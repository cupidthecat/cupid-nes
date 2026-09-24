/*
 * tas_script.c - Sandboxed one-shot Lua automation for TAS projects
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_script.h"

#include "../third_party/lua/lauxlib.h"
#include "../third_party/lua/lua.h"
#include "../third_party/lua/lualib.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    TAS_SCRIPT_MEMORY_LIMIT = 16 * 1024 * 1024,
    TAS_SCRIPT_INSTRUCTION_BUDGET = 1000000,
    TAS_SCRIPT_HOOK_STEP = 1000,
    TAS_SCRIPT_MAX_FRAME_GROWTH = 262144
};

typedef enum { TAS_SCRIPT_CHANGE_INPUT = 0, TAS_SCRIPT_CHANGE_INSERT, TAS_SCRIPT_CHANGE_DELETE } TasScriptChangeType;

typedef struct {
    TasScriptChangeType type;
    size_t frame;
    size_t count;
    unsigned controller;
    uint8_t input;
} TasScriptChange;

typedef union {
    max_align_t alignment;
    size_t size;
} TasScriptAllocation;

typedef struct {
    NesTasProject *project;
    TasScriptChange *pending;
    size_t pending_count;
    size_t pending_capacity;
    size_t memory_used;
    size_t memory_limit;
    size_t max_frame_count;
    uint32_t instructions_remaining;
} TasScriptContext;

static void set_error(char *error, size_t capacity, const char *message) {
    if (!error || !capacity) {
        return;
    }
    if (!message) {
        message = "TAS Lua script failed";
    }
    size_t length = strlen(message);
    if (length >= capacity) {
        length = capacity - 1;
    }
    memcpy(error, message, length);
    error[length] = '\0';
}

static void *script_realloc(TasScriptContext *context, void *pointer, size_t size) {
    if (!context) {
        return NULL;
    }
    TasScriptAllocation *old_block = pointer ? ((TasScriptAllocation *)pointer - 1) : NULL;
    size_t old_size = old_block ? old_block->size : 0;
    if (!size) {
        if (old_block) {
            if (context->memory_used >= old_size) {
                context->memory_used -= old_size;
            } else {
                context->memory_used = 0;
            }
            free(old_block);
        }
        return NULL;
    }
    if (size > SIZE_MAX - sizeof(TasScriptAllocation)) {
        return NULL;
    }
    size_t used_without_old = context->memory_used >= old_size ? context->memory_used - old_size : 0;
    if (size > context->memory_limit - used_without_old) {
        return NULL;
    }
    TasScriptAllocation *replacement = (TasScriptAllocation *)realloc(old_block, sizeof(*replacement) + size);
    if (!replacement) {
        return NULL;
    }
    replacement->size = size;
    context->memory_used = used_without_old + size;
    return replacement + 1;
}

static void *lua_allocator(void *opaque, void *pointer, size_t old_size, size_t new_size) {
    (void)old_size;
    return script_realloc((TasScriptContext *)opaque, pointer, new_size);
}

static TasScriptContext *script_context(lua_State *state) {
    return (TasScriptContext *)lua_touserdata(state, lua_upvalueindex(1));
}

static TasScriptContext *hook_context(lua_State *state) {
    return *(TasScriptContext **)lua_getextraspace(state);
}

static void instruction_hook(lua_State *state, lua_Debug *debug) {
    (void)debug;
    TasScriptContext *context = hook_context(state);
    if (!context || context->instructions_remaining <= TAS_SCRIPT_HOOK_STEP) {
        if (context) {
            context->instructions_remaining = 0;
        }
        luaL_error(state, "TAS Lua instruction limit exceeded");
        return;
    }
    context->instructions_remaining -= TAS_SCRIPT_HOOK_STEP;
}

static size_t check_size_argument(lua_State *state, int argument, const char *message) {
    lua_Integer value = luaL_checkinteger(state, argument);
    luaL_argcheck(state, value >= 0, argument, message);
    if (sizeof(size_t) < sizeof(lua_Integer)) {
        luaL_argcheck(state, (uint64_t)value <= (uint64_t)SIZE_MAX, argument, message);
    }
    return (size_t)value;
}

static unsigned check_controller(lua_State *state, int argument) {
    lua_Integer value = luaL_checkinteger(state, argument);
    luaL_argcheck(state, value >= 0 && value <= 4, argument, "controller must be 0 (commands) or 1..4 (NES pads)");
    return (unsigned)value;
}

static uint8_t check_input_byte(lua_State *state, int argument) {
    lua_Integer value = luaL_checkinteger(state, argument);
    luaL_argcheck(state, value >= 0 && value <= 255, argument, "input must be a byte from 0 through 255");
    return (uint8_t)value;
}

static bool model_ok(lua_State *state, const char *operation, NesTasResult result) {
    if (result == NES_TAS_OK) {
        return true;
    }
    luaL_error(state, "%s failed: %s", operation, nes_tas_result_string(result));
    return false;
}

static bool frame_count_fits(const TasScriptContext *context, size_t count) {
    return context && count <= context->max_frame_count;
}

static int frame_limit_error(lua_State *state) {
    return luaL_error(state, "TAS Lua frame growth limit exceeded");
}

static bool pending_reserve(lua_State *state, TasScriptContext *context, size_t count) {
    if (count <= context->pending_capacity) {
        return true;
    }
    size_t capacity = context->pending_capacity ? context->pending_capacity : 32;
    while (capacity < count) {
        if (capacity > SIZE_MAX / 2) {
            luaL_error(state, "too many pending TAS input changes");
            return false;
        }
        capacity *= 2;
    }
    if (capacity > SIZE_MAX / sizeof(*context->pending)) {
        luaL_error(state, "too many pending TAS input changes");
        return false;
    }
    TasScriptChange *replacement =
        (TasScriptChange *)script_realloc(context, context->pending, capacity * sizeof(*replacement));
    if (!replacement) {
        luaL_error(state, "TAS Lua memory limit exceeded");
        return false;
    }
    context->pending = replacement;
    context->pending_capacity = capacity;
    return true;
}

static int stage_change(lua_State *state, const TasScriptChange *change) {
    TasScriptContext *context = script_context(state);
    if (!context || !change) {
        return luaL_error(state, "TAS Lua internal error");
    }
    if (!pending_reserve(state, context, context->pending_count + 1)) {
        return 0;
    }
    context->pending[context->pending_count++] = *change;
    return 0;
}

static int lua_frame_count(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t count = nes_tas_project_frame_count(context->project);
    if (count > (size_t)LUA_MAXINTEGER) {
        return luaL_error(state, "TAS frame count exceeds Lua integer range");
    }
    lua_pushinteger(state, (lua_Integer)count);
    return 1;
}

static int lua_get_input(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    unsigned controller = check_controller(state, 2);
    const NesFm2Frame *input = nes_tas_project_frame(context->project, frame);
    if (!input) {
        lua_pushinteger(state, 0);
        return 1;
    }
    lua_pushinteger(state, controller ? input->pads[controller - 1] : input->commands);
    return 1;
}

static int lua_submit_input_change(lua_State *state) {
    TasScriptContext *context = script_context(state);
    TasScriptChange change;
    memset(&change, 0, sizeof(change));
    change.type = TAS_SCRIPT_CHANGE_INPUT;
    change.frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    change.controller = check_controller(state, 2);
    change.input = check_input_byte(state, 3);
    if (change.frame >= context->max_frame_count) {
        return frame_limit_error(state);
    }
    return stage_change(state, &change);
}

static int lua_submit_insert_frames(lua_State *state) {
    TasScriptContext *context = script_context(state);
    TasScriptChange change;
    memset(&change, 0, sizeof(change));
    change.type = TAS_SCRIPT_CHANGE_INSERT;
    change.frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    change.count = check_size_argument(state, 2, "count must be a positive integer");
    luaL_argcheck(state, change.count > 0, 2, "count must be a positive integer");
    if (change.frame >= context->max_frame_count || change.count > context->max_frame_count) {
        return frame_limit_error(state);
    }
    return stage_change(state, &change);
}

static int lua_submit_delete_frames(lua_State *state) {
    TasScriptContext *context = script_context(state);
    TasScriptChange change;
    memset(&change, 0, sizeof(change));
    change.type = TAS_SCRIPT_CHANGE_DELETE;
    change.frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    change.count = check_size_argument(state, 2, "count must be a positive integer");
    luaL_argcheck(state, change.count > 0, 2, "count must be a positive integer");
    if (change.frame >= context->max_frame_count) {
        return frame_limit_error(state);
    }
    return stage_change(state, &change);
}

static bool ensure_frame_exists(lua_State *state, TasScriptContext *context, size_t frame) {
    size_t current = nes_tas_project_frame_count(context->project);
    if (frame < current) {
        return true;
    }
    if (frame == SIZE_MAX) {
        frame_limit_error(state);
        return false;
    }
    size_t target = frame + 1;
    if (!frame_count_fits(context, target)) {
        frame_limit_error(state);
        return false;
    }
    return model_ok(state, "expanding TAS input", nes_tas_insert_frames(context->project, current, target - current));
}

static bool apply_input_change(lua_State *state, TasScriptContext *context, const TasScriptChange *change) {
    if (!ensure_frame_exists(state, context, change->frame)) {
        return false;
    }
    const NesFm2Frame *existing = nes_tas_project_frame(context->project, change->frame);
    if (!existing) {
        luaL_error(state, "TAS input frame disappeared during script edit");
        return false;
    }
    NesFm2Frame replacement = *existing;
    if (change->controller) {
        replacement.pads[change->controller - 1] = change->input;
    } else {
        replacement.commands = change->input;
    }
    return model_ok(state, "changing TAS input", nes_tas_set_frame(context->project, change->frame, &replacement));
}

static bool apply_insert_change(lua_State *state, TasScriptContext *context, const TasScriptChange *change) {
    size_t current = nes_tas_project_frame_count(context->project);
    if (change->frame > current) {
        size_t padding = change->frame - current;
        if (padding > context->max_frame_count - current) {
            frame_limit_error(state);
            return false;
        }
        if (!model_ok(state, "expanding TAS input", nes_tas_insert_frames(context->project, current, padding))) {
            return false;
        }
        current = change->frame;
    }
    if (change->count > context->max_frame_count - current) {
        frame_limit_error(state);
        return false;
    }
    return model_ok(state, "inserting TAS frames",
                    nes_tas_insert_frames(context->project, change->frame, change->count));
}

static bool apply_delete_change(lua_State *state, TasScriptContext *context, const TasScriptChange *change) {
    if (!ensure_frame_exists(state, context, change->frame)) {
        return false;
    }
    size_t current = nes_tas_project_frame_count(context->project);
    size_t available = current - change->frame;
    size_t count = change->count < available ? change->count : available;
    if (!count) {
        return true;
    }
    return model_ok(state, "deleting TAS frames", nes_tas_delete_frames(context->project, change->frame, count));
}

static int lua_apply_input_changes(lua_State *state) {
    TasScriptContext *context = script_context(state);
    if (!context->pending_count) {
        lua_pushinteger(state, -1);
        return 1;
    }
    uint64_t before = nes_tas_project_revision(context->project);
    for (size_t i = 0; i < context->pending_count; ++i) {
        const TasScriptChange *change = &context->pending[i];
        bool ok = change->type == TAS_SCRIPT_CHANGE_INPUT    ? apply_input_change(state, context, change)
                  : change->type == TAS_SCRIPT_CHANGE_INSERT ? apply_insert_change(state, context, change)
                                                             : apply_delete_change(state, context, change);
        if (!ok) {
            return 0;
        }
    }
    context->pending_count = 0;
    NesTasChange change = nes_tas_project_change_since(context->project, before);
    if (change.first_changed_frame == SIZE_MAX) {
        lua_pushinteger(state, -1);
    } else if (change.first_changed_frame > (size_t)LUA_MAXINTEGER) {
        return luaL_error(state, "changed TAS frame exceeds Lua integer range");
    } else {
        lua_pushinteger(state, (lua_Integer)change.first_changed_frame);
    }
    return 1;
}

static int lua_clear_input_changes(lua_State *state) {
    TasScriptContext *context = script_context(state);
    context->pending_count = 0;
    return 0;
}

static int lua_get_selection(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t frames = nes_tas_project_frame_count(context->project);
    size_t selected = nes_tas_selection_count(context->project);
    int array_hint = selected > (size_t)INT_MAX ? INT_MAX : (int)selected;
    lua_createtable(state, array_hint, 0);
    lua_Integer out = 1;
    for (size_t frame = 0; frame < frames; ++frame) {
        if (!nes_tas_selection_contains(context->project, frame)) {
            continue;
        }
        if (frame > (size_t)LUA_MAXINTEGER) {
            return luaL_error(state, "selected TAS frame exceeds Lua integer range");
        }
        lua_pushinteger(state, (lua_Integer)frame);
        lua_seti(state, -2, out++);
    }
    return 1;
}

static bool exact_marker(const NesTasProject *project, size_t frame, NesTasMarkerView *out) {
    size_t count = nes_tas_marker_count(project);
    for (size_t i = 0; i < count; ++i) {
        NesTasMarkerView view;
        if (!nes_tas_marker(project, i, &view)) {
            break;
        }
        if (view.frame == frame) {
            if (out) {
                *out = view;
            }
            return true;
        }
        if (view.frame > frame) {
            break;
        }
    }
    return false;
}

static int lua_get_marker(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    NesTasMarkerView marker;
    if (!exact_marker(context->project, frame, &marker)) {
        lua_pushnil(state);
        return 1;
    }
    lua_pushstring(state, marker.note ? marker.note : "");
    return 1;
}

static int lua_set_marker(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    size_t note_length = 0;
    const char *note = luaL_checklstring(state, 2, &note_length);
    luaL_argcheck(state, strlen(note) == note_length, 2, "marker note cannot contain NUL bytes");
    if (!model_ok(state, "setting TAS marker", nes_tas_marker_set(context->project, frame, note))) {
        return 0;
    }
    return 0;
}

static int lua_remove_marker(lua_State *state) {
    TasScriptContext *context = script_context(state);
    size_t frame = check_size_argument(state, 1, "frame must be a non-negative integer");
    if (!model_ok(state, "removing TAS marker", nes_tas_marker_remove(context->project, frame))) {
        return 0;
    }
    return 0;
}

static int lua_get_current_branch(lua_State *state) {
    TasScriptContext *context = script_context(state);
    lua_pushinteger(state, nes_tas_current_branch(context->project));
    return 1;
}

static void remove_global(lua_State *state, const char *name) {
    lua_pushnil(state);
    lua_setglobal(state, name);
}

static int initialize_environment(lua_State *state) {
    static const struct {
        const char *name;
        lua_CFunction open;
    } libraries[] = {{"_G", luaopen_base},
                     {LUA_MATHLIBNAME, luaopen_math},
                     {LUA_STRLIBNAME, luaopen_string},
                     {LUA_TABLIBNAME, luaopen_table}};

    static const luaL_Reg functions[] = {{"framecount", lua_frame_count},
                                         {"getinput", lua_get_input},
                                         {"submitinputchange", lua_submit_input_change},
                                         {"submitinsertframes", lua_submit_insert_frames},
                                         {"submitdeleteframes", lua_submit_delete_frames},
                                         {"applyinputchanges", lua_apply_input_changes},
                                         {"clearinputchanges", lua_clear_input_changes},
                                         {"getselection", lua_get_selection},
                                         {"getmarker", lua_get_marker},
                                         {"setmarker", lua_set_marker},
                                         {"removemarker", lua_remove_marker},
                                         {"getcurrentbranch", lua_get_current_branch},
                                         {NULL, NULL}};
    for (size_t i = 0; i < sizeof(libraries) / sizeof(libraries[0]); ++i) {
        luaL_requiref(state, libraries[i].name, libraries[i].open, 1);
        lua_pop(state, 1);
    }

    /* File/system entry points are excluded, and protected calls are removed so
     * the hard instruction-limit error cannot be swallowed and retried forever. */
    static const char *blocked[] = {"dofile", "loadfile", "load", "pcall", "xpcall", "print", "warn"};
    for (size_t i = 0; i < sizeof(blocked) / sizeof(blocked[0]); ++i) {
        remove_global(state, blocked[i]);
    }

    TasScriptContext *context = hook_context(state);
    lua_createtable(state, 0, 12);
    lua_pushlightuserdata(state, context);
    luaL_setfuncs(state, functions, 1);
    lua_setglobal(state, "taseditor");
    return 0;
}

static void set_lua_error(lua_State *state, int status, char *error, size_t capacity) {
    if (status == LUA_ERRMEM) {
        set_error(error, capacity, "TAS Lua memory limit exceeded");
        return;
    }
    const char *message = state ? lua_tostring(state, -1) : NULL;
    set_error(error, capacity, message ? message : "TAS Lua script failed");
}

static void free_context_allocations(TasScriptContext *context) {
    if (!context) {
        return;
    }
    context->pending = (TasScriptChange *)script_realloc(context, context->pending, 0);
    context->pending_count = 0;
    context->pending_capacity = 0;
}

bool nes_tas_run_script(NesTasProject *project, const char *source, size_t length, char *error, size_t capacity) {
    if (error && capacity) {
        error[0] = '\0';
    }
    if (!project || !source) {
        set_error(error, capacity, "TAS Lua requires a project and source text");
        return false;
    }
    if (nes_tas_edit_active(project)) {
        set_error(error, capacity, "TAS Lua cannot run during another project edit");
        return false;
    }

    TasScriptContext context;
    memset(&context, 0, sizeof(context));
    context.project = project;
    context.memory_limit = TAS_SCRIPT_MEMORY_LIMIT;
    size_t initial_frames = nes_tas_project_frame_count(project);
    context.max_frame_count = initial_frames > SIZE_MAX - TAS_SCRIPT_MAX_FRAME_GROWTH
                                  ? SIZE_MAX
                                  : initial_frames + TAS_SCRIPT_MAX_FRAME_GROWTH;
    context.instructions_remaining = TAS_SCRIPT_INSTRUCTION_BUDGET;

    lua_State *state = lua_newstate(lua_allocator, &context);
    if (!state) {
        set_error(error, capacity, "Unable to create bounded TAS Lua state");
        return false;
    }
    *(TasScriptContext **)lua_getextraspace(state) = &context;

    lua_pushcfunction(state, initialize_environment);
    int status = lua_pcall(state, 0, 0, 0);
    if (status != LUA_OK) {
        set_lua_error(state, status, error, capacity);
        lua_close(state);
        free_context_allocations(&context);
        return false;
    }

    status = luaL_loadbufferx(state, source, length, "tas-script", "t");
    if (status != LUA_OK) {
        set_lua_error(state, status, error, capacity);
        lua_close(state);
        free_context_allocations(&context);
        return false;
    }

    NesTasResult edit = nes_tas_edit_begin(project);
    if (edit != NES_TAS_OK) {
        char message[160];
        snprintf(message, sizeof(message), "Unable to begin TAS Lua edit: %s", nes_tas_result_string(edit));
        set_error(error, capacity, message);
        lua_close(state);
        free_context_allocations(&context);
        return false;
    }

    lua_sethook(state, instruction_hook, LUA_MASKCOUNT, TAS_SCRIPT_HOOK_STEP);
    status = lua_pcall(state, 0, 0, 0);
    lua_sethook(state, NULL, 0, 0);
    if (status != LUA_OK) {
        set_lua_error(state, status, error, capacity);
        nes_tas_edit_cancel(project);
        lua_close(state);
        free_context_allocations(&context);
        return false;
    }

    edit = nes_tas_edit_end(project);
    if (edit != NES_TAS_OK) {
        if (nes_tas_edit_active(project)) {
            nes_tas_edit_cancel(project);
        }
        char message[160];
        snprintf(message, sizeof(message), "Unable to finish TAS Lua edit: %s", nes_tas_result_string(edit));
        set_error(error, capacity, message);
        lua_close(state);
        free_context_allocations(&context);
        return false;
    }

    lua_close(state);
    free_context_allocations(&context);
    return true;
}
