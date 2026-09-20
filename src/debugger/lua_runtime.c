/*
 * lua_runtime.c - Sandboxed Lua debugger scripting
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "lua_runtime.h"
#include "../system/execution_policy.h"

#include "debugger.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../third_party/lua/lauxlib.h"
#include "../third_party/lua/lua.h"
#include "../third_party/lua/lualib.h"

#include <stdio.h>
#include <string.h>

enum {
    LUA_CALLBACK_LIMIT = 128,
    LUA_LOG_CAPACITY = 8192,
    LUA_ERROR_CAPACITY = 1024,
    LUA_NAME_CAPACITY = 128,
    LUA_DEFAULT_BUDGET = 100000
};

typedef enum {
    LUA_CALLBACK_READ,
    LUA_CALLBACK_WRITE,
    LUA_CALLBACK_EXEC
} LuaCallbackType;

typedef struct {
    uint32_t id;
    LuaCallbackType type;
    uint16_t first;
    uint16_t last;
    int reference;
} LuaMemoryCallback;

typedef struct {
    lua_State *state;
    LuaMemoryCallback callbacks[LUA_CALLBACK_LIMIT];
    size_t callback_count;
    uint32_t next_callback_id;
    uint32_t instruction_budget;
    uint32_t budget_remaining;
    uint32_t hook_step;
    uint32_t overlay[DEBUG_LUA_OVERLAY_WIDTH * DEBUG_LUA_OVERLAY_HEIGHT];
    char name[LUA_NAME_CAPACITY];
    char error[LUA_ERROR_CAPACITY];
    char log[LUA_LOG_CAPACITY];
    size_t log_length;
    bool loaded;
    bool faulted;
    bool inside_callback;
} LuaRuntime;

static LuaRuntime runtime = {.instruction_budget = LUA_DEFAULT_BUDGET};

static void append_log(const char *text) {
    if (!text) return;
    size_t length = strlen(text);
    if (length + 2 >= LUA_LOG_CAPACITY) {
        text += length - (LUA_LOG_CAPACITY - 3);
        length = LUA_LOG_CAPACITY - 3;
    }
    if (runtime.log_length + length + 2 >= LUA_LOG_CAPACITY) {
        size_t remove = runtime.log_length + length + 2 - LUA_LOG_CAPACITY;
        memmove(runtime.log, runtime.log + remove, runtime.log_length - remove);
        runtime.log_length -= remove;
    }
    memcpy(runtime.log + runtime.log_length, text, length);
    runtime.log_length += length;
    runtime.log[runtime.log_length++] = '\n';
    runtime.log[runtime.log_length] = '\0';
}

static void store_lua_error(lua_State *state) {
    const char *message = lua_tostring(state, -1);
    if (!message) message = "Lua runtime error";
    snprintf(runtime.error, sizeof(runtime.error), "%s", message);
    append_log(runtime.error);
    runtime.faulted = true;
    lua_pop(state, 1);
}

static void budget_hook(lua_State *state, lua_Debug *debug) {
    (void)debug;
    if (runtime.budget_remaining <= runtime.hook_step) {
        runtime.budget_remaining = 0;
        luaL_error(state, "Lua instruction budget exceeded");
        return;
    }
    runtime.budget_remaining -= runtime.hook_step;
}

static void begin_budget(lua_State *state) {
    runtime.budget_remaining = runtime.instruction_budget ? runtime.instruction_budget : 1;
    runtime.hook_step = runtime.budget_remaining < 100 ? runtime.budget_remaining : 100;
    if (!runtime.hook_step) runtime.hook_step = 1;
    lua_sethook(state, budget_hook, LUA_MASKCOUNT, (int)runtime.hook_step);
}

static void end_budget(lua_State *state) {
    lua_sethook(state, NULL, 0, 0);
}

static int lua_read_cpu(lua_State *state) {
    lua_Integer address = luaL_checkinteger(state, 1);
    luaL_argcheck(state, address >= 0 && address <= 0xFFFF, 1, "address out of range");
    lua_pushinteger(state, debugger_peek_cpu((uint16_t)address));
    return 1;
}

static int lua_read_ppu(lua_State *state) {
    lua_Integer address = luaL_checkinteger(state, 1);
    luaL_argcheck(state, address >= 0 && address <= 0x3FFF, 1, "PPU address out of range");
    lua_pushinteger(state, debugger_peek_ppu((uint16_t)address));
    return 1;
}

static int lua_write_cpu(lua_State *state) {
    lua_Integer address = luaL_checkinteger(state, 1);
    lua_Integer value = luaL_checkinteger(state, 2);
    luaL_argcheck(state, address >= 0 && address <= 0xFFFF, 1, "address out of range");
    luaL_argcheck(state, value >= 0 && value <= 0xFF, 2, "value out of range");
    bool previous = runtime.inside_callback;
    runtime.inside_callback = true;
    write_mem((uint16_t)address, (uint8_t)value);
    runtime.inside_callback = previous;
    return 0;
}

static int lua_get_register(lua_State *state) {
    const char *name = luaL_checkstring(state, 1);
    uint32_t value;
    if (!strcmp(name, "A") || !strcmp(name, "a")) value = cpu.a;
    else if (!strcmp(name, "X") || !strcmp(name, "x")) value = cpu.x;
    else if (!strcmp(name, "Y") || !strcmp(name, "y")) value = cpu.y;
    else if (!strcmp(name, "PC") || !strcmp(name, "pc")) value = cpu.pc;
    else if (!strcmp(name, "SP") || !strcmp(name, "sp")) value = cpu.sp;
    else if (!strcmp(name, "P") || !strcmp(name, "p") || !strcmp(name, "status"))
        value = cpu.status;
    else return luaL_error(state, "unknown CPU register");
    lua_pushinteger(state, value);
    return 1;
}

static int lua_set_register(lua_State *state) {
    const char *name = luaL_checkstring(state, 1);
    lua_Integer value = luaL_checkinteger(state, 2);
    luaL_argcheck(state, value >= 0 && value <= 0xFFFF, 2, "register value out of range");
    if (!debugger_set_cpu_register(name, (uint32_t)value)) return luaL_error(state, "unknown CPU register");
    return 0;
}

static bool parse_callback_type(const char *name, LuaCallbackType *type) {
    if (!strcmp(name, "read")) *type = LUA_CALLBACK_READ;
    else if (!strcmp(name, "write")) *type = LUA_CALLBACK_WRITE;
    else if (!strcmp(name, "exec") || !strcmp(name, "execute")) *type = LUA_CALLBACK_EXEC;
    else return false;
    return true;
}

static int lua_add_callback(lua_State *state) {
    const char *name = luaL_checkstring(state, 1);
    lua_Integer first = luaL_checkinteger(state, 2);
    int top = lua_gettop(state);
    lua_Integer last = first;
    int function_index = 3;
    if (top >= 4) {
        last = luaL_checkinteger(state, 3);
        function_index = 4;
    }
    luaL_argcheck(state, first >= 0 && first <= 0xFFFF, 2, "address out of range");
    luaL_argcheck(state, last >= first && last <= 0xFFFF, top >= 4 ? 3 : 2,
                  "invalid callback range");
    luaL_checktype(state, function_index, LUA_TFUNCTION);
    LuaCallbackType type;
    if (!parse_callback_type(name, &type)) return luaL_error(state, "callback type must be read, write, or exec");
    if (runtime.callback_count >= LUA_CALLBACK_LIMIT) return luaL_error(state, "too many Lua callbacks");

    lua_pushvalue(state, function_index);
    int reference = luaL_ref(state, LUA_REGISTRYINDEX);
    LuaMemoryCallback *callback = &runtime.callbacks[runtime.callback_count++];
    callback->id = ++runtime.next_callback_id;
    callback->type = type;
    callback->first = (uint16_t)first;
    callback->last = (uint16_t)last;
    callback->reference = reference;
    lua_pushinteger(state, callback->id);
    return 1;
}

static int lua_remove_callback(lua_State *state) {
    lua_Integer id = luaL_checkinteger(state, 1);
    for (size_t i = 0; i < runtime.callback_count; ++i) {
        if (runtime.callbacks[i].id != (uint32_t)id) continue;
        luaL_unref(state, LUA_REGISTRYINDEX, runtime.callbacks[i].reference);
        memmove(&runtime.callbacks[i], &runtime.callbacks[i + 1],
                (runtime.callback_count - i - 1) * sizeof(runtime.callbacks[0]));
        --runtime.callback_count;
        lua_pushboolean(state, 1);
        return 1;
    }
    lua_pushboolean(state, 0);
    return 1;
}

static int lua_pause(lua_State *state) {
    (void)state;
    debugger_pause();
    return 0;
}

static int lua_resume_execution(lua_State *state) {
    (void)state;
    debugger_resume();
    return 0;
}

static int lua_log_message(lua_State *state) {
    size_t length = 0;
    const char *message = luaL_checklstring(state, 1, &length);
    char buffer[512];
    if (length >= sizeof(buffer)) length = sizeof(buffer) - 1;
    memcpy(buffer, message, length);
    buffer[length] = '\0';
    append_log(buffer);
    return 0;
}

static void put_pixel(int x, int y, uint32_t color) {
    if (x < 0 || x >= DEBUG_LUA_OVERLAY_WIDTH || y < 0 || y >= DEBUG_LUA_OVERLAY_HEIGHT) return;
    runtime.overlay[(size_t)y * DEBUG_LUA_OVERLAY_WIDTH + (size_t)x] = color;
}

static uint32_t lua_color(lua_State *state, int index) {
    lua_Integer color = luaL_checkinteger(state, index);
    luaL_argcheck(state, color >= 0 && (lua_Unsigned)color <= UINT32_MAX, index,
                  "color must be a 32-bit unsigned value");
    return (uint32_t)color;
}

static int lua_coordinate(lua_State *state, int index, int minimum, int maximum) {
    lua_Integer value = luaL_checkinteger(state, index);
    luaL_argcheck(state, value >= minimum && value <= maximum, index,
                  "coordinate is too far outside the overlay");
    return (int)value;
}

static int lua_draw_pixel(lua_State *state) {
    int x = lua_coordinate(state, 1, -DEBUG_LUA_OVERLAY_WIDTH, DEBUG_LUA_OVERLAY_WIDTH * 2);
    int y = lua_coordinate(state, 2, -DEBUG_LUA_OVERLAY_HEIGHT, DEBUG_LUA_OVERLAY_HEIGHT * 2);
    uint32_t color = lua_color(state, 3);
    put_pixel(x, y, color);
    return 0;
}

static void draw_line(int x0, int y0, int x1, int y1, uint32_t color) {
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int sx = x0 < x1 ? 1 : -1;
    int dy_abs = y1 > y0 ? y1 - y0 : y0 - y1;
    int dy = -dy_abs;
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        put_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int twice = error * 2;
        if (twice >= dy) { error += dy; x0 += sx; }
        if (twice <= dx) { error += dx; y0 += sy; }
    }
}

static int lua_draw_line(lua_State *state) {
    int x0 = lua_coordinate(state, 1, -DEBUG_LUA_OVERLAY_WIDTH, DEBUG_LUA_OVERLAY_WIDTH * 2);
    int y0 = lua_coordinate(state, 2, -DEBUG_LUA_OVERLAY_HEIGHT, DEBUG_LUA_OVERLAY_HEIGHT * 2);
    int x1 = lua_coordinate(state, 3, -DEBUG_LUA_OVERLAY_WIDTH, DEBUG_LUA_OVERLAY_WIDTH * 2);
    int y1 = lua_coordinate(state, 4, -DEBUG_LUA_OVERLAY_HEIGHT, DEBUG_LUA_OVERLAY_HEIGHT * 2);
    uint32_t color = lua_color(state, 5);
    if ((x0 < 0 && x1 < 0) || (x0 >= DEBUG_LUA_OVERLAY_WIDTH && x1 >= DEBUG_LUA_OVERLAY_WIDTH)
        || (y0 < 0 && y1 < 0) || (y0 >= DEBUG_LUA_OVERLAY_HEIGHT && y1 >= DEBUG_LUA_OVERLAY_HEIGHT))
        return 0;
    if (x0 < 0) x0 = 0; else if (x0 >= DEBUG_LUA_OVERLAY_WIDTH) x0 = DEBUG_LUA_OVERLAY_WIDTH - 1;
    if (x1 < 0) x1 = 0; else if (x1 >= DEBUG_LUA_OVERLAY_WIDTH) x1 = DEBUG_LUA_OVERLAY_WIDTH - 1;
    if (y0 < 0) y0 = 0; else if (y0 >= DEBUG_LUA_OVERLAY_HEIGHT) y0 = DEBUG_LUA_OVERLAY_HEIGHT - 1;
    if (y1 < 0) y1 = 0; else if (y1 >= DEBUG_LUA_OVERLAY_HEIGHT) y1 = DEBUG_LUA_OVERLAY_HEIGHT - 1;
    draw_line(x0, y0, x1, y1, color);
    return 0;
}

static int lua_draw_rect(lua_State *state) {
    int x = lua_coordinate(state, 1, -DEBUG_LUA_OVERLAY_WIDTH, DEBUG_LUA_OVERLAY_WIDTH * 2);
    int y = lua_coordinate(state, 2, -DEBUG_LUA_OVERLAY_HEIGHT, DEBUG_LUA_OVERLAY_HEIGHT * 2);
    lua_Integer requested_width = luaL_checkinteger(state, 3);
    lua_Integer requested_height = luaL_checkinteger(state, 4);
    luaL_argcheck(state, requested_width >= 0 && requested_width <= DEBUG_LUA_OVERLAY_WIDTH * 3,
                  3, "rectangle width is out of range");
    luaL_argcheck(state, requested_height >= 0 && requested_height <= DEBUG_LUA_OVERLAY_HEIGHT * 3,
                  4, "rectangle height is out of range");
    int width = (int)requested_width, height = (int)requested_height;
    uint32_t color = lua_color(state, 5);
    int filled = lua_toboolean(state, 6);
    if (width <= 0 || height <= 0) return 0;
    int64_t right64 = (int64_t)x + width - 1;
    int64_t bottom64 = (int64_t)y + height - 1;
    if (right64 < 0 || bottom64 < 0 || x >= DEBUG_LUA_OVERLAY_WIDTH || y >= DEBUG_LUA_OVERLAY_HEIGHT)
        return 0;
    int left = x < 0 ? 0 : x, top = y < 0 ? 0 : y;
    int right = right64 >= DEBUG_LUA_OVERLAY_WIDTH ? DEBUG_LUA_OVERLAY_WIDTH - 1 : (int)right64;
    int bottom = bottom64 >= DEBUG_LUA_OVERLAY_HEIGHT ? DEBUG_LUA_OVERLAY_HEIGHT - 1 : (int)bottom64;
    if (filled) {
        for (int row = top; row <= bottom; ++row)
            for (int column = left; column <= right; ++column)
                put_pixel(column, row, color);
    } else {
        draw_line(left, top, right, top, color);
        draw_line(left, bottom, right, bottom, color);
        draw_line(left, top, left, bottom, color);
        draw_line(right, top, right, bottom, color);
    }
    return 0;
}

static int lua_clear_overlay(lua_State *state) {
    (void)state;
    debugger_lua_clear_overlay();
    return 0;
}

static int lua_get_ppu_state(lua_State *state) {
    DebugPpuSnapshot snapshot;
    debugger_get_ppu(&snapshot);
    lua_createtable(state, 0, 10);
#define SET_INT(name, value) do { lua_pushinteger(state, (lua_Integer)(value)); lua_setfield(state, -2, (name)); } while (0)
    SET_INT("ctrl", snapshot.ctrl); SET_INT("mask", snapshot.mask); SET_INT("status", snapshot.status);
    SET_INT("v", snapshot.v); SET_INT("t", snapshot.t); SET_INT("fineX", snapshot.fine_x);
    SET_INT("scanline", snapshot.scanline); SET_INT("dot", snapshot.dot);
    SET_INT("frame", snapshot.frame); SET_INT("cycles", snapshot.ppu_cycles);
#undef SET_INT
    return 1;
}

static int lua_get_cpu_cycles(lua_State *state) {
    lua_pushinteger(state, (lua_Integer)cpu_total_cycles);
    return 1;
}

static void open_safe_libraries(lua_State *state) {
    static const struct { const char *name; lua_CFunction open; } libraries[] = {
        {"_G", luaopen_base}, {LUA_COLIBNAME, luaopen_coroutine},
        {LUA_TABLIBNAME, luaopen_table}, {LUA_STRLIBNAME, luaopen_string},
        {LUA_MATHLIBNAME, luaopen_math}, {LUA_UTF8LIBNAME, luaopen_utf8}
    };
    for (size_t i = 0; i < sizeof(libraries) / sizeof(libraries[0]); ++i) {
        luaL_requiref(state, libraries[i].name, libraries[i].open, 1);
        lua_pop(state, 1);
    }
}

static void install_emu_api(lua_State *state) {
    static const luaL_Reg functions[] = {
        {"read", lua_read_cpu}, {"readPpu", lua_read_ppu}, {"write", lua_write_cpu},
        {"getRegister", lua_get_register}, {"setRegister", lua_set_register},
        {"getPpuState", lua_get_ppu_state}, {"getCpuCycleCount", lua_get_cpu_cycles},
        {"addMemoryCallback", lua_add_callback}, {"removeMemoryCallback", lua_remove_callback},
        {"pause", lua_pause}, {"resume", lua_resume_execution}, {"log", lua_log_message},
        {"drawPixel", lua_draw_pixel}, {"drawLine", lua_draw_line},
        {"drawRectangle", lua_draw_rect}, {"clearOverlay", lua_clear_overlay},
        {NULL, NULL}
    };
    luaL_newlib(state, functions);
    lua_setglobal(state, "emu");
}

bool debugger_lua_load(const char *name, const char *source) {
    if (!source) return false;
    if (!nes_execution_allows_host_configuration()) return false;
    debugger_lua_unload();
    runtime.state = luaL_newstate();
    if (!runtime.state) {
        snprintf(runtime.error, sizeof(runtime.error), "Unable to create Lua state");
        runtime.faulted = true;
        return false;
    }
    runtime.instruction_budget = runtime.instruction_budget ? runtime.instruction_budget : LUA_DEFAULT_BUDGET;
    runtime.next_callback_id = 0;
    runtime.error[0] = '\0'; runtime.log[0] = '\0'; runtime.log_length = 0;
    snprintf(runtime.name, sizeof(runtime.name), "%s", name ? name : "script");
    open_safe_libraries(runtime.state);
    install_emu_api(runtime.state);
    if (luaL_loadbufferx(runtime.state, source, strlen(source), runtime.name, "t") != LUA_OK) {
        store_lua_error(runtime.state);
        return false;
    }
    begin_budget(runtime.state);
    int result = lua_pcall(runtime.state, 0, 0, 0);
    end_budget(runtime.state);
    if (result != LUA_OK) {
        store_lua_error(runtime.state);
        return false;
    }
    runtime.loaded = true;
    runtime.faulted = false;
    append_log("Script loaded");
    return true;
}

void debugger_lua_unload(void) {
    if (runtime.state) lua_close(runtime.state);
    runtime.state = NULL;
    runtime.callback_count = 0;
    runtime.next_callback_id = 0;
    runtime.loaded = false;
    runtime.faulted = false;
    runtime.inside_callback = false;
    runtime.error[0] = '\0';
    runtime.name[0] = '\0';
    runtime.log[0] = '\0'; runtime.log_length = 0;
    debugger_lua_clear_overlay();
}

bool debugger_lua_loaded(void) { return runtime.loaded; }
bool debugger_lua_faulted(void) { return runtime.faulted; }
const char *debugger_lua_error(void) { return runtime.error; }
const char *debugger_lua_log(void) { return runtime.log; }
void debugger_lua_set_instruction_budget(uint32_t instructions) {
    runtime.instruction_budget = instructions ? instructions : 1;
}
uint32_t debugger_lua_instruction_budget(void) { return runtime.instruction_budget; }
const uint32_t *debugger_lua_overlay(void) { return runtime.overlay; }
void debugger_lua_clear_overlay(void) { memset(runtime.overlay, 0, sizeof(runtime.overlay)); }

static bool callback_matches(const LuaMemoryCallback *callback, LuaCallbackType type, uint16_t address) {
    return callback->type == type && address >= callback->first && address <= callback->last;
}

static bool call_callback(LuaMemoryCallback *callback, uint16_t address,
                          uint8_t *value, bool can_replace) {
    lua_State *state = runtime.state;
    if (!state) return false;
    int top = lua_gettop(state);
    lua_rawgeti(state, LUA_REGISTRYINDEX, callback->reference);
    lua_pushinteger(state, address);
    int arguments = 1;
    if (value) { lua_pushinteger(state, *value); arguments = 2; }
    begin_budget(state);
    runtime.inside_callback = true;
    int result = lua_pcall(state, arguments, can_replace ? 1 : 0, 0);
    runtime.inside_callback = false;
    end_budget(state);
    if (result != LUA_OK) {
        store_lua_error(state);
        lua_settop(state, top);
        return false;
    }
    if (can_replace && value && lua_isinteger(state, -1)) {
        lua_Integer replacement = lua_tointeger(state, -1);
        if (replacement >= 0 && replacement <= 0xFF) *value = (uint8_t)replacement;
    }
    lua_settop(state, top);
    return true;
}

static void dispatch_callbacks(LuaCallbackType type, uint16_t address,
                               uint8_t *value, bool can_replace) {
    if (!runtime.loaded || runtime.faulted || runtime.inside_callback) return;
    for (size_t i = 0; i < runtime.callback_count; ++i) {
        LuaMemoryCallback *callback = &runtime.callbacks[i];
        if (!callback_matches(callback, type, address)) continue;
        if (!call_callback(callback, address, value, can_replace)) break;
    }
}

void debugger_lua_on_execute(uint16_t address) {
    dispatch_callbacks(LUA_CALLBACK_EXEC, address, NULL, false);
}

void debugger_lua_on_read(uint16_t address, uint8_t *value) {
    dispatch_callbacks(LUA_CALLBACK_READ, address, value, true);
}

void debugger_lua_on_write(uint16_t address, uint8_t value) {
    uint8_t copy = value;
    dispatch_callbacks(LUA_CALLBACK_WRITE, address, &copy, false);
}
