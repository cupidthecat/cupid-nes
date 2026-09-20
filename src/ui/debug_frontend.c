/*
 * debug_frontend.c - Desktop debugger, viewer, and Lua controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "debug_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../debugger/debugger.h"
#include "../debugger/lua_runtime.h"
#include "../util/file_io.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    DEBUG_CONTROL_CPU,
    DEBUG_CONTROL_REGISTER,
    DEBUG_CONTROL_STOP,
    DEBUG_CONTROL_TOGGLE,
    DEBUG_CONTROL_STEP_INTO,
    DEBUG_CONTROL_STEP_OVER,
    DEBUG_CONTROL_STEP_OUT,
    DEBUG_CONTROL_BREAK_ADDRESS,
    DEBUG_CONTROL_BREAK_TYPE,
    DEBUG_CONTROL_BREAK_LIST,
    DEBUG_CONTROL_BREAK_ADD,
    DEBUG_CONTROL_BREAK_TOGGLE,
    DEBUG_CONTROL_BREAK_REMOVE,
    DEBUG_CONTROL_TRACE,
    DEBUG_CONTROL_TRACE_CLEAR
};

enum {
    VIEW_CONTROL_MODE = 1,
    VIEW_CONTROL_ADDRESS,
    VIEW_CONTROL_SNAPSHOT,
    VIEW_CONTROL_WATCH_ADDRESS,
    VIEW_CONTROL_WATCH_SIZE,
    VIEW_CONTROL_WATCH_LABEL,
    VIEW_CONTROL_WATCH_ADD,
    VIEW_CONTROL_WATCH_LIST,
    VIEW_CONTROL_WATCH_TOGGLE,
    VIEW_CONTROL_WATCH_REMOVE,
    VIEW_CONTROL_TRACE_LAST
};

enum {
    LUA_CONTROL_PATH = 1,
    LUA_CONTROL_LOAD,
    LUA_CONTROL_UNLOAD,
    LUA_CONTROL_BUDGET,
    LUA_CONTROL_STATE,
    LUA_CONTROL_OVERLAY,
    LUA_CONTROL_CLEAR_OVERLAY,
    LUA_CONTROL_ERROR,
    LUA_CONTROL_LOG
};

enum { DEBUG_BREAK_MAX = 128, DEBUG_WATCH_MAX = 64, LUA_SOURCE_LIMIT = 1024 * 1024 };

struct DebugFrontend {
    FrontendExecutionRuntime *execution;
    char status[256];
    char cpu_text[160];
    char stop_text[128];
    char register_edit[32];
    char breakpoint_address[32];
    int breakpoint_type;
    int breakpoint_selected;
    char breakpoint_labels[DEBUG_BREAK_MAX][80];
    const char *breakpoint_items[DEBUG_BREAK_MAX];
    char view_address[32];
    int view_mode;
    char view_text[256];
    char watch_address[32];
    int watch_size;
    char watch_label[48];
    int watch_selected;
    char watch_labels[DEBUG_WATCH_MAX][96];
    const char *watch_items[DEBUG_WATCH_MAX];
    char trace_text[128];
    char lua_path[NES_FILE_PATH_LIMIT];
    char lua_budget[32];
    char lua_state[96];
    char lua_overlay[64];
    char lua_error[192];
    char lua_log[256];
    bool registered;
};

static bool fail(DebugFrontend *frontend, char *error, size_t error_size,
                 const char *message) {
    if (frontend) snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && error_size) snprintf(error, error_size, "%s", message);
    return false;
}

static bool parse_hex16(const char *text, uint16_t *value) {
    if (!text || !*text || !value) return false;
    if (text[0] == '$') ++text;
    else if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) text += 2;
    char *end = NULL;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, 16);
    if (errno || end == text || *end || parsed > 0xFFFFu) return false;
    *value = (uint16_t)parsed;
    return true;
}

static bool parse_u32(const char *text, uint32_t *value) {
    if (!text || !*text || !value) return false;
    char *end = NULL;
    errno = 0;
    unsigned long long parsed = strtoull(text, &end, 0);
    if (errno || end == text || *end || parsed > UINT32_MAX) return false;
    *value = (uint32_t)parsed;
    return true;
}

static void sync_execution(DebugFrontend *frontend) {
    if (frontend && frontend->execution) frontend_execution_sync_debugger(frontend->execution);
}

static bool command_toggle(void *context, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    if (debugger_is_paused()) debugger_resume();
    else debugger_pause();
    sync_execution(frontend);
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool command_pause(void *context, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    debugger_pause();
    sync_execution(frontend);
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool command_resume(void *context, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    debugger_resume();
    sync_execution(frontend);
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool command_step(void *context, unsigned which, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    bool ok = which == DEBUGGER_STEP_INTO_COMMAND ? debugger_step_into()
            : which == DEBUGGER_STEP_OVER_COMMAND ? debugger_step_over()
                                                  : debugger_step_out();
    sync_execution(frontend);
    return ok || fail(frontend, error, error_size, "Pause the debugger before stepping");
}

static bool command_step_into(void *c, char *e, size_t n) {
    return command_step(c, DEBUGGER_STEP_INTO_COMMAND, e, n);
}
static bool command_step_over(void *c, char *e, size_t n) {
    return command_step(c, DEBUGGER_STEP_OVER_COMMAND, e, n);
}
static bool command_step_out(void *c, char *e, size_t n) {
    return command_step(c, DEBUGGER_STEP_OUT_COMMAND, e, n);
}

static bool command_trace(void *context, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    debugger_trace_enable(!debugger_trace_enabled());
    (void)frontend_command_set_checked(DEBUGGER_VIEWERS_FRONTEND_COMMAND,
                                       debugger_trace_enabled());
    snprintf(frontend->status, sizeof(frontend->status), "Trace %s",
             debugger_trace_enabled() ? "enabled" : "disabled");
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool load_lua(DebugFrontend *frontend, char *error, size_t error_size) {
    if (!frontend || !frontend->lua_path[0])
        return fail(frontend, error, error_size, "Choose a Lua script path first");
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(frontend->lua_path, LUA_SOURCE_LIMIT, &data, &size);
    if (result != NES_FILE_OK)
        return fail(frontend, error, error_size, nes_file_result_message(result));
    if (memchr(data, '\0', size)) {
        free(data);
        return fail(frontend, error, error_size, "Lua script contains an embedded NUL byte");
    }
    char *source = malloc(size + 1u);
    if (!source) {
        free(data);
        return fail(frontend, error, error_size, "Out of memory while reading Lua script");
    }
    memcpy(source, data, size);
    source[size] = '\0';
    free(data);
    bool ok = debugger_lua_load(frontend->lua_path, source);
    free(source);
    if (!ok)
        return fail(frontend, error, error_size,
                    debugger_lua_error()[0] ? debugger_lua_error() : "Lua script could not be loaded");
    snprintf(frontend->status, sizeof(frontend->status), "Lua script loaded");
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool command_lua_load(void *context, char *error, size_t error_size) {
    return load_lua(context, error, error_size);
}

static bool command_lua_unload(void *context, char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    debugger_lua_unload();
    snprintf(frontend->status, sizeof(frontend->status), "Lua script unloaded");
    if (error && error_size) error[0] = '\0';
    return true;
}

static void refresh_breakpoints(DebugFrontend *frontend, size_t *count) {
    *count = debugger_breakpoint_count();
    if (*count > DEBUG_BREAK_MAX) *count = DEBUG_BREAK_MAX;
    for (size_t i = 0; i < *count; ++i) {
        DebugBreakpoint bp;
        if (!debugger_breakpoint_at(i, &bp)) continue;
        snprintf(frontend->breakpoint_labels[i], sizeof(frontend->breakpoint_labels[i]),
                 "#%u %04X-%04X %c%c%c %s", bp.id, bp.first, bp.last,
                 (bp.type & DEBUG_BREAK_EXECUTE) ? 'X' : '-',
                 (bp.type & DEBUG_BREAK_READ) ? 'R' : '-',
                 (bp.type & DEBUG_BREAK_WRITE) ? 'W' : '-',
                 bp.enabled ? "on" : "off");
        frontend->breakpoint_items[i] = frontend->breakpoint_labels[i];
    }
    if (!*count) frontend->breakpoint_selected = 0;
    else if (frontend->breakpoint_selected >= (int)*count)
        frontend->breakpoint_selected = (int)*count - 1;
}

static bool debugger_snapshot(void *context, FrontendPanelModel *model,
                              char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    DebugCpuSnapshot cpu_state;
    debugger_get_cpu(&cpu_state);
    DebugStopInfo stop = debugger_last_stop();
    snprintf(frontend->cpu_text, sizeof(frontend->cpu_text),
             "PC=%04X A=%02X X=%02X Y=%02X SP=%02X P=%02X cycles=%llu",
             cpu_state.cpu.pc, cpu_state.cpu.a, cpu_state.cpu.x, cpu_state.cpu.y,
             cpu_state.cpu.sp, cpu_state.cpu.status,
             (unsigned long long)cpu_state.cpu_cycles);
    snprintf(frontend->stop_text, sizeof(frontend->stop_text),
             "%s | reason=%u addr=%04X cycle=%llu",
             debugger_is_paused() ? "Paused" : "Running", (unsigned)stop.reason,
             stop.address, (unsigned long long)stop.cpu_cycle);
    size_t break_count = 0;
    refresh_breakpoints(frontend, &break_count);
    static const char *const break_types[] = {"Execute", "Read", "Write", "Exec+Read+Write"};
    FrontendPanelControl controls[] = {
        {DEBUG_CONTROL_CPU, FRONTEND_PANEL_TEXT, "CPU", frontend->cpu_text, NULL, 0, 0, true, true},
        {DEBUG_CONTROL_REGISTER, FRONTEND_PANEL_TEXT, "Set register (A=FF, PC=8000)", frontend->register_edit, NULL, 0, 0, true, false},
        {DEBUG_CONTROL_STOP, FRONTEND_PANEL_TEXT, "Debugger state", frontend->stop_text, NULL, 0, 0, true, true},
        {DEBUG_CONTROL_TOGGLE, FRONTEND_PANEL_ACTION, debugger_is_paused() ? "Resume" : "Pause", "", NULL, 0, 0, true, false},
        {DEBUG_CONTROL_STEP_INTO, FRONTEND_PANEL_ACTION, "Step into", "", NULL, 0, 0, debugger_is_paused(), false},
        {DEBUG_CONTROL_STEP_OVER, FRONTEND_PANEL_ACTION, "Step over", "", NULL, 0, 0, debugger_is_paused(), false},
        {DEBUG_CONTROL_STEP_OUT, FRONTEND_PANEL_ACTION, "Step out", "", NULL, 0, 0, debugger_is_paused(), false},
        {DEBUG_CONTROL_BREAK_ADDRESS, FRONTEND_PANEL_TEXT, "Breakpoint address/range", frontend->breakpoint_address, NULL, 0, 0, true, false},
        {DEBUG_CONTROL_BREAK_TYPE, FRONTEND_PANEL_CHOICE, "Breakpoint type", NULL, break_types, 4, frontend->breakpoint_type, true, false},
        {DEBUG_CONTROL_BREAK_LIST, FRONTEND_PANEL_CHOICE, "Breakpoints", NULL, frontend->breakpoint_items, break_count, frontend->breakpoint_selected, break_count != 0, false},
        {DEBUG_CONTROL_BREAK_ADD, FRONTEND_PANEL_ACTION, "Add breakpoint", "", NULL, 0, 0, true, false},
        {DEBUG_CONTROL_BREAK_TOGGLE, FRONTEND_PANEL_ACTION, "Toggle selected breakpoint", "", NULL, 0, 0, break_count != 0, false},
        {DEBUG_CONTROL_BREAK_REMOVE, FRONTEND_PANEL_ACTION, "Remove selected breakpoint", "", NULL, 0, 0, break_count != 0, false},
        {DEBUG_CONTROL_TRACE, FRONTEND_PANEL_CHECKBOX, "Instruction trace", NULL, NULL, 0, debugger_trace_enabled(), true, false},
        {DEBUG_CONTROL_TRACE_CLEAR, FRONTEND_PANEL_ACTION, "Clear trace", "", NULL, 0, 0, true, false}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static uint8_t breakpoint_mask(int selected) {
    static const uint8_t types[] = {
        DEBUG_BREAK_EXECUTE, DEBUG_BREAK_READ, DEBUG_BREAK_WRITE,
        DEBUG_BREAK_EXECUTE | DEBUG_BREAK_READ | DEBUG_BREAK_WRITE
    };
    return selected >= 0 && selected < 4 ? types[selected] : DEBUG_BREAK_EXECUTE;
}

static bool parse_break_range(const char *text, uint16_t *first, uint16_t *last) {
    if (!text || !first || !last) return false;
    char copy[32];
    if (strlen(text) >= sizeof(copy)) return false;
    strcpy(copy, text);
    char *dash = strchr(copy, '-');
    if (dash) *dash++ = '\0';
    if (!parse_hex16(copy, first)) return false;
    *last = *first;
    return !dash || parse_hex16(dash, last);
}

static bool set_register(DebugFrontend *frontend, const char *value,
                         char *error, size_t error_size) {
    if (!value || strlen(value) >= sizeof(frontend->register_edit))
        return fail(frontend, error, error_size, "Register edit is invalid");
    strcpy(frontend->register_edit, value);
    char copy[32];
    strcpy(copy, value);
    char *equals = strchr(copy, '=');
    if (!equals) return fail(frontend, error, error_size, "Use REG=HEX, for example PC=8000");
    *equals++ = '\0';
    uint32_t parsed = 0;
    char *end = NULL;
    errno = 0;
    parsed = (uint32_t)strtoul(equals, &end, 16);
    if (errno || end == equals || *end || !debugger_set_cpu_register(copy, parsed))
        return fail(frontend, error, error_size, "Register name or value is invalid");
    snprintf(frontend->status, sizeof(frontend->status), "%s updated", copy);
    return true;
}

static bool debugger_action(void *context, unsigned id, const char *value, int selected,
                            char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    if (id == DEBUG_CONTROL_REGISTER) return set_register(frontend, value, error, error_size);
    if (id == DEBUG_CONTROL_BREAK_ADDRESS) {
        if (!value || strlen(value) >= sizeof(frontend->breakpoint_address))
            return fail(frontend, error, error_size, "Breakpoint address is too long");
        strcpy(frontend->breakpoint_address, value);
        return true;
    }
    if (id == DEBUG_CONTROL_BREAK_TYPE && selected >= 0 && selected < 4) {
        frontend->breakpoint_type = selected;
        return true;
    }
    if (id == DEBUG_CONTROL_BREAK_LIST && selected >= 0) {
        frontend->breakpoint_selected = selected;
        return true;
    }
    if (id == DEBUG_CONTROL_TOGGLE) return command_toggle(frontend, error, error_size);
    if (id == DEBUG_CONTROL_STEP_INTO) return command_step_into(frontend, error, error_size);
    if (id == DEBUG_CONTROL_STEP_OVER) return command_step_over(frontend, error, error_size);
    if (id == DEBUG_CONTROL_STEP_OUT) return command_step_out(frontend, error, error_size);
    if (id == DEBUG_CONTROL_TRACE) return command_trace(frontend, error, error_size);
    if (id == DEBUG_CONTROL_TRACE_CLEAR) {
        debugger_trace_clear();
        snprintf(frontend->status, sizeof(frontend->status), "Trace cleared");
        return true;
    }
    size_t count = debugger_breakpoint_count();
    if (id == DEBUG_CONTROL_BREAK_ADD) {
        uint16_t first, last;
        if (!parse_break_range(frontend->breakpoint_address, &first, &last))
            return fail(frontend, error, error_size, "Enter a hexadecimal address or range");
        uint32_t added = debugger_add_breakpoint((DebugBreakpointType)breakpoint_mask(frontend->breakpoint_type),
                                                first, last);
        if (!added) return fail(frontend, error, error_size, "Breakpoint could not be added");
        frontend->breakpoint_selected = (int)debugger_breakpoint_count() - 1;
        snprintf(frontend->status, sizeof(frontend->status), "Breakpoint #%u added", added);
        return true;
    }
    if ((id == DEBUG_CONTROL_BREAK_TOGGLE || id == DEBUG_CONTROL_BREAK_REMOVE)
        && frontend->breakpoint_selected >= 0
        && (size_t)frontend->breakpoint_selected < count) {
        DebugBreakpoint bp;
        if (!debugger_breakpoint_at((size_t)frontend->breakpoint_selected, &bp))
            return fail(frontend, error, error_size, "Breakpoint is unavailable");
        if (id == DEBUG_CONTROL_BREAK_REMOVE) {
            if (!debugger_remove_breakpoint(bp.id))
                return fail(frontend, error, error_size, "Breakpoint could not be removed");
            snprintf(frontend->status, sizeof(frontend->status), "Breakpoint removed");
        } else {
            if (!debugger_update_breakpoint(bp.id, (DebugBreakpointType)bp.type,
                                            bp.first, bp.last, !bp.enabled))
                return fail(frontend, error, error_size, "Breakpoint could not be updated");
            snprintf(frontend->status, sizeof(frontend->status), "Breakpoint %s",
                     bp.enabled ? "disabled" : "enabled");
        }
        return true;
    }
    return fail(frontend, error, error_size, "Unknown debugger control");
}

static void format_bytes(char *out, size_t out_size, const uint8_t *bytes, size_t count,
                         const char *prefix) {
    int used = snprintf(out, out_size, "%s", prefix ? prefix : "");
    if (used < 0 || (size_t)used >= out_size) return;
    size_t offset = (size_t)used;
    for (size_t i = 0; i < count && offset + 4 < out_size; ++i) {
        int n = snprintf(out + offset, out_size - offset, "%s%02X", i ? " " : "", bytes[i]);
        if (n < 0 || (size_t)n >= out_size - offset) break;
        offset += (size_t)n;
    }
}

static void refresh_view(DebugFrontend *frontend) {
    uint16_t address = 0;
    if (!parse_hex16(frontend->view_address, &address)) address = 0;
    uint8_t bytes[32] = {0};
    const char *prefix = "";
    if (frontend->view_mode == 0) {
        for (unsigned i = 0; i < 16; ++i) bytes[i] = debugger_peek_cpu((uint16_t)(address + i));
        prefix = "CPU: ";
    } else if (frontend->view_mode == 1) {
        for (unsigned i = 0; i < 16; ++i) bytes[i] = debugger_peek_ppu((uint16_t)(address + i));
        prefix = "PPU: ";
    } else if (frontend->view_mode == 2) {
        uint8_t table[0x400]; debugger_copy_nametable((address >> 10) & 3u, table);
        memcpy(bytes, table, 16); prefix = "Nametable: ";
    } else if (frontend->view_mode == 3) {
        uint8_t pattern[0x1000]; debugger_copy_pattern_table((address >> 12) & 1u, pattern);
        memcpy(bytes, pattern, 16); prefix = "Pattern: ";
    } else if (frontend->view_mode == 4) {
        debugger_copy_palette(bytes); prefix = "Palette: ";
    } else {
        uint8_t oam[256]; debugger_copy_oam(oam); memcpy(bytes, oam, 16); prefix = "OAM: ";
    }
    format_bytes(frontend->view_text, sizeof(frontend->view_text), bytes, 16, prefix);
    DebugDisassembly disassembly;
    if (debugger_disassemble(address, &disassembly)) {
        size_t used = strlen(frontend->view_text);
        snprintf(frontend->view_text + used, sizeof(frontend->view_text) - used,
                 " | %s", disassembly.text);
    }
}

static void refresh_watches(DebugFrontend *frontend, size_t *count) {
    *count = debugger_watch_count();
    if (*count > DEBUG_WATCH_MAX) *count = DEBUG_WATCH_MAX;
    for (size_t i = 0; i < *count; ++i) {
        DebugWatch watch; uint32_t value = 0;
        if (!debugger_watch_at(i, &watch, &value)) continue;
        snprintf(frontend->watch_labels[i], sizeof(frontend->watch_labels[i]),
                 "#%u %04X/%u %s = %08X %s", watch.id, watch.address, watch.size,
                 watch.label, value, watch.enabled ? "on" : "off");
        frontend->watch_items[i] = frontend->watch_labels[i];
    }
    if (!*count) frontend->watch_selected = 0;
    else if (frontend->watch_selected >= (int)*count)
        frontend->watch_selected = (int)*count - 1;
}

static bool viewer_snapshot(void *context, FrontendPanelModel *model,
                            char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    static const char *const modes[] = {"CPU memory", "PPU memory", "Nametable", "Pattern table", "Palette", "OAM"};
    static const char *const sizes[] = {"1 byte", "2 bytes", "4 bytes"};
    refresh_view(frontend);
    size_t watch_count = 0;
    refresh_watches(frontend, &watch_count);
    size_t trace_count = debugger_trace_count();
    if (trace_count) {
        DebugTraceEntry entry;
        if (debugger_trace_at(trace_count - 1, &entry))
            snprintf(frontend->trace_text, sizeof(frontend->trace_text),
                     "%zu entries | %llu PC=%04X OP=%02X A=%02X X=%02X Y=%02X",
                     trace_count, (unsigned long long)entry.cpu_cycle, entry.pc,
                     entry.opcode, entry.a, entry.x, entry.y);
    } else snprintf(frontend->trace_text, sizeof(frontend->trace_text), "Trace is empty");
    FrontendPanelControl controls[] = {
        {VIEW_CONTROL_MODE, FRONTEND_PANEL_CHOICE, "Viewer", NULL, modes, 6, frontend->view_mode, true, false},
        {VIEW_CONTROL_ADDRESS, FRONTEND_PANEL_TEXT, "Address/base", frontend->view_address, NULL, 0, 0, true, false},
        {VIEW_CONTROL_SNAPSHOT, FRONTEND_PANEL_TEXT, "Snapshot + disassembly", frontend->view_text, NULL, 0, 0, true, true},
        {VIEW_CONTROL_WATCH_ADDRESS, FRONTEND_PANEL_TEXT, "Watch address", frontend->watch_address, NULL, 0, 0, true, false},
        {VIEW_CONTROL_WATCH_SIZE, FRONTEND_PANEL_CHOICE, "Watch size", NULL, sizes, 3, frontend->watch_size, true, false},
        {VIEW_CONTROL_WATCH_LABEL, FRONTEND_PANEL_TEXT, "Watch label", frontend->watch_label, NULL, 0, 0, true, false},
        {VIEW_CONTROL_WATCH_ADD, FRONTEND_PANEL_ACTION, "Add watch", "", NULL, 0, 0, true, false},
        {VIEW_CONTROL_WATCH_LIST, FRONTEND_PANEL_CHOICE, "Watches", NULL, frontend->watch_items, watch_count, frontend->watch_selected, watch_count != 0, false},
        {VIEW_CONTROL_WATCH_TOGGLE, FRONTEND_PANEL_ACTION, "Toggle selected watch", "", NULL, 0, 0, watch_count != 0, false},
        {VIEW_CONTROL_WATCH_REMOVE, FRONTEND_PANEL_ACTION, "Remove selected watch", "", NULL, 0, 0, watch_count != 0, false},
        {VIEW_CONTROL_TRACE_LAST, FRONTEND_PANEL_TEXT, "Last trace", frontend->trace_text, NULL, 0, 0, true, true}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool viewer_action(void *context, unsigned id, const char *value, int selected,
                          char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    if (id == VIEW_CONTROL_MODE && selected >= 0 && selected < 6) {
        frontend->view_mode = selected; return true;
    }
    if (id == VIEW_CONTROL_ADDRESS || id == VIEW_CONTROL_WATCH_ADDRESS
        || id == VIEW_CONTROL_WATCH_LABEL) {
        char *dst = id == VIEW_CONTROL_ADDRESS ? frontend->view_address
                  : id == VIEW_CONTROL_WATCH_ADDRESS ? frontend->watch_address
                                                     : frontend->watch_label;
        size_t capacity = id == VIEW_CONTROL_WATCH_LABEL ? sizeof(frontend->watch_label)
                                                        : sizeof(frontend->view_address);
        if (!value || strlen(value) >= capacity)
            return fail(frontend, error, error_size, "Viewer value is too long");
        strcpy(dst, value); return true;
    }
    if (id == VIEW_CONTROL_WATCH_SIZE && selected >= 0 && selected < 3) {
        frontend->watch_size = selected; return true;
    }
    if (id == VIEW_CONTROL_WATCH_LIST && selected >= 0) {
        frontend->watch_selected = selected; return true;
    }
    if (id == VIEW_CONTROL_WATCH_ADD) {
        uint16_t address;
        if (!parse_hex16(frontend->watch_address, &address))
            return fail(frontend, error, error_size, "Watch address must be hexadecimal");
        static const uint8_t sizes[] = {1, 2, 4};
        uint32_t added = debugger_add_watch(address, sizes[frontend->watch_size], frontend->watch_label);
        if (!added) return fail(frontend, error, error_size, "Watch could not be added");
        frontend->watch_selected = (int)debugger_watch_count() - 1;
        snprintf(frontend->status, sizeof(frontend->status), "Watch #%u added", added);
        return true;
    }
    size_t count = debugger_watch_count();
    if ((id == VIEW_CONTROL_WATCH_TOGGLE || id == VIEW_CONTROL_WATCH_REMOVE)
        && frontend->watch_selected >= 0 && (size_t)frontend->watch_selected < count) {
        DebugWatch watch; uint32_t ignored;
        if (!debugger_watch_at((size_t)frontend->watch_selected, &watch, &ignored))
            return fail(frontend, error, error_size, "Watch is unavailable");
        bool ok = id == VIEW_CONTROL_WATCH_REMOVE
            ? debugger_remove_watch(watch.id)
            : debugger_update_watch(watch.id, watch.address, watch.size,
                                    watch.label, !watch.enabled);
        if (!ok) return fail(frontend, error, error_size, "Watch could not be updated");
        snprintf(frontend->status, sizeof(frontend->status), "%s",
                 id == VIEW_CONTROL_WATCH_REMOVE ? "Watch removed" : "Watch toggled");
        return true;
    }
    return fail(frontend, error, error_size, "Unknown viewer control");
}

static bool overlay_present(void) {
    const uint32_t *overlay = debugger_lua_overlay();
    if (!overlay) return false;
    for (size_t i = 0; i < DEBUG_LUA_OVERLAY_WIDTH * DEBUG_LUA_OVERLAY_HEIGHT; ++i)
        if (overlay[i]) return true;
    return false;
}

static bool lua_snapshot(void *context, FrontendPanelModel *model,
                         char *error, size_t error_size) {
    DebugFrontend *frontend = context;
    snprintf(frontend->lua_budget, sizeof(frontend->lua_budget), "%u",
             debugger_lua_instruction_budget());
    snprintf(frontend->lua_state, sizeof(frontend->lua_state), "%s%s",
             debugger_lua_loaded() ? "Loaded" : "No script",
             debugger_lua_faulted() ? " (faulted)" : "");
    snprintf(frontend->lua_overlay, sizeof(frontend->lua_overlay), "%s",
             overlay_present() ? "Overlay has pixels" : "Overlay empty");
    snprintf(frontend->lua_error, sizeof(frontend->lua_error), "%.180s",
             debugger_lua_error());
    snprintf(frontend->lua_log, sizeof(frontend->lua_log), "%.244s", debugger_lua_log());
    FrontendPanelControl controls[] = {
        {LUA_CONTROL_PATH, FRONTEND_PANEL_TEXT, "Lua script path", frontend->lua_path, NULL, 0, 0, true, false},
        {LUA_CONTROL_LOAD, FRONTEND_PANEL_ACTION, "Load script", "", NULL, 0, 0, true, false},
        {LUA_CONTROL_UNLOAD, FRONTEND_PANEL_ACTION, "Unload script", "", NULL, 0, 0, debugger_lua_loaded(), false},
        {LUA_CONTROL_BUDGET, FRONTEND_PANEL_TEXT, "Instruction budget", frontend->lua_budget, NULL, 0, 0, true, false},
        {LUA_CONTROL_STATE, FRONTEND_PANEL_TEXT, "Script state", frontend->lua_state, NULL, 0, 0, true, true},
        {LUA_CONTROL_OVERLAY, FRONTEND_PANEL_TEXT, "Overlay", frontend->lua_overlay, NULL, 0, 0, true, true},
        {LUA_CONTROL_CLEAR_OVERLAY, FRONTEND_PANEL_ACTION, "Clear overlay", "", NULL, 0, 0, true, false},
        {LUA_CONTROL_ERROR, FRONTEND_PANEL_TEXT, "Last Lua error", frontend->lua_error, NULL, 0, 0, true, true},
        {LUA_CONTROL_LOG, FRONTEND_PANEL_TEXT, "Lua log", frontend->lua_log, NULL, 0, 0, true, true}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    model->status = frontend->status;
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool lua_action(void *context, unsigned id, const char *value, int selected,
                       char *error, size_t error_size) {
    (void)selected;
    DebugFrontend *frontend = context;
    if (id == LUA_CONTROL_PATH) {
        if (!value || strlen(value) >= sizeof(frontend->lua_path))
            return fail(frontend, error, error_size, "Lua path is too long");
        strcpy(frontend->lua_path, value); return true;
    }
    if (id == LUA_CONTROL_LOAD) return load_lua(frontend, error, error_size);
    if (id == LUA_CONTROL_UNLOAD) return command_lua_unload(frontend, error, error_size);
    if (id == LUA_CONTROL_BUDGET) {
        uint32_t budget;
        if (!parse_u32(value, &budget) || !budget)
            return fail(frontend, error, error_size, "Lua budget must be a positive integer");
        debugger_lua_set_instruction_budget(budget);
        snprintf(frontend->status, sizeof(frontend->status), "Lua instruction budget updated");
        return true;
    }
    if (id == LUA_CONTROL_CLEAR_OVERLAY) {
        debugger_lua_clear_overlay();
        snprintf(frontend->status, sizeof(frontend->status), "Lua overlay cleared");
        return true;
    }
    return fail(frontend, error, error_size, "Unknown Lua control");
}

DebugFrontend *debug_frontend_create(FrontendExecutionRuntime *execution) {
    if (!execution) return NULL;
    DebugFrontend *frontend = calloc(1, sizeof(*frontend));
    if (!frontend) return NULL;
    frontend->execution = execution;
    strcpy(frontend->register_edit, "PC=8000");
    strcpy(frontend->breakpoint_address, "8000");
    strcpy(frontend->view_address, "8000");
    strcpy(frontend->watch_address, "0000");
    frontend->breakpoint_type = 0;
    return frontend;
}

bool debug_frontend_register_ui(DebugFrontend *frontend) {
    if (!frontend || frontend->registered) return false;
    const FrontendCommandSpec commands[] = {
        {DEBUGGER_FRONTEND_COMMAND, "Debugger Pause/Resume", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_toggle, frontend},
        {DEBUGGER_VIEWERS_FRONTEND_COMMAND, "Toggle Instruction Trace", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_CHECKABLE, command_trace, frontend},
        {DEBUGGER_LUA_FRONTEND_COMMAND, "Load Lua Script", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_lua_load, frontend},
        {DEBUGGER_STEP_INTO_COMMAND, "Debugger Step Into", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_step_into, frontend},
        {DEBUGGER_STEP_OVER_COMMAND, "Debugger Step Over", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_step_over, frontend},
        {DEBUGGER_STEP_OUT_COMMAND, "Debugger Step Out", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_step_out, frontend},
        {DEBUGGER_RESUME_COMMAND, "Debugger Resume", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_resume, frontend},
        {DEBUGGER_PAUSE_COMMAND, "Debugger Pause", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_pause, frontend},
        {DEBUGGER_LUA_LOAD_COMMAND, "Load Configured Lua Script", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_lua_load, frontend},
        {DEBUGGER_LUA_UNLOAD_COMMAND, "Unload Lua Script", "Tools", "", FRONTEND_COMMAND_NEEDS_SESSION, command_lua_unload, frontend}
    };
    size_t registered_commands = 0;
    for (; registered_commands < sizeof(commands) / sizeof(commands[0]); ++registered_commands)
        if (!frontend_command_register(&commands[registered_commands])) break;
    if (registered_commands != sizeof(commands) / sizeof(commands[0])) {
        while (registered_commands) (void)frontend_command_unregister(commands[--registered_commands].id);
        return false;
    }
    const FrontendPanelSpec panels[] = {
        {DEBUGGER_FRONTEND_PANEL, "Debugger", "Tools", FRONTEND_PANEL_NEEDS_SESSION, debugger_snapshot, debugger_action, frontend},
        {DEBUGGER_VIEWERS_FRONTEND_PANEL, "Memory and PPU Viewers", "Tools", FRONTEND_PANEL_NEEDS_SESSION, viewer_snapshot, viewer_action, frontend},
        {DEBUGGER_LUA_FRONTEND_PANEL, "Lua Scripting", "Tools", FRONTEND_PANEL_NEEDS_SESSION, lua_snapshot, lua_action, frontend}
    };
    size_t registered_panels = 0;
    for (; registered_panels < sizeof(panels) / sizeof(panels[0]); ++registered_panels)
        if (!frontend_panel_register(&panels[registered_panels])) break;
    if (registered_panels != sizeof(panels) / sizeof(panels[0])) {
        while (registered_panels) (void)frontend_panel_unregister(panels[--registered_panels].id);
        for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
            (void)frontend_command_unregister(commands[i].id);
        return false;
    }
    frontend->registered = true;
    (void)frontend_command_set_checked(DEBUGGER_VIEWERS_FRONTEND_COMMAND, debugger_trace_enabled());
    return true;
}

void debug_frontend_image_changed(DebugFrontend *frontend) {
    if (!frontend) return;
    debugger_reset_session();
    sync_execution(frontend);
    frontend->breakpoint_selected = 0;
    frontend->watch_selected = 0;
    frontend->status[0] = '\0';
}

void debug_frontend_destroy(DebugFrontend *frontend) {
    if (!frontend) return;
    if (frontend->registered) {
        const unsigned commands[] = {
            DEBUGGER_FRONTEND_COMMAND, DEBUGGER_VIEWERS_FRONTEND_COMMAND,
            DEBUGGER_LUA_FRONTEND_COMMAND, DEBUGGER_STEP_INTO_COMMAND,
            DEBUGGER_STEP_OVER_COMMAND, DEBUGGER_STEP_OUT_COMMAND,
            DEBUGGER_RESUME_COMMAND, DEBUGGER_PAUSE_COMMAND,
            DEBUGGER_LUA_LOAD_COMMAND, DEBUGGER_LUA_UNLOAD_COMMAND
        };
        for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
            (void)frontend_command_unregister(commands[i]);
        (void)frontend_panel_unregister(DEBUGGER_FRONTEND_PANEL);
        (void)frontend_panel_unregister(DEBUGGER_VIEWERS_FRONTEND_PANEL);
        (void)frontend_panel_unregister(DEBUGGER_LUA_FRONTEND_PANEL);
    }
    debugger_lua_unload();
    free(frontend);
}
