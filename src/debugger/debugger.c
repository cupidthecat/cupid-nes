/*
 * debugger.c - Runtime breakpoints, stepping, tracing and inspection
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "debugger.h"
#include "lua_runtime.h"

#include "../ppu/ppu.h"

#include <stdio.h>
#include <string.h>

enum { DEBUG_MAX_BREAKPOINTS = 128, DEBUG_MAX_WATCHES = 64, DEBUG_TRACE_CAPACITY = 4096 };

typedef enum {
    STEP_NONE,
    STEP_INTO,
    STEP_OVER,
    STEP_OUT
} StepMode;

typedef struct {
    DebugBreakpoint breakpoints[DEBUG_MAX_BREAKPOINTS];
    size_t breakpoint_count;
    DebugWatch watches[DEBUG_MAX_WATCHES];
    size_t watch_count;
    DebugTraceEntry trace[DEBUG_TRACE_CAPACITY];
    size_t trace_head, trace_count;
    uint32_t next_id;
    bool paused, trace_enabled;
    DebugStopInfo stop;
    StepMode step;
    bool step_started;
    uint16_t step_target_pc;
    uint8_t step_target_sp;
    bool skip_exec_once;
    uint16_t skip_exec_pc;
    DebugPauseCallback pause_callback;
    void *pause_userdata;
} DebuggerState;

static DebuggerState debug_state;

static void set_paused(bool paused) {
    if (debug_state.paused == paused) return;
    debug_state.paused = paused;
    if (debug_state.pause_callback)
        debug_state.pause_callback(paused, debug_state.pause_userdata);
}

static void stop_at(DebugStopReason reason, uint16_t address, uint8_t value, uint8_t access) {
    debug_state.stop.reason = reason;
    debug_state.stop.address = address;
    debug_state.stop.value = value;
    debug_state.stop.access = access;
    debug_state.stop.cpu_cycle = cpu_total_cycles;
    debug_state.step = STEP_NONE;
    debug_state.step_started = false;
    set_paused(true);
}

static bool breakpoint_match(uint8_t type, uint16_t address) {
    for (size_t i = 0; i < debug_state.breakpoint_count; ++i) {
        const DebugBreakpoint *bp = &debug_state.breakpoints[i];
        if (bp->enabled && (bp->type & type) && address >= bp->first && address <= bp->last)
            return true;
    }
    return false;
}

void debugger_init(void) {
    DebugPauseCallback callback = debug_state.pause_callback;
    void *userdata = debug_state.pause_userdata;
    memset(&debug_state, 0, sizeof(debug_state));
    debug_state.next_id = 1;
    debug_state.pause_callback = callback;
    debug_state.pause_userdata = userdata;
}

void debugger_shutdown(void) {
    debugger_lua_unload();
    DebugPauseCallback callback = debug_state.pause_callback;
    void *userdata = debug_state.pause_userdata;
    memset(&debug_state, 0, sizeof(debug_state));
    debug_state.pause_callback = callback;
    debug_state.pause_userdata = userdata;
}

void debugger_reset_session(void) {
    debug_state.stop = (DebugStopInfo){0};
    debug_state.step = STEP_NONE;
    debug_state.step_started = false;
    debug_state.skip_exec_once = false;
    debugger_trace_clear();
    set_paused(false);
}

void debugger_set_pause_callback(DebugPauseCallback callback, void *userdata) {
    debug_state.pause_callback = callback;
    debug_state.pause_userdata = userdata;
}

void debugger_pause(void) {
    debug_state.step = STEP_NONE;
    stop_at(DEBUG_STOP_PAUSE, cpu.pc, debugger_peek_cpu(cpu.pc), 0);
}

void debugger_resume(void) {
    if (debug_state.stop.reason == DEBUG_STOP_BREAKPOINT
        && (debug_state.stop.access & DEBUG_BREAK_EXECUTE)) {
        debug_state.skip_exec_once = true;
        debug_state.skip_exec_pc = debug_state.stop.address;
    }
    debug_state.stop = (DebugStopInfo){0};
    debug_state.step = STEP_NONE;
    debug_state.step_started = false;
    set_paused(false);
}

bool debugger_is_paused(void) { return debug_state.paused; }
DebugStopInfo debugger_last_stop(void) { return debug_state.stop; }

static bool begin_step(StepMode mode) {
    if (!debug_state.paused) return false;
    debug_state.stop = (DebugStopInfo){0};
    debug_state.step = mode;
    debug_state.step_started = false;
    debug_state.skip_exec_once = true;
    debug_state.skip_exec_pc = cpu.pc;
    set_paused(false);
    return true;
}

bool debugger_step_into(void) { return begin_step(STEP_INTO); }

bool debugger_step_over(void) {
    if (!debug_state.paused) return false;
    if (debugger_peek_cpu(cpu.pc) != JSR_OPCODE) return begin_step(STEP_INTO);
    debug_state.step_target_pc = (uint16_t)(cpu.pc + 3u);
    debug_state.step_target_sp = cpu.sp;
    return begin_step(STEP_OVER);
}

bool debugger_step_out(void) {
    if (!debug_state.paused) return false;
    debug_state.step_target_sp = (uint8_t)(cpu.sp + 2u);
    return begin_step(STEP_OUT);
}

bool debugger_run_until_break(CPU *target, uint64_t instruction_limit) {
    if (!target || !instruction_limit) return false;
    if (debug_state.paused) debugger_resume();
    for (uint64_t i = 0; i < instruction_limit; ++i) {
        (void)cpu_step(target);
        if (debug_state.paused) return true;
    }
    return false;
}

uint32_t debugger_add_breakpoint(DebugBreakpointType type, uint16_t first, uint16_t last) {
    if (!type || first > last || debug_state.breakpoint_count >= DEBUG_MAX_BREAKPOINTS) return 0;
    uint32_t id = debug_state.next_id++;
    debug_state.breakpoints[debug_state.breakpoint_count++] =
        (DebugBreakpoint){id, first, last, (uint8_t)type, true};
    return id;
}

bool debugger_update_breakpoint(uint32_t id, DebugBreakpointType type,
                                uint16_t first, uint16_t last, bool enabled) {
    if (!id || !type || first > last) return false;
    for (size_t i = 0; i < debug_state.breakpoint_count; ++i) {
        DebugBreakpoint *bp = &debug_state.breakpoints[i];
        if (bp->id != id) continue;
        bp->type = (uint8_t)type; bp->first = first; bp->last = last; bp->enabled = enabled;
        return true;
    }
    return false;
}

bool debugger_remove_breakpoint(uint32_t id) {
    for (size_t i = 0; i < debug_state.breakpoint_count; ++i) {
        if (debug_state.breakpoints[i].id != id) continue;
        memmove(&debug_state.breakpoints[i], &debug_state.breakpoints[i + 1],
                (debug_state.breakpoint_count - i - 1) * sizeof(debug_state.breakpoints[0]));
        --debug_state.breakpoint_count;
        return true;
    }
    return false;
}

size_t debugger_breakpoint_count(void) { return debug_state.breakpoint_count; }
bool debugger_breakpoint_at(size_t index, DebugBreakpoint *out) {
    if (!out || index >= debug_state.breakpoint_count) return false;
    *out = debug_state.breakpoints[index]; return true;
}

void debugger_get_cpu(DebugCpuSnapshot *out) {
    if (!out) return;
    out->cpu = cpu;
    out->cpu_cycles = cpu_total_cycles;
}

bool debugger_set_cpu_register(const char *name, uint32_t value) {
    if (!name) return false;
    if (!strcmp(name, "A") || !strcmp(name, "a")) cpu.a = (uint8_t)value;
    else if (!strcmp(name, "X") || !strcmp(name, "x")) cpu.x = (uint8_t)value;
    else if (!strcmp(name, "Y") || !strcmp(name, "y")) cpu.y = (uint8_t)value;
    else if (!strcmp(name, "PC") || !strcmp(name, "pc")) cpu.pc = (uint16_t)value;
    else if (!strcmp(name, "SP") || !strcmp(name, "sp")) cpu.sp = (uint8_t)value;
    else if (!strcmp(name, "P") || !strcmp(name, "p") || !strcmp(name, "status"))
        cpu.status = (uint8_t)value;
    else return false;
    return true;
}

void debugger_get_ppu(DebugPpuSnapshot *out) {
    if (!out) return;
    *out = (DebugPpuSnapshot){
        ppu.ctrl, ppu.mask, ppu.status, ppu.oam_addr, ppu.v, ppu.t, ppu.x, ppu.w,
        ppu.scanline, ppu.dot, ppu.frame_count, ppu.total_cycles
    };
}

uint8_t debugger_peek_cpu(uint16_t address) { return cpu_debug_peek(address); }
uint8_t debugger_peek_ppu(uint16_t address) { return ppu_debug_peek(address); }

void debugger_copy_nametable(unsigned table, uint8_t out[0x400]) {
    if (!out) return;
    table &= 3u;
    uint16_t base = (uint16_t)(0x2000u + table * 0x400u);
    for (unsigned i = 0; i < 0x400; ++i) out[i] = ppu_debug_peek((uint16_t)(base + i));
}

void debugger_copy_pattern_table(unsigned table, uint8_t out[0x1000]) {
    if (!out) return;
    uint16_t base = (uint16_t)((table & 1u) * 0x1000u);
    for (unsigned i = 0; i < 0x1000; ++i) out[i] = ppu_debug_peek((uint16_t)(base + i));
}

void debugger_copy_palette(uint8_t out[32]) {
    if (!out) return;
    for (unsigned i = 0; i < 32; ++i) out[i] = ppu_debug_peek((uint16_t)(0x3F00u + i));
}

void debugger_copy_oam(uint8_t out[256]) { if (out) memcpy(out, ppu.oam, 256); }

uint32_t debugger_add_watch(uint16_t address, uint8_t size, const char *label) {
    if ((size != 1 && size != 2 && size != 4) || debug_state.watch_count >= DEBUG_MAX_WATCHES) return 0;
    DebugWatch *watch = &debug_state.watches[debug_state.watch_count++];
    memset(watch, 0, sizeof(*watch));
    watch->id = debug_state.next_id++; watch->address = address; watch->size = size; watch->enabled = true;
    if (label) snprintf(watch->label, sizeof(watch->label), "%s", label);
    return watch->id;
}

bool debugger_update_watch(uint32_t id, uint16_t address, uint8_t size,
                           const char *label, bool enabled) {
    if (size != 1 && size != 2 && size != 4) return false;
    for (size_t i = 0; i < debug_state.watch_count; ++i) {
        DebugWatch *watch = &debug_state.watches[i];
        if (watch->id != id) continue;
        watch->address = address; watch->size = size; watch->enabled = enabled;
        snprintf(watch->label, sizeof(watch->label), "%s", label ? label : "");
        return true;
    }
    return false;
}

bool debugger_remove_watch(uint32_t id) {
    for (size_t i = 0; i < debug_state.watch_count; ++i) {
        if (debug_state.watches[i].id != id) continue;
        memmove(&debug_state.watches[i], &debug_state.watches[i + 1],
                (debug_state.watch_count - i - 1) * sizeof(debug_state.watches[0]));
        --debug_state.watch_count; return true;
    }
    return false;
}

size_t debugger_watch_count(void) { return debug_state.watch_count; }
bool debugger_watch_at(size_t index, DebugWatch *out, uint32_t *value) {
    if (index >= debug_state.watch_count || !out) return false;
    *out = debug_state.watches[index];
    if (value) {
        uint32_t result = 0;
        if (out->enabled) for (unsigned i = 0; i < out->size; ++i)
            result |= (uint32_t)debugger_peek_cpu((uint16_t)(out->address + i)) << (8u * i);
        *value = result;
    }
    return true;
}

static uint8_t opcode_length(uint8_t opcode) {
    switch (opcode) {
        case 0x20: case 0x4C: case 0x6C:
        case 0x0C: case 0x0D: case 0x0E: case 0x19: case 0x1C: case 0x1D: case 0x1E:
        case 0x2C: case 0x2D: case 0x2E: case 0x39: case 0x3C: case 0x3D: case 0x3E:
        case 0x4D: case 0x4E: case 0x59: case 0x5C: case 0x5D: case 0x5E:
        case 0x6D: case 0x6E: case 0x79: case 0x7C: case 0x7D: case 0x7E:
        case 0x8C: case 0x8D: case 0x8E: case 0x99: case 0x9C: case 0x9D: case 0x9E: case 0x9F:
        case 0xAC: case 0xAD: case 0xAE: case 0xB9: case 0xBC: case 0xBD: case 0xBE: case 0xBF:
        case 0xCC: case 0xCD: case 0xCE: case 0xD9: case 0xDC: case 0xDD: case 0xDE:
        case 0xEC: case 0xED: case 0xEE: case 0xF9: case 0xFC: case 0xFD: case 0xFE:
            return 3;
        default: break;
    }
    if ((opcode & 0x1Fu) == 0x10 || (opcode & 0x1Fu) == 0x11 ||
        (opcode & 0x1Fu) == 0x15 || (opcode & 0x1Fu) == 0x16 ||
        (opcode & 0x1Fu) == 0x19 || (opcode & 0x1Fu) == 0x1A ||
        (opcode & 0x1Fu) == 0x01 || (opcode & 0x1Fu) == 0x05 ||
        (opcode & 0x1Fu) == 0x06 || (opcode & 0x1Fu) == 0x09)
        return 2;
    switch (opcode) {
        case 0x24: case 0x84: case 0x85: case 0x86: case 0x94: case 0x95: case 0x96:
        case 0xA0: case 0xA2: case 0xA4: case 0xA5: case 0xA6: case 0xB4: case 0xB5: case 0xB6:
        case 0xC0: case 0xC4: case 0xC5: case 0xC6: case 0xD5: case 0xD6:
        case 0xE0: case 0xE4: case 0xE5: case 0xE6: case 0xF5: case 0xF6:
            return 2;
        default: return 1;
    }
}

static const char *opcode_name(uint8_t op) {
    switch (op) {
        case 0x00: return "BRK"; case 0x20: return "JSR"; case 0x40: return "RTI"; case 0x60: return "RTS";
        case 0x4C: case 0x6C: return "JMP"; case 0xEA: return "NOP";
        case 0xA9: case 0xA5: case 0xAD: case 0xB5: case 0xBD: case 0xB9: case 0xA1: case 0xB1: return "LDA";
        case 0xA2: case 0xA6: case 0xAE: case 0xB6: case 0xBE: return "LDX";
        case 0xA0: case 0xA4: case 0xAC: case 0xB4: case 0xBC: return "LDY";
        case 0x85: case 0x8D: case 0x95: case 0x9D: case 0x99: case 0x81: case 0x91: return "STA";
        case 0x86: case 0x8E: case 0x96: return "STX"; case 0x84: case 0x8C: case 0x94: return "STY";
        case 0x69: case 0x65: case 0x6D: case 0x75: case 0x7D: case 0x79: case 0x61: case 0x71: return "ADC";
        case 0xE9: case 0xE5: case 0xED: case 0xF5: case 0xFD: case 0xF9: case 0xE1: case 0xF1: return "SBC";
        case 0x29: case 0x25: case 0x2D: return "AND"; case 0x09: case 0x05: case 0x0D: return "ORA";
        case 0x49: case 0x45: case 0x4D: return "EOR"; case 0xC9: case 0xC5: case 0xCD: return "CMP";
        case 0xE8: return "INX"; case 0xC8: return "INY"; case 0xCA: return "DEX"; case 0x88: return "DEY";
        case 0x18: return "CLC"; case 0x38: return "SEC"; case 0x58: return "CLI"; case 0x78: return "SEI";
        case 0xD8: return "CLD"; case 0xF8: return "SED"; case 0xB8: return "CLV";
        case 0x10: return "BPL"; case 0x30: return "BMI"; case 0x50: return "BVC"; case 0x70: return "BVS";
        case 0x90: return "BCC"; case 0xB0: return "BCS"; case 0xD0: return "BNE"; case 0xF0: return "BEQ";
        default: return "???";
    }
}

bool debugger_disassemble(uint16_t address, DebugDisassembly *out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out)); out->address = address;
    out->bytes[0] = debugger_peek_cpu(address); out->length = opcode_length(out->bytes[0]);
    for (unsigned i = 1; i < out->length; ++i) out->bytes[i] = debugger_peek_cpu((uint16_t)(address + i));
    if (out->length == 1) snprintf(out->text, sizeof(out->text), "%04X  %02X       %s", address, out->bytes[0], opcode_name(out->bytes[0]));
    else if (out->length == 2) snprintf(out->text, sizeof(out->text), "%04X  %02X %02X    %s $%02X", address, out->bytes[0], out->bytes[1], opcode_name(out->bytes[0]), out->bytes[1]);
    else snprintf(out->text, sizeof(out->text), "%04X  %02X %02X %02X %s $%04X", address, out->bytes[0], out->bytes[1], out->bytes[2], opcode_name(out->bytes[0]), (unsigned)(out->bytes[1] | (out->bytes[2] << 8)));
    return true;
}

void debugger_trace_enable(bool enabled) { debug_state.trace_enabled = enabled; }
bool debugger_trace_enabled(void) { return debug_state.trace_enabled; }
void debugger_trace_clear(void) { debug_state.trace_head = debug_state.trace_count = 0; }
size_t debugger_trace_count(void) { return debug_state.trace_count; }
bool debugger_trace_at(size_t index, DebugTraceEntry *out) {
    if (!out || index >= debug_state.trace_count) return false;
    size_t oldest = (debug_state.trace_head + DEBUG_TRACE_CAPACITY - debug_state.trace_count) % DEBUG_TRACE_CAPACITY;
    *out = debug_state.trace[(oldest + index) % DEBUG_TRACE_CAPACITY]; return true;
}

static void trace_instruction(const CPU *state) {
    if (!debug_state.trace_enabled) return;
    DebugTraceEntry *entry = &debug_state.trace[debug_state.trace_head];
    *entry = (DebugTraceEntry){cpu_total_cycles, state->pc, debugger_peek_cpu(state->pc),
                              state->a, state->x, state->y, state->sp, state->status};
    debug_state.trace_head = (debug_state.trace_head + 1) % DEBUG_TRACE_CAPACITY;
    if (debug_state.trace_count < DEBUG_TRACE_CAPACITY) ++debug_state.trace_count;
}

bool debugger_before_instruction(CPU *state) {
    if (!state) return false;
    if (debug_state.paused) return false;

    if (debug_state.step != STEP_NONE) {
        if (!debug_state.step_started) debug_state.step_started = true;
        else if (debug_state.step == STEP_INTO
                 || (debug_state.step == STEP_OVER && state->pc == debug_state.step_target_pc
                     && state->sp == debug_state.step_target_sp)
                 || (debug_state.step == STEP_OUT && state->sp == debug_state.step_target_sp)) {
            stop_at(DEBUG_STOP_STEP, state->pc, debugger_peek_cpu(state->pc), DEBUG_BREAK_EXECUTE);
            return false;
        }
    }

    bool skip = debug_state.skip_exec_once && state->pc == debug_state.skip_exec_pc;
    debug_state.skip_exec_once = false;
    if (!skip && breakpoint_match(DEBUG_BREAK_EXECUTE, state->pc)) {
        stop_at(DEBUG_STOP_BREAKPOINT, state->pc, debugger_peek_cpu(state->pc), DEBUG_BREAK_EXECUTE);
        return false;
    }
    trace_instruction(state);
    debugger_lua_on_execute(state->pc);
    return !debug_state.paused;
}

void debugger_on_cpu_read(uint16_t address, uint8_t *value) {
    if (!value) return;
    debugger_lua_on_read(address, value);
    if (breakpoint_match(DEBUG_BREAK_READ, address))
        stop_at(DEBUG_STOP_BREAKPOINT, address, *value, DEBUG_BREAK_READ);
}

void debugger_on_cpu_write(uint16_t address, uint8_t value) {
    debugger_lua_on_write(address, value);
    if (breakpoint_match(DEBUG_BREAK_WRITE, address))
        stop_at(DEBUG_STOP_BREAKPOINT, address, value, DEBUG_BREAK_WRITE);
}
