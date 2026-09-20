/*
 * debugger.c - Runtime breakpoints, stepping, tracing and inspection
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../system/execution_policy.h"
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
static uint64_t pause_revision;

static void set_paused(bool paused) {
    if (debug_state.paused == paused) return;
    debug_state.paused = paused;
    ++pause_revision;
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
    memset(&debug_state, 0, sizeof(debug_state));
    debug_state.next_id = 1;
    ++pause_revision;
}

void debugger_shutdown(void) {
    debugger_lua_unload();
    memset(&debug_state, 0, sizeof(debug_state));
    ++pause_revision;
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
    if (nes_execution_policy() != NES_EXECUTION_LIVE) return;
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
uint64_t debugger_pause_revision(void) { return pause_revision; }
DebugStopInfo debugger_last_stop(void) { return debug_state.stop; }

static bool begin_step(StepMode mode) {
    if (!debug_state.paused || nes_execution_policy() != NES_EXECUTION_LIVE) return false;
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
    if (!debug_state.paused || nes_execution_policy() != NES_EXECUTION_LIVE) return false;
    if (debugger_peek_cpu(cpu.pc) != JSR_OPCODE) return begin_step(STEP_INTO);
    debug_state.step_target_pc = (uint16_t)(cpu.pc + 3u);
    debug_state.step_target_sp = cpu.sp;
    return begin_step(STEP_OVER);
}

bool debugger_step_out(void) {
    if (!debug_state.paused || nes_execution_policy() != NES_EXECUTION_LIVE) return false;
    debug_state.step_target_sp = (uint8_t)(cpu.sp + 2u);
    return begin_step(STEP_OUT);
}

bool debugger_run_until_break(CPU *target, uint64_t instruction_limit) {
    if (!target || !instruction_limit || nes_execution_policy() != NES_EXECUTION_LIVE) return false;
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
    if (!name || !nes_execution_allows_host_configuration()) return false;
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
    if (nes_execution_policy() != NES_EXECUTION_LIVE) { trace_instruction(state); return true; }
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
    if (!value || nes_execution_policy() != NES_EXECUTION_LIVE) return;
    debugger_lua_on_read(address, value);
    if (breakpoint_match(DEBUG_BREAK_READ, address))
        stop_at(DEBUG_STOP_BREAKPOINT, address, *value, DEBUG_BREAK_READ);
}

void debugger_on_cpu_write(uint16_t address, uint8_t value) {
    if (nes_execution_policy() != NES_EXECUTION_LIVE) return;
    debugger_lua_on_write(address, value);
    if (breakpoint_match(DEBUG_BREAK_WRITE, address))
        stop_at(DEBUG_STOP_BREAKPOINT, address, value, DEBUG_BREAK_WRITE);
}
