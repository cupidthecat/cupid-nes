/*
 * debugger.h - Runtime debugger and side-effect-free inspection API
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_DEBUGGER_H
#define CUPID_DEBUGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../cpu/cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    DEBUGGER_FRONTEND_COMMAND = 0x1340,
    DEBUGGER_VIEWERS_FRONTEND_COMMAND = 0x1341,
    DEBUGGER_LUA_FRONTEND_COMMAND = 0x1342,
    DEBUGGER_FRONTEND_PANEL = 0x1380,
    DEBUGGER_VIEWERS_FRONTEND_PANEL = 0x1381,
    DEBUGGER_LUA_FRONTEND_PANEL = 0x1382,
    DEBUGGER_STEP_INTO_COMMAND = 0x1350,
    DEBUGGER_STEP_OVER_COMMAND = 0x1351,
    DEBUGGER_STEP_OUT_COMMAND = 0x1352,
    DEBUGGER_RESUME_COMMAND = 0x1353,
    DEBUGGER_PAUSE_COMMAND = 0x1354,
    DEBUGGER_LUA_LOAD_COMMAND = 0x1360,
    DEBUGGER_LUA_UNLOAD_COMMAND = 0x1361
};

typedef enum {
    DEBUG_BREAK_EXECUTE = 1u << 0,
    DEBUG_BREAK_READ = 1u << 1,
    DEBUG_BREAK_WRITE = 1u << 2
} DebugBreakpointType;

typedef enum {
    DEBUG_STOP_NONE,
    DEBUG_STOP_PAUSE,
    DEBUG_STOP_BREAKPOINT,
    DEBUG_STOP_STEP,
    DEBUG_STOP_SCRIPT
} DebugStopReason;

typedef struct {
    uint32_t id;
    uint16_t first;
    uint16_t last;
    uint8_t type;
    bool enabled;
} DebugBreakpoint;

typedef struct {
    DebugStopReason reason;
    uint16_t address;
    uint8_t value;
    uint8_t access;
    uint64_t cpu_cycle;
} DebugStopInfo;

typedef struct {
    CPU cpu;
    uint64_t cpu_cycles;
} DebugCpuSnapshot;

typedef struct {
    uint8_t ctrl, mask, status, oam_addr;
    uint16_t v, t;
    uint8_t fine_x, write_toggle;
    int scanline, dot;
    uint64_t frame, ppu_cycles;
} DebugPpuSnapshot;

typedef struct {
    uint32_t id;
    uint16_t address;
    uint8_t size;
    bool enabled;
    char label[48];
} DebugWatch;

typedef struct {
    uint16_t address;
    uint8_t bytes[3];
    uint8_t length;
    char text[64];
} DebugDisassembly;

typedef struct {
    uint64_t cpu_cycle;
    uint16_t pc;
    uint8_t opcode;
    uint8_t a, x, y, sp, status;
} DebugTraceEntry;

typedef void (*DebugPauseCallback)(bool paused, void *userdata);

void debugger_init(void);
void debugger_shutdown(void);
void debugger_reset_session(void);
void debugger_set_pause_callback(DebugPauseCallback callback, void *userdata);
void debugger_pause(void);
void debugger_resume(void);
bool debugger_is_paused(void);
/* Changes whenever debugger pause/resume or initialization changes its state.
 * Frontends poll this token instead of retaining callbacks to their storage. */
uint64_t debugger_pause_revision(void);
DebugStopInfo debugger_last_stop(void);

bool debugger_step_into(void);
bool debugger_step_over(void);
bool debugger_step_out(void);
bool debugger_run_until_break(CPU *target, uint64_t instruction_limit);

uint32_t debugger_add_breakpoint(DebugBreakpointType type, uint16_t first, uint16_t last);
bool debugger_update_breakpoint(uint32_t id, DebugBreakpointType type,
                                uint16_t first, uint16_t last, bool enabled);
bool debugger_remove_breakpoint(uint32_t id);
size_t debugger_breakpoint_count(void);
bool debugger_breakpoint_at(size_t index, DebugBreakpoint *out);

void debugger_get_cpu(DebugCpuSnapshot *out);
bool debugger_set_cpu_register(const char *name, uint32_t value);
void debugger_get_ppu(DebugPpuSnapshot *out);
uint8_t debugger_peek_cpu(uint16_t address);
uint8_t debugger_peek_ppu(uint16_t address);
void debugger_copy_nametable(unsigned table, uint8_t out[0x400]);
void debugger_copy_pattern_table(unsigned table, uint8_t out[0x1000]);
void debugger_copy_palette(uint8_t out[32]);
void debugger_copy_oam(uint8_t out[256]);

uint32_t debugger_add_watch(uint16_t address, uint8_t size, const char *label);
bool debugger_update_watch(uint32_t id, uint16_t address, uint8_t size,
                           const char *label, bool enabled);
bool debugger_remove_watch(uint32_t id);
size_t debugger_watch_count(void);
bool debugger_watch_at(size_t index, DebugWatch *out, uint32_t *value);

bool debugger_disassemble(uint16_t address, DebugDisassembly *out);
void debugger_trace_enable(bool enabled);
bool debugger_trace_enabled(void);
void debugger_trace_clear(void);
size_t debugger_trace_count(void);
bool debugger_trace_at(size_t index, DebugTraceEntry *out);

/* Core hooks. CPU bus code calls these only for real emulated accesses. */
bool debugger_before_instruction(CPU *state);
void debugger_on_cpu_read(uint16_t address, uint8_t *value);
void debugger_on_cpu_write(uint16_t address, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
