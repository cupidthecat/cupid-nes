/*
 * debug_analysis.h - Host-side execution analysis
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_DEBUG_ANALYSIS_H
#define CUPID_DEBUG_ANALYSIS_H
#include "debugger.h"

#define DEBUG_KEY_CPU UINT64_C(0x100000000)
#define DEBUG_KEY_NONE UINT64_MAX

enum { DEBUG_CDL_CODE = 1, DEBUG_CDL_DATA = 2, DEBUG_CDL_OPERAND = 4 };

enum {
    DEBUG_EVENT_READ = 1,
    DEBUG_EVENT_WRITE = 2,
    DEBUG_EVENT_PPU = 4,
    DEBUG_EVENT_APU = 8,
    DEBUG_EVENT_DMA = 16,
    DEBUG_EVENT_NMI = 32,
    DEBUG_EVENT_IRQ = 64,
    DEBUG_EVENT_VBLANK = 128,
    DEBUG_EVENT_SPRITE0 = 256,
    DEBUG_EVENT_MAPPER = 512
};

typedef struct {
    size_t code, data, operand, untouched, total;
} DebugCoverage;

typedef struct {
    uint64_t key, instructions, cycles;
    uint16_t address;
} DebugProfileRow;

typedef struct {
    uint64_t source_key, target_key;
    uint16_t source, target, return_address;
    uint8_t return_sp;
    bool interrupt, uncertain;
} DebugStackFrame;

typedef struct {
    uint64_t frame, cycle, key, pc_key;
    uint32_t category;
    uint16_t pc, address;
    int16_t scanline, dot;
    uint8_t value;
} DebugNesEvent;

/* A physical PRG offset, or DEBUG_KEY_CPU + address for other memory. */
uint64_t debug_analysis_key(uint16_t address);
void debug_analysis_shutdown(void);
bool debug_analysis_image_changed(void);
void debug_analysis_discontinuity(void);
void debug_analysis_begin(const CPU *state);
void debug_analysis_end(const CPU *state);
void debug_analysis_read(uint16_t address, uint8_t value);
void debug_analysis_opcode(uint8_t opcode);
void debug_analysis_write(uint16_t address, uint8_t value);
/* Supply the state immediately before interrupt entry and its resolved target. */
void debug_analysis_interrupt(const CPU *before, uint16_t target, bool nmi);
void debug_analysis_event(uint32_t category, uint16_t address, uint8_t value);
void debug_analysis_signals(bool nmi, bool irq, bool mapper_irq);
void debug_cdl_enable(bool enabled);
bool debug_cdl_enabled(void);
void debug_cdl_clear(void);
DebugCoverage debug_cdl_coverage(void);
uint8_t debug_cdl_at(size_t offset);
bool debug_cdl_save(const char *path);
bool debug_cdl_load(const char *path);
bool debug_cdl_export(const char *path);
void debug_profile_enable(bool enabled);
bool debug_profile_enabled(void);
void debug_profile_clear(void);
size_t debug_profile_rows(DebugProfileRow *out, size_t capacity, bool by_cycles);
size_t debug_profile_function_rows(DebugProfileRow *out, size_t capacity, bool by_cycles);
uint64_t debug_profile_total_cycles(void);
uint64_t debug_profile_dropped(void);
size_t debug_stack_count(void);
void debug_stack_enable(bool enabled);
void debug_stack_clear(void);
bool debug_stack_enabled(void);
bool debug_stack_at(size_t index, DebugStackFrame *out);
void debug_events_enable(bool enabled);
bool debug_events_enabled(void);
void debug_events_clear(void);
/* Completed frame only; filtering never modifies capture storage. */
size_t debug_events_count(uint32_t mask);
bool debug_events_at(size_t index, uint32_t mask, DebugNesEvent *out);
uint64_t debug_events_dropped(void);
#endif
