/*
 * debug_analysis.c - Physical coverage, cycle profiles, stack and frame events
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "debug_analysis.h"
#include "debug_catalog.h"
#include "opcodes.h"
#include "../rom/rom.h"
#include "../ppu/ppu.h"
#include "../system/execution_policy.h"
#include "../util/file_io.h"
#include "../util/sha1.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { PROFILE_CAP = 131072, EVENT_CAP = 131072, STACK_CAP = 128, CDL_LIMIT = 64 * 1024 * 1024 };

static struct {
    uint8_t *cdl;
    size_t size;
    char hash[41];
    DebugCoverage coverage;
    bool nmi, irq, mapper_irq;
    bool cdl_on, profile_on, events_on, stack_on, pending, executed;
    CPU before;
    uint8_t opcode, length, fetch_mask, bytes[3];
    uint64_t key, start, cycles, profile_lost;
    DebugProfileRow *profile;
    DebugStackFrame stack[STACK_CAP];
    size_t depth;
    DebugNesEvent *events[2];
    size_t event_count[2];
    uint64_t event_lost[2], frame;
    unsigned writing;
    bool frame_valid, completed;
} analysis;

static uint64_t add_sat(uint64_t value, uint64_t amount) {
    return UINT64_MAX - value < amount ? UINT64_MAX : value + amount;
}

uint64_t debug_analysis_key(uint16_t address) {
    NesMemoryLocation location;
    if (prg_rom && cpu_debug_memory_location(address, &location)) {
        uintptr_t ptr = (uintptr_t)location.data, base = (uintptr_t)prg_rom;
        if (ptr >= base && ptr - base < prg_size) {
            return (uint64_t)(ptr - base);
        }
    }
    return DEBUG_KEY_CPU + address;
}

void debug_analysis_shutdown(void) {
    free(analysis.cdl);
    free(analysis.profile);
    free(analysis.events[0]);
    free(analysis.events[1]);
    memset(&analysis, 0, sizeof(analysis));
}

bool debug_analysis_image_changed(void) {
    debug_analysis_shutdown();
    if (!prg_rom || !prg_size) {
        return true;
    }
    if (prg_size > CDL_LIMIT) {
        return false;
    }
    analysis.cdl = calloc(prg_size, 1);
    if (!analysis.cdl) {
        return false;
    }
    analysis.size = prg_size;
    analysis.coverage.total = analysis.coverage.untouched = prg_size;
    nes_sha1(prg_rom, prg_size, analysis.hash);
    return true;
}

void debug_analysis_discontinuity(void) {
    analysis.pending = false;
    analysis.depth = 0;
    debug_events_clear();
}

void debug_cdl_enable(bool enabled) {
    analysis.cdl_on = enabled && analysis.cdl;
}

bool debug_cdl_enabled(void) {
    return analysis.cdl_on;
}

void debug_cdl_clear(void) {
    if (analysis.cdl) {
        memset(analysis.cdl, 0, analysis.size);
    }
    analysis.coverage = (DebugCoverage){0, 0, 0, analysis.size, analysis.size};
}

uint8_t debug_cdl_at(size_t offset) {
    return offset < analysis.size ? analysis.cdl[offset] : 0;
}

DebugCoverage debug_cdl_coverage(void) {
    return analysis.coverage;
}

static void cdl_mark(size_t index, uint8_t flags) {
    uint8_t old = analysis.cdl[index], added = flags & (uint8_t)~old;
    analysis.cdl[index] |= flags;
    analysis.coverage.code += !!(added & DEBUG_CDL_CODE);
    analysis.coverage.data += !!(added & DEBUG_CDL_DATA);
    analysis.coverage.operand += !!(added & DEBUG_CDL_OPERAND);
    if (!old && flags) {
        --analysis.coverage.untouched;
    }
}

bool debug_cdl_save(const char *path) {
    if (!analysis.cdl) {
        return false;
    }
    char header[96];
    int n = snprintf(header, sizeof(header), "CUPID-CDL 1 %zu %s\n", analysis.size, analysis.hash);
    NesFileTransaction *tx = NULL;
    if (nes_file_transaction_begin(path, (uint64_t)analysis.size + sizeof(header), &tx) != NES_FILE_OK) {
        return false;
    }
    if (nes_file_transaction_write(tx, header, (size_t)n) != NES_FILE_OK ||
        nes_file_transaction_write(tx, analysis.cdl, analysis.size) != NES_FILE_OK) {
        nes_file_transaction_abort(&tx);
        return false;
    }
    return nes_file_transaction_commit(&tx) == NES_FILE_OK;
}

bool debug_cdl_load(const char *path) {
    uint8_t *bytes = NULL;
    size_t size;
    if (!analysis.cdl || nes_file_read_all(path, CDL_LIMIT + 96, &bytes, &size) != NES_FILE_OK) {
        return false;
    }
    char header[96];
    int n = snprintf(header, sizeof(header), "CUPID-CDL 1 %zu %s\n", analysis.size, analysis.hash);
    bool ok = size == (size_t)n + analysis.size && !memcmp(bytes, header, (size_t)n);
    for (size_t i = (size_t)n; ok && i < size; ++i) {
        ok = !(bytes[i] & ~7u);
    }
    if (ok) {
        debug_cdl_clear();
        for (size_t i = 0; i < analysis.size; ++i) {
            cdl_mark(i, bytes[n + i]);
        }
    }
    free(bytes);
    return ok;
}

bool debug_cdl_export(const char *path) {
    NesFileTransaction *tx = NULL;
    if (!analysis.cdl || nes_file_transaction_begin(path, (uint64_t)analysis.size * 32 + 64, &tx) != NES_FILE_OK) {
        return false;
    }
    const char *header = "prg_offset,code,data,operand\n";
    bool ok = nes_file_transaction_write(tx, header, strlen(header)) == NES_FILE_OK;
    for (size_t i = 0; ok && i < analysis.size; ++i) {
        char line[64];
        unsigned v = analysis.cdl[i];
        int n = snprintf(line, sizeof(line), "%zu,%u,%u,%u\n", i, !!(v & 1), !!(v & 2), !!(v & 4));
        ok = nes_file_transaction_write(tx, line, (size_t)n) == NES_FILE_OK;
    }
    if (!ok) {
        nes_file_transaction_abort(&tx);
        return false;
    }
    return nes_file_transaction_commit(&tx) == NES_FILE_OK;
}

void debug_profile_enable(bool enabled) {
    if (enabled && !analysis.profile) {
        analysis.profile = calloc(PROFILE_CAP, sizeof(*analysis.profile));
    }
    analysis.profile_on = enabled && analysis.profile;
}

bool debug_profile_enabled(void) {
    return analysis.profile_on;
}

void debug_profile_clear(void) {
    if (analysis.profile) {
        memset(analysis.profile, 0, PROFILE_CAP * sizeof(*analysis.profile));
    }
    analysis.cycles = analysis.profile_lost = 0;
}

uint64_t debug_profile_total_cycles(void) {
    return analysis.cycles;
}

uint64_t debug_profile_dropped(void) {
    return analysis.profile_lost;
}

static void profile_add(uint64_t elapsed) {
    size_t slot = (size_t)((analysis.key * UINT64_C(11400714819323198485)) >> 47);
    analysis.cycles = add_sat(analysis.cycles, elapsed);
    for (size_t i = 0; i < PROFILE_CAP; ++i) {
        DebugProfileRow *row = &analysis.profile[(slot + i) & (PROFILE_CAP - 1)];
        if (!row->instructions || row->key == analysis.key) {
            row->key = analysis.key;
            row->address = analysis.before.pc;
            row->instructions = add_sat(row->instructions, 1);
            row->cycles = add_sat(row->cycles, elapsed);
            return;
        }
    }
    analysis.profile_lost = add_sat(analysis.profile_lost, 1);
}

static int profile_cycles(const void *a, const void *b) {
    const DebugProfileRow *x = a, *y = b;
    if (x->cycles != y->cycles) {
        return x->cycles < y->cycles ? 1 : -1;
    }
    return x->key > y->key ? 1 : x->key < y->key ? -1 : 0;
}

static int profile_instructions(const void *a, const void *b) {
    const DebugProfileRow *x = a, *y = b;
    if (x->instructions != y->instructions) {
        return x->instructions < y->instructions ? 1 : -1;
    }
    return profile_cycles(a, b);
}

size_t debug_profile_rows(DebugProfileRow *out, size_t capacity, bool by_cycles) {
    if (!analysis.profile) {
        return 0;
    }
    size_t count = 0;
    /* Sort the complete set before truncating so a small view still gets the hottest rows. */
    DebugProfileRow *rows = malloc(PROFILE_CAP * sizeof(*rows));
    if (!rows) {
        return 0;
    }
    for (size_t i = 0; i < PROFILE_CAP; ++i) {
        if (analysis.profile[i].instructions) {
            rows[count++] = analysis.profile[i];
        }
    }
    qsort(rows, count, sizeof(*rows), by_cycles ? profile_cycles : profile_instructions);
    size_t copied = count < capacity ? count : capacity;
    if (out && copied) {
        memcpy(out, rows, copied * sizeof(*out));
    }
    free(rows);
    return out ? copied : count;
}

static int profile_key(const void *a, const void *b) {
    const DebugProfileRow *x = a, *y = b;
    return x->key > y->key ? 1 : x->key < y->key ? -1 : 0;
}

size_t debug_profile_function_rows(DebugProfileRow *out, size_t capacity, bool by_cycles) {
    DebugProfileRow *rows = malloc(PROFILE_CAP * sizeof(*rows));
    if (!rows) {
        return 0;
    }
    size_t count = debug_profile_rows(rows, PROFILE_CAP, by_cycles);
    for (size_t i = 0; i < count; ++i) {
        DebugSymbol symbol;
        if (debug_symbol_find(rows[i].key, &symbol) && symbol.function) {
            rows[i].key = symbol.key;
            rows[i].address = symbol.address;
        }
    }
    qsort(rows, count, sizeof(*rows), profile_key);
    size_t total = 0;
    for (size_t i = 0; i < count; ++i) {
        if (total && rows[total - 1].key == rows[i].key) {
            rows[total - 1].cycles = add_sat(rows[total - 1].cycles, rows[i].cycles);
            rows[total - 1].instructions = add_sat(rows[total - 1].instructions, rows[i].instructions);
        } else {
            rows[total++] = rows[i];
        }
    }
    qsort(rows, total, sizeof(*rows), by_cycles ? profile_cycles : profile_instructions);
    size_t copied = total < capacity ? total : capacity;
    if (out && copied) {
        memcpy(out, rows, copied * sizeof(*out));
    }
    free(rows);
    return out ? copied : total;
}

static void stack_push(DebugStackFrame frame) {
    if (analysis.depth == STACK_CAP) {
        memmove(analysis.stack, analysis.stack + 1, (STACK_CAP - 1) * sizeof(frame));
        --analysis.depth;
        for (size_t i = 0; i < analysis.depth; ++i) {
            analysis.stack[i].uncertain = true;
        }
        frame.uncertain = true;
    }
    analysis.stack[analysis.depth++] = frame;
}

size_t debug_stack_count(void) {
    return analysis.depth;
}

void debug_stack_enable(bool enabled) {
    analysis.stack_on = enabled;
}

void debug_stack_clear(void) {
    analysis.depth = 0;
}

bool debug_stack_enabled(void) {
    return analysis.stack_on;
}

bool debug_stack_at(size_t index, DebugStackFrame *out) {
    if (!out || index >= analysis.depth) {
        return false;
    }
    *out = analysis.stack[analysis.depth - 1 - index];
    return true;
}

void debug_analysis_begin(const CPU *state) {
    analysis.pending = false;
    if (!state || state->halted || nes_execution_policy() != NES_EXECUTION_LIVE ||
        !(analysis.cdl_on || analysis.profile_on || analysis.stack_on || analysis.events_on)) {
        return;
    }
    analysis.before = *state;
    analysis.start = cpu_get_bus_cycle();
    analysis.key = debug_analysis_key(state->pc);
    analysis.opcode = 0;
    analysis.length = 1;
    memset(analysis.bytes, 0, sizeof(analysis.bytes));
    analysis.fetch_mask = 0;
    analysis.executed = false;
    analysis.pending = true;
}

void debug_analysis_end(const CPU *state) {
    if (!analysis.pending || !state || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return;
    }
    analysis.pending = false;
    uint64_t end = cpu_get_bus_cycle();
    if (analysis.profile_on && analysis.executed && end >= analysis.start) {
        profile_add(end - analysis.start);
    }
    if (!analysis.stack_on || !analysis.executed) {
        return;
    }
    const CPU *before = &analysis.before;
    if (analysis.opcode == 0x20) {
        uint16_t target = (uint16_t)(analysis.bytes[1] | (analysis.bytes[2] << 8));
        DebugStackFrame frame = {analysis.key,
                                 debug_analysis_key(target),
                                 before->pc,
                                 target,
                                 (uint16_t)(before->pc + 3),
                                 before->sp,
                                 false,
                                 state->sp != (uint8_t)(before->sp - 2) || state->pc != target};
        stack_push(frame);
    } else if (analysis.opcode == 0x00) {
        DebugStackFrame frame = {analysis.key,
                                 debug_analysis_key(state->pc),
                                 before->pc,
                                 state->pc,
                                 (uint16_t)(before->pc + 2),
                                 before->sp,
                                 true,
                                 state->sp != (uint8_t)(before->sp - 3)};
        stack_push(frame);
    } else if (analysis.opcode == 0x60 || analysis.opcode == 0x40) {
        bool found = false;
        for (size_t i = analysis.depth; i; --i) {
            DebugStackFrame *frame = &analysis.stack[i - 1];
            if (frame->return_sp == state->sp && frame->return_address == state->pc &&
                frame->interrupt == (analysis.opcode == 0x40)) {
                analysis.depth = i - 1;
                found = true;
                break;
            }
        }
        if (!found) {
            for (size_t i = 0; i < analysis.depth; ++i) {
                analysis.stack[i].uncertain = true;
            }
        }
    } else if (analysis.opcode == 0x9A || analysis.opcode == 0x9B) {
        for (size_t i = 0; i < analysis.depth; ++i) {
            analysis.stack[i].uncertain = true;
        }
    }
}

void debug_analysis_interrupt(const CPU *before, uint16_t target, bool nmi) {
    if (!before || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return;
    }
    /* Complete the interrupted instruction before recording its interrupt frame. */
    debug_analysis_end(before);
    DebugStackFrame frame = {debug_analysis_key(before->pc),
                             debug_analysis_key(target),
                             before->pc,
                             target,
                             before->pc,
                             before->sp,
                             true,
                             false};
    if (analysis.stack_on) {
        stack_push(frame);
    }
    debug_analysis_event(nmi ? DEBUG_EVENT_NMI : DEBUG_EVENT_IRQ, target, 1);
}

void debug_events_enable(bool enabled) {
    if (enabled && !analysis.events[0]) {
        DebugNesEvent *a = malloc(EVENT_CAP * sizeof(*a)), *b = malloc(EVENT_CAP * sizeof(*b));
        if (!a || !b) {
            free(a);
            free(b);
            return;
        }
        analysis.events[0] = a;
        analysis.events[1] = b;
    }
    analysis.events_on = enabled;
}

bool debug_events_enabled(void) {
    return analysis.events_on;
}

void debug_events_clear(void) {
    analysis.event_count[0] = analysis.event_count[1] = 0;
    analysis.event_lost[0] = analysis.event_lost[1] = 0;
    analysis.frame_valid = analysis.completed = false;
}

void debug_analysis_event(uint32_t category, uint16_t address, uint8_t value) {
    if (!analysis.events_on || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return;
    }
    if (analysis.frame_valid && analysis.frame != ppu.frame_count) {
        analysis.completed = true;
        analysis.writing ^= 1;
        analysis.event_count[analysis.writing] = 0;
        analysis.event_lost[analysis.writing] = 0;
    }
    analysis.frame_valid = true;
    analysis.frame = ppu.frame_count;
    unsigned w = analysis.writing;
    if (analysis.event_count[w] == EVENT_CAP) {
        ++analysis.event_lost[w];
        return;
    }
    DebugNesEvent *event = &analysis.events[w][analysis.event_count[w]++];
    *event = (DebugNesEvent){ppu.frame_count,
                             cpu_get_bus_cycle(),
                             debug_analysis_key(address),
                             analysis.pending ? analysis.key : debug_analysis_key(cpu.pc),
                             category,
                             analysis.pending ? analysis.before.pc : cpu.pc,
                             address,
                             (int16_t)ppu.scanline,
                             (int16_t)ppu.dot,
                             value};
}

size_t debug_events_count(uint32_t mask) {
    if (!analysis.completed) {
        return 0;
    }
    unsigned r = analysis.writing ^ 1;
    size_t count = 0;
    for (size_t i = 0; i < analysis.event_count[r]; ++i) {
        count += !!(analysis.events[r][i].category & mask);
    }
    return count;
}

bool debug_events_at(size_t index, uint32_t mask, DebugNesEvent *out) {
    if (!analysis.completed || !out) {
        return false;
    }
    unsigned r = analysis.writing ^ 1;
    for (size_t i = 0; i < analysis.event_count[r]; ++i) {
        if (!(analysis.events[r][i].category & mask)) {
            continue;
        }
        if (!index--) {
            *out = analysis.events[r][i];
            return true;
        }
    }
    return false;
}

uint64_t debug_events_dropped(void) {
    return analysis.completed ? analysis.event_lost[analysis.writing ^ 1] : 0;
}

void debug_analysis_signals(bool nmi, bool irq, bool mapper_irq) {
    if (!analysis.events_on || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return;
    }
    if (nmi != analysis.nmi) {
        debug_analysis_event(DEBUG_EVENT_NMI, 0, nmi);
    }
    if (irq != analysis.irq) {
        debug_analysis_event(DEBUG_EVENT_IRQ, 0, irq);
    }
    if (mapper_irq != analysis.mapper_irq) {
        debug_analysis_event(DEBUG_EVENT_MAPPER | DEBUG_EVENT_IRQ, 0, mapper_irq);
    }
    analysis.nmi = nmi;
    analysis.irq = irq;
    analysis.mapper_irq = mapper_irq;
}

void debug_analysis_read(uint16_t address, uint8_t value) {
    unsigned flag = DEBUG_CDL_DATA;
    uint16_t delta = (uint16_t)(address - analysis.before.pc);
    if (analysis.pending && analysis.executed && delta < analysis.length && !(analysis.fetch_mask & (1u << delta))) {
        analysis.fetch_mask |= (uint8_t)(1u << delta);
        analysis.bytes[delta] = value;
        flag = delta ? DEBUG_CDL_OPERAND : DEBUG_CDL_CODE;
    }
    if (analysis.cdl_on) {
        uint64_t key = debug_analysis_key(address);
        /* The opcode hook supplies its classification after the fetch finishes. */
        if (key < analysis.size && !(analysis.pending && !analysis.executed && !delta)) {
            cdl_mark((size_t)key, (uint8_t)flag);
        }
    }
    uint32_t category = DEBUG_EVENT_READ;
    if (address >= 0x2000 && address < 0x4000) {
        category |= DEBUG_EVENT_PPU;
    }
    if (address >= 0x4000 && address < 0x4016) {
        category |= DEBUG_EVENT_APU;
    }
    debug_analysis_event(category, address, value);
}

void debug_analysis_opcode(uint8_t opcode) {
    if (!analysis.pending) {
        return;
    }
    analysis.executed = true;
    analysis.opcode = analysis.bytes[0] = opcode;
    analysis.length = debugger_opcode(opcode).length;
    analysis.fetch_mask = 1;
    if (analysis.cdl_on && analysis.key < analysis.size) {
        cdl_mark((size_t)analysis.key, DEBUG_CDL_CODE);
    }
}

void debug_analysis_write(uint16_t address, uint8_t value) {
    uint32_t category = DEBUG_EVENT_WRITE;
    if (address >= 0x2000 && address < 0x4000) {
        category |= DEBUG_EVENT_PPU;
    }
    if (address >= 0x4000 && address < 0x4016) {
        category |= DEBUG_EVENT_APU;
    }
    if (address == 0x4014) {
        category |= DEBUG_EVENT_DMA;
    }
    if (address >= 0x4020) {
        category |= DEBUG_EVENT_MAPPER;
    }
    /* Any explicit stack write can invalidate the saved return bytes. */
    if (address >= 0x100 && address < 0x200) {
        for (size_t i = 0; i < analysis.depth; ++i) {
            DebugStackFrame *f = &analysis.stack[i];
            uint8_t slot = (uint8_t)address;
            if (slot == f->return_sp || slot == (uint8_t)(f->return_sp - 1)) {
                f->uncertain = true;
            }
        }
    }
    debug_analysis_event(category, address, value);
}
