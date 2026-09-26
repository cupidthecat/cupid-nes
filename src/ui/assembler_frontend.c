/*
 * assembler_frontend.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Inline assembler controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "assembler_frontend.h"
#include "frontend_panels.h"
#include "../debugger/assembler.h"
#include "../debugger/debugger.h"
#include "../debugger/expression.h"
#include "../debugger/memory_editor.h"
#include "../system/execution_policy.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const DebugMemorySpace spaces[] = {DEBUG_MEMORY_CPU, DEBUG_MEMORY_RAM, DEBUG_MEMORY_CART_RAM};

struct AssemblerFrontend {
    FrontendExecutionRuntime *execution;
    DebugAssembly preview;
    DebugMemoryByte before[3];
    uint64_t session, pause_revision;
    int space;
    bool registered, valid, writable;
    char address[DEBUG_EXPRESSION_LIMIT + 1], source[DEBUG_ASSEMBLY_TEXT_LIMIT + 1];
    char bytes[96], backing[192], status[256];
    char lines[6][64];
    const char *items[6];
};

static bool report(AssemblerFrontend *frontend, char *error, size_t size, const char *message, bool ok) {
    snprintf(frontend->status, sizeof(frontend->status), "%s", message);
    if (error && size) {
        snprintf(error, size, "%s", ok ? "" : message);
    }

    return ok;
}

static bool current(const AssemblerFrontend *frontend) {
    return frontend->valid && frontend_panel_session_active() && frontend->session == debugger_session_revision() &&
           frontend->pause_revision == debugger_pause_revision();
}

static bool editable(const AssemblerFrontend *frontend) {
    return current(frontend) && frontend->writable && debugger_is_paused() &&
           frontend_execution_paused(frontend->execution) && nes_execution_policy() == NES_EXECUTION_LIVE;
}

static bool preview(AssemblerFrontend *frontend, char *error, size_t error_size) {
    uint32_t address = 0;
    DebugAssembly result;
    char message[256];
    frontend->valid = false;
    if (!debug_expression_eval(frontend->address, &address, message, sizeof(message))) {
        return report(frontend, error, error_size, message, false);
    }

    if (address > UINT16_MAX) {
        return report(frontend, error, error_size, "The instruction address must fit in 16 bits", false);
    }

    if (!debugger_assemble((uint16_t)address, frontend->source, &result, message, sizeof(message))) {
        return report(frontend, error, error_size, message, false);
    }

    frontend_execution_begin_machine_change(frontend->execution);
    frontend->preview = result;
    frontend->session = debugger_session_revision();
    frontend->pause_revision = debugger_pause_revision();
    frontend->writable = true;
    memset(frontend->before, 0, sizeof(frontend->before));
    for (unsigned i = 0; i < result.length; ++i) {
        DebugMemoryByte *byte = &frontend->before[i];
        if (!debug_memory_inspect(spaces[frontend->space], address + i, byte) || !byte->physical || !byte->writable) {
            frontend->writable = false;
        }
    }

    uint16_t pc = result.address;
    for (unsigned i = 0; i < 6; ++i) {
        DebugDisassembly dis;
        (void)debugger_disassemble(pc, &dis);
        snprintf(frontend->lines[i], sizeof(frontend->lines[i]), "%s", dis.text);
        frontend->items[i] = frontend->lines[i];
        pc = (uint16_t)(pc + dis.length);
    }

    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    char bytes[9];
    (void)debug_memory_format_hex(result.bytes, result.length, bytes, sizeof(bytes));
    snprintf(frontend->bytes, sizeof(frontend->bytes), "$%04X: %s (%u bytes)", result.address, bytes, result.length);
    const DebugMemoryByte *first = &frontend->before[0];
    snprintf(frontend->backing, sizeof(frontend->backing), "%s + $%zX | %s",
             first->backing ? first->backing : "Unavailable", first->offset,
             frontend->writable ? "Writable backing" : "Read only, protected or outside the selected memory space");
    frontend->valid = true;
    return report(frontend, error, error_size, "Preview ready. Apply writes only the displayed instruction bytes",
                  true);
}

static bool apply(AssemblerFrontend *frontend, char *error, size_t error_size) {
    if (!editable(frontend)) {
        return report(frontend, error, error_size,
                      "Pause a live game and preview a writable target again before applying", false);
    }

    frontend_execution_begin_machine_change(frontend->execution);
    bool unchanged = current(frontend);
    for (unsigned i = 0; unchanged && i < frontend->preview.length; ++i) {
        DebugMemoryByte now;
        const DebugMemoryByte *before = &frontend->before[i];
        unchanged = debug_memory_inspect(spaces[frontend->space], frontend->preview.address + i, &now) &&
                    now.writable && now.physical && now.value == before->value && now.offset == before->offset &&
                    now.backing && before->backing && !strcmp(now.backing, before->backing);
    }

    if (!unchanged) {
        frontend_execution_end_machine_change_preserving_audio(frontend->execution);
        frontend->valid = false;
        return report(frontend, error, error_size, "The target changed; preview it again before writing", false);
    }

    bool changed = false;
    char message[256];
    bool ok = debug_memory_edit(spaces[frontend->space], frontend->preview.address, frontend->preview.bytes,
                                frontend->preview.length, frontend->session, &changed, message, sizeof(message));
    if (ok && changed) {
        frontend_execution_clear_timeline(frontend->execution);
    }

    frontend_execution_end_machine_change_preserving_audio(frontend->execution);
    if (!ok) {
        return report(frontend, error, error_size, message, false);
    }

    (void)preview(frontend, NULL, 0);
    return report(frontend, error, error_size,
                  changed ? "Instruction written; rewind history cleared" : "The instruction bytes already match",
                  true);
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    AssemblerFrontend *frontend = context;
    static const char *const names[] = {"CPU address space", "Internal CPU RAM", "Cartridge RAM"};
    FrontendPanelControl controls[] = {
        {ASSEMBLER_SPACE, FRONTEND_PANEL_CHOICE, "Write target", NULL, names, 3, frontend->space, true, false},
        {ASSEMBLER_ADDRESS, FRONTEND_PANEL_TEXT, "CPU address / expression (hex)", frontend->address, NULL, 0, 0, true,
         false},
        {ASSEMBLER_PC, FRONTEND_PANEL_ACTION, "Use current PC", "", NULL, 0, 0, true, false},
        {ASSEMBLER_SOURCE, FRONTEND_PANEL_TEXT, "6502 instruction", frontend->source, NULL, 0, 0, true, false},
        {ASSEMBLER_PREVIEW, FRONTEND_PANEL_ACTION, "Preview instruction", "", NULL, 0, 0, true, false},
        {ASSEMBLER_BYTES, FRONTEND_PANEL_TEXT, "Generated bytes",
         current(frontend) ? frontend->bytes : "Preview required", NULL, 0, 0, true, true},
        {ASSEMBLER_BACKING, FRONTEND_PANEL_TEXT, "Target backing", current(frontend) ? frontend->backing : "", NULL, 0,
         0, true, true},
        {ASSEMBLER_APPLY, FRONTEND_PANEL_ACTION, "Apply instruction", "", NULL, 0, 0, editable(frontend), false},
        {ASSEMBLER_CONTEXT, FRONTEND_PANEL_LIST, "Existing disassembly at preview", "", frontend->items,
         current(frontend) ? 6u : 0u, 0, true, true},
        {ASSEMBLER_SYNTAX, FRONTEND_PANEL_TEXT, "Operand syntax",
         "$hex / 0xhex, %binary, decimal; $0010 forces absolute. One instruction; semicolon comments.", NULL, 0, 0,
         true, true}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }

    model->status = frontend->status;
    if (error && error_size) {
        error[0] = '\0';
    }

    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t error_size) {
    AssemblerFrontend *frontend = context;
    if (id == ASSEMBLER_PREVIEW) {
        return preview(frontend, error, error_size);
    }

    if (id == ASSEMBLER_APPLY) {
        return apply(frontend, error, error_size);
    }

    if (id == ASSEMBLER_ADDRESS || id == ASSEMBLER_SOURCE) {
        char *target = id == ASSEMBLER_ADDRESS ? frontend->address : frontend->source;
        size_t capacity = id == ASSEMBLER_ADDRESS ? sizeof(frontend->address) : sizeof(frontend->source);
        if (!value || strlen(value) >= capacity) {
            return report(frontend, error, error_size, "The assembler field is too long", false);
        }

        strcpy(target, value);
    } else if (id == ASSEMBLER_PC) {
        DebugCpuSnapshot cpu;
        debugger_get_cpu(&cpu);
        snprintf(frontend->address, sizeof(frontend->address), "$%04X", cpu.cpu.pc);
    } else if (id == ASSEMBLER_SPACE && selected >= 0 && selected < 3) {
        frontend->space = selected;
    } else {
        return report(frontend, error, error_size, "Unknown assembler control", false);
    }

    frontend->valid = false;
    return report(frontend, error, error_size, "Preview the instruction before applying", true);
}

AssemblerFrontend *assembler_frontend_create(FrontendExecutionRuntime *execution) {
    if (!execution) {
        return NULL;
    }

    AssemblerFrontend *frontend = calloc(1, sizeof(*frontend));
    if (frontend) {
        frontend->execution = execution;
        strcpy(frontend->address, "$0000");
        strcpy(frontend->source, "NOP");
    }

    return frontend;
}

bool assembler_frontend_register(AssemblerFrontend *frontend) {
    if (!frontend || frontend->registered) {
        return false;
    }

    FrontendPanelSpec panel = {
        ASSEMBLER_FRONTEND_PANEL, "6502 Assembler", "Tools", FRONTEND_PANEL_NEEDS_SESSION, snapshot, action, frontend};
    frontend->registered = frontend_panel_register(&panel);
    return frontend->registered;
}

void assembler_frontend_unregister(AssemblerFrontend *frontend) {
    if (frontend && frontend->registered) {
        (void)frontend_panel_unregister(ASSEMBLER_FRONTEND_PANEL);
        frontend->registered = false;
    }
}

void assembler_frontend_image_changed(AssemblerFrontend *frontend) {
    if (frontend) {
        frontend->valid = false;
        frontend->status[0] = '\0';
    }
}

void assembler_frontend_destroy(AssemblerFrontend *frontend) {
    assembler_frontend_unregister(frontend);
    free(frontend);
}
