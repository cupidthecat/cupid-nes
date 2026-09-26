/*
 * debug_tools_accuracy.c - Execution analysis and native panel regression coverage
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../debugger/debug_analysis.h"
#include "../debugger/debug_catalog.h"
#include "../debugger/debug_capture.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../ui/debug_tools_frontend.h"
#include "../ui/frontend_panels.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(c)                                                                                                       \
    do {                                                                                                               \
        ++checks;                                                                                                      \
        if (!(c)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #c);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)
static uint8_t image[16 + 0x10000 + 0x2000];

static int machine(bool banked) {
    debugger_shutdown();
    if (!unload_rom() || !nes_set_region_mode(NES_REGION_MODE_NTSC)) {
        return -1;
    }
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = banked ? 4 : 2;
    image[5] = 1;
    image[6] = banked ? 0x40 : 0;
    size_t prg = banked ? 0x10000 : 0x8000;
    memset(image + 16, 0xEA, prg);
    const uint8_t program[] = {0xA9, 0x41, 0x8D, 0x10, 0x00, 0x20, 0x10, 0x80, 0xAD, 0x00, 0x90, 0x4C, 0x0B, 0x80};
    memcpy(image + 16, program, sizeof(program));
    image[16 + 0x10] = 0xE8;
    image[16 + 0x11] = 0x60;
    image[16 + 0x20] = 0x40;
    image[16 + 0x30] = 0x00;
    image[16 + 0x1000] = 0x42;
    for (unsigned offset = 0xFFFA; offset <= 0xFFFE; offset += 2) {
        image[16 + prg - 0x10000 + offset] = offset == 0xFFFC ? 0 : 0x20;
        image[16 + prg - 0x10000 + offset + 1] = 0x80;
    }
    if (load_rom_memory(image, 16 + prg + 0x2000) != 0) {
        return -1;
    }
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_total_cycles = 0;
    if (!cpu_power_on(&cpu)) {
        return -1;
    }
    debugger_init();
    return 0;
}

static int uxrom_physical_mapping(void) {
    CHECK(unload_rom());
    for (unsigned mapper = 2; mapper <= 180; mapper = mapper == 2 ? 94 : 180) {
        memset(image, 0, sizeof(image));
        memcpy(image, "NES\x1A", 4);
        image[4] = 4;
        image[5] = 1;
        image[6] = (uint8_t)((mapper & 0x0f) << 4);
        image[7] = (uint8_t)(mapper & 0xf0);
        for (unsigned bank = 0; bank < 4; ++bank) {
            image[16 + bank * 0x4000] = (uint8_t)(0x40 + bank);
        }
        image[16 + 0xFFFC] = 0;
        image[16 + 0xFFFD] = 0x80;
        CHECK(load_rom_memory(image, 16 + 0x10000 + 0x2000) == 0);
        NesMemoryLocation low, high;
        CHECK(cart_debug_cpu_location(0x8000, &low) && cart_debug_cpu_location(0xC000, &high));
        CHECK(low.offset == 0 && high.offset == (mapper == 180 ? 0 : 0xC000));
        CHECK(!low.writable && !high.writable);
        cart_cpu_write(0x8000, mapper == 94 ? 4 : 1);
        CHECK(cart_debug_cpu_location(0x8000, &low) && cart_debug_cpu_location(0xC000, &high));
        CHECK(low.offset == (mapper == 180 ? 0 : 0x4000));
        CHECK(high.offset == (mapper == 180 ? 0x4000 : 0xC000));
        CHECK(!memcmp(low.data, image + 16 + low.offset, 1) && !memcmp(high.data, image + 16 + high.offset, 1));
        CHECK(unload_rom());
        if (mapper == 180) {
            break;
        }
    }
    return 0;
}

static int coverage_profile_stack_trace(void) {
    CHECK(machine(false) == 0);
    debug_cdl_enable(true);
    debug_profile_enable(true);
    debug_stack_enable(true);
    DebugTraceFilter filter = {0x8000, 0xFFFF, "", DEBUG_TRACE_ALL};
    char error[256];
    CHECK(debug_log_configure(3, &filter, error, sizeof(error)));
    debug_log_enable(true);
    DebugCoverage before = debug_cdl_coverage();
    CHECK(before.total == 0x8000 && before.untouched == before.total);
    CHECK(debugger_peek_cpu(0x8000) == 0xA9 && debug_cdl_at(0) == 0);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 6);
    CHECK(debug_stack_count() == 1);
    DebugStackFrame frame;
    CHECK(debug_stack_at(0, &frame) && frame.source == 0x8005 && frame.target == 0x8010 &&
          frame.return_address == 0x8008 && !frame.uncertain);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 6);
    CHECK(debug_stack_count() == 0 && cpu.pc == 0x8008);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x42);
    CHECK(debug_cdl_at(0) == DEBUG_CDL_CODE && debug_cdl_at(1) == DEBUG_CDL_OPERAND);
    CHECK(debug_cdl_at(0x1000) & DEBUG_CDL_DATA);
    CHECK(debug_profile_total_cycles() == 24);
    DebugProfileRow rows[8];
    CHECK(debug_profile_rows(rows, 8, true) == 6 && rows[0].cycles == 6);
    DebugSymbol function = {0, 0x8000, 14, 0, true, "main", "", ""};
    CHECK(debug_symbol_set(&function));
    CHECK(debug_profile_function_rows(rows, 8, true) == 3 && rows[0].key == 0 && rows[0].cycles == 16);
    debug_catalog_clear();
    CHECK(debug_log_count() == 3 && debug_log_overwritten() == 3);
    DebugLogEntry entry;
    CHECK(debug_log_at(2, &entry) && entry.cpu.pc == 0x8008 && entry.bytes[2] == 0x90);
    CHECK(!debug_log_at(3, &entry));
    CHECK(debug_cdl_save("build/debugger-test.cdl"));
    DebugCoverage saved = debug_cdl_coverage();
    debug_cdl_clear();
    CHECK(debug_cdl_load("build/debugger-test.cdl"));
    CHECK(debug_cdl_coverage().untouched == saved.untouched);
    CHECK(nes_file_write_atomic("build/debugger-invalid.cdl", "bad", 3) == NES_FILE_OK);
    CHECK(!debug_cdl_load("build/debugger-invalid.cdl") && debug_cdl_coverage().untouched == saved.untouched);
    CHECK(debug_cdl_export("build/debugger-test.csv"));
    CHECK(debug_log_export("build/debugger-trace.txt"));
    DebugTraceFilter invalid = filter;
    invalid.first = 5;
    invalid.last = 4;
    CHECK(!debug_log_configure(5, &invalid, error, sizeof(error)) && debug_log_count() == 3);
    debugger_pause();
    uint64_t cycles = debug_profile_total_cycles();
    CHECK(cpu_step(&cpu) == 0 && debug_profile_total_cycles() == cycles);
    debugger_resume();
    CHECK(nes_execution_set_policy(NES_EXECUTION_SPECULATIVE));
    CHECK(cpu_step(&cpu) == 3 && debug_profile_total_cycles() == cycles && debug_log_count() == 3);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    cpu.pc = 0x8030;
    CHECK(cpu_step(&cpu) == 7 && debug_stack_count() == 1);
    CHECK(debug_stack_at(0, &frame) && frame.interrupt && frame.return_address == 0x8032);
    CHECK(cpu_step(&cpu) == 6 && debug_stack_count() == 0);
    cpu.pc = 0x800B;
    cycles = debug_profile_total_cycles();
    cpu_request_nmi();
    CHECK(cpu_step(&cpu) == 7 && debug_stack_count() == 1 && debug_profile_total_cycles() == cycles);
    CHECK(debug_stack_at(0, &frame) && frame.interrupt && frame.return_address == 0x800B);
    CHECK(cpu_step(&cpu) == 6 && cpu.pc == 0x800B && debug_stack_count() == 0);
    cpu.pc = 0x8005;
    CHECK(cpu_step(&cpu) == 6 && debug_stack_count() == 1);
    CHECK(debug_stack_at(0, &frame));
    write_mem((uint16_t)(0x100u + frame.return_sp), 0x90);
    CHECK(debug_stack_at(0, &frame) && frame.uncertain);
    debugger_invalidate_memory();
    CHECK(debug_stack_count() == 0);
    return 0;
}

static int bank_identity_symbols_source_references(void) {
    CHECK(machine(true) == 0);
    CHECK(!debug_cdl_load("build/debugger-test.cdl"));
    debug_cdl_enable(true);
    debug_profile_enable(true);
    CHECK(cpu_step(&cpu) == 2 && debug_analysis_key(0x8000) == 0);
    DebugSymbol symbol = {0, 0x8000, 14, 2, true, "entry", "debugger-source.asm", "entry point"};
    CHECK(debug_symbol_set(&symbol));
    CHECK(debug_symbol_export("build/debugger-symbols.txt"));
    debug_catalog_clear();
    char error[256];
    CHECK(debug_symbol_import("build/debugger-symbols.txt", error, sizeof(error)) && debug_symbol_count() == 1);
    CHECK(nes_file_write_atomic("build/debugger-bad-symbols.txt", "invalid\n", 8) == NES_FILE_OK);
    CHECK(!debug_symbol_import("build/debugger-bad-symbols.txt", error, sizeof(error)) && debug_symbol_count() == 1);
    DebugSymbol wrong = symbol;
    strcpy(wrong.file, "../outside.asm");
    CHECK(!debug_symbol_set(&wrong));
    const char source[] = "; synthetic source\nentry: lda #$41\n";
    CHECK(nes_file_write_atomic("build/debugger-source.asm", source, sizeof(source) - 1) == NES_FILE_OK);
    char line[128];
    CHECK(debug_source_line("build", &symbol, line, sizeof(line), error, sizeof(error)));
    CHECK(!strcmp(line, "entry: lda #$41"));
    CHECK(!debug_source_line("build/missing", &symbol, line, sizeof(line), error, sizeof(error)));
    DebugDisassembly dis;
    CHECK(debugger_disassemble(0x8000, &dis) && strstr(dis.text, "entry"));
    DebugReference refs[8];
    CHECK(debug_references(0x8000, 0x800D, DEBUG_KEY_NONE, 0x8010, false, refs, 8) == 1);
    CHECK(refs[0].address == 0x8005 && refs[0].key == 5 && !refs[0].constant);
    CHECK(debug_references(0x8000, 0x800D, 0, 0x41, true, refs, 8) == 1 && refs[0].constant);
    uint32_t breakpoint = debug_source_breakpoint(&symbol);
    CHECK(breakpoint != 0);
    write_mem(0x8000, 6);
    write_mem(0x8001, 2);
    CHECK(debug_analysis_key(0x8000) == 0x4000);
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 2 && !debugger_is_paused());
    CHECK(debug_cdl_at(0x4000) & DEBUG_CDL_CODE);
    DebugProfileRow rows[8];
    CHECK(debug_profile_rows(rows, 8, false) == 2);
    CHECK(rows[0].key != rows[1].key);
    write_mem(0x8001, 0);
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 0 && debugger_is_paused());
    CHECK(debugger_remove_breakpoint(breakpoint));
    debugger_resume();
    CHECK(machine(false) == 0);
    CHECK(debug_symbol_count() == 0);
    return 0;
}

static int event_and_text_capture(void) {
    CHECK(machine(false) == 0);
    debug_events_enable(true);
    uint64_t frame = ppu.frame_count;
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(debug_events_count(0x3FF) == 0);
    ppu.frame_count = frame + 1;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(debug_events_count(DEBUG_EVENT_READ) == 5 && debug_events_count(DEBUG_EVENT_WRITE) == 1);
    DebugNesEvent event;
    CHECK(debug_events_at(0, DEBUG_EVENT_WRITE, &event) && event.address == 0x10 && event.value == 0x41 &&
          event.frame == frame);
    CHECK(!debug_events_at(1, DEBUG_EVENT_WRITE, &event));
    char error[256];
    const char table[] = "41=A\n4243=BC\n00=\\n\n";
    CHECK(debug_text_table(table, sizeof(table) - 1, error, sizeof(error)));
    const char ambiguous[] = "41=A\n4142=B\n";
    CHECK(!debug_text_table(ambiguous, sizeof(ambiguous) - 1, error, sizeof(error)));
    debug_text_configure(0x10, 0x10, false, true, true);
    debug_text_enable(true);
    for (unsigned repeat = 0; repeat < 2; ++repeat) {
        write_mem(0x10, 0x41);
        write_mem(0x10, 0x42);
        write_mem(0x10, 0x43);
        write_mem(0x10, 0);
    }
    CHECK(!strcmp(debug_text_output(), "ABC\n"));
    write_mem(0x11, 0x41);
    write_mem(0x11, 0);
    CHECK(debug_text_size() == 4);
    CHECK(debug_text_export("build/debugger-text.txt"));
    debug_text_enable(false);
    write_mem(0x10, 0x41);
    debug_text_flush();
    CHECK(debug_text_size() == 4);
    debug_text_clear();
    CHECK(debug_text_size() == 0);
    debug_events_clear();
    for (unsigned i = 0; i < 131075; ++i) {
        debug_analysis_event(DEBUG_EVENT_IRQ, 0, 1);
    }
    ++ppu.frame_count;
    debug_analysis_event(DEBUG_EVENT_IRQ, 0, 0);
    CHECK(debug_events_count(DEBUG_EVENT_IRQ) == 131072 && debug_events_dropped() == 3);
    return 0;
}

static uint8_t observed_memory[65536];
static unsigned observed_reads, observed_writes;

static uint8_t observe_read(uint16_t address) {
    ++observed_reads;
    return observed_memory[address];
}

static void observe_write(uint16_t address, uint8_t value) {
    ++observed_writes;
    observed_memory[address] = value;
}

static int observational_bus(void) {
    CHECK(machine(false) == 0);
    Mapper *saved = cart;
    Mapper observer = {0};
    observer.cpu_read = observe_read;
    observer.cpu_write = observe_write;
    cart = &observer;
    observed_memory[0x8000] = 0xA9;
    observed_memory[0x8001] = 0x7F;
    cpu.pc = 0x8000;
    observed_reads = observed_writes = 0;
    int elapsed = cpu_step(&cpu);
    unsigned reads = observed_reads, writes = observed_writes;
    cart = saved;
    CHECK(elapsed == 2 && reads == 2 && writes == 0);
    debug_profile_enable(true);
    debug_stack_enable(true);
    debug_events_enable(true);
    debug_log_enable(true);
    cart = &observer;
    cpu.pc = 0x8000;
    observed_reads = observed_writes = 0;
    elapsed = cpu_step(&cpu);
    reads = observed_reads;
    writes = observed_writes;
    cart = saved;
    CHECK(elapsed == 2 && reads == 2 && writes == 0);
    CHECK(debug_profile_total_cycles() == 2 && debug_log_count() == 1);
    DebugLogEntry entry;
    CHECK(debug_log_at(0, &entry) && entry.bytes[0] == 0xA9 && entry.bytes[1] == 0x7F);
    return 0;
}

static int native_panels(void) {
    CHECK(machine(false) == 0);
    FrontendExecutionRuntime execution = {0};
    frontend_execution_init(&execution, NULL, 0, NULL, NULL, NULL, NULL);
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    DebugToolsFrontend *frontend = debug_tools_frontend_create(&execution);
    CHECK(frontend && debug_tools_frontend_register(frontend));
    CHECK(frontend_panel_count() == DEBUG_TOOLS_PANEL_COUNT);
    char error[256];
    for (unsigned i = 0; i < DEBUG_TOOLS_PANEL_COUNT; ++i) {
        FrontendPanelControl controls[64];
        FrontendPanelModel model = {controls, 64, 0, NULL};
        CHECK(frontend_panel_snapshot(DEBUG_TOOLS_PANEL_FIRST + i, &model, error, sizeof(error)) && model.count >= 8);
    }
    CHECK(frontend_panel_action(0x2400, DEBUG_TOOL_START, NULL, 0, error, sizeof(error)) && debug_cdl_enabled());
    CHECK(frontend_panel_action(0x2408, DEBUG_TOOL_CAPACITY, "4", 0, error, sizeof(error)));
    CHECK(frontend_panel_action(0x2408, DEBUG_TOOL_START, NULL, 0, error, sizeof(error)) && debug_log_enabled());
    CHECK(cpu_step(&cpu) == 2 && debug_log_count() == 1);
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {controls, 64, 0, NULL};
    CHECK(frontend_panel_snapshot(0x2408, &model, error, sizeof(error)));
    CHECK(frontend_panel_action(0x2408, DEBUG_TOOL_DISASSEMBLY, NULL, 0, error, sizeof(error)));
    CHECK(frontend_panel_action(0x2408, DEBUG_TOOL_BREAK, NULL, 0, error, sizeof(error)));
    CHECK(debugger_breakpoint_count() == 1);
    debug_tools_frontend_image_changed(frontend);
    CHECK(!debug_log_enabled() && debug_log_count() == 0 && debug_symbol_count() == 0);
    debug_tools_frontend_destroy(frontend);
    CHECK(frontend_panel_count() == 0);
    frontend_execution_shutdown(&execution);
    frontend_panels_reset();
    return 0;
}

int run_debug_tools_accuracy_tests(void) {
    checks = 0;
    int failures = uxrom_physical_mapping() + coverage_profile_stack_trace() +
                   bank_identity_symbols_source_references() + event_and_text_capture() + observational_bus() +
                   native_panels();
    debugger_shutdown();
    (void)unload_rom();
    printf("Debugger tools: %s (%u checks, %d failing groups)\n", failures ? "FAIL" : "PASS", checks, failures);
    return failures;
}
