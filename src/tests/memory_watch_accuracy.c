/*
 * memory_watch_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Watch expressions, comparison history and observational reads. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cheats/cheats.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_watch.h"
#include "../joypad/joypad.h"
#include "../state/state.h"
#include "../system/execution_policy.h"

static int machine(unsigned mapper) {
    BoardImage image = {0};
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    BOARD_CHECK(board_image_create(&image, mapper, 0x10000, 0x8000, false));
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    debugger_init();
    return 0;
}

static DebugWatchSpec spec_at(const char *expression) {
    DebugWatchSpec spec = {.space = DEBUG_MEMORY_CPU, .width = 1, .format = DEBUG_WATCH_HEX, .enabled = true};
    snprintf(spec.expression, sizeof(spec.expression), "%s", expression);
    return spec;
}

static int expressions(void) {
    BOARD_CHECK(machine(0) == 0);
    cpu.pc = 0x8123;
    cpu.sp = 0xFD;
    cpu.a = 0x28;
    cpu.x = 3;
    cpu.y = 5;
    const struct { const char *text; uint32_t value; } valid[] = {
        {"10", 16}, {"$a", 10}, {"#10", 10}, {"%101", 5}, {"0xABCD", 0xABCD},
        {" PC + X + Y ", 0x812B}, {"SP | $100", 0x1FD}, {"A+#2", 0x2A},
        {"#1 + #2 << #3", 24}, {"#4 | #1 * #3", 7}, {"(#4 | #1) * #3", 15},
        {"#20/#2/#2", 5}, {"~$FFFF & $FFFF0000", 0xFFFF0000}, {"+$1 ^ $3", 2}
    };
    char error[96];
    for (size_t i = 0; i < sizeof(valid) / sizeof(*valid); ++i) {
        uint32_t value = 0;
        BOARD_CHECK(debug_expression_eval(valid[i].text, &value, error, sizeof(error)));
        BOARD_CHECK(value == valid[i].value && !error[0]);
    }
    const char *const invalid[] = {"", "(", "(1", "1)", "0x", "$", "%", "1/0", "-1", "0-1",
        "FFFFFFFF+1", "FFFFFFFF*2", "1<<#32", "FFFFFFFF<<1", "1||2", "missing_label", "1 trailing",
        "((((((((((((((((((1))))))))))))))))))", "\xFF"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        uint32_t value = 0xCAFE;
        BOARD_CHECK(!debug_expression_eval(invalid[i], &value, error, sizeof(error)));
        BOARD_CHECK(value == 0xCAFE && error[0]);
    }
    char unterminated[DEBUG_EXPRESSION_LIMIT];
    memset(unterminated, '9', sizeof(unterminated));
    uint32_t unchanged = 0xCAFE;
    BOARD_CHECK(!debug_expression_eval(unterminated, &unchanged, NULL, 0) && unchanged == 0xCAFE);
    BOARD_CHECK(!debug_expression_eval(NULL, &unchanged, NULL, 0));
    BOARD_CHECK(!debug_expression_eval("1", NULL, NULL, 0));
    return 0;
}

static int history_and_bounds(void) {
    BOARD_CHECK(machine(0) == 0);
    write_mem(0x10, 10);
    DebugWatchSpec spec = spec_at("$10");
    char error[96];
    uint32_t id = debug_watch_add(&spec, 1, error, sizeof(error));
    BOARD_CHECK(id && !error[0]);
    DebugWatchSample row;
    BOARD_CHECK(debug_watch_at(0, &row) && row.current == 10 && row.previous == 10 && row.previous_valid);
    write_mem(0x10, 20);
    debug_watch_refresh();
    BOARD_CHECK(debug_watch_at(0, &row) && row.current == 20 && row.previous == 10 && row.changes == 1);
    debug_watch_refresh();
    BOARD_CHECK(debug_watch_at(0, &row) && row.previous == 10 && row.changes == 1);
    debug_watch_freeze_previous(true);
    BOARD_CHECK(debug_watch_previous_frozen());
    write_mem(0x10, 30);
    debug_watch_refresh();
    BOARD_CHECK(debug_watch_at(0, &row) && row.current == 30 && row.previous == 10 && row.changes == 2);
    debug_watch_capture_previous();
    BOARD_CHECK(debug_watch_at(0, &row) && row.previous == 30 && row.changes == 0);
    debug_watch_freeze_previous(false);
    spec.enabled = false;
    BOARD_CHECK(debug_watch_update(id, &spec, error, sizeof(error)));
    BOARD_CHECK(debug_watch_at(0, &row) && !row.valid && !strcmp(row.error, "Disabled"));
    spec.enabled = true;
    BOARD_CHECK(debug_watch_update(id, &spec, error, sizeof(error)));
    spec.format = DEBUG_WATCH_SIGNED;
    BOARD_CHECK(debug_watch_update(id, &spec, error, sizeof(error)));
    strcpy(spec.expression, "PC");
    cpu.pc = 0x10;
    BOARD_CHECK(debug_watch_update(id, &spec, error, sizeof(error)));
    cpu.pc = 0x4000;
    debug_watch_refresh();
    BOARD_CHECK(debug_watch_at(0, &row) && !row.valid && row.error[0] && row.id == id);
    BOARD_CHECK(debugger_watch_count() == 1);
    cpu.pc = 0x10;
    debug_watch_refresh();
    BOARD_CHECK(debug_watch_at(0, &row) && row.valid);
    DebugWatchSample before = row;
    const char *const invalid[] = {"$4000", "$5000", "$10000", "1/0", "$2000"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        strcpy(spec.expression, invalid[i]);
        BOARD_CHECK(!debug_watch_add(&spec, 1, error, sizeof(error)) && error[0]);
        BOARD_CHECK(!debug_watch_update(id, &spec, error, sizeof(error)) && error[0]);
        BOARD_CHECK(debugger_watch_count() == 1 && debug_watch_at(0, &row));
        BOARD_CHECK(!memcmp(&row, &before, sizeof(row)));
    }
    spec = spec_at("$FFFF");
    spec.width = 2;
    BOARD_CHECK(!debug_watch_add(&spec, 1, NULL, 0));
    spec = spec_at("$7FD");
    spec.space = DEBUG_MEMORY_RAM;
    spec.width = 2;
    BOARD_CHECK(!debug_watch_add(&spec, 2, NULL, 0) && debugger_watch_count() == 1);
    spec = spec_at("$20");
    strcpy(spec.label, "array");
    uint32_t array = debug_watch_add(&spec, 63, NULL, 0);
    BOARD_CHECK(array && debugger_watch_count() == 64);
    BOARD_CHECK(debug_watch_at(63, &row) && row.address == 0x5E && !strcmp(row.spec.label, "array[62]"));
    BOARD_CHECK(!debug_watch_add(&spec, 1, NULL, 0) && debugger_watch_count() == 64);
    BOARD_CHECK(!debug_watch_add(&spec, 0, NULL, 0));
    BOARD_CHECK(!debug_watch_add(&spec, SIZE_MAX, NULL, 0));
    debugger_reset_session();
    debug_watch_refresh();
    BOARD_CHECK(debugger_watch_count() == 64 && debug_watch_at(0, &row) && row.changes == 0 && row.id == id);
    BOARD_CHECK(row.current == row.previous);
    debugger_shutdown();
    BOARD_CHECK(!debugger_watch_count());
    return 0;
}

static int formats_sort_and_export(void) {
    BOARD_CHECK(machine(0) == 0);
    char text[DEBUG_WATCH_EXPORT_LIMIT];
    const struct { uint32_t value; uint8_t width; DebugWatchFormat format; const char *text; } values[] = {
        {0x89ABCDEF, 4, DEBUG_WATCH_HEX, "89ABCDEF"}, {0x89ABCDEF, 3, DEBUG_WATCH_HEX, "ABCDEF"},
        {0x80, 1, DEBUG_WATCH_SIGNED, "-128"}, {0x8000, 2, DEBUG_WATCH_SIGNED, "-32768"},
        {0x800000, 3, DEBUG_WATCH_SIGNED, "-8388608"}, {0x80000000, 4, DEBUG_WATCH_SIGNED, "-2147483648"},
        {UINT32_MAX, 4, DEBUG_WATCH_UNSIGNED, "4294967295"}, {0x81, 1, DEBUG_WATCH_BINARY, "10000001"},
        {0x7F, 1, DEBUG_WATCH_ASCII, "."}, {0x00424120, 4, DEBUG_WATCH_ASCII, " AB."}
    };
    for (size_t i = 0; i < sizeof(values) / sizeof(*values); ++i) {
        BOARD_CHECK(debug_watch_format(values[i].value, values[i].width, values[i].format, text, sizeof(text)));
        BOARD_CHECK(!strcmp(text, values[i].text));
    }
    strcpy(text, "untouched");
    BOARD_CHECK(!debug_watch_format(0xFF, 1, DEBUG_WATCH_HEX, text, 2) && !strcmp(text, "untouched"));
    BOARD_CHECK(!debug_watch_format(0, 0, DEBUG_WATCH_HEX, text, sizeof(text)));
    BOARD_CHECK(!debug_watch_format(0, 5, DEBUG_WATCH_HEX, text, sizeof(text)));
    BOARD_CHECK(!debug_watch_format(0, 1, DEBUG_WATCH_FORMAT_COUNT, text, sizeof(text)));
    const uint8_t bytes[] = {0xFF, 0x80, 0x00, 0x7F};
    for (unsigned i = 0; i < 4; ++i) write_mem((uint16_t)(0x10 + i), bytes[i]);
    DebugWatchSpec spec = spec_at("$10");
    spec.format = DEBUG_WATCH_SIGNED;
    strcpy(spec.label, "quoted,\"value\"\n");
    uint32_t first = debug_watch_add(&spec, 4, NULL, 0);
    BOARD_CHECK(first && debug_watch_sort(DEBUG_WATCH_SORT_CURRENT, false));
    const uint32_t ascending[] = {0x11, 0x10, 0x12, 0x13};
    DebugWatchSample row;
    for (size_t i = 0; i < 4; ++i) {
        BOARD_CHECK(debug_watch_at(i, &row) && row.address == ascending[i]);
    }
    BOARD_CHECK(debug_watch_move(first, 3));
    BOARD_CHECK(debug_watch_at(3, &row) && row.id == first);
    BOARD_CHECK(!debug_watch_move(first, 4) && !debug_watch_move(0, 0));
    BOARD_CHECK(!debug_watch_sort(DEBUG_WATCH_SORT_COUNT, false));
    BOARD_CHECK(debug_watch_export(text, sizeof(text)));
    BOARD_CHECK(strstr(text, "id,space,address,expression,width,format,enabled,current,previous,changes,status,label\n"));
    BOARD_CHECK(strstr(text, "\"quoted,\"\"value\"\"\n[0]\""));
    BOARD_CHECK(strstr(text, ",signed,1,\"-128\",\"-128\",0,"));
    strcpy(text, "untouched");
    BOARD_CHECK(!debug_watch_export(text, 3) && !strcmp(text, "untouched"));
    BOARD_CHECK(debugger_remove_watch(first));
    BOARD_CHECK(debugger_watch_count() == 3 && !debugger_remove_watch(first));
    return 0;
}

static int observational(void) {
    const unsigned mappers[] = {0, 4, 5, 9};
    const uint16_t addresses[] = {0x10, 0x2002, 0x2004, 0x2007, 0x4015, 0x4016, 0x4017, 0x8000};
    for (size_t board = 0; board < sizeof(mappers) / sizeof(*mappers); ++board) {
        BOARD_CHECK(machine(mappers[board]) == 0);
        BOARD_CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
        BOARD_CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
        BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
        pad1.strobe = 0;
        pad1.shift = 0x5A;
        ppu.status = 0xE0;
        ppu.w = 1;
        ppu.open_bus = 0x1F;
        apu.frame_irq = apu.frame_irq_source = true;
        apu.frame_irq_clear_delay = 0;
        NesStateBlob before = {0}, after = {0};
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        for (size_t i = 0; i < sizeof(addresses) / sizeof(*addresses); ++i) {
            DebugWatchSpec spec = spec_at("0");
            snprintf(spec.expression, sizeof(spec.expression), "$%04X", addresses[i]);
            BOARD_CHECK(debug_watch_add(&spec, 1, NULL, 0));
        }
        for (unsigned space = DEBUG_MEMORY_PPU; space < DEBUG_MEMORY_SPACE_COUNT; ++space) {
            DebugWatchSpec spec = spec_at(space == DEBUG_MEMORY_PALETTE ? "$3F00" : "0");
            spec.space = space;
            BOARD_CHECK(debug_watch_add(&spec, 8, NULL, 0));
        }
        for (unsigned repeat = 0; repeat < 8; ++repeat) {
            debug_watch_refresh();
            debug_watch_capture_previous();
            BOARD_CHECK(debug_watch_sort(DEBUG_WATCH_SORT_CURRENT, (repeat & 1) != 0));
            char csv[DEBUG_WATCH_EXPORT_LIMIT];
            BOARD_CHECK(debug_watch_export(csv, sizeof(csv)));
        }
        BOARD_CHECK(nes_state_capture(&after) == NES_STATE_OK);
        BOARD_CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
        BOARD_CHECK(ppu.status == 0xE0 && ppu.w == 1 && ppu.open_bus == 0x1F && pad1.shift == 0x5A);
        BOARD_CHECK(apu.frame_irq && apu.frame_irq_source && apu.frame_irq_clear_delay == 0);
        nes_state_blob_free(&before);
        nes_state_blob_free(&after);
    }
    BOARD_CHECK(machine(0) == 0);
    write_mem(0x10, 0x42);
    BOARD_CHECK(cheats_add("0010:FF", "Read override", true, NULL) == CHEAT_OK);
    DebugWatchSpec spec = spec_at("$10");
    BOARD_CHECK(debug_watch_add(&spec, 1, NULL, 0));
    DebugWatchSample row;
    BOARD_CHECK(debug_watch_at(0, &row) && row.current == 0x42);
    BOARD_CHECK(cheats_clear() == CHEAT_OK);
    return 0;
}

int test_memory_watch_accuracy(void) {
    int failures = expressions() + history_and_bounds() + formats_sort_and_export() + observational();
    debugger_shutdown();
    (void)unload_rom();
    printf("Memory watches: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
