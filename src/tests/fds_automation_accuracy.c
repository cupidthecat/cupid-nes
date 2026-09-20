/*
 * fds_automation_accuracy.c - BIOS side selection and loading controls
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../rom/fds.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../ui/execution_control.h"

enum { AUTO_SIDE_BYTES = 65500, AUTO_SIDE_COUNT = 3 };
static uint8_t auto_disk[16 + AUTO_SIDE_COUNT * AUTO_SIDE_BYTES];
static uint8_t auto_bios[0x2000];

static void make_automatic_disk(void) {
    memset(auto_disk, 0, sizeof(auto_disk));
    memcpy(auto_disk, "FDS\x1A", 4);
    auto_disk[4] = AUTO_SIDE_COUNT;
    for (unsigned side = 0; side < AUTO_SIDE_COUNT; side++) {
        uint8_t *raw = auto_disk + 16 + side * AUTO_SIDE_BYTES;
        raw[0] = 1;
        raw[1] = (uint8_t)(0x50 + side);
        for (unsigned i = 0; i < 10; i++) raw[14 + i] = (uint8_t)(0x20 + side + i);
        raw[56] = 2;
    }

    memset(auto_bios, 0xEA, sizeof(auto_bios));
    auto_bios[0] = 0x4C;
    auto_bios[1] = 0;
    auto_bios[2] = 0xE0;
    for (unsigned vector = 0x1FFA; vector <= 0x1FFE; vector += 2) {
        auto_bios[vector] = 0;
        auto_bios[vector + 1] = 0xE0;
    }
}

static int start_automatic_disk(bool automatic, bool fast_forward) {
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    fds_set_automation_options((FdsAutomationOptions){false, false});
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    cpu_select_machine(NULL);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(load_fds_memory(auto_disk, sizeof(auto_disk), auto_bios, sizeof(auto_bios), NULL, false) == 0);
    BOARD_CHECK(cpu_power_on(&cpu));
    fds_set_automation_options((FdsAutomationOptions){automatic, fast_forward});
    return 0;
}

static void request_side(unsigned side, bool wildcard) {
    write_mem(0, 0x00);
    write_mem(1, 0x06);
    for (unsigned i = 0; i < 10; i++) {
        uint8_t value = auto_disk[16 + side * AUTO_SIDE_BYTES + 14 + i];
        write_mem((uint16_t)(0x0600 + i), wildcard && i != 0 ? 0xFF : value);
    }
}

static void tick_frame(void) {
    ppu.frame_count++;
    fds_clock_cpu(1);
}

static int test_bios_selection(void) {
    BOARD_CHECK(start_automatic_disk(true, true) == 0);
    BOARD_CHECK(fds_automatic_insert_active() && fds_current_side() == 0);
    BOARD_CHECK(fds_loading_fast_forward());
    request_side(2, false);
    uint64_t cycles = cpu_total_cycles, frame = ppu.frame_count;
    BOARD_CHECK(cart_cpu_read(0xE445) == auto_bios[0x445]);
    BOARD_CHECK(fds_current_side() == 2 && !fds_loading_fast_forward());
    BOARD_CHECK(cpu_total_cycles == cycles && ppu.frame_count == frame);

    request_side(1, true);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_current_side() == 1 && fds_automatic_insert_active());
    for (unsigned i = 0; i < 10; i++) write_mem((uint16_t)(0x0600 + i), 0x91);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_current_side() == 1 && !fds_automatic_insert_ambiguous());

    // A bad requested-header pointer must not acknowledge a pending timer.
    cart_cpu_write(0x4020, 1);
    cart_cpu_write(0x4021, 0);
    cart_cpu_write(0x4022, 2);
    fds_clock_cpu(2);
    BOARD_CHECK(fds_irq_pending());
    write_mem(0, 0x30);
    write_mem(1, 0x40);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_irq_pending() && fds_current_side() == 1);
    (void)cart_cpu_read(0x4030);

    // If several headers match, stop automatic selection for this session.
    write_mem(0, 0x00);
    write_mem(1, 0x06);
    for (unsigned i = 0; i < 10; i++) write_mem((uint16_t)(0x0600 + i), 0xFF);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_automatic_insert_ambiguous() && !fds_automatic_insert_active());
    BOARD_CHECK(fds_current_side() == 1 && fds_insert_disk(2));
    request_side(0, false);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_current_side() == 2);
    BOARD_CHECK(start_automatic_disk(true, false) == 0);
    BOARD_CHECK(fds_automatic_insert_active() && !fds_automatic_insert_ambiguous());
    BOARD_CHECK(unload_rom());
    return 0;
}

static int test_status_poll_delay(void) {
    BOARD_CHECK(start_automatic_disk(true, false) == 0);
    uint8_t last_status = 0;
    for (unsigned i = 0; i < 21; i++) last_status = cart_cpu_read_bus(0x4032, 0xA8);
    BOARD_CHECK(!(last_status & 1) && !fds_disk_inserted());
    BOARD_CHECK((cart_cpu_read_bus(0x4032, 0xA8) & 0xAF) == 0xAF);
    for (unsigned i = 0; i < 76; i++) {
        tick_frame();
        fds_automation_frame(ppu.frame_count); // Same frame does not count twice.
        BOARD_CHECK(!fds_disk_inserted());
    }

    tick_frame();
    BOARD_CHECK(fds_disk_inserted() && fds_current_side() == 0);
    request_side(2, false);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_current_side() == 2);
    BOARD_CHECK(unload_rom());
    return 0;
}

static int test_policy_gates(void) {
    const uint32_t policies[] = {NES_EXECUTION_MOVIE_RECORDING, NES_EXECUTION_MOVIE_PLAYBACK,
                                 NES_EXECUTION_NETPLAY, NES_EXECUTION_SPECULATIVE, NES_EXECUTION_REWIND};
    BOARD_CHECK(start_automatic_disk(true, true) == 0);
    for (size_t i = 0; i < sizeof(policies) / sizeof(policies[0]); i++) {
        BOARD_CHECK(fds_insert_disk(0));
        BOARD_CHECK(nes_execution_set_policy(policies[i]));
        BOARD_CHECK(!fds_automatic_insert_active() && !fds_loading_fast_forward());
        request_side(2, false);
        (void)cart_cpu_read(0xE445);
        for (unsigned check = 0; check < 50; check++) (void)cart_cpu_read(0x4032);
        for (unsigned frame = 0; frame < 80; frame++) tick_frame();
        BOARD_CHECK(fds_disk_inserted() && fds_current_side() == 0);
        BOARD_CHECK(!nes_execution_set_policy(UINT32_C(0x80000000)));
        BOARD_CHECK(nes_execution_policy() == policies[i]);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(fds_automatic_insert_active() && fds_loading_fast_forward());
    }

    request_side(2, false);
    (void)cart_cpu_read(0xE445);
    BOARD_CHECK(fds_current_side() == 2);
    BOARD_CHECK(unload_rom());
    return 0;
}

static int test_loading_speed(void) {
    BOARD_CHECK(start_automatic_disk(false, true) == 0);
    BOARD_CHECK(fds_loading_fast_forward());
    ExecutionControl control;
    execution_control_init(&control);
    BOARD_CHECK(execution_control_set_speed(&control, 0.5));
    execution_control_set_loading_fast_forward(&control, fds_loading_fast_forward());
    BOARD_CHECK(execution_control_effective_speed(&control) == 4.0);
    write_mem(0x0100, 0);
    (void)cart_cpu_read(0xE18C);
    BOARD_CHECK(fds_loading_fast_forward());
    write_mem(0x0100, 0x40);
    (void)cart_cpu_read(0xE18C);
    BOARD_CHECK(!fds_loading_fast_forward());
    execution_control_set_loading_fast_forward(&control, false);
    BOARD_CHECK(execution_control_effective_speed(&control) == 0.5);

    cart_cpu_write(0x4025, 0xC5);
    fds_clock_cpu(600000);
    BOARD_CHECK(fds_loading_fast_forward());
    execution_control_set_loading_fast_forward(&control, true);
    execution_control_set_fast_forward_held(&control, true);
    cart_cpu_write(0x4025, 0xC7); // Bit 1 high stops the motor; bit 0 stays unchanged.
    fds_clock_cpu(1);
    BOARD_CHECK(!fds_loading_fast_forward());
    execution_control_set_loading_fast_forward(&control, false);
    BOARD_CHECK(execution_control_fast_forward_active(&control));
    execution_control_set_fast_forward_held(&control, false);
    BOARD_CHECK(!execution_control_fast_forward_active(&control));
    BOARD_CHECK(execution_control_effective_speed(&control) == 0.5);
    BOARD_CHECK(unload_rom());
    return 0;
}

typedef struct {
    uint64_t cycles;
    uint32_t scanline;
    uint32_t dot;
    uint64_t frames;
    uint8_t first_byte;
    uint8_t second_byte;
    uint32_t first_transfer;
    uint32_t second_transfer;
} DiskTimingTrace;

static int record_disk_trace(bool automatic, DiskTimingTrace *trace) {
    BOARD_CHECK(start_automatic_disk(automatic, automatic) == 0);
    request_side(1, false);
    if (automatic) (void)cart_cpu_read(0xE445);
    else BOARD_CHECK(fds_insert_disk(1));
    cart_cpu_write(0x4025, 0xC5);
    unsigned transfers = 0;
    memset(trace, 0, sizeof(*trace));
    for (uint32_t step = 1; step < 220000 && transfers < 2; step++) {
        BOARD_CHECK(cpu_step(&cpu) > 0);
        if (fds_irq_pending()) {
            uint8_t byte = cart_cpu_read(0x4031);
            if (!transfers) {
                trace->first_byte = byte;
                trace->first_transfer = (uint32_t)cpu_total_cycles;
            } else {
                trace->second_byte = byte;
                trace->second_transfer = (uint32_t)cpu_total_cycles;
            }

            transfers++;
        }
    }

    BOARD_CHECK(transfers == 2 && trace->first_byte == 1 && trace->second_byte == 0x51);
    trace->cycles = cpu_total_cycles;
    trace->scanline = (uint32_t)ppu.scanline;
    trace->dot = (uint32_t)ppu.dot;
    trace->frames = ppu.frame_count;
    BOARD_CHECK(unload_rom());
    return 0;
}

static int test_automatic_manual_timing(void) {
    DiskTimingTrace manual, automatic;
    BOARD_CHECK(record_disk_trace(false, &manual) == 0);
    BOARD_CHECK(record_disk_trace(true, &automatic) == 0);
    BOARD_CHECK(!memcmp(&manual, &automatic, sizeof(manual)));
    BOARD_CHECK(manual.second_transfer - manual.first_transfer == 150);
    return 0;
}

int test_fds_automation_accuracy(void) {
    FdsAutomationOptions previous_options = fds_automation_options();
    uint32_t previous_policy = nes_execution_policy();
    NesRegionMode previous_region = nes_region_mode();
    make_automatic_disk();
    int failures = test_bios_selection() + test_status_poll_delay() + test_policy_gates()
                 + test_loading_speed() + test_automatic_manual_timing();
    fds_set_automation_options(previous_options);
    if (!nes_execution_set_policy(previous_policy)) ++failures;
    if (!nes_set_region_mode(previous_region)) ++failures;
    printf("FDS automatic controls: 5 groups, %d failures\n", failures);
    return failures;
}
