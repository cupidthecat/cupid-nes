/*
 * cpu_accuracy_reset_nmi.h - NMI edges during the CPU reset sequence
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 */

static int reset_nmi_window_case(NesRegion region, unsigned dots_before_line,
                                 bool suppress_ppu_reset) {
    reset_fixture_region(region);
    ppu_set_reset_suppression(suppress_ppu_reset);
    program(0xEA, 0xEA, 0xEA);
    fixture_memory[0x9000] = 0xE6; /* INC $10 */
    fixture_memory[0x9001] = 0x10;
    fixture_memory[0x9002] = 0x40; /* RTI */
    ppu_reg_write(PPUCTRL, 0x80);

    const NesTiming *timing = nes_timing();
    int target_line = (int)timing->vblank_scanline - 1;
    int target_dot = 340 - (int)dots_before_line;
    unsigned limit = timing->scanlines * 341u;
    for (unsigned dot = 0; dot < limit; ++dot) {
        if (ppu.scanline == target_line && ppu.dot == target_dot) break;
        ppu_step_dots(1);
    }
    CHECK(ppu.scanline == target_line && ppu.dot == target_dot);
    CHECK(!ppu.nmi_out && !(ppu.status & 0x80));

    uint64_t reset_start = cpu_total_cycles;
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    CHECK(cpu_total_cycles - reset_start == 7 && cpu.pc == 0x8000);
    CHECK(cpu.status & INTERRUPT_FLAG);
    if (suppress_ppu_reset) {
        CHECK(ppu.nmi_out && (ppu.status & 0x80));
        CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x9000);
        CHECK(cpu_step(&cpu) == 5 && ram[0x10] == 1);
        CHECK(cpu_step(&cpu) == 6 && cpu.pc == 0x8001);
        CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8002);
        CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8003);
        CHECK(ram[0x10] == 1); /* The held NMI line is not another edge. */
    } else {
        CHECK(!ppu.nmi_out);
        CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
        CHECK(ram[0x10] == 0);
    }
    return 0;
}

static int reset_nmi_window(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    static const unsigned offsets[] = {0, 3, 9, 15, 18};
    bool saved_suppression = ppu_reset_suppression_enabled();
    bool saved_restriction = ppu_startup_write_restriction_enabled();
    NesRegion saved_region = nes_timing()->region;
    cpu_use_default_startup_alignment();
    ppu_set_startup_write_restriction(false);
    int failures = 0;
    for (size_t region = 0; region < sizeof(regions) / sizeof(regions[0]); ++region) {
        for (size_t offset = 0; offset < sizeof(offsets) / sizeof(offsets[0]); ++offset) {
            failures += reset_nmi_window_case(regions[region], offsets[offset], true);
            failures += reset_nmi_window_case(regions[region], offsets[offset], false);
        }
    }
    ppu_set_reset_suppression(saved_suppression);
    ppu_set_startup_write_restriction(saved_restriction);
    nes_set_region(saved_region);
    return failures ? 1 : 0;
}
