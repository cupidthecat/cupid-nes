/*
 * board_rainbow_accuracy.c - Rainbow cartridge bus, video, flash, and audio tests
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../system/hardware.h"
#include <math.h>
#include <time.h>

static bool rainbow_image(BoardImage *image, size_t prg, size_t chr) {
    if (!board_image_create(image, 682, prg, chr, true)) return false;
    for (size_t offset = 0; offset < chr; ++offset)
        image->data[16 + prg + offset] = (offset & 0x1FF) == 1 ? (uint8_t)(offset >> 17)
                                                             : (uint8_t)(offset >> 9);
    return true;
}

static int rainbow_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int rainbow_read(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 4 && cpu.a == expected);
    return 0;
}

static int rainbow_nops(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned step = 0; step < count; ++step) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static void rainbow_prg(unsigned slot, uint16_t bank) {
    write_mem((uint16_t)(0x4108 + slot), (uint8_t)(bank >> 8));
    write_mem((uint16_t)(0x4118 + slot), (uint8_t)bank);
}

static void rainbow_low(unsigned slot, uint16_t bank) {
    write_mem((uint16_t)(0x4106 + slot), (uint8_t)(bank >> 8));
    write_mem((uint16_t)(0x4116 + slot), (uint8_t)bank);
}

static void rainbow_chr(unsigned slot, uint16_t bank) {
    write_mem((uint16_t)(0x4130 + slot), (uint8_t)(bank >> 8));
    write_mem((uint16_t)(0x4140 + slot), (uint8_t)bank);
}

static uint8_t rainbow_begin_frame(void) {
    (void)ppu_read(0); // Start the idle detector before allowing three CPU clocks to expire.
    for (unsigned cycle = 0; cycle < 3; ++cycle) cart_clock_cpu_cycle(false);
    (void)ppu_read(0);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    return ppu_read(0x2000);
}

static void rainbow_sprite_phase(void) {
    for (unsigned tile = 1; tile < 33; ++tile) {
        (void)ppu_read(0x23C0);
        (void)ppu_read(0x2000);
    }
}

static int test_rainbow_prg_modes(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x80000, 0x40000));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7 && read_mem(0x6000) == 0);
    BOARD_CHECK(read_mem(0x4160) == 0x21 && read_mem(0x4100) == 0);
    const uint8_t selected[] = {1, 7, 11, 15, 19, 23, 27, 31};
    const uint8_t pages[5][8] = {
        {8, 9, 10, 11, 12, 13, 14, 15}, {4, 5, 6, 7, 76, 77, 78, 79},
        {4, 5, 6, 7, 38, 39, 54, 55}, {2, 3, 22, 23, 38, 39, 54, 55},
        {1, 7, 11, 15, 19, 23, 27, 31}
    };
    for (unsigned bank = 0; bank < 8; ++bank) rainbow_prg(bank, selected[bank]);
    for (unsigned mode = 0; mode < 8; ++mode) {
        BOARD_CHECK(rainbow_store(0x4100, (uint8_t)mode) == 0);
        BOARD_CHECK(read_mem(0x4100) == mode);
        for (unsigned slot = 0; slot < 8; ++slot) {
            uint16_t address = (uint16_t)(0x8000 + slot * 0x1000);
            BOARD_CHECK(read_mem(address) == pages[mode > 4 ? 4 : mode][slot]);
            BOARD_CHECK(read_mem((uint16_t)(address + 0xFFF)) == pages[mode > 4 ? 4 : mode][slot]);
        }
    }
    write_mem(0x8100, 0xAB);
    BOARD_CHECK(read_mem(0x8100) == 1);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x4101) == 0xA5 && read_mem(0x41FE) == 0xA5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7 && read_mem(0x4120) == 0);
    write_mem(0x4100, 1);
    BOARD_CHECK(read_mem(0xC000) == 76); // Reset only writes high-bank register zero.
    board_image_free(&image);
    return 0;
}

static int test_rainbow_ram_sources(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x80000, 0x2000));
    image.data[10] = 9;
    BOARD_CHECK(board_image_add_trainer(&image, 0x65));
    BOARD_CHECK(board_image_load(&image) == 0);
    rainbow_low(0, 0x8000);
    BOARD_CHECK(read_mem(0x7000) == 0x65);
    write_mem(0x4100, 0x84);
    rainbow_low(1, 0x8001);
    BOARD_CHECK(rainbow_store(0x6123, 0xA1) == 0 && rainbow_store(0x7123, 0xB2) == 0);
    rainbow_prg(0, 0x8001);
    BOARD_CHECK(read_mem(0x8123) == 0xB2);
    write_mem(0x8123, 0xC3);
    BOARD_CHECK(read_mem(0x7123) == 0xC3);
    rainbow_low(0, 0x8001);
    BOARD_CHECK(read_mem(0x6123) == 0xC3);
    rainbow_low(0, 0xC000);
    rainbow_low(1, 0xC001);
    write_mem(0x6123, 0xD4);
    write_mem(0x7123, 0xE5);
    BOARD_CHECK(read_mem(0x5123) == 0xD4);
    write_mem(0x4115, 1);
    BOARD_CHECK(read_mem(0x5123) == 0xE5);
    write_mem(0x4823, 0xF6);
    BOARD_CHECK(read_mem(0x7823) == 0xF6 && read_mem(0x5823) == 0xF6);
    write_mem(0x415C, 0x1F);
    write_mem(0x415D, 0xFF);
    write_mem(0x415E, 2);
    write_mem(0x415F, 0x87);
    write_mem(0x415F, 0x98);
    BOARD_CHECK(read_mem(0x4FFF) == 0x87 && read_mem(0x6001) == 0x98);
    write_mem(0x415C, 0x1F);
    write_mem(0x415D, 0xFF);
    BOARD_CHECK(read_mem(0x415F) == 0x87 && read_mem(0x415F) == 0x98);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x5123) == 0xE5 && read_mem(0x6123) == 0xD4 && read_mem(0x7823) == 0xF6);
    rainbow_low(0, 0x8000);
    BOARD_CHECK(read_mem(0x6123) == 0xA1 && read_mem(0x7123) == 0xC3);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_chr_modes(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x80000));
    image.data[11] = 9;
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned slot = 0; slot < 16; ++slot) rainbow_chr(slot, (uint16_t)(slot + 1));
    const uint8_t pages[5][4] = {{16, 20, 24, 28}, {8, 12, 16, 20}, {4, 8, 12, 16},
                                {2, 6, 10, 14}, {1, 5, 9, 13}};
    for (unsigned mode = 0; mode < 8; ++mode) {
        BOARD_CHECK(rainbow_store(0x4120, (uint8_t)mode) == 0);
        BOARD_CHECK(read_mem(0x4120) == mode);
        for (unsigned chunk = 0; chunk < 4; ++chunk)
            BOARD_CHECK(ppu_read((uint16_t)(chunk * 0x800)) == pages[mode > 4 ? 4 : mode][chunk]);
    }
    write_mem(0x4120, 4);
    rainbow_chr(15, 0x1FFF);
    BOARD_CHECK(ppu_read(0x1E00) == 0xFF && ppu_read(0x1E01) == 3);
    ppu_write(0x1E00, 0x76);
    BOARD_CHECK(ppu_read(0x1E00) == 0xFF);
    write_mem(0x4120, 0x44);
    rainbow_chr(0, 3);
    ppu_write(0x0023, 0xA5);
    rainbow_chr(0, 67);
    BOARD_CHECK(ppu_read(0x0023) == 0xA5);
    write_mem(0x4120, 0x80);
    ppu_write(0x1023, 0xB6);
    BOARD_CHECK(ppu_read(0x0023) == 0xB6 && read_mem(0x5023) == 0xB6);
    write_mem(0x4120, 0xC0);
    ppu_write(0x0423, 0xC7);
    BOARD_CHECK(ppu_read(0x0C23) == 0xC7 && ppu_read(0x1C23) == 0xC7 && ppu_read(0x2823) == 0xC7);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_nametables(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x80000));
    image.data[11] = 9;
    BOARD_CHECK(board_image_load(&image) == 0);
    const uint8_t banks[] = {1, 3, 1, 9};
    for (unsigned quadrant = 0; quadrant < 4; ++quadrant) {
        write_mem((uint16_t)(0x4126 + quadrant), banks[quadrant]);
        write_mem((uint16_t)(0x412A + quadrant), (uint8_t)(quadrant << 6));
    }
    ppu_write(0x2023, 0xA1);
    ppu_write(0x2423, 0xB2);
    ppu_write(0x2823, 0xC3);
    ppu_write(0x2C23, 0xD4);
    BOARD_CHECK(ppu_read(0x2023) == 0xA1 && ppu_read(0x2423) == 0xB2);
    BOARD_CHECK(ppu_read(0x2823) == 0xC3 && ppu_read(0x2C23) == 18);
    BOARD_CHECK(read_mem(0x5423) == 0xC3 && ppu_read(0x3023) == 0xA1);
    write_mem(0x4120, 0xC0);
    BOARD_CHECK(ppu_read(0x0423) == 0xA1);
    write_mem(0x4120, 0x44);
    rainbow_chr(0, 6);
    BOARD_CHECK(ppu_read(0x0023) == 0xB2);
    write_mem(0x4124, 0x37);
    write_mem(0x4125, 2);
    write_mem(0x412A, 0x20);
    BOARD_CHECK(rainbow_nops(2) == 0 && ppu_read(0x2023) == 0xA1);
    BOARD_CHECK(rainbow_begin_frame() == 0x37);
    BOARD_CHECK(ppu_read(0x2023) == 0x37 && ppu_read(0x23C0) == 0xAA);
    ppu_write(0x2023, 0xE5);
    write_mem(0x412A, 0);
    BOARD_CHECK(ppu_read(0x2023) == 0xE5); // Fill affects fetches, not writes to the mapped chip.
    board_image_free(&image);
    return 0;
}

static int test_rainbow_cpu_irq(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x2000));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4158, 0);
    write_mem(0x4159, 4);
    write_mem(0x415A, 1);
    BOARD_CHECK(rainbow_nops(1) == 0 && !cart_irq_pending());
    BOARD_CHECK(rainbow_nops(1) == 0 && cart_irq_pending());
    BOARD_CHECK(read_mem(0x4161) == 0x40 && read_mem(0x4154) == 0);
    BOARD_CHECK(rainbow_nops(128) == 0 && read_mem(0x4154) == 0);
    write_mem(0x415B, 0);
    BOARD_CHECK(!cart_irq_pending() && rainbow_nops(8) == 0 && !cart_irq_pending());
    write_mem(0x415A, 3);
    BOARD_CHECK(rainbow_nops(2) == 0 && cart_irq_pending());
    write_mem(0x415B, 0);
    BOARD_CHECK(!cart_irq_pending());
    BOARD_CHECK(rainbow_nops(1) == 0 && !cart_irq_pending());
    BOARD_CHECK(rainbow_nops(1) == 0 && cart_irq_pending());
    write_mem(0x415A, 5);
    BOARD_CHECK(rainbow_nops(2) == 0 && cart_irq_pending());
    BOARD_CHECK(rainbow_read(0x4011, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(rainbow_nops(8) == 0 && read_mem(0x4161) == 0);
    write_mem(0x4159, 0);
    write_mem(0x415A, 3);
    BOARD_CHECK(rainbow_nops(64) == 0 && !cart_irq_pending());
    write_mem(0x415A, 0);

    write_mem(0x4157, 0);
    BOARD_CHECK(read_mem(0x4157) == 0x80);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4157) == 0x80);
    write_mem(0x0200, 0xA5);
    write_mem(0x0201, 0);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 3 && read_mem(0x4157) == 0);

    write_mem(0x416C, 0xAB);
    write_mem(0x416D, 0xCD);
    write_mem(0x416E, 0x12);
    write_mem(0x416F, 0x34);
    write_mem(0x416B, 3);
    BOARD_CHECK(read_mem(0xFFFA) == 0xCD && read_mem(0xFFFB) == 0xAB);
    BOARD_CHECK(read_mem(0xFFFE) == 0x34 && read_mem(0xFFFF) == 0x12);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xFFFA) == 7 && read_mem(0xFFFE) == 7);
    write_mem(0x416B, 3);
    BOARD_CHECK(read_mem(0xFFFA) == 0xCD && read_mem(0xFFFE) == 0x34);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_scanline_irq(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x2000));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4150, 0);
    write_mem(0x4153, 2);
    write_mem(0x4151, 0);
    (void)rainbow_begin_frame();
    BOARD_CHECK(read_mem(0x4150) == 0 && !cart_irq_pending());
    (void)ppu_read(0x23C0);
    BOARD_CHECK(!cart_irq_pending());
    (void)ppu_read(0);
    BOARD_CHECK(cart_irq_pending() && read_mem(0x4161) == 0x80);
    write_mem(0x4150, 1);
    BOARD_CHECK(cart_irq_pending()); // Changing the target does not acknowledge the IRQ.
    BOARD_CHECK(read_mem(0x4151) == 0x40 && !cart_irq_pending());
    write_mem(0x4153, 0); // Zero selects the first PPU read, not a CPU-cycle delay.
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    BOARD_CHECK(read_mem(0x4150) == 1 && !cart_irq_pending());
    (void)ppu_read(0);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0x4158, 0);
    write_mem(0x4159, 2);
    write_mem(0x415A, 1);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4161) == 0xC0);
    BOARD_CHECK(read_mem(0x4151) == 0x40 && cart_irq_pending() && read_mem(0x4161) == 0x40);
    write_mem(0x415B, 0);
    write_mem(0x4152, 0);
    BOARD_CHECK(!cart_irq_pending());
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4151) == 0 && read_mem(0x4150) == 0xFF);

    (void)rainbow_begin_frame();
    for (unsigned tile = 1; tile < 32; ++tile) {
        (void)ppu_read(0x23C0);
        (void)ppu_read(0x2000);
    }
    BOARD_CHECK(read_mem(0x4151) == 0x40);
    (void)ppu_read(0x23C0);
    (void)ppu_read(0x2000);
    BOARD_CHECK(read_mem(0x4151) == 0xC0);
    (void)read_mem(0xFFFA);
    BOARD_CHECK((read_mem(0x4151) & 0x40) == 0 && read_mem(0x4150) == 0);
    (void)rainbow_begin_frame();
    BOARD_CHECK(rainbow_nops(1) == 0 && (read_mem(0x4151) & 0x40));
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4151) == 0 && read_mem(0x4150) == 0xFF);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_audio(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x2000));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x41A0, 0x8F);
    write_mem(0x41A1, 0);
    write_mem(0x41A2, 0x80);
    write_mem(0x41A3, 0x87);
    write_mem(0x41A4, 0);
    write_mem(0x41A5, 0x80);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 44);
    BOARD_CHECK(cart_expansion_audio() == 0.0f);
    write_mem(0x41A9, 1);
    BOARD_CHECK(fabsf(cart_expansion_audio() + 0.33f) < 0.00001f);
    write_mem(0x41AA, 3);
    BOARD_CHECK(fabsf(cart_expansion_audio() + 0.066f) < 0.00001f);
    write_mem(0x41A9, 2);
    BOARD_CHECK(fabsf(cart_expansion_audio() + 0.066f) < 0.00001f);
    write_mem(0x41A9, 4);
    BOARD_CHECK(cart_expansion_audio() == 0.0f);
    write_mem(0x4158, 0);
    write_mem(0x4159, 2);
    write_mem(0x415A, 5);
    write_mem(0x0200, 0xEE); // INC $4011 uses the cartridge sample, then writes the native DAC.
    write_mem(0x0201, 0x11);
    write_mem(0x0202, 0x40);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 6 && apu.dmc.output_level == 45 && !cart_irq_pending());
    BOARD_CHECK(read_mem(0x4161) == 0 && read_mem(0x4011) == 44);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_expansion_audio() == 0.0f && read_mem(0x4011) == 44);
    write_mem(0x41A9, 1);
    BOARD_CHECK(fabsf(cart_expansion_audio() + 0.33f) < 0.00001f);

    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x41A0, 0x0F);
    write_mem(0x41A1, 0);
    write_mem(0x41A2, 0x80);
    BOARD_CHECK(rainbow_nops(7) == 0 && read_mem(0x4011) == 0);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 30);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 0);
    write_mem(0x41A2, 0);
    write_mem(0x41A6, 8);
    write_mem(0x41A7, 0);
    write_mem(0x41A8, 0x80);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 2);
    BOARD_CHECK(rainbow_nops(5) == 0 && read_mem(0x4011) == 12);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 0);
    write_mem(0x41A8, 0);
    BOARD_CHECK(rainbow_nops(1) == 0 && read_mem(0x4011) == 0);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_extended_background(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x80000));
    image.data[11] = 9;
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x2023, 0x2E);
    write_mem(0x5023, 0xC3);
    write_mem(0x5823, 0x44);
    write_mem(0x412A, 3);
    (void)rainbow_begin_frame();
    BOARD_CHECK(ppu_read(0x2023) == 0x2E && ppu_read(0x23C0) == 0xFF);
    BOARD_CHECK(ppu_read(0) == 24 && ppu_read(1) == 0);
    write_mem(0x4121, 1);
    BOARD_CHECK(ppu_read(0) == 24 && ppu_read(1) == 2);
    write_mem(0x412A, 0x0B);
    BOARD_CHECK(ppu_read(0x2023) == 0x2E && ppu_read(0x23C0) == 0x55);
    BOARD_CHECK(ppu_read(0) == 32 && ppu_read(1) == 2);
    write_mem(0x412A, 9);
    (void)ppu_read(0x2023);
    BOARD_CHECK(ppu_read(0x23C0) == 0x55 && ppu_read(0) == 0);
    write_mem(0x4124, 0x37);
    write_mem(0x4125, 2);
    write_mem(0x412A, 0x2B);
    BOARD_CHECK(ppu_read(0x2023) == 0x37 && ppu_read(0x23C0) == 0xAA);
    BOARD_CHECK(ppu_read(0) == 32); // Fill and extended pattern addressing can be used together.

    write_mem(0x4121, 0);
    write_mem(0x4120, 0x44);
    rainbow_chr(0, 24);
    ppu_write(0x0123, 0xAB);
    rainbow_chr(0, 0);
    write_mem(0x412A, 3);
    (void)ppu_read(0x2023);
    BOARD_CHECK(ppu_read(0x0123) == 0xAB);
    write_mem(0x4115, 1);
    write_mem(0x5123, 0xCD);
    write_mem(0x4115, 0);
    write_mem(0x4120, 0x80);
    (void)ppu_read(0x2023);
    BOARD_CHECK(ppu_read(0x0123) == 0xCD);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_window(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x80000));
    for (unsigned fine = 0; fine < 8; ++fine) image.data[16 + 0x8000 + 0x120 + fine] = (uint8_t)(0x40 + fine);
    image.data[16 + 0x8000 + 0x3121] = 0xBA;
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x2000, 0x19);
    ppu_write(0x2023, 0x2B);
    write_mem(0x5823, 0x37);
    write_mem(0x5BC0, 0x08);
    write_mem(0x412E, 2);
    write_mem(0x412F, 0x80);
    write_mem(0x4170, 0);
    write_mem(0x4171, 31);
    write_mem(0x4172, 0);
    write_mem(0x4173, 239);
    write_mem(0x4174, 1);
    write_mem(0x4175, 9);
    write_mem(0x4120, 0x10);
    BOARD_CHECK(rainbow_begin_frame() == 0x37);
    BOARD_CHECK(ppu_read(0x23C0) == 0xAA && ppu_read(0x0120) == 0x41);
    write_mem(0x5023, 0xC3);
    write_mem(0x412F, 0x81);
    BOARD_CHECK(read_mem(0x412F) == 0x82);
    write_mem(0x412F, 0x83);
    BOARD_CHECK(rainbow_begin_frame() == 0x37);
    BOARD_CHECK(ppu_read(0x23C0) == 0xFF && ppu_read(0x0120) == 0xBA);
    write_mem(0x4124, 0x6E);
    write_mem(0x4125, 0xFD);
    write_mem(0x412F, 0xA0);
    BOARD_CHECK(rainbow_begin_frame() == 0x6E && ppu_read(0x23C0) == 0x55);

    write_mem(0x412F, 0x80);
    write_mem(0x412E, 0xFF);
    write_mem(0x4C23, 0xC7);
    BOARD_CHECK(rainbow_begin_frame() == 0xC7);
    write_mem(0x4170, 30);
    write_mem(0x4171, 2);
    write_mem(0x4172, 239);
    write_mem(0x4173, 0);
    BOARD_CHECK(rainbow_begin_frame() == 0xC7);
    BOARD_CHECK(ppu_read(0x2023) == 0x2B);
    (void)ppu_read(0);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    BOARD_CHECK(ppu_read(0x2000) == 0x19 && read_mem(0x4150) == 1);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_sprites_and_oam(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x200000));
    image.data[16 + 0x8000 + 0x3000] = 0x31;
    image.data[16 + 0x8000 + 0x3008] = 0x32;
    image.data[16 + 0x8000 + 0xA000] = 0x41;
    image.data[16 + 0x8000 + 0xA008] = 0x42;
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned byte = 0; byte < 256; ++byte)
        write_mem((uint16_t)(0x0400 + byte), (byte & 3) == 0 ? 0xFF : 0);
    write_mem(0x040C, 0);
    BOARD_CHECK(rainbow_store(0x2003, 0) == 0 && rainbow_store(0x4014, 4) == 0);
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) >= 515);
    BOARD_CHECK(ppu.oam[0] == 0xFF && ppu.oam[12] == 0);
    write_mem(0x4203, 3);
    write_mem(0x4120, 0x20);
    (void)rainbow_begin_frame();
    rainbow_sprite_phase();
    BOARD_CHECK(ppu_read(0) == 0x31 && ppu_read(8) == 0x32);
    write_mem(0x4203, 5);
    BOARD_CHECK(rainbow_store(0x2000, 0x20) == 0);
    (void)rainbow_begin_frame();
    rainbow_sprite_phase();
    BOARD_CHECK(ppu_read(0) == 0x41 && ppu_read(8) == 0x42);

    write_mem(0x4F00, 0x2A);
    write_mem(0x4F01, 0x11);
    write_mem(0x4F02, 0x22);
    write_mem(0x4F03, 0x33);
    write_mem(0x4243, 0);
    BOARD_CHECK(rainbow_store(0x2003, 0) == 0);
    cpu.pc = 0x4280;
    for (unsigned byte = 0; byte < 4; ++byte)
        BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    BOARD_CHECK(cpu.pc == 0x4294 && read_mem(0x4294) == 0x60);
    BOARD_CHECK(ppu.oam[0] == 0x2A && ppu.oam[1] == 0x11 && ppu.oam[2] == 0x22 && ppu.oam[3] == 0x33);
    write_mem(0x4E00, 6);
    cpu.pc = 0x4282;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu.a == 6 && cpu_step(&cpu) == 4);
    BOARD_CHECK(cpu.pc == 0x4287 && read_mem(0x4287) == 0x60);
    BOARD_CHECK(rainbow_store(0x2000, 0) == 0 && rainbow_store(0x2003, 0) == 0);
    BOARD_CHECK(rainbow_store(0x2004, 0) == 0);
    (void)rainbow_begin_frame();
    rainbow_sprite_phase();
    BOARD_CHECK(ppu_read(0) == 48);

    BOARD_CHECK(read_mem(0x4280) == 0xA9);
    write_mem(0x4F00, 0x99);
    BOARD_CHECK(read_mem(0x4281) == 0x2A && read_mem(0x4280) == 0xA9 && read_mem(0x4281) == 0x2A);
    (void)read_mem(0x4282);
    BOARD_CHECK(read_mem(0x4280) == 0xA9 && read_mem(0x4281) == 0x99);
    (void)read_mem(0x4283);
    write_mem(0x4243, 0xFF);
    BOARD_CHECK(read_mem(0x4280) == 0xA9 && read_mem(0x4780) == 0x60);
    BOARD_CHECK(read_mem(0x4282) == 0xA9 && read_mem(0x43C2) == 0x60);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x4786) == 0xA5);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_rendered_video(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x2000));
    BOARD_CHECK(board_image_load(&image) == 0);
    bool startup = ppu_startup_write_restriction_enabled();
    ppu_set_startup_write_restriction(false);
    write_mem(0x4124, 0x37);
    write_mem(0x4125, 2);
    for (unsigned quadrant = 0; quadrant < 4; ++quadrant) write_mem((uint16_t)(0x412A + quadrant), 0x20);
    BOARD_CHECK(rainbow_store(0x2000, 8) == 0 && rainbow_store(0x2001, 0x18) == 0);
    unsigned dots = 0;
    do { ppu_step_dots(1); ++dots; }
    while (dots < 1000 && !(ppu.fetches_enabled && ppu.scanline < 240
                           && ppu.dot >= 16 && ppu.dot < 248 && (ppu.dot & 7) == 5));
    BOARD_CHECK(dots < 1000 && ppu.nt_byte == 0x37 && ppu.at_byte == 2);
    write_mem(0x4124, 0x49);
    write_mem(0x4125, 1);
    write_mem(0x412F, 0xA0);
    write_mem(0x4120, 0x10);
    dots = 0;
    do { ppu_step_dots(1); ++dots; }
    while (dots < 1000 && !(ppu.fetches_enabled && ppu.scanline < 240
                           && ppu.dot >= 16 && ppu.dot < 248 && (ppu.dot & 7) == 5));
    BOARD_CHECK(dots < 1000 && ppu.nt_byte == 0x49 && ppu.at_byte == 1);
    write_mem(0x4150, 1);
    write_mem(0x4153, 1);
    write_mem(0x4151, 0);
    for (unsigned step = 0; step < 2000 && !cart_irq_pending(); ++step)
        BOARD_CHECK(rainbow_nops(1) == 0);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(rainbow_store(0x2001, 0) == 0);
    write_mem(0x4152, 0);
    ppu_set_startup_write_restriction(startup);
    board_image_free(&image);
    return 0;
}

static void rainbow_flash_command(uint8_t command) {
    write_mem(0x8AAA, 0xAA);
    write_mem(0x8555, 0x55);
    write_mem(0x8AAA, command);
}

static void rainbow_chr_flash_command(uint8_t command) {
    ppu_write(0x0AAA, 0xAA);
    ppu_write(0x0555, 0x55);
    ppu_write(0x0AAA, command);
}

static int rainbow_program_prg(uint32_t address, uint8_t value) {
    write_mem(0x4100, 4);
    rainbow_prg(0, (uint16_t)(address >> 12));
    rainbow_flash_command(0xA0);
    return rainbow_store((uint16_t)(0x8000 | (address & 0xFFF)), value);
}

static uint8_t rainbow_peek_prg(uint32_t address) {
    write_mem(0x4100, 4);
    rainbow_prg(0, (uint16_t)(address >> 12));
    return read_mem((uint16_t)(0x8000 | (address & 0xFFF)));
}

static void rainbow_erase_prg(uint32_t address, bool whole_chip) {
    write_mem(0x4100, 4);
    rainbow_prg(0, (uint16_t)(address >> 12));
    rainbow_flash_command(0x80);
    write_mem(0x8AAA, 0xAA);
    write_mem(0x8555, 0x55);
    write_mem(whole_chip ? 0x8AAA : (uint16_t)(0x8000 | (address & 0xFFF)), whole_chip ? 0x10 : 0x30);
}

static void rainbow_program_chr(uint32_t address, uint8_t value) {
    write_mem(0x4120, 0);
    rainbow_chr(0, (uint16_t)(address >> 13));
    rainbow_chr_flash_command(0xA0);
    ppu_write((uint16_t)(address & 0x1FFF), value);
}

static uint8_t rainbow_peek_chr(uint32_t address) {
    write_mem(0x4120, 0);
    rainbow_chr(0, (uint16_t)(address >> 13));
    return ppu_read((uint16_t)(address & 0x1FFF));
}

static void rainbow_erase_chr(uint32_t address, bool whole_chip) {
    write_mem(0x4120, 0);
    rainbow_chr(0, (uint16_t)(address >> 13));
    rainbow_chr_flash_command(0x80);
    ppu_write(0x0AAA, 0xAA);
    ppu_write(0x0555, 0x55);
    ppu_write(whole_chip ? 0x0AAA : (uint16_t)(address & 0x1FFF), whole_chip ? 0x10 : 0x30);
}

static int test_rainbow_flash(void) {
    const size_t sizes[] = {0x100000, 0x200000, 0x400000, 0x800000};
    const uint8_t ids[] = {0x5B, 0x49, 0x7E, 0x7E}, extra[] = {0xFF, 0xFF, 0x0A, 0x10};
    for (unsigned model = 0; model < 4; ++model) {
        BoardImage image;
        BOARD_CHECK(rainbow_image(&image, sizes[model], sizes[model]));
        memset(image.data + 16, 0xFF, sizes[model] * 2);
        image.data[10] = image.data[11] = 7;
        BOARD_CHECK(board_image_load(&image) == 0);
        rainbow_flash_command(0x90);
        rainbow_chr_flash_command(0x90);
        BOARD_CHECK(read_mem(0x8000) == 1 && read_mem(0x8002) == ids[model]);
        BOARD_CHECK(read_mem(0x801C) == extra[model] && read_mem(0x801E) == (model < 2 ? 0xFF : 0));
        BOARD_CHECK(ppu_read(0) == 1 && ppu_read(2) == ids[model] && ppu_read(0x1C) == extra[model]);
        rainbow_prg(0, 0x8000);
        write_mem(0x8000, 0x66);
        BOARD_CHECK(read_mem(0x8000) == 0x66 && read_mem(0x6000) == 1);
        write_mem(0x4120, 0x40);
        ppu_write(0, 0x77);
        BOARD_CHECK(ppu_read(0) == 0x77);
        rainbow_prg(0, 0);
        write_mem(0x4120, 0);
        BOARD_CHECK(read_mem(0x8000) == 1 && ppu_read(0) == 1);
        write_mem(0x8000, 0xF0);
        ppu_write(0, 0xF0);
        BOARD_CHECK(read_mem(0x8000) == 0xFF && ppu_read(0) == 0xFF);
        BOARD_CHECK(rainbow_program_prg(0x2123, 0x5A) == 0);
        BOARD_CHECK(rainbow_program_prg(0x2123, 0xA5) == 0 && rainbow_peek_prg(0x2123) == 0);
        rainbow_program_chr(0x6123, 0x5A);
        rainbow_program_chr(0x6123, 0xA5);
        BOARD_CHECK(rainbow_peek_chr(0x6123) == 0 && rainbow_peek_prg(0x6123) == 0xFF);

        uint32_t top = (uint32_t)sizes[model] - 0x10000;
        BOARD_CHECK(rainbow_program_prg(top + 0x0123, 0x11) == 0);
        BOARD_CHECK(rainbow_program_prg(top + 0x1123, 0x22) == 0);
        BOARD_CHECK(rainbow_program_prg(top + 0x3123, 0x33) == 0);
        BOARD_CHECK(rainbow_program_prg(top + 0x9123, 0x44) == 0);
        BOARD_CHECK(rainbow_program_prg(top + 0xA123, 0x55) == 0);
        rainbow_erase_prg(top + 0x0123, false);
        BOARD_CHECK(rainbow_peek_prg(top + 0x0123) == 0xFF && rainbow_peek_prg(top + 0x1123) == 0xFF);
        BOARD_CHECK(rainbow_peek_prg(top + 0x3123) == (model < 2 ? 0xFF : 0x33));
        BOARD_CHECK(rainbow_peek_prg(top + 0x9123) == 0x44 && rainbow_peek_prg(top + 0xA123) == 0x55);
        rainbow_erase_prg(top + 0x8123, false);
        BOARD_CHECK(rainbow_peek_prg(top + 0x9123) == 0xFF && rainbow_peek_prg(top + 0xA123) == 0x55);
        rainbow_program_chr(top + 0x0123, 0x11);
        rainbow_program_chr(top + 0x3123, 0x22);
        rainbow_program_chr(top + 0x9123, 0x33);
        rainbow_erase_chr(top + 0x0123, false);
        BOARD_CHECK(rainbow_peek_chr(top + 0x0123) == 0xFF);
        BOARD_CHECK(rainbow_peek_chr(top + 0x3123) == (model < 2 ? 0xFF : 0x22));
        BOARD_CHECK(rainbow_peek_chr(top + 0x9123) == 0x33);

        if (model == 0) {
            write_mem(0x4100, 4);
            rainbow_prg(0, 2);
            rainbow_flash_command(0x20);
            write_mem(0x8000, 0xA0);
            write_mem(0x8124, 0xF0);
            write_mem(0x8000, 0xA0);
            write_mem(0x8124, 0x5A);
            write_mem(0x8000, 0x90);
            write_mem(0x8000, 0);
            BOARD_CHECK(read_mem(0x8124) == 0x50);
            write_mem(0x8AAA, 0xAA);
            write_mem(0x8555, 0);
            write_mem(0x8AAA, 0xA0);
            write_mem(0x8125, 0);
            BOARD_CHECK(read_mem(0x8125) == 0xFF);
            rainbow_flash_command(0xA0);
            cpu_soft_reset(&cpu);
            BOARD_CHECK(rainbow_store(0x8126, 0xA6) == 0);
            BOARD_CHECK(rainbow_peek_prg(0x0126) == 0xA6 && rainbow_peek_prg(0x2126) == 0xFF);
        }
        rainbow_erase_prg(0, true);
        rainbow_erase_chr(0, true);
        BOARD_CHECK(rainbow_peek_prg(0x2123) == 0xFF && rainbow_peek_prg(top + 0xA123) == 0xFF);
        BOARD_CHECK(rainbow_peek_chr(0x6123) == 0xFF && rainbow_peek_chr(top + 0x9123) == 0xFF);
        board_image_free(&image);
    }
    return 0;
}

static int test_rainbow_geometry_and_register_bounds(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x6000, 0x2800));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xD000) == 5 && read_mem(0xFFFF) == 0);
    write_mem(0x4100, 4);
    rainbow_prg(0, 5);
    write_mem(0x4120, 4);
    rainbow_chr(0, 31);
    BOARD_CHECK(read_mem(0x8000) == 5 && ppu_read(0) == 11);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 5 && ppu_read(0) == 11);
    image.data[7] |= 2;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 5 && ppu_read(0) == 11);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && read_mem(0x8000) == 5);
    board_image_free(&image);

    BOARD_CHECK(rainbow_image(&image, 0x2000, 0x100));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x9000) == 1 && read_mem(0xFFFF) == 0);
    write_mem(0x4120, 4);
    rainbow_chr(15, 0xFFFF);
    BOARD_CHECK(ppu_read(0x1E23) == 0 && ppu_read(0x1F23) == 0x23);
    board_image_free(&image);
    BOARD_CHECK(rainbow_image(&image, 0x8000, 0x2800));
    image.data[6] |= 2;
    image.data[10] = image.data[11] = 0x22;
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned value = 0; value < 256; ++value) {
        write_mem(0x4100, (uint8_t)value);
        rainbow_prg(value & 7, (uint16_t)(value * 257));
        rainbow_low(value & 1, (uint16_t)(value * 257));
        write_mem(0x4120, (uint8_t)value);
        rainbow_chr(value & 15, (uint16_t)(value * 257));
        write_mem((uint16_t)(0x4126 + (value & 3)), (uint8_t)value);
        write_mem((uint16_t)(0x412A + (value & 3)), (uint8_t)value);
        write_mem(0x412E, (uint8_t)value);
        write_mem(0x412F, (uint8_t)value);
        for (unsigned slot = 0; slot < 16; ++slot) {
            uint16_t address = (uint16_t)(slot * 0x200 + 0x123);
            ppu_write(address, (uint8_t)value);
            (void)ppu_read(address);
        }
        for (unsigned slot = 0; slot < 4; ++slot) {
            uint16_t address = (uint16_t)(0x2000 + slot * 0x400 + 0x123);
            ppu_write(address, (uint8_t)value);
            (void)ppu_read(address);
            (void)ppu_read((uint16_t)(address | 0x3C0));
        }
        (void)read_mem(0x6000);
        (void)read_mem(0x8000);
        (void)read_mem(0xFFFF);
    }
    write_mem(0x4190, 0xFF);
    write_mem(0x4191, 0xFF);
    write_mem(0x4192, 0xFF);
    BOARD_CHECK(read_mem(0x4190) == 3 && read_mem(0x4191) == 0 && read_mem(0x4192) == 0);
    write_mem(0x41FF, 0);
    BOARD_CHECK(read_mem(0x4190) == 3);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x4190) == 0 && read_mem(0x4120) == 0 && read_mem(0x8000) == 0);
    board_image_free(&image);
    return 0;
}

static int test_rainbow_persistence(void) {
    BoardImage image;
    BOARD_CHECK(rainbow_image(&image, 0x20000, 0x8000));
    memset(image.data + 16, 0xFF, 0x28000);
    image.data[6] |= 2;
    image.data[10] = image.data[11] = 0x77;
    char stem[100], path[120], save[120], chr_save[128], flash_save[132], chr_flash_save[136];
    snprintf(stem, sizeof(stem), "build/board-rainbow-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    snprintf(chr_save, sizeof(chr_save), "%s.chr.sav", stem);
    snprintf(flash_save, sizeof(flash_save), "%s.flash.sav", stem);
    snprintf(chr_flash_save, sizeof(chr_flash_save), "%s.chr.flash.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    uint8_t saved[0x2000] = {0};
    saved[0x123] = 0x87;
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    count = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    rainbow_low(0, 0x8000);
    BOARD_CHECK(read_mem(0x6123) == 0x87);
    write_mem(0x6123, 0x98);
    write_mem(0x4120, 0x44);
    rainbow_chr(0, 0);
    ppu_write(0x0023, 0xB6);
    rainbow_chr(0, 16);
    ppu_write(0x0023, 0xC7);
    write_mem(0x5023, 0xD8);
    write_mem(0x4823, 0xE9);
    BOARD_CHECK(rainbow_program_prg(0x10123, 0x5A) == 0);
    rainbow_program_chr(0x6123, 0xA6);
    BOARD_CHECK(read_mem(0x8123) == 0x5A && ppu_read(0x0123) == 0xA6);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8123) == 0x5A && read_mem(0x5023) == 0xD8 && read_mem(0x6123) == 0x98);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x5023) == 0xD8 && read_mem(0x4823) == 0xE9 && read_mem(0x6123) == 0x98);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    BOARD_CHECK(read_mem(0x5023) == 0 && read_mem(0x4823) == 0);
    rainbow_low(0, 0x8000);
    BOARD_CHECK(read_mem(0x6123) == 0x98);
    write_mem(0x4120, 0x44);
    rainbow_chr(0, 0);
    BOARD_CHECK(ppu_read(0x0023) == 0);
    rainbow_chr(0, 16);
    BOARD_CHECK(ppu_read(0x0023) == 0xC7);
    BOARD_CHECK(rainbow_peek_prg(0x10123) == 0x5A && rainbow_peek_chr(0x6123) == 0xA6);
    BOARD_CHECK(unload_rom());

    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL);
    count = fread(saved, 1, sizeof(saved), file);
    BOARD_CHECK(count == sizeof(saved) && fgetc(file) == EOF && fclose(file) == 0 && saved[0x123] == 0x98);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL);
    count = fread(saved, 1, sizeof(saved), file);
    BOARD_CHECK(count == sizeof(saved) && fgetc(file) == EOF && fclose(file) == 0 && saved[0x23] == 0xC7);
    file = fopen(flash_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x20000);
    BOARD_CHECK(fseek(file, 0x10123, SEEK_SET) == 0 && fgetc(file) == 0x5A && fclose(file) == 0);
    file = fopen(chr_flash_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x8000);
    BOARD_CHECK(fseek(file, 0x6123, SEEK_SET) == 0 && fgetc(file) == 0xA6 && fclose(file) == 0);
    file = fopen(path, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 16 + 0x10123, SEEK_SET) == 0 && fgetc(file) == 0xFF);
    BOARD_CHECK(fseek(file, 16 + 0x20000 + 0x6123, SEEK_SET) == 0 && fgetc(file) == 0xFF && fclose(file) == 0);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
    BOARD_CHECK(remove(flash_save) == 0 && remove(chr_flash_save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_rainbow_accuracy(void) {
    int failures = 0;
    failures += test_rainbow_prg_modes();
    failures += test_rainbow_ram_sources();
    failures += test_rainbow_chr_modes();
    failures += test_rainbow_nametables();
    failures += test_rainbow_cpu_irq();
    failures += test_rainbow_scanline_irq();
    failures += test_rainbow_audio();
    failures += test_rainbow_extended_background();
    failures += test_rainbow_window();
    failures += test_rainbow_sprites_and_oam();
    failures += test_rainbow_rendered_video();
    failures += test_rainbow_flash();
    failures += test_rainbow_geometry_and_register_bounds();
    failures += test_rainbow_persistence();
    unload_rom();
    printf("Rainbow accuracy: 14 groups, %d failures\n", failures);
    return failures;
}
