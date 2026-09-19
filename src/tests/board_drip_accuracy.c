/*
 * board_drip_accuracy.c - Drip Game cartridge, FIFO audio, and PPU tests
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

static int drip_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int drip_nops(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned step = 0; step < count; ++step) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static uint8_t drip_fetch(uint16_t address, CartPpuFetchSource source) {
    cart_set_ppu_fetch_source(source);
    uint8_t value = cart_ppu_read(address);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    return value;
}

static bool drip_sample(int output) {
    return fabsf(cart_expansion_audio() - output * (3.0f / 5000.0f)) < 0.000001f;
}

static int test_drip_banks(void) {
    BoardImage image;
    BOARD_CHECK(cart_set_dip_switches(0));
    BOARD_CHECK(board_image_create(&image, 284, 0x80000, 0x10000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x65));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xC000) == 124 && read_mem(0xFFFF) == 127);
    BOARD_CHECK(ppu_read(0x1234) == 0x34 && read_mem(0x7000) == 0x65);
    BOARD_CHECK(read_mem(0x4800) == 0x64 && read_mem(0x4FFF) == 0x64);
    BOARD_CHECK(cart_set_dip_switches(1));
    BOARD_CHECK(read_mem(0x4800) == 0xE4 && read_mem(0x4A75) == 0xE4);
    BOARD_CHECK(read_mem(0x5000) == 0x40 && read_mem(0x57FF) == 0x40);
    BOARD_CHECK(read_mem(0x5800) == 0x40 && read_mem(0x5FFF) == 0x40);
    BOARD_CHECK(drip_store(0xBFEB, 0xF3) == 0 && read_mem(0x8000) == 12);
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(drip_store((uint16_t)(0xBFFC + slot), (uint8_t)(0xF4 + slot)) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x800)) == 8 + slot * 2);
    }
    write_mem(0xC000, 0xFF);
    ppu_write(0x0123, 0xEE);
    BOARD_CHECK(read_mem(0xC000) == 124 && ppu_read(0x0123) == 8);
    write_mem(0x6123, 0x91);
    BOARD_CHECK(read_mem(0x6123) == 0x91);
    write_mem(0x800A, 0);
    write_mem(0x6123, 0xA2);
    BOARD_CHECK(read_mem(0x6123) == 0x91 && cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x800A, 9);
    BOARD_CHECK(drip_store(0x6123, 0xA2) == 0 && read_mem(0x6123) == 0xA2);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x800A, 10);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    write_mem(0x800A, 11);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1 && read_mem(0x6123) == 0xA2);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0x0123) == 8);
    BOARD_CHECK(cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_drip_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    write_mem(0x8008, 4);
    BOARD_CHECK(drip_store(0x8009, 0x80) == 0);
    BOARD_CHECK(drip_nops(1) == 0 && !cart_irq_pending());
    write_mem(0x8008, 0xFE);
    BOARD_CHECK(drip_nops(1) == 0 && cart_irq_pending());
    (void)read_mem(0x4800);
    write_mem(0x8008, 0);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(drip_store(0xBFF9, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(drip_store(0x8009, 0x80) == 0);
    BOARD_CHECK(drip_nops(1000) == 0 && !cart_irq_pending());
    write_mem(0x8009, 0);
    write_mem(0x8008, 5);
    BOARD_CHECK(drip_store(0x8009, 0x80) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0x8009, 0);
    write_mem(0x8008, 0);
    BOARD_CHECK(drip_store(0x8009, 0x81) == 0);
    BOARD_CHECK(drip_store(0x4014, 0) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());
    BOARD_CHECK(drip_store(0x8009, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(drip_nops(200) == 0 && !cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_drip_attributes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xC123, 1);
    write_mem(0xC523, 2);
    write_mem(0xF524, 3);
    ppu_write(0x23D2, 0x1B);
    write_mem(0x800A, 4);
    BOARD_CHECK(drip_fetch(0x2123, CART_PPU_FETCH_BG) == 0);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0x55);
    BOARD_CHECK(ppu_read(0x23D2) == 0x1B);
    (void)drip_fetch(0x2523, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x27D2, CART_PPU_FETCH_BG) == 0xAA);
    (void)drip_fetch(0x2524, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x27D2, CART_PPU_FETCH_BG) == 0xFF);
    (void)ppu_read(0x2523);
    (void)drip_fetch(0x0123, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x27D2, CART_PPU_FETCH_BG) == 0xFF);
    write_mem(0x800A, 5);
    (void)drip_fetch(0x2523, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x27D2, CART_PPU_FETCH_BG) == 0x55);
    (void)drip_fetch(0x2923, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x2BD2, CART_PPU_FETCH_BG) == 0xAA);
    write_mem(0x800A, 6);
    BOARD_CHECK(drip_fetch(0x2FD2, CART_PPU_FETCH_SPRITE) == 0x55);
    write_mem(0x800A, 7);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0xAA);
    write_mem(0xE523, 0xFD);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0x55);
    (void)drip_fetch(0x3124, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x37D2, CART_PPU_FETCH_BG) == 0xFF);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0xFF);
    write_mem(0x800A, 3);
    ppu_write(0x23D2, 0x2C);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0x2C);
    board_image_free(&image);
    return 0;
}

static int test_drip_rendered_attributes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned offset = 0; offset < 0x3C0; ++offset) write_mem((uint16_t)(0xC000 + offset), 2);
    bool startup = ppu_startup_write_restriction_enabled();
    ppu_set_startup_write_restriction(false);
    BOARD_CHECK(drip_store(0x800A, 6) == 0);
    BOARD_CHECK(drip_store(0x2000, 0) == 0);
    BOARD_CHECK(drip_store(0x2001, 0x08) == 0);
    unsigned dots = 0;
    do {
        ppu_step_dots(1);
        ++dots;
    } while (dots < 1000 && !(ppu.fetches_enabled && ppu.scanline < 240
               && ppu.dot >= 8 && ppu.dot < 256 && (ppu.dot & 7) == 5));
    BOARD_CHECK(dots < 1000 && ppu.at_byte == 2);
    write_mem(0x800A, 2);
    ppu_step_dots(8);
    BOARD_CHECK(ppu.at_byte == 0);
    BOARD_CHECK(drip_store(0x2001, 0) == 0);
    ppu_set_startup_write_restriction(startup);
    board_image_free(&image);
    return 0;
}

static int test_drip_audio_timing(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && drip_sample(0));
    apu_audio_init(48000);
    write_mem(0x8002, 4);
    write_mem(0x8003, 0x20);
    write_mem(0x8000, 0);
    uint64_t transitions = apu.audio_transition_count;
    BOARD_CHECK(drip_store(0x8001, 0xC0) == 0 && drip_sample(128));
    BOARD_CHECK(apu.audio_transition_count > transitions);
    write_mem(0x8001, 0x40);
    write_mem(0x8001, 0x80);
    BOARD_CHECK(read_mem(0x5000) == 0 && read_mem(0x5800) == 0x40);
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(128));
    transitions = apu.audio_transition_count;
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(-128));
    BOARD_CHECK(apu.audio_transition_count > transitions);
    write_mem(0x8002, 6);
    write_mem(0x8003, 0x30);
    BOARD_CHECK(drip_sample(-192));
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(-192));
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(0) && read_mem(0x5000) == 0);
    BOARD_CHECK(drip_nops(2) == 0 && read_mem(0x5000) == 0);
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(-384) && read_mem(0x5000) == 0x40);
    write_mem(0x8003, 0xF0);
    BOARD_CHECK(drip_sample(-384));
    BOARD_CHECK(drip_nops(10) == 0 && drip_sample(-384));
    write_mem(0x8006, 0xFF);
    write_mem(0x8007, 0x1F);
    write_mem(0x8004, 0);
    BOARD_CHECK(drip_store(0x8005, 0xF0) == 0 && drip_sample(-272));
    write_mem(0x8000, 0);
    BOARD_CHECK(drip_sample(112) && read_mem(0x5000) == 0x40 && read_mem(0x5800) == 0);
    write_mem(0xBFF7, 0x2F);
    BOARD_CHECK(drip_sample(224));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(drip_sample(224) && read_mem(0x5800) == 0);
    BOARD_CHECK(drip_store(0xBFF4, 0xFF) == 0 && drip_sample(0) && read_mem(0x5800) == 0x40);
    BOARD_CHECK(board_image_load(&image) == 0 && drip_sample(0));
    apu_audio_shutdown_state(&apu);
    board_image_free(&image);
    return 0;
}

static int test_drip_audio_fifo(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8002, 0);
    write_mem(0x8003, 0x10);
    write_mem(0x8000, 0);
    for (unsigned byte = 0; byte < 256; ++byte) write_mem(0x8001, (uint8_t)(0x80 + byte));
    BOARD_CHECK(read_mem(0x5000) == 0x80 && drip_sample(0));
    BOARD_CHECK(drip_nops(32767) == 0 && read_mem(0x5000) == 0x80 && drip_sample(0));
    BOARD_CHECK(drip_nops(1) == 0 && read_mem(0x5000) == 0 && drip_sample(1));
    write_mem(0x8000, 0);
    for (unsigned byte = 0; byte < 256; ++byte) write_mem(0x8001, (uint8_t)(0x80 + byte));
    write_mem(0x8001, 0xCF);
    BOARD_CHECK(read_mem(0x5000) == 0x80 && drip_sample(79));
    // Overwriting a full FIFO moves its write pointer without clearing the
    // full flag. Draining to that pointer sets empty as well.
    BOARD_CHECK(drip_nops(32768) == 0 && read_mem(0x5000) == 0xC0 && drip_sample(1));
    write_mem(0x8002, 2);
    write_mem(0x8000, 0);
    for (unsigned byte = 0; byte < 256; ++byte) write_mem(0x8001, (uint8_t)(0x80 + byte));
    for (unsigned byte = 1; byte <= 256; ++byte) {
        BOARD_CHECK(drip_nops(1) == 0);
        BOARD_CHECK(drip_sample((int)(uint8_t)(0x80 + byte) - 0x80));
        BOARD_CHECK(read_mem(0x5000) == (byte == 256 ? 0x40 : 0));
    }
    BOARD_CHECK(drip_store(0x8001, 0xFF) == 0 && drip_sample(127) && read_mem(0x5000) == 0);
    BOARD_CHECK(drip_nops(1) == 0 && drip_sample(1) && read_mem(0x5000) == 0x40);
    board_image_free(&image);
    return 0;
}

static int test_drip_geometry_and_power(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x6000, 0x2800, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 0 && read_mem(0xFFFF) == 3);
    write_mem(0x800B, 15);
    write_mem(0x800F, 14);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1800) == 8 && ppu_read(0x1FFF) == 9);
    write_mem(0x800A, 8);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x6000) == 0xA5);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1800) == 8);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1800) == 8);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && ppu_read(0x1800) == 8);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 284, 0x2000, 0x100, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xFFFF) == 1);
    write_mem(0x800F, 15);
    BOARD_CHECK(ppu_read(0x0323) == 0 && ppu_read(0x1823) == 0x23);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 284, 0x8000, 0, true));
    image.data[11] = 2;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x800F, 15);
    ppu_write(0x0323, 0xA6);
    BOARD_CHECK(ppu_read(0x0323) == 0xA6 && ppu_read(0x1023) == 0xA6);
    NesRamPowerOnState previous = nes_ram_power_on_state();
    const NesRamPowerOnState modes[] = {NES_RAM_POWER_ZERO, NES_RAM_POWER_ONES};
    for (unsigned mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode) {
        BOARD_CHECK(nes_set_ram_power_on_state(modes[mode]));
        BOARD_CHECK(board_image_load(&image) == 0);
        write_mem(0x800A, 6);
        (void)drip_fetch(0x2123, CART_PPU_FETCH_BG);
        BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == (mode ? 0xFF : 0));
        BOARD_CHECK(ppu_read(0x0023) == (mode ? 0xFF : 0));
        BOARD_CHECK(read_mem(0x5000) == 0x40 && drip_sample(0));
    }
    BOARD_CHECK(nes_set_ram_power_on_state(previous));
    board_image_free(&image);
    return 0;
}

static int test_drip_persistence(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 284, 0x10000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x77;
    image.data[11] = 0x70;
    BOARD_CHECK(board_image_add_trainer(&image, 0x65));
    char path[128], save[128], chr_save[136];
    snprintf(path, sizeof(path), "build/board-drip-%lu-%lu.nes", (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    memcpy(chr_save, path, strlen(path) + 1);
    strcpy(strrchr(chr_save, '.'), ".chr.sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    uint8_t saved[0x2000] = {0};
    saved[0x1000] = 0x76;
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    count = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x76);
    write_mem(0x7000, 0x87);
    write_mem(0x800A, 0);
    BOARD_CHECK(read_mem(0x7000) == 0x65);
    write_mem(0x7000, 0x98);
    BOARD_CHECK(read_mem(0x7000) == 0x65);
    BOARD_CHECK(drip_store(0x800A, 8) == 0);
    write_mem(0x7000, 0xA9);
    BOARD_CHECK(read_mem(0x7000) == 0xA9);
    ppu_write(0x0023, 0xB6);
    write_mem(0x800C, 2);
    ppu_write(0x0023, 0xC7);
    write_mem(0xC123, 3);
    write_mem(0x800A, 6);
    (void)drip_fetch(0x2123, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0xFF);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x87 && ppu_read(0x0023) == 0xB6);
    write_mem(0x800C, 2);
    BOARD_CHECK(ppu_read(0x0023) == 0xC7);
    write_mem(0x800A, 6);
    BOARD_CHECK(read_mem(0x7000) == 0x65);
    (void)drip_fetch(0x2123, CART_PPU_FETCH_BG);
    BOARD_CHECK(drip_fetch(0x23D2, CART_PPU_FETCH_BG) == 0);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL);
    count = fread(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0 && saved[0x1000] == 0x87);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x2000 && closed == 0);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_drip_accuracy(void) {
    int failures = 0;
    failures += test_drip_banks();
    cart_set_dip_switches(0);
    failures += test_drip_irq();
    failures += test_drip_attributes();
    failures += test_drip_rendered_attributes();
    failures += test_drip_audio_timing();
    failures += test_drip_audio_fifo();
    failures += test_drip_geometry_and_power();
    failures += test_drip_persistence();
    unload_rom();
    printf("Drip Game accuracy: 8 groups, %d failures\n", failures);
    return failures;
}
