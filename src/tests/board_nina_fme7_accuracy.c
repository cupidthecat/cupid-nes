/*
 * board_nina_fme7_accuracy.c - NINA and FME-7 memory ownership and reset tests
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../system/hardware.h"
#include <time.h>

static bool power_cart(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool cpu_load_is(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 4 && cpu.a == expected;
}

static bool fme_command(uint8_t command, uint8_t value) {
    return cpu_store(0x8000, command) && cpu_store(0xA000, value);
}

static bool image_create(BoardImage *image, unsigned mapper, uint8_t prg_ram,
                         size_t chr_bytes, uint8_t chr_ram) {
    if (!board_image_create(image, mapper, 0x20000, chr_bytes, true)) return false;
    image->data[8] = mapper == 34 ? 0x10 : 0;
    image->data[10] = prg_ram;
    image->data[11] = chr_ram;
    if ((prg_ram | chr_ram) & 0xF0) image->data[6] |= 2;
    return true;
}

static int test_nina_register_ram_and_reset(void) {
    BoardImage image;
    BOARD_CHECK(image_create(&image, 34, 0x87, 0x8000, 0));
    BOARD_CHECK(board_image_add_trainer(&image, 0xC1));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
    BOARD_CHECK(cpu_load_is(0x7000, 0));
    BOARD_CHECK(ppu_read(0x0123) == 0x23 && ppu_read(0x1ABC) == 0xBC);
    BOARD_CHECK(cpu_store(0x7FFD, 3));
    BOARD_CHECK(cpu_store(0x7FFE, 2) && cpu_store(0x7FFF, 5));
    BOARD_CHECK(cpu_load_is(0x8000, 24) && cpu_load_is(0x7FFD, 3));
    BOARD_CHECK(cpu_load_is(0x7FFE, 2) && cpu_load_is(0x7FFF, 5));
    BOARD_CHECK(ppu_read(0x0123) == 8 && ppu_read(0x1123) == 20);
    ppu_write(0x0123, 0xA6);
    BOARD_CHECK(ppu_read(0x0123) == 8);
    BOARD_CHECK(cpu_store(0x6001, 0x5A) && cpu_store(0x8000, 0));
    BOARD_CHECK(cpu_load_is(0x8000, 24));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cpu_load_is(0x8000, 24) && cpu_load_is(0x6001, 0x5A));
    BOARD_CHECK(ppu_read(0x0123) == 8 && ppu_read(0x1123) == 20);
    uint8_t *old_prg = prg_rom;
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
    BOARD_CHECK(prg_rom == old_prg && cpu_load_is(0x7FFD, 3));
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
    BOARD_CHECK(cpu_load_is(0x8000, 0) && ppu_read(0x0123) == 0x23);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_nina_banked_chr_ram(void) {
    BoardImage image;
    BOARD_CHECK(image_create(&image, 34, 0, 0, 9));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
    ppu_write(0x0123, 0xA0);
    ppu_write(0x1123, 0xA1);
    for (uint8_t bank = 2; bank < 8; ++bank) {
        BOARD_CHECK(cpu_store(0x7FFE, bank));
        ppu_write(0x0123, (uint8_t)(0xA0 + bank));
    }
    for (uint8_t bank = 0; bank < 8; ++bank) {
        BOARD_CHECK(cpu_store(0x7FFE, bank) && cpu_store(0x7FFF, bank + 8));
        BOARD_CHECK(ppu_read(0x0123) == 0xA0 + bank);
        BOARD_CHECK(ppu_read(0x1123) == 0xA0 + bank);
    }
    BOARD_CHECK(cart_cpu_read_bus(0x7FFE, 0x39) == 0x39);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_fme_selected_chip_and_small_pages(void) {
    const uint8_t layouts[] = {0x00, 0x01, 0x07};
    for (size_t i = 0; i < sizeof(layouts); ++i) {
        BoardImage image;
        BOARD_CHECK(image_create(&image, 69, layouts[i], 0x2000, 0));
        image.data[6] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(fme_command(8, 3) && cpu_load_is(0x6123, 6));
        BOARD_CHECK(fme_command(8, 0xC0));
        BOARD_CHECK(cpu_store(0x6123, 0xA6) && cpu_load_is(0x6123, 6));
        BOARD_CHECK(fme_command(8, 0x40) && cpu_load_is(0x6123, 6));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }

    for (uint8_t shift = 1; shift <= 2; ++shift) {
        BoardImage image;
        BOARD_CHECK(image_create(&image, 69, shift, 0x2000, 0));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(fme_command(8, 2) && cpu_load_is(0x6123, 4));
        BOARD_CHECK(fme_command(8, 0xFF) && cpu_store(0x6023, 0xA5));
        if (shift == 1) {
            BOARD_CHECK(cpu_load_is(0x6023, 4) && cpu_load_is(0x7FFF, 5));
            BOARD_CHECK(fme_command(8, 0x40));
            BOARD_CHECK(cpu_load_is(0x6023, 4) && cpu_load_is(0x7FFF, 5));
        } else {
            BOARD_CHECK(cpu_load_is(0x6123, 0xA5) && cpu_load_is(0x7F23, 0xA5));
            BOARD_CHECK(fme_command(8, 0x40));
            BOARD_CHECK(cart_cpu_read_bus(0x6023, 0x39) == 0x39);
            BOARD_CHECK(cpu_store(0x6023, 0x77));
            BOARD_CHECK(fme_command(8, 0xC0) && cpu_load_is(0x6023, 0xA5));
        }
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_fme_reset_keeps_banks_irq_and_audio(void) {
    BoardImage image;
    BOARD_CHECK(image_create(&image, 69, 9, 0, 9));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
    BOARD_CHECK(fme_command(9, 5) && fme_command(8, 0xC3));
    BOARD_CHECK(cpu_store(0x6123, 0x53));
    BOARD_CHECK(fme_command(0, 31));
    ppu_write(0x0123, 0xA6);
    BOARD_CHECK(cpu_store(0xC000, 7) && cpu_store(0xE000, 0x3F));
    BOARD_CHECK(cpu_store(0xC000, 8) && cpu_store(0xE000, 15));
    float audio = cart_expansion_audio();
    BOARD_CHECK(audio < -0.125f && audio > -0.127f);
    BOARD_CHECK(fme_command(14, 1) && fme_command(15, 0) && fme_command(13, 0x81));
    write_mem(0x0200, 0xEA);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending() && cart_expansion_audio() == audio);
    BOARD_CHECK(cpu_load_is(0x8000, 10) && cpu_load_is(0x6123, 0x53));
    BOARD_CHECK(ppu_read(0x0123) == 0xA6);
    uint8_t *old_prg = prg_rom;
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
    BOARD_CHECK(prg_rom == old_prg && cart_irq_pending());
    BOARD_CHECK(fme_command(13, 0) && !cart_irq_pending());
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(cart_expansion_audio() == 0.0f && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static bool write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(data, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static bool read_file_exact(const char *path, uint8_t *data, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool ok = fread(data, 1, size, file) == size && fgetc(file) == EOF;
    return fclose(file) == 0 && ok;
}

static int test_split_ram_save_and_trainer(void) {
    const unsigned mappers[] = {34, 69};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        BoardImage image;
        BOARD_CHECK(image_create(&image, mapper, 0x87, 0x2000, 0));
        BOARD_CHECK(board_image_add_trainer(&image, 0xC1));
        char path[160], save_path[160];
        int used = snprintf(path, sizeof(path), "build/nina-fme-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used < sizeof(path));
        memcpy(save_path, path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".sav");
        uint8_t expected[0x4000], saved[0x4000];
        memset(expected, 0x12, 0x2000);
        memset(expected + 0x2000, 0x34, 0x2000);
        BOARD_CHECK(write_file(path, image.data, image.size));
        BOARD_CHECK(write_file(save_path, expected, sizeof(expected)));
        BOARD_CHECK(load_rom(path) == 0 && power_cart());
        if (mapper == 69) BOARD_CHECK(fme_command(8, 0xC0));
        BOARD_CHECK(cpu_load_is(0x7000, 0x12));
        if (mapper == 34) {
            BOARD_CHECK(cpu_store(0x7FFD, 3) && cpu_store(0x7FFE, 5));
            expected[0x1FFD] = 3;
            expected[0x1FFE] = 5;
        } else {
            BOARD_CHECK(fme_command(8, 0xC1) && cpu_load_is(0x7000, 0x34));
            BOARD_CHECK(cpu_store(0x6123, 0xA6));
            expected[0x2123] = 0xA6;
            BOARD_CHECK(fme_command(8, 0xC2) && cpu_load_is(0x6123, 0x12));
        }
        cart_battery_flush();
        BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
        BOARD_CHECK(memcmp(saved, expected, sizeof(saved)) == 0);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(path) == 0 && power_cart());
        if (mapper == 34) {
            BOARD_CHECK(cpu_load_is(0x7FFD, 3) && cpu_load_is(0x7FFE, 5));
            BOARD_CHECK(cpu_load_is(0x8000, 0));
        } else {
            BOARD_CHECK(fme_command(8, 0xC1) && cpu_load_is(0x6123, 0xA6));
        }
        BOARD_CHECK(unload_rom() && remove(save_path) == 0 && remove(path) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_board_nina_fme7_accuracy(void) {
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_ZERO)) return 1;
    int failures = test_nina_register_ram_and_reset();
    failures += test_nina_banked_chr_ram();
    failures += test_fme_selected_chip_and_small_pages();
    failures += test_fme_reset_keeps_banks_irq_and_audio();
    failures += test_split_ram_save_and_trainer();
    unload_rom();
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT)) ++failures;
    printf("NINA and FME-7 ownership: 5 groups, %d failures\n", failures);
    return failures;
}
