/*
 * board_ffe_accuracy.c - Front Fareast cartridge bus and IRQ regressions
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
#include <time.h>

static void ffe_chr_bank(unsigned mapper, uint8_t bank) {
    if (mapper == 17) {
        for (unsigned slot = 0; slot < 8; ++slot)
            write_mem((uint16_t)(0x4510 + slot), (uint8_t)(bank * 8 + slot));
    } else {
        write_mem(0x8000, bank);
    }
}

static int test_ffe_bank_registers(void) {
    static const unsigned mappers[] = {6, 8, 17};
    static const uint8_t startup[3][4] = {{0, 2, 28, 30}, {0, 2, 4, 6}, {120, 122, 124, 126}};
    for (unsigned variant = 0; variant < 3; ++variant) {
        unsigned mapper = mappers[variant];
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, mapper, 0x80000, 0x40000, true));
        image.data[8] |= 0xF0; // These board IDs do not decode the submapper.
        image.data[10] = 7;
        BOARD_CHECK(board_image_load(&image) == 0);
        for (unsigned slot = 0; slot < 4; ++slot) {
            BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x2000)) == startup[variant][slot]);
            BOARD_CHECK(read_mem((uint16_t)(0x9FFF + slot * 0x2000)) == startup[variant][slot] + 1);
        }
        BOARD_CHECK(ppu_read(0x1234) == 0x34);
        if (mapper == 6) {
            write_mem(0xFFFF, 0x17);
            BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xBFFF) == 23);
            BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xFFFF) == 31);
            BOARD_CHECK(ppu_read(0) == 24 && ppu_read(0x1FFF) == 31);
            write_mem(0x42FE, 0x80);
            write_mem(0x8000, 0x1B);
            BOARD_CHECK(read_mem(0x8000) == 20 && ppu_read(0) == 216);
            BOARD_CHECK(ppu_read(0x1FFF) == 223);
            write_mem(0x42FF, 0x10); // This mirror register does not change the mode.
            write_mem(0x8000, 0x12);
            BOARD_CHECK(read_mem(0x8000) == 20 && ppu_read(0) == 144);
        } else if (mapper == 8) {
            write_mem(0xFFFF, 0x2B);
            BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xBFFF) == 23);
            BOARD_CHECK(read_mem(0xC000) == 4 && read_mem(0xFFFF) == 7);
            BOARD_CHECK(ppu_read(0) == 24 && ppu_read(0x1FFF) == 31);
        } else {
            for (unsigned slot = 0; slot < 4; ++slot)
                write_mem((uint16_t)(0x4504 + slot), (uint8_t)(1 + slot * 3));
            for (unsigned slot = 0; slot < 4; ++slot)
                BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x2000)) == 2 + slot * 6);
            for (unsigned slot = 0; slot < 8; ++slot)
                write_mem((uint16_t)(0x4510 + slot), (uint8_t)(13 + slot * 2));
            for (unsigned slot = 0; slot < 8; ++slot)
                BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == 13 + slot * 2);
            write_mem(0x8000, 0xFF);
            write_mem(0x4508, 0xFF);
            write_mem(0x4518, 0xFF);
            BOARD_CHECK(read_mem(0x8000) == 2 && read_mem(0xE000) == 20);
        }
        write_mem(0x6000, 0xAD);
        write_mem(0x42FE, 0);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
        ppu_write(0x2000, 0x92);
        BOARD_CHECK(ppu_read(0x2C00) == 0x92);
        write_mem(0x42FE, 0x10);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
        ppu_write(0x2400, 0x83);
        BOARD_CHECK(ppu_read(0x2000) == 0x83 && ppu_read(0x2C00) == 0x83);
        write_mem(0x42FF, 0);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        BOARD_CHECK(ppu_read(0x2000) == 0x92 && ppu_read(0x2400) == 0x83);
        write_mem(0x42FF, 0x10);
        write_mem(0x43FF, 0);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        BOARD_CHECK(ppu_read(0x2400) == 0x92 && ppu_read(0x2800) == 0x83);
        uint8_t prg = read_mem(0x8000), chr = ppu_read(0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == prg && ppu_read(0) == chr && read_mem(0x6000) == 0xAD);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        write_mem(0x4800, 0xB6);
        BOARD_CHECK(read_mem(0x4502) == 0xB6);
        board_image_free(&image);
    }
    // Legacy mapper 6 also has CHR RAM. It keeps PRG selection active even
    // with a CHR ROM present and the alternate-mode control bit cleared.
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 6, 0x80000, 0x40000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x42FE, 0x80);
    write_mem(0x8000, 0x1B);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 24);
    board_image_free(&image);
    return 0;
}

static int ffe_cpu_store(uint16_t address, uint8_t value) {
    const uint8_t program[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(program); ++byte)
        write_mem((uint16_t)(0x0200 + byte), program[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int ffe_nop(void) {
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == 2);
    return 0;
}

static int test_ffe_irq(void) {
    static const unsigned mappers[] = {6, 8, 17};
    for (unsigned variant = 0; variant < 3; ++variant) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, mappers[variant], 0x20000, 0, false));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4502, 0xFD) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFF) == 0);
        BOARD_CHECK(!cart_irq_pending());
        BOARD_CHECK(ffe_nop() == 0 && !cart_irq_pending());
        BOARD_CHECK(ffe_nop() == 0 && cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4502, 0xFE) == 0 && !cart_irq_pending());
        for (unsigned instruction = 0; instruction < 4; ++instruction)
            BOARD_CHECK(ffe_nop() == 0 && !cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFF) == 0 && !cart_irq_pending());
        BOARD_CHECK(ffe_nop() == 0 && cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4501, 0) == 0 && !cart_irq_pending());

        // Overflow on the fourth cycle of STA must count its write cycle.
        BOARD_CHECK(ffe_cpu_store(0x4502, 0xFC) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFF) == 0);
        write_mem(0x0300, 0x8D);
        write_mem(0x0301, 0);
        write_mem(0x0302, 4);
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 4 && cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4501, 0) == 0);

        // Replacing the counter low byte does not disable an armed timer.
        BOARD_CHECK(ffe_cpu_store(0x4502, 0) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFF) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4502, 0xFE) == 0 && !cart_irq_pending());
        BOARD_CHECK(ffe_nop() == 0 && cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4501, 0) == 0);

        BOARD_CHECK(ffe_cpu_store(0x4502, 0) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFE) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4014, 0) == 0 && !cart_irq_pending());
        write_mem(0x0300, 0xEA);
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());
        BOARD_CHECK(ffe_cpu_store(0x4501, 0) == 0);

        BOARD_CHECK(ffe_cpu_store(0x4502, 0xFA) == 0);
        BOARD_CHECK(ffe_cpu_store(0x4503, 0xFF) == 0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(cart_irq_pending());
        BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
        board_image_free(&image);
    }
    return 0;
}

static int test_ffe_ram_and_geometry(void) {
    static const unsigned mappers[] = {6, 8, 17};
    for (unsigned variant = 0; variant < 3; ++variant) {
        unsigned mapper = mappers[variant];
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, mapper, 0x20000, 0, false));
        BOARD_CHECK(board_image_load(&image) == 0);
        for (uint8_t bank = 0; bank < 4; ++bank) {
            ffe_chr_bank(mapper, bank);
            ppu_write(0x0123, (uint8_t)(0x70 + bank));
            ppu_write(0x1FFF, (uint8_t)(0x80 + bank));
        }
        for (uint8_t bank = 0; bank < 4; ++bank) {
            ffe_chr_bank(mapper, bank);
            BOARD_CHECK(ppu_read(0x0123) == 0x70 + bank && ppu_read(0x1FFF) == 0x80 + bank);
        }
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(ppu_read(0x0123) == 0x73);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(ppu_read(0x1FFF) == 0x83);
        image.data[0] = 0;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(ppu_read(0x0123) == 0x73);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, mapper, 0x14000, 0x2800, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        ffe_chr_bank(mapper, 3);
        BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x1FFF) == 1);
        if (mapper == 17) {
            write_mem(0x4504, 13);
            BOARD_CHECK(read_mem(0x8000) == 6 && read_mem(0xFFFF) == 19);
        } else {
            write_mem(0x8000, mapper == 6 ? 0x2C : 0x58);
            BOARD_CHECK(read_mem(0x8000) == 4 && read_mem(0xBFFF) == 7);
        }
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, mapper, 0x1800, 0x100, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0x9700) == 1);
        write_mem(0x4800, 0xDA);
        BOARD_CHECK(read_mem(0xF800) == 0xDA);
        ffe_chr_bank(mapper, 0);
        BOARD_CHECK(ppu_read(0x0700) == 0 && ppu_read(0x0801) == 1);
        board_image_free(&image);
    }
    return 0;
}

static int test_ffe_battery(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 17, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70; // 8 KiB persistent PRG RAM.
    image.data[11] = 0x90; // 32 KiB persistent CHR RAM.
    BOARD_CHECK(board_image_add_trainer(&image, 0x14));
    char path[128], save[128], chr_save[136];
    snprintf(path, sizeof(path), "build/board-ffe-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    memcpy(chr_save, path, strlen(path) + 1);
    strcpy(strrchr(chr_save, '.'), ".chr.sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x14);
    write_mem(0x7000, 0x72);
    for (uint8_t bank = 0; bank < 4; ++bank) {
        ffe_chr_bank(17, bank);
        ppu_write(0x1234, (uint8_t)(0x84 + bank));
    }
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x72);
    for (uint8_t bank = 0; bank < 4; ++bank) {
        ffe_chr_bank(17, bank);
        BOARD_CHECK(ppu_read(0x1234) == 0x84 + bank);
    }
    BOARD_CHECK(unload_rom());
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x8000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(chr_save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_ffe_accuracy(void) {
    int failures = 0;
    failures += test_ffe_bank_registers();
    failures += test_ffe_irq();
    failures += test_ffe_ram_and_geometry();
    failures += test_ffe_battery();
    unload_rom();
    printf("Front Fareast accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
