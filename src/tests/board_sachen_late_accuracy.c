/*
 * board_sachen_late_accuracy.c - Sachen 243 and 9602 cartridge regressions
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

static bool sachen_prg8(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 2)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 7);
}

static int sachen_late_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int sachen_late_cycles(unsigned cycles) {
    if (cycles == 2) write_mem(0x0200, 0xEA);
    else {
        write_mem(0x0200, 0x4C);
        write_mem(0x0201, 0);
        write_mem(0x0202, 2);
    }
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == (int)cycles);
    return 0;
}

static void sachen243_reg(uint8_t reg, uint8_t value) {
    write_mem(0x4100, reg);
    write_mem(0x4101, value);
}

static int test_sachen243(void) {
    BoardImage image;
    BOARD_CHECK(cart_set_dip_switches(0));
    BOARD_CHECK(board_image_create(&image, 243, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x2000, 0x31);
    ppu_write(0x2C00, 0x42);
    BOARD_CHECK(ppu_read(0x2400) == 0x31 && ppu_read(0x2800) == 0x31 && ppu_read(0x2C00) == 0x42);
    sachen243_reg(2, 1);
    BOARD_CHECK(ppu_read(0) == 8);
    sachen243_reg(4, 1);
    BOARD_CHECK(ppu_read(0) == 24);
    sachen243_reg(6, 2);
    sachen243_reg(5, 3);
    BOARD_CHECK(ppu_read(0) == 88 && ppu_read(0x1FFF) == 95 && read_mem(0x8000) == 24);
    BOARD_CHECK(cart_set_dip_switches(1));
    write_mem(0x4100, 1);
    BOARD_CHECK(sachen_late_store(0x4101, 2) == 0);
    write_mem(0x4018, 0xD0);
    BOARD_CHECK(read_mem(0x4101) == 0xD2 && read_mem(0x8000) == 24 && ppu_read(0) == 88);
    write_mem(0x4018, 0xA7);
    BOARD_CHECK(read_mem(0x4100) == 0xA7 && read_mem(0x6000) == 0xA7);
    sachen243_reg(7, 4);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x7FFE, 6);
    write_mem(0x7FFF, 3);
    BOARD_CHECK(ppu_read(0) == 120 && read_mem(0x8000) == 24);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0) == 120 && read_mem(0x8000) == 24);
    BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_sachen513_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 513, 0x200000, 0, true));
    image.data[10] = 7;
    image.data[11] = 9;
    BOARD_CHECK(board_image_add_trainer(&image, 0x66));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(sachen_prg8(0x8000, 0) && sachen_prg8(0xA000, 1));
    BOARD_CHECK(sachen_prg8(0xC000, 62) && sachen_prg8(0xE000, 63));
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x7000) == 0xA5);
    write_mem(0xA001, 0x80);
    BOARD_CHECK(read_mem(0x7000) == 0x66);
    write_mem(0x6123, 0xAB);
    write_mem(0xA001, 0xC0);
    write_mem(0x6123, 0xCD);
    BOARD_CHECK(read_mem(0x6123) == 0xAB);
    write_mem(0x8002, 2);
    BOARD_CHECK(sachen_late_store(0x8003, 0xC7) == 0);
    ppu_write(0x1023, 0xE7);
    BOARD_CHECK(sachen_prg8(0x8000, 192) && sachen_prg8(0xA000, 193));
    BOARD_CHECK(sachen_prg8(0xC000, 62) && sachen_prg8(0xE000, 63));
    write_mem(0x8000, 6);
    write_mem(0x8001, 0xE5);
    BOARD_CHECK(sachen_prg8(0x8000, 229));
    write_mem(0x8000, 0x46);
    BOARD_CHECK(sachen_prg8(0x8000, 62) && sachen_prg8(0xC000, 229));
    write_mem(0x8000, 0x40);
    write_mem(0x8001, 0x83);
    BOARD_CHECK(sachen_prg8(0x8000, 62) && sachen_prg8(0xC000, 165));
    ppu_write(0x0023, 0x32);
    ppu_write(0x0423, 0x43);
    write_mem(0x8000, 0x82);
    BOARD_CHECK(ppu_read(0x0023) == 0xE7 && ppu_read(0x1023) == 0x32 && ppu_read(0x1423) == 0x43);
    BOARD_CHECK(sachen_prg8(0x8000, 165) && sachen_prg8(0xC000, 62));
    write_mem(0x8001, 0x1F);
    ppu_write(0x0023, 0xF1);
    write_mem(0x8001, 0);
    ppu_write(0x0023, 0x02);
    write_mem(0x8001, 0x1F);
    BOARD_CHECK(ppu_read(0x0023) == 0xF1 && sachen_prg8(0x8000, 37));
    write_mem(0xA000, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0x0023) == 0xF1 && sachen_prg8(0x8000, 37));
    BOARD_CHECK(read_mem(0x6123) == 0xAB && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(board_image_load(&image) == 0 && sachen_prg8(0x8000, 0) && ppu_read(0x0023) == 0);
    board_image_free(&image);
    return 0;
}

static int test_sachen513_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 513, 0x80000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_mmc3_revision_name("standard"));
    cart_notify_ppu_address(0x1000, 1);
    write_mem(0xC000, 1);
    write_mem(0xC001, 0);
    write_mem(0xE001, 0);
    cart_notify_ppu_address(0, 2);
    BOARD_CHECK(sachen_late_cycles(2) == 0);
    cart_notify_ppu_address(0x1000, 1000);
    BOARD_CHECK(!cart_irq_pending());
    for (unsigned edge = 0; edge < 2; ++edge) {
        cart_notify_ppu_address(0, 1001 + edge * 20);
        BOARD_CHECK(sachen_late_cycles(3) == 0);
        cart_notify_ppu_address(0x1000, 1011 + edge * 20);
        BOARD_CHECK(cart_irq_pending() == (edge == 1));
    }
    BOARD_CHECK(sachen_late_store(0xE000, 0) == 0 && !cart_irq_pending());
    write_mem(0xC000, 0);
    write_mem(0xC001, 0);
    write_mem(0xE001, 0);
    cart_notify_ppu_address(0, 1200);
    cpu_soft_reset(&cpu);
    cart_notify_ppu_address(0x1000, 1221);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_sachen513_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 513, 0x6000, 0x2800, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(sachen_prg8(0x8000, 0) && sachen_prg8(0xA000, 1));
    BOARD_CHECK(sachen_prg8(0xC000, 2) && sachen_prg8(0xE000, 0));
    write_mem(0x8000, 6);
    write_mem(0x8001, 5);
    write_mem(0x8000, 2);
    write_mem(0x8001, 0xDF);
    BOARD_CHECK(sachen_prg8(0x8000, 2) && ppu_read(0x1000) == 1);
    ppu_write(0x1000, 0xE7);
    BOARD_CHECK(ppu_read(0x1000) == 1);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(sachen_prg8(0x8000, 2) && ppu_read(0x1000) == 1);
    image.data[7] |= 2;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(sachen_prg8(0x8000, 2) && ppu_read(0x1000) == 1);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && sachen_prg8(0x8000, 2));
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 513, 0x2000, 0x100, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xFFFF) == 1 && ppu_read(0x0010) == 0 && ppu_read(0x0810) == 0x10);
    write_mem(0x8000, 2);
    write_mem(0x8001, 31);
    BOARD_CHECK(ppu_read(0x0410) == 0 && ppu_read(0x1010) == 0x10);
    board_image_free(&image);
    return 0;
}

static int test_sachen513_persistence(void) {
    const uint8_t layouts[] = {9, 0x77};
    for (unsigned layout = 0; layout < sizeof(layouts); ++layout) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 513, 0x20000, 0, true));
        image.data[11] = layouts[layout];
        size_t bytes = layout == 0 ? 0x8000 : 0x4000;
        char path[128], save[136];
        snprintf(path, sizeof(path), "build/board-sachen513-%u-%lu-%lu.nes", layout,
                 (unsigned long)time(NULL), (unsigned long)clock());
        memcpy(save, path, strlen(path) + 1);
        strcpy(strrchr(save, '.'), ".chr.sav");
        FILE *file = fopen(path, "wb");
        BOARD_CHECK(file != NULL);
        size_t written = fwrite(image.data, 1, image.size, file);
        int closed = fclose(file);
        BOARD_CHECK(written == image.size && closed == 0);
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        const uint8_t banks[] = {0, (uint8_t)(bytes / 0x800), (uint8_t)(bytes / 0x400 - 1)};
        for (unsigned bank = 0; bank < sizeof(banks); ++bank) {
            write_mem(0x8000, 2);
            write_mem(0x8001, banks[bank]);
            ppu_write(0x1023, (uint8_t)(0x91 + bank));
        }
        BOARD_CHECK(unload_rom());
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
        long length = ftell(file);
        BOARD_CHECK(length == (long)bytes);
        for (unsigned bank = 0; bank < sizeof(banks); ++bank) {
            BOARD_CHECK(fseek(file, banks[bank] * 0x400 + 0x23, SEEK_SET) == 0);
            BOARD_CHECK(fgetc(file) == (int)(0x91 + bank));
        }
        BOARD_CHECK(fclose(file) == 0 && load_rom(path) == 0);
        for (unsigned bank = 0; bank < sizeof(banks); ++bank) {
            write_mem(0x8000, 2);
            write_mem(0x8001, banks[bank]);
            BOARD_CHECK(ppu_read(0x1023) == (uint8_t)(0x91 + bank));
        }
        BOARD_CHECK(unload_rom() && remove(path) == 0 && remove(save) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_board_sachen_late_accuracy(void) {
    int failures = 0;
    failures += test_sachen243();
    cart_set_dip_switches(0);
    failures += test_sachen513_banks();
    failures += test_sachen513_irq();
    failures += test_sachen513_geometry();
    failures += test_sachen513_persistence();
    unload_rom();
    printf("Sachen 243/9602 accuracy: 5 groups, %d failures\n", failures);
    return failures;
}
