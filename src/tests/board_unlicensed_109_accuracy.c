/*
 * board_unlicensed_109_accuracy.c - Unlicensed board regressions for issue 109
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

static void clock_mapper(unsigned cycles) {
    for (unsigned i = 0; i < cycles; ++i) cart_clock_cpu_cycle(false);
}

static int cpu_store_109(uint16_t address, uint8_t value) {
    const uint8_t program[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(program); ++byte)
        write_mem((uint16_t)(0x0200 + byte), program[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int cpu_nops_109(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned instruction = 0; instruction < count; ++instruction) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int mirroring_109(Mirroring expected) {
    BOARD_CHECK(cart_get_mirroring() == expected);
    ppu_write(0x2000, 0x12);
    ppu_write(expected == MIRROR_HORIZONTAL ? 0x2800 : 0x2400, 0x34);
    BOARD_CHECK(ppu_read(expected == MIRROR_HORIZONTAL ? 0x2400 : 0x2800) == 0x12);
    BOARD_CHECK(ppu_read(0x2C00) == 0x34 && ppu_read(0x3000) == 0x12);
    return 0;
}

static uint32_t crc32_109(const uint8_t *bytes, size_t size) {
    uint32_t crc = UINT32_MAX;
    for (size_t byte = 0; byte < size; ++byte) {
        crc ^= bytes[byte];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc ^ UINT32_MAX;
}

// Construct synthetic EPROM bytes with the board-layout checksum. No game ROM is needed.
static bool make_eprom_109(uint8_t *bytes, size_t size) {
    if (size < 4) return false;
    uint32_t base = crc32_109(bytes, size);
    uint32_t basis[32] = {0}, patches[32] = {0};
    for (unsigned bit = 0; bit < 32; ++bit) {
        size_t index = size - 4 + bit / 8;
        uint8_t mask = (uint8_t)(1u << (bit & 7));
        bytes[index] ^= mask;
        uint32_t delta = crc32_109(bytes, size) ^ base;
        bytes[index] ^= mask;
        uint32_t patch = 1u << bit;
        for (int pivot = 31; pivot >= 0; --pivot) {
            if (!(delta & (1u << pivot))) continue;
            if (basis[pivot]) { delta ^= basis[pivot]; patch ^= patches[pivot]; }
            else { basis[pivot] = delta; patches[pivot] = patch; break; }
        }
    }
    uint32_t needed = base ^ 0x63794E25u, patch = 0;
    for (int pivot = 31; pivot >= 0; --pivot) {
        if (!(needed & (1u << pivot))) continue;
        if (!basis[pivot]) return false;
        needed ^= basis[pivot];
        patch ^= patches[pivot];
    }
    for (unsigned byte = 0; byte < 4; ++byte)
        bytes[size - 4 + byte] ^= (uint8_t)(patch >> (byte * 8));
    return crc32_109(bytes, size) == 0x63794E25u;
}

static int test_109_address_and_value_banks(void) {
    BoardImage image;

    BOARD_CHECK(board_image_create(&image, 38, 0x20000, 0x8000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
    write_mem(0x7000, 0x0D);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 24);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 39, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 2);
    BOARD_CHECK(read_mem(0x8000) == 16);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 54, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8003, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 24);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 58, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x80D5, 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20);
    BOARD_CHECK(ppu_read(0) == 16 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_109_register_banks_and_mirroring(void) {
    BoardImage image;

    BOARD_CHECK(board_image_create(&image, 46, 0x100000, 0x100000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0xA3);
    write_mem(0x8000, 0x51);
    BOARD_CHECK(read_mem(0x8000) == 56 && ppu_read(0) == 0xA8);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 51, 0x100000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6000) == 70 && cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0xC000, 0x1A);
    BOARD_CHECK(read_mem(0x8000) == 80 && read_mem(0x6000) == 86);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 57, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 0x40);
    write_mem(0x8800, 0xB8);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20);
    BOARD_CHECK(ppu_read(0) == 64 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_109_irq_boards(void) {
    BoardImage image;

    BOARD_CHECK(board_image_create(&image, 42, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xE000) == 30);
    write_mem(0xE000, 5);
    write_mem(0x8000, 3);
    write_mem(0xE001, 8);
    BOARD_CHECK(read_mem(0x6000) == 10 && ppu_read(0) == 24);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && !cart_irq_pending());
    write_mem(0xE002, 2);
    clock_mapper(0x5FFF);
    BOARD_CHECK(!cart_irq_pending());
    clock_mapper(1);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xE002, 0);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 43, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x5000) == 16 && read_mem(0x6000) == 4);
    write_mem(0x4022, 6);
    BOARD_CHECK(read_mem(0xC000) == 14);
    write_mem(0x4120, 1);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0xE000) == 16);
    write_mem(0x4122, 1);
    clock_mapper(4095);
    BOARD_CHECK(!cart_irq_pending());
    clock_mapper(1);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0x8122, 0);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 50, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6000) == 30 && read_mem(0x8000) == 16);
    write_mem(0x4020, 0x0D);
    BOARD_CHECK(read_mem(0xC000) == 28);
    write_mem(0x4120, 1);
    clock_mapper(0x0FFF);
    BOARD_CHECK(!cart_irq_pending());
    clock_mapper(1);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0x4120, 0);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_109_supervision_and_dips(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 53, 0x220000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xA000) == 2);
    write_mem(0x6000, 0x30);
    write_mem(0x8000, 3);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xC000) == 28);
    BOARD_CHECK(read_mem(0x6000) == 30 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 59, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_dip_switches(3));
    write_mem(0x818B, 0);
    BOARD_CHECK(read_mem(0x8000) == 3 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x8082, 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 16);
    BOARD_CHECK(cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_109_transaction_and_save(void) {
    BoardImage active, replacement;
    BOARD_CHECK(board_image_create(&active, 42, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&active) == 0);
    write_mem(0xE000, 7);
    BOARD_CHECK(read_mem(0x6000) == 14);
    BOARD_CHECK(board_image_create(&replacement, 39, 0x20000, 0x2000, false));
    BOARD_CHECK(load_rom_memory(replacement.data, replacement.size - 1) < 0);
    BOARD_CHECK(read_mem(0x6000) == 14);
    board_image_free(&replacement);
    board_image_free(&active);

    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 39, 0x20000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0x6C));
    char path[128], save[128];
    unsigned long stamp = (unsigned long)time(NULL);
    snprintf(path, sizeof(path), "build/board-unlicensed109-%lu-%lu.nes", stamp, (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    BOARD_CHECK(fwrite(image.data, 1, image.size, file) == image.size && fclose(file) == 0);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x6C);
    write_mem(0x7000, 0xA5);
    write_mem(0x6123, 0x5A);
    cart_battery_flush();
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0xA5 && read_mem(0x6123) == 0x5A);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_109_register_aliases_and_reset(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 38, 0x20000, 0x8000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0x6FFF, 0x0D) == 0 && read_mem(0x8000) == 0 && read_mem(0x6FFF) == 0x0D);
    BOARD_CHECK(cpu_store_109(0x7FFF, 0xFE) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xFFFF) == 23);
    BOARD_CHECK(ppu_read(0) == 24 && ppu_read(0x1FFF) == 31);
    BOARD_CHECK(cpu_store_109(0x8000, 0) == 0 && read_mem(0x8000) == 16 && ppu_read(0) == 24);
    write_mem(0x7000, 0xFC);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 24 && read_mem(0x7000) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 24);
    BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 39, 0x60000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0xFFFF, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xFFFF) == 31);
    ppu_write(0, 0xEF);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1FFF) == 7);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 46, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0x7FFF, 0xA3) == 0 && cpu_store_109(0xFFFF, 0x51) == 0);
    BOARD_CHECK(read_mem(0x8000) == 56 && read_mem(0xFFFF) == 63);
    BOARD_CHECK(ppu_read(0) == 0xA8 && ppu_read(1) == 2 && ppu_read(0x1FFF) == 0xAF);
    write_mem(0x8000, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 56 && ppu_read(0) == 0xB8);
    write_mem(0x5FFF, 0);
    BOARD_CHECK(read_mem(0x8000) == 56 && ppu_read(0) == 0xB8);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 54, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0xFFF6, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xFFFF) == 23 && ppu_read(0) == 48);
    write_mem(0xFFF6, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 48);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 48);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 57, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0xF7FF, 0x45) == 0 && cpu_store_109(0xFFFF, 0xA8) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 104);
    BOARD_CHECK(mirroring_109(MIRROR_HORIZONTAL) == 0);
    write_mem(0x8800, 0xF0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28);
    BOARD_CHECK(mirroring_109(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 104);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 58, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
    BOARD_CHECK(cpu_store_109(0x80D5, 0) == 0 && mirroring_109(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 16);
    BOARD_CHECK(cpu_store_109(0xFFAD, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
    write_mem(0x802F, 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28 && mirroring_109(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 40);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 59, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_set_dip_switches(2));
    BOARD_CHECK(cpu_store_109(0x81DB, 0xFF) == 0 && mirroring_109(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 2 && read_mem(0xFFFF) == 2 && ppu_read(0) == 24);
    write_mem(0x4018, 0xFC);
    BOARD_CHECK(read_mem(0xB123) == 2); // The DIP pins replace all eight CPU data bits.
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 2 && ppu_read(0) == 24);
    write_mem(0x80D3, 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && mirroring_109(MIRROR_VERTICAL) == 0);
    write_mem(0x8066, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28 && ppu_read(0) == 48);
    BOARD_CHECK(cart_set_dip_switches(0));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);
    return 0;
}

static int test_109_real_cpu_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 42, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0xFFFC, 7) == 0 && read_mem(0x6000) == 14);
    BOARD_CHECK(cpu_store_109(0x9FFC, 0xF3) == 0 && ppu_read(0) == 24);
    BOARD_CHECK(cpu_store_109(0xFFFD, 0xFF) == 0 && mirroring_109(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(cpu_store_109(0xFFFE, 2) == 0);
    BOARD_CHECK(cpu_nops_109(0x2FFF) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(0x0FFF) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(0x3000) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0xE002, 0x82) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(0x4000) == 0 && !cart_irq_pending());
    BOARD_CHECK(read_mem(0x6000) == 14 && read_mem(0x8000) == 24 && read_mem(0xFFFF) == 31);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 14 && ppu_read(0) == 24);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 42, 0x20000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0123, 0xA5);
    BOARD_CHECK(cpu_store_109(0x8000, 0x0F) == 0 && ppu_read(0x0123) == 0xA5);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 43, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_store_109(0x4E22, 4) == 0 && read_mem(0xC000) == 12);
    BOARD_CHECK(cpu_store_109(0x4F20, 1) == 0 && read_mem(0x6000) == 0 && read_mem(0xE000) == 16);
    BOARD_CHECK(cpu_store_109(0x8F22, 1) == 0);
    BOARD_CHECK(cpu_nops_109(2047) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(4096) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4F22, 1) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(2047) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4122, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(read_mem(0x5000) == 16 && read_mem(0x5FFF) == 16);
    BOARD_CHECK(read_mem(0x8000) == 2 && read_mem(0xA000) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0xC000) == 12 && read_mem(0xE000) == 16);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 50, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0xC000, 0xA5) == 0xA5);
    BOARD_CHECK(cpu_store_109(0x5EFF, 0x0D) == 0 && read_mem(0xC000) == 28);
    BOARD_CHECK(cpu_store_109(0x5FFF, 1) == 0);
    BOARD_CHECK(cpu_nops_109(2047) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4120, 1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4120, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4120, 1) == 0);
    BOARD_CHECK(cpu_nops_109(1000) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4120, 1) == 0); // Six CPU clocks pass; enabling again retains them.
    BOARD_CHECK(cpu_nops_109(1044) == 0 && !cart_irq_pending());
    BOARD_CHECK(cpu_nops_109(1) == 0 && cart_irq_pending());
    BOARD_CHECK(cpu_store_109(0x4120, 0) == 0 && !cart_irq_pending());
    write_mem(0x6123, 0x88);
    write_mem(0x8123, 0x99);
    BOARD_CHECK(read_mem(0x6123) == 30 && read_mem(0x8123) == 16);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xC000) == 28 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_109_multicart_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 51, 0x100000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0123, 0xA5);
    BOARD_CHECK(cpu_store_109(0x8000, 5) == 0);
    const uint8_t modes[] = {0, 2, 0x10, 0x12};
    const uint8_t cpu_banks[4][4] = {{40, 42, 60, 62}, {40, 42, 44, 46},
                                    {44, 46, 60, 62}, {40, 42, 44, 46}};
    for (unsigned mode = 0; mode < 4; ++mode) {
        BOARD_CHECK(cpu_store_109(0x7FFF, modes[mode]) == 0);
        for (unsigned slot = 0; slot < 4; ++slot) {
            uint16_t address = (uint16_t)(0x8000 + slot * 0x2000);
            BOARD_CHECK(read_mem(address) == cpu_banks[mode][slot]);
            BOARD_CHECK(read_mem((uint16_t)(address + 0x1FFF)) == cpu_banks[mode][slot] + 1);
        }
        BOARD_CHECK(read_mem(0x6000) == (mode & 1 ? 110 : 126));
        BOARD_CHECK(ppu_read(0x0123) == 0xA5);
        BOARD_CHECK(mirroring_109(mode == 3 ? MIRROR_HORIZONTAL : MIRROR_VERTICAL) == 0);
    }
    BOARD_CHECK(cpu_store_109(0xDFFF, 0x0A) == 0);
    BOARD_CHECK(read_mem(0x8000) == 80 && read_mem(0xC000) == 84 && read_mem(0x6000) == 86);
    BOARD_CHECK(mirroring_109(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(cpu_store_109(0xC000, 0x1A) == 0 && mirroring_109(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(cpu_store_109(0xE000, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 120 && read_mem(0x6000) == 126);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 120 && ppu_read(0x0123) == 0xA5);
    BOARD_CHECK(mirroring_109(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    for (unsigned first = 0; first < 2; ++first) {
        BOARD_CHECK(board_image_create(&image, 53, 0x220000, 0x2000, true));
        if (first) BOARD_CHECK(make_eprom_109(image.data + 16, 0x8000));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0x8001) == (first ? 0 : 2));
        BOARD_CHECK(read_mem(0xC000) == (first ? 4 : 0) && read_mem(0xC001) == (first ? 0 : 2));
        BOARD_CHECK(read_mem(0x6000) == (first ? 38 : 30));
        BOARD_CHECK(cpu_store_109(0x7FFF, 0x31) == 0 && cpu_store_109(0xFFFF, 3) == 0);
        BOARD_CHECK(read_mem(0x8000) == (first ? 52 : 44));
        BOARD_CHECK(read_mem(0xC000) == (first ? 68 : 60));
        BOARD_CHECK(read_mem(0x6000) == (first ? 70 : 62));
        BOARD_CHECK(mirroring_109(MIRROR_HORIZONTAL) == 0);
        BOARD_CHECK(cpu_store_109(0x8000, 0xFC) == 0);
        BOARD_CHECK(read_mem(0x8000) == (first ? 56 : 48));
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == (first ? 56 : 48) && read_mem(0xC000) == (first ? 68 : 60));
        write_mem(0x6000, 0x01);
        BOARD_CHECK(read_mem(0x8001) == (first ? 0 : 2));
        BOARD_CHECK(mirroring_109(MIRROR_VERTICAL) == 0 && ppu_read(0) == 0);
        board_image_free(&image);
    }
    return 0;
}

static int test_109_small_rom_geometry(void) {
    const unsigned ids[] = {38, 39, 42, 43, 46, 50, 51, 53, 54, 57, 58, 59};
    const uint16_t registers[] = {0x7FFF, 0xFFFF, 0xE000, 0x4022, 0x7FFF, 0x4020,
                                   0x8000, 0x7FFF, 0xFFFF, 0x8800, 0xFFFF, 0x80FF};
    for (unsigned index = 0; index < sizeof(ids) / sizeof(ids[0]); ++index) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0x600, true));
        image.data[8] |= 0xF0; // These boards do not select a different circuit by submapper.
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(cpu_store_109(registers[index], 0xFF) == 0);
        if (ids[index] == 53) write_mem(0x8000, 0xFF);
        // These two CHR-RAM multicart circuits never select a CHR-ROM page.
        uint8_t pattern = ids[index] == 51 || ids[index] == 53 ? 0x34 : 0xB8;
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0234) == pattern);
        BOARD_CHECK(ppu_read(0x0634) == 0x34);
        ppu_write(0x0234, 0xEE);
        BOARD_CHECK(ppu_read(0x0234) == pattern);
        uint8_t active = read_mem(0x8000);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == active && ppu_read(0x0234) == pattern);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == active && ppu_read(0x0234) == pattern);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0, true));
        image.data[11] = 3; // A 512-byte RAM chip retains its startup aliases outside selected slots.
        memset(image.data + 16, 0xA7, 0x6000);
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x0123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        BOARD_CHECK(cpu_store_109(registers[index], 0xFF) == 0);
        BOARD_CHECK(ppu_read(0x0123) == 0x5A && ppu_read(0x1F23) == 0x5A);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x0123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

int test_board_unlicensed_109_accuracy(void) {
    int failures = 0;
    failures += test_109_address_and_value_banks();
    failures += test_109_register_banks_and_mirroring();
    failures += test_109_irq_boards();
    failures += test_109_supervision_and_dips();
    failures += test_109_transaction_and_save();
    failures += test_109_register_aliases_and_reset();
    failures += test_109_real_cpu_irq();
    failures += test_109_multicart_modes();
    failures += test_109_small_rom_geometry();
    unload_rom();
    printf("Unlicensed issue 109: 9 groups, %d failures\n", failures);
    return failures;
}
