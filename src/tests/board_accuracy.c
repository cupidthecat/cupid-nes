/*
 * board_accuracy.c - Cartridge board bus and loader regressions
 *
 * Author: @frankischilling
 *
 * These tests load generated images through the normal loader and exercise
 * banking, mirroring, open bus, reset signals, and persistent RAM.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../system/hardware.h"
#include <time.h>

static bool image_rom_size(size_t bytes, size_t unit, uint8_t *low, uint8_t *high) {
    if (!(bytes % unit) && bytes / unit <= 0xFFF) {
        *low = (uint8_t)(bytes / unit);
        *high = (uint8_t)((bytes / unit) >> 8);
        return true;
    }
    unsigned exponent = 0;
    size_t multiplier = bytes;
    while (multiplier && !(multiplier & 1)) { multiplier >>= 1; ++exponent; }
    if (!multiplier || multiplier > 7 || exponent > 63) return false;
    *low = (uint8_t)((exponent << 2) | ((multiplier - 1) / 2));
    *high = 15;
    return true;
}

bool board_image_create(BoardImage *image, unsigned mapper, size_t prg_bytes,
                         size_t chr_bytes, bool nes20) {
    if (!image || mapper > 0xFFF || prg_bytes > SIZE_MAX - sizeof(iNESHeader)
        || chr_bytes > SIZE_MAX - sizeof(iNESHeader) - prg_bytes) return false;
    *image = (BoardImage){0};
    iNESHeader header = {0};
    memcpy(header.signature, "NES\x1A", 4);
    header.flags6 = (uint8_t)(mapper << 4);
    header.flags7 = (uint8_t)(mapper & 0xF0);
    if (nes20) {
        uint8_t prg_high, chr_high;
        if (!image_rom_size(prg_bytes, 0x4000, &header.prg_rom_chunks, &prg_high)
            || !image_rom_size(chr_bytes, 0x2000, &header.chr_rom_chunks, &chr_high)) return false;
        header.flags7 |= 8;
        header.flags9 = (uint8_t)(prg_high | (chr_high << 4));
        header.prg_ram_size = (uint8_t)(mapper >> 8);
        if (!chr_bytes) header.zero[0] = 7;
    } else {
        if (mapper > 255 || (prg_bytes % 0x4000) || (chr_bytes % 0x2000)
            || prg_bytes / 0x4000 > 255 || chr_bytes / 0x2000 > 255) return false;
        header.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
        header.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    }
    image->size = sizeof(header) + prg_bytes + chr_bytes;
    image->data = (uint8_t *)malloc(image->size);
    if (!image->data) { image->size = 0; return false; }
    memcpy(image->data, &header, sizeof(header));
    uint8_t *prg = image->data + sizeof(header);
    uint8_t *chr = prg + prg_bytes;
    for (size_t i = 0; i < prg_bytes; ++i)
        prg[i] = (i & 0xFFF) == 1 ? (uint8_t)(i >> 20) : (uint8_t)(i >> 12);
    for (size_t i = 0; i < chr_bytes; ++i)
        chr[i] = (i & 0x3FF) == 1 ? (uint8_t)(i >> 18) : (uint8_t)(i >> 10);
    return true;
}

bool board_image_add_trainer(BoardImage *image, uint8_t fill) {
    if (!image || !image->data || image->size < sizeof(iNESHeader)
        || image->size > SIZE_MAX - 512 || (image->data[6] & 4)) return false;
    uint8_t *expanded = (uint8_t *)realloc(image->data, image->size + 512);
    if (!expanded) return false;
    memmove(expanded + sizeof(iNESHeader) + 512, expanded + sizeof(iNESHeader),
            image->size - sizeof(iNESHeader));
    memset(expanded + sizeof(iNESHeader), fill, 512);
    expanded[6] |= 4;
    image->data = expanded;
    image->size += 512;
    return true;
}

int board_image_load(const BoardImage *image) {
    int result = load_rom_memory(image->data, image->size);
    if (result == 0) {
        ppu_power_on(&ppu);
        if (!cpu_power_on(&cpu)) return -1;
    }
    return result;
}

void board_image_free(BoardImage *image) {
    if (!image) return;
    free(image->data);
    *image = (BoardImage){0};
}

static int test_bandai_discrete_banks(void) {
    const unsigned ids[] = {70, 152};
    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x20000, 0x20000, false));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 31);
        BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x1FFF) == 7);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        write_mem(0x6123, 0xAD);
        BOARD_CHECK(read_mem(0x6123) == 0xAD);
        ppu_write(0x2000, 0x12);
        ppu_write(0x2400, 0x34);
        BOARD_CHECK(ppu_read(0x2800) == 0x12 && ppu_read(0x2C00) == 0x34);
        // The current ROM drives zero here. A bus conflict would erase this write.
        write_mem(0x8000, 0x25);
        BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xC000) == 28);
        BOARD_CHECK(ppu_read(0x0000) == 40 && ppu_read(0x1FFF) == 47);
        BOARD_CHECK(cart_get_mirroring() == (ids[i] == 70 ? MIRROR_VERTICAL : MIRROR_SINGLE0));
        write_mem(0xA123, 0xA5);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
        ppu_write(0x2000, 0x56);
        BOARD_CHECK(ppu_read(0x2400) == 0x56 && ppu_read(0x3C00) == 0x56);
        write_mem(0xFFFF, 0x13);
        BOARD_CHECK(read_mem(0x8000) == 4 && read_mem(0xC000) == 28);
        BOARD_CHECK(ppu_read(0x0000) == 24 && cart_get_mirroring() == MIRROR_SINGLE0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == 4 && ppu_read(0x0000) == 24);
        BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0 && read_mem(0x6123) == 0xAD);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
        board_image_free(&image);
    }
    return 0;
}

static int test_bandai_discrete_page_sizes(void) {
    const unsigned ids[] = {70, 152};
    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0xC000, 0x6000, true));
        image.data[8] |= 0xF0; // Neither variant decodes a submapper value.
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0xC000) == 8 && read_mem(0xFFFF) == 11);
        write_mem(0x8000, 0x7F);
        BOARD_CHECK(read_mem(0x8000) == 4 && ppu_read(0x0000) == 0);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[i], 0x80000, 0x40000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        write_mem(0xFFFF, 0xFF);
        BOARD_CHECK(read_mem(0x8000) == 28 && read_mem(0xC000) == 124);
        BOARD_CHECK(ppu_read(0x0000) == 120);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[i], 0x2000, 0x1000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xBFFF) == 1);
        BOARD_CHECK(read_mem(0xC000) == 0 && read_mem(0xFFFF) == 1);
        BOARD_CHECK(ppu_read(0x0FFF) == 3 && ppu_read(0x1234) == 0x34);
        ppu_write(0x0000, 0xEE);
        BOARD_CHECK(ppu_read(0x0000) == 0);
        write_mem(0x5000, 0xA6);
        BOARD_CHECK(read_mem(0x6000) == 0xA6);
        board_image_free(&image);
    }
    return 0;
}

static int test_bandai_discrete_failed_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 70, 0x20000, 0x8000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 0x31);
    write_mem(0x7123, 0x89);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0x0000) == 8 && read_mem(0x7123) == 0x89);
    image.data[7] |= 2; // PlayChoice is a different console.
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0x7123) == 0x89);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0x7123) == 0x89);
    board_image_free(&image);
    return 0;
}

static int test_bandai_discrete_saves(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 152, 0x20000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0x44));
    char path[128], save[128];
    unsigned long stamp = (unsigned long)time(NULL);
    snprintf(path, sizeof(path), "build/board-bandai-%lu-%lu.nes", stamp, (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x44 && read_mem(0x71FF) == 0x44);
    write_mem(0x7000, 0x96);
    write_mem(0x6123, 0xB4);
    ppu_write(0x0456, 0x71);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x7000) == 0x96 && ppu_read(0x0456) == 0x71);
    cart_battery_flush();
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x96 && read_mem(0x6123) == 0xB4);
    BOARD_CHECK(ppu_read(0x0456) == 0); // Volatile CHR RAM is not part of the save.
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x2000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

static void karaoke_make_register_writes_visible(BoardImage *image, size_t prg_bytes) {
    if (!image || !image->data || image->size < sizeof(iNESHeader)) return;
    const iNESHeader *header = (const iNESHeader *)image->data;
    uint8_t *prg = image->data + sizeof(iNESHeader) + ((header->flags6 & 4) ? 512 : 0);
    for (size_t bank = 0; bank + 0x4000 <= prg_bytes; bank += 0x4000)
        prg[bank + 0x3FFF] = 0xFF;
    if (prg_bytes >= 0x4000) prg[0x3FFE] = 0x13;
}

static int test_bandai_karaoke_banking_and_input(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 188, 0x20000, 0, true));
    karaoke_make_register_writes_visible(&image, 0x20000);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
    ppu_write(0x0123, 0xA6);
    BOARD_CHECK(ppu_read(0x0123) == 0xA6);

    // ROM bus conflicts mask register data before the board sees it.
    write_mem(0xBFFE, 0x3F);
    BOARD_CHECK(read_mem(0x8000) == 12 && cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0xBFFF, 0x35);
    BOARD_CHECK(read_mem(0x8000) == 20 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xBFFF, 0x15);
    BOARD_CHECK(read_mem(0x8000) == 20 && cart_get_mirroring() == MIRROR_VERTICAL);

    // With less than 256 KiB of PRG, selecting the optional expansion ROM
    // disconnects only the lower window. The fixed internal bank remains mapped.
    write_mem(0xBFFF, 0x02);
    write_mem(0x4018, 0xA6);
    BOARD_CHECK(read_mem(0x8000) == 0xA6 && read_mem(0xBFFF) == 0xA6);
    BOARD_CHECK(read_mem(0xC000) == 28);
    write_mem(0xBFFF, 0x17); // An unmapped window cannot create a ROM bus conflict.
    BOARD_CHECK(read_mem(0x8000) == 28 && read_mem(0xC000) == 28);

    BOARD_CHECK(cart_set_karaoke_input(CART_KARAOKE_A, true));
    BOARD_CHECK(cart_set_karaoke_input(CART_KARAOKE_B, true));
    BOARD_CHECK(cart_set_karaoke_input(CART_KARAOKE_MICROPHONE, true));
    write_mem(0x4018, 0xA8);
    BOARD_CHECK(ppu.frame_count == 0 && read_mem(0x6000) == 0xAC);
    start_frame();
    for (unsigned clocks = 0; clocks < 40000 && !ppu.frame_complete; ++clocks) ppu_step(1);
    BOARD_CHECK(ppu.frame_complete && ppu.frame_count == 1);
    write_mem(0x4018, 0xA8);
    BOARD_CHECK(read_mem(0x7FFF) == 0xA8);
    start_frame();
    for (unsigned clocks = 0; clocks < 40000 && !ppu.frame_complete; ++clocks) ppu_step(1);
    BOARD_CHECK(ppu.frame_complete && ppu.frame_count == 2);
    write_mem(0x4018, 0xA8);
    BOARD_CHECK(read_mem(0x6000) == 0xAC);

    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA8);
    BOARD_CHECK(read_mem(0x6000) == 0xAB); // A replacement starts with released mapper inputs.
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 188, 0x50000, 0x3000, true));
    image.data[8] |= 0xF0; // Nonzero submappers keep the same board behavior.
    karaoke_make_register_writes_visible(&image, 0x50000);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xC000) == 28);
    write_mem(0xBFFF, 0x03);
    BOARD_CHECK(read_mem(0x8000) == 44 && read_mem(0xC000) == 28);
    write_mem(0xBFFF, 0x07);
    BOARD_CHECK(read_mem(0x8000) == 60);
    write_mem(0xBFFF, 0x11);
    BOARD_CHECK(read_mem(0x8000) == 4);
    BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x1FFF) == 7);
    ppu_write(0x0123, 0xE7);
    BOARD_CHECK(ppu_read(0x0123) == 0); // CHR ROM remains read-only.
    board_image_free(&image);
    return 0;
}

static int test_bandai_karaoke_page_sizes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 188, 0xC000, 0x1000, true));
    image.data[8] |= 0xA0;
    karaoke_make_register_writes_visible(&image, 0xC000);
    BOARD_CHECK(board_image_load(&image) == 0);
    // Bank seven wraps across the three complete 16 KiB pages.
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
    write_mem(0xBFFF, 0x17);
    BOARD_CHECK(read_mem(0x8000) == 4 && read_mem(0xC000) == 4);
    BOARD_CHECK(ppu_read(0x0FFF) == 3 && ppu_read(0x1234) == 0x34);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 188, 0x5000, 0, true));
    image.data[11] = 6; // 4 KiB CHR RAM uses the shared small-page mapping.
    karaoke_make_register_writes_visible(&image, 0x5000);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xBFFF) == 0xFF);
    BOARD_CHECK(read_mem(0xC000) == 0 && read_mem(0xFFFF) == 0xFF);
    ppu_write(0x0234, 0xD6);
    BOARD_CHECK(ppu_read(0x0234) == 0xD6 && ppu_read(0x1234) == 0xD6);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 188, 0x2000, 0, true));
    karaoke_make_register_writes_visible(&image, 0x2000);
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned window = 0; window < 4; ++window) {
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + window * 0x2000)) == 0);
        BOARD_CHECK(read_mem((uint16_t)(0x9FFF + window * 0x2000)) == 1);
    }
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 188, 0x80000, 0x2000, true));
    image.data[11] = 0x07; // CHR ROM plus separately declared volatile CHR RAM is accepted.
    karaoke_make_register_writes_visible(&image, 0x80000);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xBFFF, 0x07);
    BOARD_CHECK(read_mem(0x8000) == 60 && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

static int file_byte_at(const char *path, long offset) {
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    if (fseek(file, offset, SEEK_SET) != 0) { fclose(file); return -1; }
    int value = fgetc(file);
    fclose(file);
    return value;
}

static int test_bandai_karaoke_saves(void) {
    for (unsigned nes20 = 0; nes20 < 2; ++nes20) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 188, 0x20000, 0x2000, nes20 != 0));
        image.data[6] |= 2;
        if (nes20) image.data[10] = 0x70; // Explicit 8 KiB battery-backed PRG RAM.
        karaoke_make_register_writes_visible(&image, 0x20000);
        char path[128], save[128];
        unsigned long stamp = (unsigned long)time(NULL);
        snprintf(path, sizeof(path), "build/board-karaoke-%u-%lu-%lu.nes",
                 nes20, stamp, (unsigned long)clock());
        memcpy(save, path, strlen(path) + 1);
        strcpy(strrchr(save, '.'), ".sav");
        FILE *file = fopen(path, "wb");
        BOARD_CHECK(file != NULL);
        size_t count = fwrite(image.data, 1, image.size, file);
        int closed = fclose(file);
        BOARD_CHECK(count == image.size && closed == 0);
        BOARD_CHECK(load_rom(path) == 0);
        write_mem(0x6123, (uint8_t)(0xB4 + nes20));
        write_mem(0x7FFF, (uint8_t)(0xD5 + nes20));
        write_mem(0x4018, 0xA8);
        BOARD_CHECK(read_mem(0x6123) == 0xAB && read_mem(0x7FFF) == 0xAB);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(file_byte_at(save, 0x123) == 0xB4 + (int)nes20);
        BOARD_CHECK(file_byte_at(save, 0x1FFF) == 0xD5 + (int)nes20);
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
        long length = ftell(file);
        closed = fclose(file);
        BOARD_CHECK(length == 0x2000 && closed == 0);
        BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
        board_image_free(&image);
    }
    return 0;
}

static int test_board_power_on_ram_case(unsigned mapper) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, mapper, 0x8000, 0, false));
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ONES));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6123) == 0xFF && ppu_read(0x1234) == 0xFF);
    BOARD_CHECK(ppu_read(0x2222) == 0xFF && read_mem(0x7000) == 0x9A);
    if (mapper == 5) {
        write_mem(0x5102, 2);
        write_mem(0x5103, 1);
    }
    write_mem(0x6123, 0x34);
    ppu_write(0x1234, 0x56);
    ppu_write(0x2222, 0x78);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6123) == 0x34 && ppu_read(0x1234) == 0x56 && ppu_read(0x2222) == 0x78);
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6123) == 0 && ppu_read(0x1234) == 0);
    BOARD_CHECK(ppu_read(0x2222) == 0 && read_mem(0x7000) == 0x9A);
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_RANDOM));
    nes_seed_power_on_random(0x12345678);
    BOARD_CHECK(board_image_load(&image) == 0);
    uint8_t expected[768];
    for (unsigned i = 0; i < 256; ++i) {
        expected[i] = read_mem((uint16_t)(0x6000 + i));
        expected[i + 256] = ppu_read((uint16_t)i);
        expected[i + 512] = ppu_read((uint16_t)(0x2000 + i));
    }
    nes_seed_power_on_random(0x12345678);
    BOARD_CHECK(board_image_load(&image) == 0);
    bool varied = false;
    for (unsigned i = 0; i < 256; ++i) {
        BOARD_CHECK(read_mem((uint16_t)(0x6000 + i)) == expected[i]);
        BOARD_CHECK(ppu_read((uint16_t)i) == expected[i + 256]);
        BOARD_CHECK(ppu_read((uint16_t)(0x2000 + i)) == expected[i + 512]);
        varied |= expected[i] != expected[0];
    }
    BOARD_CHECK(varied && read_mem(0x7000) == 0x9A);
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    board_image_free(&image);
    return 0;
}

static int test_board_power_on_ram(void) {
    const unsigned mappers[] = {0, 1, 5, 155, 70};
    int failures = 0;
    for (unsigned board = 0; board < sizeof(mappers) / sizeof(mappers[0]); ++board) {
        failures += test_board_power_on_ram_case(mappers[board]);
        BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    }
    return failures;
}

int test_board_accuracy(void) {
    int failures = 0;
    failures += test_bandai_discrete_banks();
    failures += test_bandai_discrete_page_sizes();
    failures += test_bandai_discrete_failed_replacement();
    failures += test_bandai_discrete_saves();
    failures += test_bandai_karaoke_banking_and_input();
    failures += test_bandai_karaoke_page_sizes();
    failures += test_bandai_karaoke_saves();
    failures += test_board_power_on_ram();
    unload_rom();
    printf("Board accuracy: 8 groups, %d failures\n", failures);
    return failures;
}
