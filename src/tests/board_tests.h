/*
 * board_tests.h - Synthetic cartridge images for hardware regression tests
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_BOARD_TESTS_H
#define CUPID_BOARD_TESTS_H
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t *data;
    size_t size;
} BoardImage;

// PRG patterns identify 4KB pages; CHR patterns identify 1KB pages.
// Offset 1 in each page carries the high byte of its page number.
bool board_image_create(BoardImage *image, unsigned mapper, size_t prg_bytes,
                         size_t chr_bytes, bool nes20);
bool board_image_add_trainer(BoardImage *image, uint8_t fill);
int board_image_load(const BoardImage *image);
void board_image_free(BoardImage *image);

#define BOARD_CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

#endif
