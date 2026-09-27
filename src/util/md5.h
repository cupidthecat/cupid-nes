/*
 * md5.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Movie ROM identity. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MD5_H
#define CUPID_MD5_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t state[4];
    uint64_t bytes;
    uint8_t block[64];
    size_t used;
} NesMd5;

void nes_md5_init(NesMd5 *hash);
bool nes_md5_update(NesMd5 *hash, const void *data, size_t size);
bool nes_md5_final(const NesMd5 *hash, uint8_t digest[16]);

#ifdef __cplusplus
}
#endif
#endif
