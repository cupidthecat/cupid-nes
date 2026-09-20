/*
 * netplay_hash.h - Canonical hardware checkpoint hashing for netplay
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_NETPLAY_HASH_H
#define CUPID_NETPLAY_HASH_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t first;
    uint64_t second;
} NesNetplayHardwareHash;

bool nes_netplay_hardware_hash(NesNetplayHardwareHash *hash);
bool nes_netplay_hardware_hash_equal(const NesNetplayHardwareHash *left,
                                     const NesNetplayHardwareHash *right);

#endif
