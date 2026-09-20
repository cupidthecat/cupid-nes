/*
 * netplay_hash.c - Canonical hardware checkpoint hashing for netplay
 *
 * Author: @frankischilling
 *
 * Host audio consumers, resampling/filter state, and presentation snapshots
 * are deliberately excluded from this digest.
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "netplay_hash.h"
#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../cpu/cpu.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../joypad/special_peripherals.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/mapper_state.h"
#include "../state/state_io.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"

#include <stddef.h>

typedef bool (*NetplayCapture)(NesStateWriter *writer);

static void hash_bytes(NesNetplayHardwareHash *hash, const void *data, size_t size) {
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < size; ++i) {
        hash->first = (hash->first ^ bytes[i]) * UINT64_C(1099511628211);
        hash->second ^= bytes[i] + UINT64_C(0x9e3779b97f4a7c15)
                      + (hash->second << 6) + (hash->second >> 2);
    }
}

static bool hash_component(NesNetplayHardwareHash *hash, uint32_t tag,
                           NetplayCapture capture) {
    NesStateWriter writer;
    nes_state_writer_init(&writer, NES_STATE_MAX_SIZE);
    bool ok = capture(&writer) && !writer.failed;
    if (ok) {
        uint64_t size = writer.size;
        hash_bytes(hash, &tag, sizeof(tag));
        hash_bytes(hash, &size, sizeof(size));
        hash_bytes(hash, writer.data, writer.size);
    }
    nes_state_writer_destroy(&writer);
    return ok;
}

bool nes_netplay_hardware_hash(NesNetplayHardwareHash *hash) {
    if (!hash) return false;
    *hash = (NesNetplayHardwareHash){
        UINT64_C(1469598103934665603), UINT64_C(0x6a09e667f3bcc909)
    };
    static const struct {
        uint32_t tag;
        NetplayCapture capture;
    } components[] = {
        {0x454D4954u, timing_state_capture},             /* TIME */
        {0x52505748u, hardware_state_capture},           /* HWPR */
        {0x20555043u, cpu_state_capture},                /* CPU  */
        {0x20555050u, ppu_hardware_state_capture},       /* PPU  */
        {0x20555041u, apu_hardware_state_capture},       /* APU  */
        {0x5250414Du, mapper_hardware_state_capture},    /* MAPR */
        {0x4D535045u, epsm_state_capture},               /* EPSM */
        {0x54504E49u, joypad_hardware_state_capture},    /* INPT */
        {0x53414246u, family_basic_state_capture},       /* FBAS */
        {0x52455053u, special_peripherals_hardware_state_capture}, /* SPER */
        {0x20534446u, fds_hardware_state_capture},       /* FDS  */
        {0x53595356u, vs_hardware_state_capture}         /* VSYS */
    };
    for (size_t i = 0; i < sizeof(components) / sizeof(components[0]); ++i)
        if (!hash_component(hash, components[i].tag, components[i].capture))
            return false;
    return true;
}

bool nes_netplay_hardware_hash_equal(const NesNetplayHardwareHash *left,
                                     const NesNetplayHardwareHash *right) {
    return left && right && left->first == right->first
        && left->second == right->second;
}
