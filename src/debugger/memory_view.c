/*
 * memory_view.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Bounded, observational memory access. SPDX-License-Identifier: GPL-3.0-or-later */
#include "memory_view.h"
#include "debugger.h"
#include "../rom/mapper.h"

#include <string.h>

static const struct {
    const char *name;
    uint32_t first, last;
} spaces[] = {{"CPU RAM", 0, 0x7FF},
              {"Cartridge RAM window", 0x6000, 0x7FFF},
              {"CPU address space", 0, 0xFFFF},
              {"PPU address space", 0, 0x3FFF},
              {"Sprite OAM", 0, 0xFF},
              {"Palette", 0x3F00, 0x3F1F}};

const char *debug_memory_name(DebugMemorySpace space) {
    return (unsigned)space < DEBUG_MEMORY_SPACE_COUNT ? spaces[space].name : "Invalid memory space";
}

bool debug_memory_bounds(DebugMemorySpace space, uint32_t *first, uint32_t *last) {
    if ((unsigned)space >= DEBUG_MEMORY_SPACE_COUNT || !first || !last) {
        return false;
    }

    *first = spaces[space].first;
    *last = spaces[space].last;
    if (space == DEBUG_MEMORY_RAM && cart_cpu_ram_8k()) *last = 0x1FFF;
    return true;
}

bool debug_memory_cpu_space(DebugMemorySpace space) {
    return space == DEBUG_MEMORY_RAM || space == DEBUG_MEMORY_CART_RAM || space == DEBUG_MEMORY_CPU;
}

bool debug_memory_readable(DebugMemorySpace space, uint32_t first, uint32_t last) {
    uint32_t low, high;
    if (!debug_memory_bounds(space, &low, &high) || first < low || last > high || first > last) return false;
    if (!debug_memory_cpu_space(space)) return space != DEBUG_MEMORY_PPU || first >= 0x2000 || cart != NULL;
    for (uint32_t address = first; address <= last; ++address) {
        if (address < 0x2000) continue;
        if (address < 0x4000) {
            unsigned reg = address & 7;
            if (reg == 2 || reg == 4 || reg == 7) continue;
            return false;
        }
        if (address == 0x4015 || address == 0x4016 || address == 0x4017) continue;
        if (address < 0x4020) return false;
        /* Open-bus inputs distinguish floating lines without assuming that a
         * particular data byte (00 or FF) means an unmapped address. */
        uint8_t zero = cart_cpu_peek_bus((uint16_t)address, 0);
        uint8_t ones = cart_cpu_peek_bus((uint16_t)address, 0xFF);
        if ((uint8_t)(zero ^ ones) == 0xFF) return false;
    }
    return true;
}

bool debug_memory_sample(DebugMemorySpace space, uint32_t first, uint32_t last, uint8_t *out, size_t capacity) {
    uint32_t low, high;
    if (!out || !debug_memory_bounds(space, &low, &high) || first < low || last > high || first > last ||
        capacity < (size_t)(last - first) + 1) {
        return false;
    }

    if (space == DEBUG_MEMORY_OAM) {
        uint8_t oam[256];
        debugger_copy_oam(oam);
        memcpy(out, oam + first, (size_t)(last - first) + 1);
        return true;
    }

    bool cpu_space = debug_memory_cpu_space(space);
    for (uint32_t address = first; address <= last; ++address) {
        out[address - first] = cpu_space ? debugger_peek_cpu((uint16_t)address) : debugger_peek_ppu((uint16_t)address);
    }

    return true;
}
