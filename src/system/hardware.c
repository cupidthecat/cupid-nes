/*
 * hardware.c - NES console hardware selection
 *
 * Author: @frankischilling
 *
 * Console wiring is a property of the machine. Loading a cartridge or
 * changing the clock region does not change this setting.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
#include "hardware.h"
#include "execution_policy.h"
#include <string.h>

static NesConsoleModel console_model = NES_CONSOLE_NES001;
static NesRamPowerOnState ram_power_on_state = NES_RAM_POWER_DEFAULT;
static uint32_t power_on_random_state = 0x4E455300u;
static bool randomize_vblank = false;
static const char *const console_names[] = {
    "nes-001", "nes-101", "famicom", "av-famicom"
};
static const char *const ram_power_names[] = {
    "default", "zero", "ones", "random"
};

NesConsoleModel nes_console_model(void) {
    return console_model;
}

bool nes_set_console_model(NesConsoleModel model) {
    if ((unsigned)model > NES_CONSOLE_HVC101) return false;
    if (!nes_execution_allows_host_configuration()) return false;
    console_model = model;
    return true;
}

bool nes_set_console_model_name(const char *name) {
    if (!name) return false;
    for (unsigned i = 0; i < sizeof(console_names) / sizeof(console_names[0]); ++i) {
        if (strcmp(name, console_names[i]) == 0)
            return nes_set_console_model((NesConsoleModel)i);
    }
    return false;
}

const char *nes_console_model_name(void) {
    return console_names[console_model];
}

NesRamPowerOnState nes_ram_power_on_state(void) {
    return ram_power_on_state;
}

bool nes_set_ram_power_on_state(NesRamPowerOnState state) {
    if ((unsigned)state > NES_RAM_POWER_RANDOM) return false;
    if (!nes_execution_allows_host_configuration()) return false;
    ram_power_on_state = state;
    return true;
}

bool nes_set_ram_power_on_state_name(const char *name) {
    if (!name) return false;
    for (unsigned i = 0; i < sizeof(ram_power_names) / sizeof(ram_power_names[0]); ++i) {
        if (strcmp(name, ram_power_names[i]) == 0)
            return nes_set_ram_power_on_state((NesRamPowerOnState)i);
    }
    return false;
}

const char *nes_ram_power_on_state_name(void) {
    return ram_power_names[ram_power_on_state];
}

void nes_seed_power_on_random(uint32_t seed) {
    if (!nes_execution_allows_host_configuration()) return;
    power_on_random_state = seed;
}

uint32_t nes_power_on_random_state(void) {
    return power_on_random_state;
}

static uint32_t next_power_on_random(void) {
    power_on_random_state = power_on_random_state * 1664525u + 1013904223u;
    uint32_t value = power_on_random_state;
    value ^= value >> 16;
    value *= 0x7FEB352Du;
    value ^= value >> 15;
    return value;
}

void nes_initialize_power_on_ram(void *data, size_t size, uint8_t default_value) {
    if (!data || !size) return;
    if (ram_power_on_state == NES_RAM_POWER_DEFAULT) {
        memset(data, default_value, size);
        return;
    }
    if (ram_power_on_state == NES_RAM_POWER_ZERO) {
        memset(data, 0, size);
        return;
    }
    if (ram_power_on_state == NES_RAM_POWER_ONES) {
        memset(data, 0xFF, size);
        return;
    }

    uint8_t *bytes = data;
    size_t offset = 0;
    while (offset < size) {
        uint32_t value = next_power_on_random();
        for (unsigned byte = 0; byte < 4 && offset < size; ++byte)
            bytes[offset++] = (uint8_t)(value >> (byte * 8));
    }
}

bool nes_power_on_random_bool(void) {
    return (next_power_on_random() & 1u) != 0;
}

void nes_set_randomize_vblank(bool enabled) {
    if (!nes_execution_allows_host_configuration()) return;
    randomize_vblank = enabled;
}

bool nes_randomize_vblank_enabled(void) {
    return randomize_vblank;
}

static bool hardware_state_decode(NesStateReader *reader, NesConsoleModel *model,
                                  NesRamPowerOnState *power_state, uint32_t *random_state,
                                  bool *random_vblank) {
    uint8_t encoded_model, encoded_power;
    if (!nes_state_read_u8(reader, &encoded_model)
        || !nes_state_read_u8(reader, &encoded_power)
        || !nes_state_read_u32(reader, random_state)
        || !nes_state_read_bool(reader, random_vblank)
        || encoded_model > NES_CONSOLE_HVC101
        || encoded_power > NES_RAM_POWER_RANDOM
        || nes_state_reader_remaining(reader) != 0) return false;
    *model = (NesConsoleModel)encoded_model;
    *power_state = (NesRamPowerOnState)encoded_power;
    return true;
}

bool hardware_state_capture(NesStateWriter *writer) {
    return writer
        && nes_state_write_u8(writer, (uint8_t)console_model)
        && nes_state_write_u8(writer, (uint8_t)ram_power_on_state)
        && nes_state_write_u32(writer, power_on_random_state)
        && nes_state_write_bool(writer, randomize_vblank);
}

bool hardware_state_validate(NesStateReader *reader) {
    NesConsoleModel model;
    NesRamPowerOnState power_state;
    uint32_t random_state;
    bool random_vblank;
    return reader && hardware_state_decode(reader, &model, &power_state,
                                           &random_state, &random_vblank);
}

bool hardware_state_apply(NesStateReader *reader) {
    NesConsoleModel model;
    NesRamPowerOnState power_state;
    uint32_t random_state;
    bool random_vblank;
    if (!reader || !hardware_state_decode(reader, &model, &power_state,
                                          &random_state, &random_vblank)) return false;
    console_model = model;
    ram_power_on_state = power_state;
    power_on_random_state = random_state;
    randomize_vblank = random_vblank;
    return true;
}
