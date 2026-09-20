/*
 * hardware.h - NES console hardware selection
 *
 * Author: @frankischilling
 *
 * This header selects the console wiring independently from the cartridge's
 * video timing region. Power and reset operations preserve the selection.
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
#ifndef NES_HARDWARE_H
#define NES_HARDWARE_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "../state/state_io.h"

typedef enum {
    NES_CONSOLE_NES001,
    NES_CONSOLE_NES101,
    NES_CONSOLE_HVC001,
    NES_CONSOLE_HVC101
} NesConsoleModel;

typedef enum {
    NES_RAM_POWER_DEFAULT,
    NES_RAM_POWER_ZERO,
    NES_RAM_POWER_ONES,
    NES_RAM_POWER_RANDOM
} NesRamPowerOnState;

NesConsoleModel nes_console_model(void);
bool nes_set_console_model(NesConsoleModel model);
bool nes_set_console_model_name(const char *name);
const char *nes_console_model_name(void);

NesRamPowerOnState nes_ram_power_on_state(void);
bool nes_set_ram_power_on_state(NesRamPowerOnState state);
bool nes_set_ram_power_on_state_name(const char *name);
const char *nes_ram_power_on_state_name(void);
void nes_seed_power_on_random(uint32_t seed);
void nes_initialize_power_on_ram(void *data, size_t size, uint8_t default_value);
bool nes_power_on_random_bool(void);
void nes_set_randomize_vblank(bool enabled);
bool nes_randomize_vblank_enabled(void);

bool hardware_state_capture(NesStateWriter *writer);
bool hardware_state_validate(NesStateReader *reader);
bool hardware_state_apply(NesStateReader *reader);

#endif
