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

#include <stdbool.h>

typedef enum {
    NES_CONSOLE_NES001,
    NES_CONSOLE_NES101,
    NES_CONSOLE_HVC001,
    NES_CONSOLE_HVC101
} NesConsoleModel;

NesConsoleModel nes_console_model(void);
bool nes_set_console_model(NesConsoleModel model);
bool nes_set_console_model_name(const char *name);
const char *nes_console_model_name(void);

#endif
