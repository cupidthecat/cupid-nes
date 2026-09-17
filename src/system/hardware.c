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
#include <string.h>

static NesConsoleModel console_model = NES_CONSOLE_NES001;
static const char *const console_names[] = {
    "nes-001", "nes-101", "famicom", "av-famicom"
};

NesConsoleModel nes_console_model(void) {
    return console_model;
}

bool nes_set_console_model(NesConsoleModel model) {
    if ((unsigned)model > NES_CONSOLE_HVC101) return false;
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
