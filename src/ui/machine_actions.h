/*
 * machine_actions.h - Desktop frontend machine lifecycle actions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef MACHINE_ACTIONS_H
#define MACHINE_ACTIONS_H

#include <stdbool.h>

bool frontend_machine_soft_reset(void);
bool frontend_machine_power_cycle(void);

#endif
