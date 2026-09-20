/*
 * nsf_player_runtime.h - Music controls in the desktop execution loop
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NSF_PLAYER_RUNTIME_H
#define NSF_PLAYER_RUNTIME_H

#include "nsf_player.h"
#include "frontend_execution.h"

bool nsf_player_bind_frontend(NsfPlayer *player, FrontendExecutionRuntime *runtime,
                               uint32_t shuffle_seed);
bool nsf_player_handle_shortcut(NsfPlayer *player, const SDL_KeyboardEvent *event,
                                 char *error, size_t error_size);

#endif
