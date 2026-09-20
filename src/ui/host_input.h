/*
 * host_input.h - SDL controller assignment and bound host input
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_HOST_INPUT_H
#define FRONTEND_HOST_INPUT_H

#include <SDL2/SDL.h>
#include "frontend_execution.h"
#include "settings.h"

void frontend_host_input_open_controllers(const FrontendSettings *settings);
void frontend_host_input_event(const SDL_Event *event, const FrontendSettings *settings,
                               FrontendExecutionRuntime *execution);
void frontend_host_input_release_all(void);
bool frontend_host_input_bound_player_key(const FrontendBindingProfile *profile,
                                          const SDL_KeyboardEvent *event);
void frontend_host_input_shutdown(void);

#endif
