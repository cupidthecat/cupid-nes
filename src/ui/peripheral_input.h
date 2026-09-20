/*
 * peripheral_input.h - Host keyboard and pointer routing for NES peripherals
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_PERIPHERAL_INPUT_H
#define FRONTEND_PERIPHERAL_INPUT_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>

bool extended_port_key_event(const SDL_KeyboardEvent *event);
bool mat_key_event(const SDL_KeyboardEvent *event);
bool finish_tape_capture(const char *path);
bool family_basic_key_event(const SDL_KeyboardEvent *event,
                            const char *play_path, const char *record_path,
                            uint64_t cpu_cycles);
bool subor_key_event(const SDL_KeyboardEvent *event);
bool party_tap_key_event(const SDL_KeyboardEvent *event);
bool boxing_key_event(const SDL_KeyboardEvent *event);
bool jissen_key_event(const SDL_KeyboardEvent *event);
void oeka_kids_pointer_event(int pointer_x, int pointer_y, bool pointer_on_screen,
                             uint32_t buttons);

#endif
