/*
 * special_peripherals.h - Famicom expansion peripheral helpers
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef SPECIAL_PERIPHERALS_H
#define SPECIAL_PERIPHERALS_H

#include <stdbool.h>
#include <stdint.h>

void turbo_file_reset_protocol(void);
uint8_t turbo_file_read(unsigned port);
void turbo_file_write(uint8_t value);
bool turbo_file_configure(const char *rom_path);
bool turbo_file_flush(void);
bool turbo_file_shutdown(void);

void battle_box_reset_protocol(void);
uint8_t battle_box_read(unsigned port);
void battle_box_write(uint8_t value);
bool battle_box_configure(const char *rom_path);
bool battle_box_flush(void);
bool battle_box_shutdown(void);

void subor_keyboard_reset(void);
uint8_t subor_keyboard_read(unsigned port);
void subor_keyboard_write(uint8_t value);
bool subor_keyboard_set_key(unsigned key, bool pressed);

void subor_mouse_reset(void);
uint8_t subor_mouse_read(void);
void subor_mouse_write(uint8_t value);
void subor_mouse_add_motion(int dx, int dy);
void subor_mouse_set_buttons(bool left, bool right);

void hori_track_reset(void);
void hori_track_add_motion(int dx, int dy);
void hori_track_write(uint8_t value, uint8_t buttons);
uint8_t hori_track_read(uint8_t buttons);

#endif
