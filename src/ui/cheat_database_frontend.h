/*
 * cheat_database_frontend.h - Matching cheat catalog browser
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_CHEAT_DATABASE_FRONTEND_H
#define CUPID_CHEAT_DATABASE_FRONTEND_H
#include <stdbool.h>

enum { CHEAT_DATABASE_PANEL = 0x2900 };

bool cheat_database_frontend_register(void);
void cheat_database_frontend_unregister(void);
void cheat_database_frontend_image_changed(void);
#endif
