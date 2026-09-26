/*
 * game_genie_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Game Genie utility ownership. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_GAME_GENIE_FRONTEND_H
#define CUPID_GAME_GENIE_FRONTEND_H

#include "cheat_frontend.h"

typedef struct GameGenieFrontend GameGenieFrontend;

enum {
    GAME_GENIE_CODE = 1,
    GAME_GENIE_ADDRESS,
    GAME_GENIE_VALUE,
    GAME_GENIE_COMPARE,
    GAME_GENIE_DESCRIPTION,
    GAME_GENIE_SEND,
    GAME_GENIE_COPY
};

GameGenieFrontend *game_genie_frontend_create(CheatFrontend *editor);
bool game_genie_frontend_register(GameGenieFrontend *frontend);
void game_genie_frontend_destroy(GameGenieFrontend *frontend);

#endif
