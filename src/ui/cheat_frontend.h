/*
 * cheat_frontend.h - Desktop cheat-code controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CHEAT_FRONTEND_H
#define CHEAT_FRONTEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct CheatFrontend CheatFrontend;

enum {
    CHEAT_CONTROL_LIST = 1,
    CHEAT_CONTROL_CODE,
    CHEAT_CONTROL_DESCRIPTION,
    CHEAT_CONTROL_ENABLED,
    CHEAT_CONTROL_ADD,
    CHEAT_CONTROL_REMOVE,
    CHEAT_CONTROL_UP,
    CHEAT_CONTROL_DOWN,
    CHEAT_CONTROL_CLEAR,
    CHEAT_CONTROL_PATH,
    CHEAT_CONTROL_LOAD,
    CHEAT_CONTROL_SAVE,
    CHEAT_CONTROL_INFO
};

CheatFrontend *cheat_frontend_create(const char *storage_directory);
bool cheat_frontend_register_ui(CheatFrontend *frontend);
bool cheat_frontend_prepare(CheatFrontend *frontend, const char *code, const char *description,
                            char *error, size_t error_size);
void cheat_frontend_image_changed(CheatFrontend *frontend, uint32_t game_identity);
void cheat_frontend_destroy(CheatFrontend *frontend);

#endif
