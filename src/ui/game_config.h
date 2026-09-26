/*
 * game_config.h - Per-image settings layered over global preferences
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GAME_CONFIG_H
#define CUPID_GAME_CONFIG_H
#include "settings.h"
#include "frontend_session.h"

enum { GAME_CONFIG_PANEL = 0x2600, GAME_CONFIG_FIELDS = 48 };

typedef struct {
    uint64_t present;
    char values[GAME_CONFIG_FIELDS][1024];
} GameConfig;

typedef struct {
    FrontendSettings *global;
    FrontendSettings launch;
    GameConfig config;
    char directory[2048], key[41];
    uint64_t cli_fields;
    int selected;
    char names[GAME_CONFIG_FIELDS][128];
    const char *items[GAME_CONFIG_FIELDS];
    char value[1024], inherited[1024], status[192];
} GameConfigFrontend;

size_t game_config_field_count(void);
const char *game_config_field_name(size_t field);
int game_config_find_field(const char *name);
bool game_config_set(GameConfig *config, size_t field, const char *value);
void game_config_reset(GameConfig *config, size_t field);
bool game_config_load(const char *directory, const char *key, GameConfig *config, char *error, size_t error_size);
bool game_config_save(const char *directory, const char *key, const GameConfig *config, char *error, size_t error_size);
bool game_config_resolve(const FrontendSettings *global, const GameConfig *config, uint64_t cli_fields,
                         FrontendSettings *effective, char *cheat_path, size_t cheat_capacity, char *error,
                         size_t error_size);
/* Set before each image activation; apply effective settings before loading/power-on. */
bool game_config_prepare(GameConfigFrontend *frontend, const FrontendImageResult *image, FrontendSettings *effective,
                         char *cheat_path, size_t cheat_capacity, char *error, size_t error_size);
bool game_config_init(GameConfigFrontend *frontend, FrontendSettings *global, const char *directory);
bool game_config_save_globals(GameConfigFrontend *frontend, const char *path, const FrontendSettings *effective,
                              FrontendSettingsReport *report);
bool game_config_register_ui(GameConfigFrontend *frontend, FrontendSettings *global, const char *directory);
void game_config_unregister_ui(void);
#endif
