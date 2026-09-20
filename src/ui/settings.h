/*
 * settings.h - Persistent desktop application settings
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_SETTINGS_H
#define FRONTEND_SETTINGS_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../joypad/joypad.h"
#include "../capture/capture_session.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../util/file_io.h"
#include "nsf_player.h"

enum {
    FRONTEND_SETTINGS_VERSION = 3,
    FRONTEND_SETTINGS_MAX_PROFILES = 8,
    FRONTEND_SETTINGS_PROFILE_NAME = 32,
    FRONTEND_SETTINGS_GUID_TEXT = 64,
    FRONTEND_SETTINGS_PATH_TEXT = 1024,
    FRONTEND_SETTINGS_MESSAGE = 256
};

typedef enum {
    FRONTEND_SHORTCUT_PAUSE,
    FRONTEND_SHORTCUT_FRAME_ADVANCE,
    FRONTEND_SHORTCUT_SOFT_RESET,
    FRONTEND_SHORTCUT_POWER_CYCLE,
    FRONTEND_SHORTCUT_RELOAD,
    FRONTEND_SHORTCUT_FAST_FORWARD_HOLD,
    FRONTEND_SHORTCUT_FAST_FORWARD_TOGGLE,
    FRONTEND_SHORTCUT_SPEED_HALF,
    FRONTEND_SHORTCUT_SPEED_NORMAL,
    FRONTEND_SHORTCUT_SPEED_DOUBLE,
    FRONTEND_SHORTCUT_OPEN,
    FRONTEND_SHORTCUT_COUNT
} FrontendShortcut;

typedef struct {
    SDL_Scancode key;
    SDL_Keymod modifiers;
    SDL_GameControllerButton gamepad_button;
} FrontendHostBinding;

typedef struct {
    char name[FRONTEND_SETTINGS_PROFILE_NAME];
    FrontendHostBinding players[NES_INPUT_PLAYERS][8];
    FrontendHostBinding shortcuts[FRONTEND_SHORTCUT_COUNT];
} FrontendBindingProfile;

typedef enum {
    FRONTEND_OVERRIDE_REGION = 1u << 0,
    FRONTEND_OVERRIDE_CONSOLE = 1u << 1,
    FRONTEND_OVERRIDE_VIDEO_FILTER = 1u << 2,
    FRONTEND_OVERRIDE_SPEED = 1u << 3,
    FRONTEND_OVERRIDE_FAST_FORWARD_SPEED = 1u << 4,
    FRONTEND_OVERRIDE_ADAPTER = 1u << 5,
    FRONTEND_OVERRIDE_PORT1 = 1u << 6,
    FRONTEND_OVERRIDE_PORT2 = 1u << 7,
    FRONTEND_OVERRIDE_EXPANSION = 1u << 8,
    FRONTEND_OVERRIDE_ZAPPER_RADIUS = 1u << 9
} FrontendSettingOverride;

typedef struct {
    unsigned version;
    NesRegionMode region_mode;
    NesConsoleModel console_model;
    bool ntsc_composite;
    double speed;
    double fast_forward_speed;
    bool reopen_last_image;
    bool pause_on_focus_loss;
    bool pause_on_ui;
    bool show_fps;
    bool fullscreen;
    bool integer_scaling;
    bool muted;
    unsigned window_width;
    unsigned window_height;
    FdsSaveMode disk_save_mode;
    char disk_overlay_path[FRONTEND_SETTINGS_PATH_TEXT];
    char fds_bios_path[FRONTEND_SETTINGS_PATH_TEXT];
    char studybox_bios_path[FRONTEND_SETTINGS_PATH_TEXT];
    bool fds_write_protected;
    bool fds_auto_insert;
    bool fds_loading_fast_forward;
    NsfPlayerOptions nsf_player;
    NesCaptureOptions capture;
    char capture_paths[3][FRONTEND_SETTINGS_PATH_TEXT];
    unsigned state_slot;
    char state_file_path[FRONTEND_SETTINGS_PATH_TEXT];
    NesInputConfiguration input;
    unsigned zapper_radius;
    uint8_t saved_input_overrides;
    char active_profile[FRONTEND_SETTINGS_PROFILE_NAME];
    FrontendBindingProfile profiles[FRONTEND_SETTINGS_MAX_PROFILES];
    size_t profile_count;
    char device_guid[NES_INPUT_PLAYERS][FRONTEND_SETTINGS_GUID_TEXT];
    uint32_t cli_overrides;
} FrontendSettings;

typedef struct {
    NesFileResult file_result;
    bool found;
    bool migrated;
    unsigned unknown_settings;
    unsigned line;
    char message[FRONTEND_SETTINGS_MESSAGE];
} FrontendSettingsReport;

void frontend_settings_defaults(FrontendSettings *settings);
bool frontend_settings_load(const char *path, FrontendSettings *settings,
                            FrontendSettingsReport *report);
bool frontend_settings_save(const char *path, const FrontendSettings *settings,
                            FrontendSettingsReport *report);
bool frontend_settings_apply_core(const FrontendSettings *settings,
                                  char *error, size_t error_size);

void frontend_settings_mark_cli_override(FrontendSettings *settings,
                                         FrontendSettingOverride override_flag);
bool frontend_settings_cli_overridden(const FrontendSettings *settings,
                                      FrontendSettingOverride override_flag);

FrontendBindingProfile *frontend_settings_active_profile(FrontendSettings *settings);
const FrontendBindingProfile *frontend_settings_active_profile_const(
    const FrontendSettings *settings);
bool frontend_settings_select_profile(FrontendSettings *settings, const char *name);
bool frontend_settings_add_profile(FrontendSettings *settings, const char *name,
                                   const char *preset);
bool frontend_binding_profile_preset(FrontendBindingProfile *profile,
                                     const char *name, const char *preset);

const char *frontend_shortcut_name(FrontendShortcut shortcut);
bool frontend_shortcut_from_name(const char *name, FrontendShortcut *shortcut);
const char *frontend_player_button_name(unsigned button);
bool frontend_player_button_from_name(const char *name, unsigned *button);
const char *frontend_gamepad_button_name(SDL_GameControllerButton button);
bool frontend_gamepad_button_from_name(const char *name,
                                       SDL_GameControllerButton *button);
bool frontend_profile_player_key(const FrontendBindingProfile *profile,
                                 const SDL_KeyboardEvent *event,
                                 unsigned *player, unsigned *button);
bool frontend_profile_player_gamepad(const FrontendBindingProfile *profile,
                                     unsigned player,
                                     SDL_GameControllerButton gamepad_button,
                                     unsigned *button);
bool frontend_profile_shortcut_key(const FrontendBindingProfile *profile,
                                   const SDL_KeyboardEvent *event,
                                   FrontendShortcut *shortcut);
bool frontend_profile_shortcut_gamepad(const FrontendBindingProfile *profile,
                                       SDL_GameControllerButton gamepad_button,
                                       FrontendShortcut *shortcut);

#endif
