/*
 * frontend_commands.h - Shared desktop frontend command registry
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_COMMANDS_H
#define FRONTEND_COMMANDS_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    FRONTEND_COMMAND_OPEN = 1,
    FRONTEND_COMMAND_PAUSE,
    FRONTEND_COMMAND_FRAME_ADVANCE,
    FRONTEND_COMMAND_SOFT_RESET,
    FRONTEND_COMMAND_POWER_CYCLE,
    FRONTEND_COMMAND_RELOAD,
    FRONTEND_COMMAND_FAST_FORWARD_HOLD,
    FRONTEND_COMMAND_FAST_FORWARD_TOGGLE,
    FRONTEND_COMMAND_SPEED_HALF,
    FRONTEND_COMMAND_SPEED_NORMAL,
    FRONTEND_COMMAND_SPEED_DOUBLE,
    FRONTEND_COMMAND_SETTINGS,
    FRONTEND_COMMAND_FULLSCREEN,
    FRONTEND_COMMAND_MUTE,
    FRONTEND_COMMAND_EXTENSION_BASE = 0x1000
} FrontendCommandId;

typedef enum {
    FRONTEND_COMMAND_NEEDS_SESSION = 1u << 0,
    FRONTEND_COMMAND_CHECKABLE = 1u << 1,
    FRONTEND_COMMAND_MOMENTARY = 1u << 2
} FrontendCommandFlags;

typedef bool (*FrontendCommandHandler)(void *userdata, char *error, size_t error_size);

typedef struct {
    unsigned id;
    const char *label;
    const char *menu;
    const char *shortcut;
    unsigned flags;
    FrontendCommandHandler handler;
    void *userdata;
} FrontendCommandSpec;

typedef struct {
    unsigned id;
    const char *label;
    const char *menu;
    const char *shortcut;
    unsigned flags;
    bool enabled;
    bool checked;
} FrontendCommandInfo;

void frontend_commands_reset(void);
bool frontend_command_register(const FrontendCommandSpec *spec);
bool frontend_command_unregister(unsigned id);
bool frontend_command_invoke(unsigned id, char *error, size_t error_size);
bool frontend_command_set_enabled(unsigned id, bool enabled);
bool frontend_command_set_checked(unsigned id, bool checked);
void frontend_command_set_session_active(bool active);
bool frontend_command_session_active(void);
bool frontend_command_get(unsigned id, FrontendCommandInfo *info);
size_t frontend_command_count(void);
bool frontend_command_at(size_t index, FrontendCommandInfo *info);

#endif
