/*
 * nsf_player_runtime.c - Music controls in the desktop execution loop
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "nsf_player_runtime.h"
#include "frontend_commands.h"
#include "../rom/rom.h"

static bool audio_active(void *context) {
    FrontendExecutionRuntime *runtime = context;
    return runtime->audio_device && *runtime->audio_device != 0;
}

static void lock_audio(void *context) {
    FrontendExecutionRuntime *runtime = context;
    if (audio_active(context)) SDL_LockAudioDevice(*runtime->audio_device);
}

static void unlock_audio(void *context) {
    FrontendExecutionRuntime *runtime = context;
    if (audio_active(context)) SDL_UnlockAudioDevice(*runtime->audio_device);
}

static void set_paused(void *context, bool paused) {
    FrontendExecutionRuntime *runtime = context;
    if (runtime->execution.paused != paused) {
        char error[160];
        (void)frontend_command_invoke(FRONTEND_COMMAND_PAUSE, error, sizeof(error));
    }
}

bool nsf_player_bind_frontend(NsfPlayer *player, FrontendExecutionRuntime *runtime,
                               uint32_t shuffle_seed) {
    if (!runtime) return false;
    NsfPlayerHooks hooks = {lock_audio, unlock_audio, set_paused, audio_active, runtime};
    if (!nsf_player_init(player, &runtime->execution, &hooks, shuffle_seed)) return false;
    if (!nsf_player_register_ui(player)) {
        nsf_player_shutdown(player);
        return false;
    }
    return true;
}

bool nsf_player_handle_shortcut(NsfPlayer *player, const SDL_KeyboardEvent *event,
                                 char *error, size_t error_size) {
    if (!player || !event || !rom_is_nsf()) return false;
    SDL_Keymod mods = (SDL_Keymod)event->keysym.mod;
    bool control = (mods & KMOD_CTRL) != 0;
    if (mods & (KMOD_ALT | KMOD_GUI)) return false;

    unsigned command = 0;
    switch (event->keysym.scancode) {
        case SDL_SCANCODE_PAGEUP: command = NSF_COMMAND_NEXT; break;
        case SDL_SCANCODE_PAGEDOWN: command = NSF_COMMAND_PREVIOUS; break;
        case SDL_SCANCODE_SPACE: if (control) command = NSF_COMMAND_PLAY_PAUSE; break;
        case SDL_SCANCODE_HOME: if (control) command = NSF_COMMAND_RESTART; break;
        case SDL_SCANCODE_END: if (control) command = NSF_COMMAND_STOP; break;
        default: break;
    }
    if (!command) return false;
    if (event->type == SDL_KEYDOWN && !event->repeat) {
        nsf_player_refresh_ui(player);
        (void)frontend_command_invoke(command, error, error_size);
    }
    return true;
}
