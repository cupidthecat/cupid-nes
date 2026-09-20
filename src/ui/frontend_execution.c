/*
 * frontend_execution.c - SDL execution controls and machine commands
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "frontend_execution.h"
#include "frontend_commands.h"
#include "machine_actions.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/rom.h"
#include "../system/vs_system.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
}

static void refresh_audio(FrontendExecutionRuntime *runtime) {
    if (!runtime || !runtime->audio_device || !*runtime->audio_device
        || runtime->audio_output_rate <= 0) return;
    SDL_AudioDeviceID device = *runtime->audio_device;
    SDL_PauseAudioDevice(device, 1);
    SDL_LockAudioDevice(device);
    double speed = execution_control_effective_speed(&runtime->execution);
    int emulated_sample_rate = (int)lround((double)runtime->audio_output_rate / speed);
    if (emulated_sample_rate < 1000) emulated_sample_rate = 1000;
    vs_audio_init(emulated_sample_rate);
    SDL_UnlockAudioDevice(device);
    if (!runtime->execution.paused) SDL_PauseAudioDevice(device, 0);
}

static void lock_audio_for_machine_change(FrontendExecutionRuntime *runtime) {
    if (!runtime || !runtime->audio_device || !*runtime->audio_device) return;
    SDL_PauseAudioDevice(*runtime->audio_device, 1);
    SDL_LockAudioDevice(*runtime->audio_device);
}

static void unlock_audio_after_machine_change(FrontendExecutionRuntime *runtime) {
    if (!runtime || !runtime->audio_device || !*runtime->audio_device) return;
    SDL_UnlockAudioDevice(*runtime->audio_device);
    refresh_audio(runtime);
}

static bool command_pause(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    execution_control_toggle_paused(&runtime->execution);
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, runtime->execution.paused);
    refresh_audio(runtime);
    return true;
}

static bool command_frame_advance(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!execution_control_request_frame(&runtime->execution)) {
        set_error(error, error_size, "Frame advance requires paused emulation");
        return false;
    }
    return true;
}

static bool command_soft_reset(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    lock_audio_for_machine_change(runtime);
    bool result = frontend_machine_soft_reset();
    unlock_audio_after_machine_change(runtime);
    return result;
}

static bool command_power_cycle(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    lock_audio_for_machine_change(runtime);
    bool result = frontend_machine_power_cycle();
    unlock_audio_after_machine_change(runtime);
    if (!result)
        set_error(error, error_size, "Power cycle failed for the selected startup alignment");
    return result;
}

static bool command_reload(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!runtime->rom_path) {
        set_error(error, error_size, "No image is loaded");
        return false;
    }
    if (!joypad_persistent_flush()) {
        set_error(error, error_size, "Peripheral storage could not be saved");
        return false;
    }

    bool was_fds = rom_is_fds();
    bool disk_inserted = was_fds && fds_disk_inserted();
    bool write_protected = was_fds && fds_write_protected();
    lock_audio_for_machine_change(runtime);
    int load_result = runtime->fds_bios_path
        ? load_fds(runtime->rom_path, runtime->fds_bios_path, write_protected)
        : runtime->studybox_bios_path
            ? load_studybox(runtime->rom_path, runtime->studybox_bios_path)
            : load_rom(runtime->rom_path);
    if (load_result != 0) {
        unlock_audio_after_machine_change(runtime);
        set_error(error, error_size,
                  "The image could not be reloaded; the current session was kept");
        return false;
    }
    if (rom_is_fds() && runtime->fds_side) {
        if (disk_inserted) {
            if (!fds_insert_disk(*runtime->fds_side)) {
                unlock_audio_after_machine_change(runtime);
                set_error(error, error_size, "The previous disk side is unavailable after reload");
                return false;
            }
        } else {
            fds_eject_disk();
        }
    }
    bool powered = frontend_machine_power_cycle();
    unlock_audio_after_machine_change(runtime);
    if (!powered)
        set_error(error, error_size, "Reloaded image has an invalid startup alignment");
    return powered;
}

static bool set_speed(void *userdata, double speed) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!execution_control_set_speed(&runtime->execution, speed)) return false;
    refresh_audio(runtime);
    return true;
}

static bool command_speed_half(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    return set_speed(userdata, 0.5);
}

static bool command_speed_normal(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    return set_speed(userdata, 1.0);
}

static bool command_speed_double(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    return set_speed(userdata, 2.0);
}

static bool command_fast_forward_toggle(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    execution_control_toggle_fast_forward(&runtime->execution);
    frontend_command_set_checked(FRONTEND_COMMAND_FAST_FORWARD_TOGGLE,
                                 runtime->execution.fast_forward_toggled);
    refresh_audio(runtime);
    return true;
}

static bool command_fast_forward_hold(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    execution_control_set_fast_forward_held(&runtime->execution, true);
    refresh_audio(runtime);
    return true;
}

void frontend_execution_init(FrontendExecutionRuntime *runtime,
                             SDL_AudioDeviceID *audio_device, int audio_output_rate,
                             const char *rom_path, const char *fds_bios_path,
                             const char *studybox_bios_path, size_t *fds_side) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    execution_control_init(&runtime->execution);
    runtime->audio_device = audio_device;
    runtime->audio_output_rate = audio_output_rate;
    runtime->rom_path = rom_path;
    runtime->fds_bios_path = fds_bios_path;
    runtime->studybox_bios_path = studybox_bios_path;
    runtime->fds_side = fds_side;
}

bool frontend_execution_register_commands(FrontendExecutionRuntime *runtime) {
    static const struct {
        unsigned id;
        const char *label;
        const char *shortcut;
        unsigned flags;
        FrontendCommandHandler handler;
    } specs[] = {
        {FRONTEND_COMMAND_PAUSE, "Pause", "Ctrl+P",
         FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_CHECKABLE, command_pause},
        {FRONTEND_COMMAND_FRAME_ADVANCE, "Frame Advance", "Ctrl+.",
         FRONTEND_COMMAND_NEEDS_SESSION, command_frame_advance},
        {FRONTEND_COMMAND_SOFT_RESET, "Soft Reset", "Ctrl+R",
         FRONTEND_COMMAND_NEEDS_SESSION, command_soft_reset},
        {FRONTEND_COMMAND_POWER_CYCLE, "Power Cycle", "Ctrl+Shift+R",
         FRONTEND_COMMAND_NEEDS_SESSION, command_power_cycle},
        {FRONTEND_COMMAND_RELOAD, "Reload", "Ctrl+Alt+R",
         FRONTEND_COMMAND_NEEDS_SESSION, command_reload},
        {FRONTEND_COMMAND_FAST_FORWARD_HOLD, "Fast Forward", "Ctrl+F (hold)",
         FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_MOMENTARY,
         command_fast_forward_hold},
        {FRONTEND_COMMAND_FAST_FORWARD_TOGGLE, "Toggle Fast Forward", "Ctrl+Shift+F",
         FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_CHECKABLE,
         command_fast_forward_toggle},
        {FRONTEND_COMMAND_SPEED_HALF, "Speed 50%", "Ctrl+1",
         FRONTEND_COMMAND_NEEDS_SESSION, command_speed_half},
        {FRONTEND_COMMAND_SPEED_NORMAL, "Speed 100%", "Ctrl+2",
         FRONTEND_COMMAND_NEEDS_SESSION, command_speed_normal},
        {FRONTEND_COMMAND_SPEED_DOUBLE, "Speed 200%", "Ctrl+3",
         FRONTEND_COMMAND_NEEDS_SESSION, command_speed_double}
    };
    if (!runtime) return false;
    frontend_commands_reset();
    for (size_t i = 0; i < sizeof(specs) / sizeof(specs[0]); ++i) {
        FrontendCommandSpec spec = {
            .id = specs[i].id,
            .label = specs[i].label,
            .menu = "Emulation",
            .shortcut = specs[i].shortcut,
            .flags = specs[i].flags,
            .handler = specs[i].handler,
            .userdata = runtime
        };
        if (!frontend_command_register(&spec)) return false;
    }
    return true;
}

bool frontend_execution_handle_shortcut(FrontendExecutionRuntime *runtime,
                                        const SDL_KeyboardEvent *event) {
    static bool held_fast_forward_shortcut;
    if (!event || !runtime) return false;
    bool down = event->type == SDL_KEYDOWN;
    SDL_Keymod mods = (SDL_Keymod)event->keysym.mod;
    bool control = (mods & KMOD_CTRL) != 0;
    bool shift = (mods & KMOD_SHIFT) != 0;
    bool alt = (mods & KMOD_ALT) != 0;
    char error[160] = {0};
    unsigned command = 0;

    if (event->keysym.scancode == SDL_SCANCODE_F && held_fast_forward_shortcut && !down) {
        held_fast_forward_shortcut = false;
        execution_control_set_fast_forward_held(&runtime->execution, false);
        refresh_audio(runtime);
        return true;
    }
    if (!control) return false;
    if (event->keysym.scancode == SDL_SCANCODE_F && !shift) {
        if (down && !event->repeat) {
            held_fast_forward_shortcut = true;
            (void)frontend_command_invoke(FRONTEND_COMMAND_FAST_FORWARD_HOLD,
                                          error, sizeof(error));
        }
        return true;
    }
    if (!down || event->repeat) return false;
    switch (event->keysym.scancode) {
        case SDL_SCANCODE_P: command = FRONTEND_COMMAND_PAUSE; break;
        case SDL_SCANCODE_PERIOD: command = FRONTEND_COMMAND_FRAME_ADVANCE; break;
        case SDL_SCANCODE_R:
            command = alt ? FRONTEND_COMMAND_RELOAD
                          : shift ? FRONTEND_COMMAND_POWER_CYCLE
                                  : FRONTEND_COMMAND_SOFT_RESET;
            break;
        case SDL_SCANCODE_F:
            if (shift) command = FRONTEND_COMMAND_FAST_FORWARD_TOGGLE;
            break;
        case SDL_SCANCODE_1: command = FRONTEND_COMMAND_SPEED_HALF; break;
        case SDL_SCANCODE_2: command = FRONTEND_COMMAND_SPEED_NORMAL; break;
        case SDL_SCANCODE_3: command = FRONTEND_COMMAND_SPEED_DOUBLE; break;
        default: break;
    }
    if (!command) return false;
    if (!frontend_command_invoke(command, error, sizeof(error)) && error[0])
        fprintf(stderr, "%s\n", error);
    return true;
}

static bool run_emulation_frame(void *userdata) {
    (void)userdata;
    vs_start_frame();
    while (!ppu.frame_complete) vs_cpu_step();
    return true;
}

bool frontend_execution_run_frame(FrontendExecutionRuntime *runtime) {
    if (!runtime) return false;
    bool ran = execution_control_run_frame(&runtime->execution, run_emulation_frame, NULL);
    if (ran && runtime->execution.paused) refresh_audio(runtime);
    return ran;
}

bool frontend_execution_paused(const FrontendExecutionRuntime *runtime) {
    return runtime && runtime->execution.paused;
}

double frontend_execution_speed(const FrontendExecutionRuntime *runtime) {
    return runtime ? execution_control_effective_speed(&runtime->execution) : 1.0;
}
