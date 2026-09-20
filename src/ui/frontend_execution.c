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
#include "frontend_panels.h"
#include "machine_actions.h"
#include "../debugger/debugger.h"
#include "replay_frontend.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../rom/rom.h"
#include "../system/vs_system.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
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
    if (!runtime->execution.paused && !runtime->muted) SDL_PauseAudioDevice(device, 0);
}

static void update_audio_pause(FrontendExecutionRuntime *runtime) {
    if (runtime && runtime->audio_device && *runtime->audio_device)
        SDL_PauseAudioDevice(*runtime->audio_device,
                             runtime->execution.paused || runtime->muted ? 1 : 0);
}

void frontend_execution_sync_debugger(FrontendExecutionRuntime *runtime) {
    if (!runtime || runtime->debugger_pause_revision == debugger_pause_revision()) return;
    runtime->debugger_pause_revision = debugger_pause_revision();
    bool paused = debugger_is_paused();
    execution_control_set_paused(&runtime->execution, paused);
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, paused);
    update_audio_pause(runtime);
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

static void unlock_audio_without_refresh(FrontendExecutionRuntime *runtime) {
    if (!runtime || !runtime->audio_device || !*runtime->audio_device) return;
    SDL_UnlockAudioDevice(*runtime->audio_device);
    if (!runtime->execution.paused && !runtime->muted)
        SDL_PauseAudioDevice(*runtime->audio_device, 0);
}

static bool command_pause(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (debugger_is_paused()) {
        debugger_resume();
        frontend_execution_sync_debugger(runtime);
        return true;
    }
    execution_control_toggle_paused(&runtime->execution);
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, runtime->execution.paused);
    update_audio_pause(runtime);
    return true;
}

static bool command_open(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!runtime || !runtime->open_handler) {
        set_error(error, error_size, "Open is unavailable");
        return false;
    }
    return runtime->open_handler(runtime->open_userdata, error, error_size);
}

static bool command_frame_advance(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (debugger_is_paused()) {
        debugger_resume();
        runtime->debugger_pause_revision = debugger_pause_revision();
        execution_control_set_paused(&runtime->execution, true);
    }
    if (!execution_control_request_frame(&runtime->execution)) {
        set_error(error, error_size, "Frame advance requires paused emulation");
        return false;
    }
    return true;
}

static bool command_soft_reset(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (runtime->before_machine_change
        && !runtime->before_machine_change(runtime->machine_change_context,
                                           error, error_size)) return false;
    frontend_execution_clear_timeline(runtime);
    lock_audio_for_machine_change(runtime);
    bool result = frontend_machine_soft_reset();
    unlock_audio_after_machine_change(runtime);
    return result;
}

static bool command_power_cycle(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (runtime->before_machine_change
        && !runtime->before_machine_change(runtime->machine_change_context,
                                           error, error_size)) return false;
    frontend_execution_clear_timeline(runtime);
    lock_audio_for_machine_change(runtime);
    bool result = frontend_machine_power_cycle();
    unlock_audio_after_machine_change(runtime);
    if (!result)
        set_error(error, error_size, "Power cycle failed for the selected startup alignment");
    return result;
}

static bool command_reload(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (runtime && runtime->reload_handler) {
        return runtime->reload_handler(runtime->reload_userdata, error, error_size);
    }
    if (!runtime->rom_path) {
        set_error(error, error_size, "No image is loaded");
        return false;
    }
    if (!joypad_persistent_flush()) {
        set_error(error, error_size, "Peripheral storage could not be saved");
        return false;
    }
    if (runtime->before_machine_change
        && !runtime->before_machine_change(runtime->machine_change_context, error, error_size)) return false;

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
    frontend_execution_clear_timeline(runtime);
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
    debugger_init();
    runtime->debugger_pause_revision = debugger_pause_revision();
    runtime->replay_status = NES_REPLAY_OK;
    runtime->replay_state_status = NES_STATE_OK;
    nes_rewind_init(&runtime->rewind);
    (void)frontend_execution_set_rewind_seconds(runtime, 10);
}

void frontend_execution_set_open_handler(FrontendExecutionRuntime *runtime,
                                         FrontendOpenHandler handler, void *userdata) {
    if (!runtime) return;
    runtime->open_handler = handler;
    runtime->open_userdata = userdata;
}

void frontend_execution_set_reload_handler(FrontendExecutionRuntime *runtime,
                                           FrontendReloadHandler handler, void *userdata) {
    if (!runtime) return;
    runtime->reload_handler = handler;
    runtime->reload_userdata = userdata;
}

void frontend_execution_begin_machine_change(FrontendExecutionRuntime *runtime) {
    lock_audio_for_machine_change(runtime);
}

void frontend_execution_end_machine_change(FrontendExecutionRuntime *runtime) {
    unlock_audio_after_machine_change(runtime);
}

bool frontend_execution_register_commands(FrontendExecutionRuntime *runtime) {
    static const struct {
        unsigned id;
        const char *label;
        const char *shortcut;
        unsigned flags;
        FrontendCommandHandler handler;
    } specs[] = {
        {FRONTEND_COMMAND_OPEN, "Open Game...", "Ctrl+O", 0, command_open},
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
    bool session_active = rom_metadata_source() != ROM_METADATA_NONE;
    frontend_command_set_session_active(session_active);
    frontend_panel_set_session_active(session_active);
    for (size_t i = 0; i < sizeof(specs) / sizeof(specs[0]); ++i) {
        FrontendCommandSpec spec = {
            .id = specs[i].id,
            .label = specs[i].label,
            .menu = specs[i].id == FRONTEND_COMMAND_OPEN ? "File" : "Emulation",
            .shortcut = specs[i].shortcut,
            .flags = specs[i].flags,
            .handler = specs[i].handler,
            .userdata = runtime
        };
        if (!frontend_command_register(&spec)) return false;
    }
    replay_frontend_unregister();
    return replay_frontend_register(runtime);
}

bool frontend_execution_handle_shortcut(FrontendExecutionRuntime *runtime,
                                        const SDL_KeyboardEvent *event) {
    if (!event || !runtime) return false;
    bool down = event->type == SDL_KEYDOWN;
    SDL_Keymod mods = (SDL_Keymod)event->keysym.mod;
    bool control = (mods & KMOD_CTRL) != 0;
    bool shift = (mods & KMOD_SHIFT) != 0;
    bool alt = (mods & KMOD_ALT) != 0;
    if (!control) return false;
    if (event->keysym.scancode == SDL_SCANCODE_F && !shift) {
        return frontend_execution_handle_shortcut_action(
            runtime, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, down, event->repeat != 0);
    }
    if (!down || event->repeat) return false;
    FrontendShortcut shortcut;
    switch (event->keysym.scancode) {
        case SDL_SCANCODE_P: shortcut = FRONTEND_SHORTCUT_PAUSE; break;
        case SDL_SCANCODE_PERIOD: shortcut = FRONTEND_SHORTCUT_FRAME_ADVANCE; break;
        case SDL_SCANCODE_R:
            shortcut = alt ? FRONTEND_SHORTCUT_RELOAD
                           : shift ? FRONTEND_SHORTCUT_POWER_CYCLE
                                   : FRONTEND_SHORTCUT_SOFT_RESET;
            break;
        case SDL_SCANCODE_F:
            if (!shift) return false;
            shortcut = FRONTEND_SHORTCUT_FAST_FORWARD_TOGGLE;
            break;
        case SDL_SCANCODE_1: shortcut = FRONTEND_SHORTCUT_SPEED_HALF; break;
        case SDL_SCANCODE_2: shortcut = FRONTEND_SHORTCUT_SPEED_NORMAL; break;
        case SDL_SCANCODE_3: shortcut = FRONTEND_SHORTCUT_SPEED_DOUBLE; break;
        default: return false;
    }
    return frontend_execution_handle_shortcut_action(runtime, shortcut, down,
                                                     event->repeat != 0);
}

bool frontend_execution_handle_shortcut_action(FrontendExecutionRuntime *runtime,
                                               FrontendShortcut shortcut,
                                               bool down, bool repeat) {
    if (!runtime || shortcut >= FRONTEND_SHORTCUT_COUNT) return false;
    if (shortcut == FRONTEND_SHORTCUT_FAST_FORWARD_HOLD) {
        if (!repeat) {
            execution_control_set_fast_forward_held(&runtime->execution, down);
            refresh_audio(runtime);
        }
        return true;
    }
    if (!down || repeat) return true;
    static const unsigned commands[FRONTEND_SHORTCUT_COUNT] = {
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
        FRONTEND_COMMAND_OPEN
    };
    char error[160] = {0};
    if (!frontend_command_invoke(commands[shortcut], error, sizeof(error)) && error[0])
        fprintf(stderr, "%s\n", error);
    return true;
}

void frontend_execution_release_host_input(FrontendExecutionRuntime *runtime) {
    if (!runtime) return;
    if (runtime->execution.fast_forward_held) {
        execution_control_set_fast_forward_held(&runtime->execution, false);
        refresh_audio(runtime);
    }
}

bool frontend_execution_set_speeds(FrontendExecutionRuntime *runtime,
                                   double speed, double fast_forward_speed) {
    if (!runtime || !execution_control_set_speed(&runtime->execution, speed)
        || !execution_control_set_fast_forward_speed(&runtime->execution,
                                                    fast_forward_speed)) return false;
    refresh_audio(runtime);
    return true;
}

void frontend_execution_set_muted(FrontendExecutionRuntime *runtime, bool muted) {
    if (!runtime) return;
    runtime->muted = muted;
    refresh_audio(runtime);
}

bool frontend_execution_muted(const FrontendExecutionRuntime *runtime) {
    return runtime && runtime->muted;
}

static bool run_emulation_frame(void *userdata) {
    (void)userdata;
    vs_start_frame();
    while (!ppu.frame_complete && !debugger_is_paused()) vs_cpu_step();
    return ppu.frame_complete;
}

bool frontend_execution_run_frame(FrontendExecutionRuntime *runtime) {
    if (!runtime) return false;
    frontend_execution_sync_debugger(runtime);
    bool loading_fast_forward = fds_loading_fast_forward();
    if (runtime->execution.loading_fast_forward != loading_fast_forward) {
        execution_control_set_loading_fast_forward(&runtime->execution, loading_fast_forward);
        refresh_audio(runtime);
    }
    if (!execution_control_should_run_frame(&runtime->execution)) return false;

    if (nes_execution_policy() == NES_EXECUTION_LIVE && runtime->rewind.capacity) {
        lock_audio_for_machine_change(runtime);
        runtime->replay_status = nes_rewind_capture(&runtime->rewind,
                                                    &runtime->replay_state_status);
        unlock_audio_without_refresh(runtime);
        if (runtime->replay_status == NES_REPLAY_DISABLED)
            runtime->replay_status = NES_REPLAY_OK;
    }

    bool ran = false;
    if (runtime->run_ahead_frames && nes_execution_policy() == NES_EXECUTION_LIVE) {
        lock_audio_for_machine_change(runtime);
        runtime->replay_status = nes_runahead_execute(runtime->run_ahead_frames,
                                                      run_emulation_frame, NULL,
                                                      &runtime->replay_state_status);
        unlock_audio_without_refresh(runtime);
        ran = runtime->replay_status == NES_REPLAY_OK;
    } else {
        ran = run_emulation_frame(NULL);
    }
    if (ran) execution_control_frame_complete(&runtime->execution);
    frontend_execution_sync_debugger(runtime);
    if (runtime->execution.paused) update_audio_pause(runtime);
    return ran;
}

bool frontend_execution_paused(const FrontendExecutionRuntime *runtime) {
    return runtime && runtime->execution.paused;
}

double frontend_execution_speed(const FrontendExecutionRuntime *runtime) {
    return runtime ? execution_control_effective_speed(&runtime->execution) : 1.0;
}

bool frontend_execution_set_rewind_seconds(FrontendExecutionRuntime *runtime,
                                           unsigned seconds) {
    if (!runtime || seconds > 60) return false;
    const NesTiming *timing = nes_timing();
    double fps = timing && timing->fps > 0.0 ? timing->fps : 60.0;
    size_t frames = seconds ? (size_t)ceil((double)seconds * fps) : 0;
    if (frames > NES_REWIND_MAX_FRAMES) return false;
    if (!nes_rewind_configure(&runtime->rewind, frames,
                              NES_REWIND_DEFAULT_MEMORY_LIMIT)) return false;
    runtime->rewind_seconds = seconds;
    runtime->replay_status = NES_REPLAY_OK;
    runtime->replay_state_status = NES_STATE_OK;
    return true;
}

unsigned frontend_execution_rewind_seconds(const FrontendExecutionRuntime *runtime) {
    return runtime ? runtime->rewind_seconds : 0;
}

size_t frontend_execution_rewind_available(const FrontendExecutionRuntime *runtime) {
    return runtime ? nes_rewind_count(&runtime->rewind) : 0;
}

bool frontend_execution_rewind_step(FrontendExecutionRuntime *runtime,
                                    char *error, size_t error_size) {
    if (!runtime) return false;
    if (runtime->before_machine_change
        && !runtime->before_machine_change(runtime->machine_change_context, error, error_size)) return false;
    lock_audio_for_machine_change(runtime);
    runtime->replay_status = nes_rewind_step(&runtime->rewind,
                                             &runtime->replay_state_status);
    unlock_audio_after_machine_change(runtime);
    if (runtime->replay_status == NES_REPLAY_OK) return true;
    if (runtime->replay_status == NES_REPLAY_STATE_ERROR) {
        char message[160];
        snprintf(message, sizeof(message), "Rewind failed: %s",
                 nes_state_result_string(runtime->replay_state_status));
        set_error(error, error_size, message);
    } else {
        set_error(error, error_size, nes_replay_result_string(runtime->replay_status));
    }
    return false;
}

void frontend_execution_clear_timeline(FrontendExecutionRuntime *runtime) {
    if (!runtime) return;
    nes_rewind_clear(&runtime->rewind);
    nes_runahead_clear_presented_frame();
    runtime->replay_status = NES_REPLAY_OK;
    runtime->replay_state_status = NES_STATE_OK;
}

void frontend_execution_shutdown(FrontendExecutionRuntime *runtime) {
    if (!runtime) return;
    replay_frontend_unregister();
    nes_rewind_destroy(&runtime->rewind);
    nes_runahead_clear_presented_frame();
    runtime->rewind_seconds = 0;
    runtime->run_ahead_frames = 0;
}

bool frontend_execution_set_run_ahead(FrontendExecutionRuntime *runtime, unsigned frames) {
    if (!runtime || frames > 4) return false;
    runtime->run_ahead_frames = frames;
    if (!frames) nes_runahead_clear_presented_frame();
    runtime->replay_status = NES_REPLAY_OK;
    runtime->replay_state_status = NES_STATE_OK;
    return true;
}

unsigned frontend_execution_run_ahead(const FrontendExecutionRuntime *runtime) {
    return runtime ? runtime->run_ahead_frames : 0;
}

NesReplayResult frontend_execution_replay_status(const FrontendExecutionRuntime *runtime,
                                                 NesStateResult *state_result) {
    if (!runtime) {
        if (state_result) *state_result = NES_STATE_ERROR_ARGUMENT;
        return NES_REPLAY_STATE_ERROR;
    }
    if (state_result) *state_result = runtime->replay_state_status;
    return runtime->replay_status;
}
