/*
 * frontend_execution.h - SDL execution controls and machine commands
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_EXECUTION_H
#define FRONTEND_EXECUTION_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include "execution_control.h"
#include "../replay/movie.h"
#include "../replay/rewind.h"
#include "settings.h"

typedef bool (*FrontendOpenHandler)(void *userdata, char *error, size_t error_size);
typedef bool (*FrontendReloadHandler)(void *userdata, char *error, size_t error_size);
typedef void (*FrontendRestoreHandler)(void *userdata);

enum { FRONTEND_MOVIE_PATH_CAPACITY = 1024 };

typedef struct FrontendExecutionRuntime {
    ExecutionControl execution;
    SDL_AudioDeviceID *audio_device;
    int audio_output_rate;
    bool muted;
    const char *rom_path;
    const char *save_identity;
    const char *fds_bios_path;
    const char *studybox_bios_path;
    size_t *fds_side;
    uint64_t debugger_pause_revision;
    NesRewindHistory rewind;
    unsigned rewind_seconds;
    unsigned run_ahead_frames;
    NesReplayResult replay_status;
    NesStateResult replay_state_status;
    NesMovieSession *movie;
    char movie_path[FRONTEND_MOVIE_PATH_CAPACITY];
    NesMovieStartKind movie_start_kind;
    char movie_status[192];
    const char *const *protected_paths;
    size_t protected_path_count;
    FrontendRestoreHandler restore_handler;
    void *restore_userdata;
    bool (*before_machine_change)(void *context, char *error, size_t error_size);
    void *machine_change_context;
    FrontendOpenHandler open_handler;
    void *open_userdata;
    FrontendReloadHandler reload_handler;
    void *reload_userdata;
} FrontendExecutionRuntime;

void frontend_execution_init(FrontendExecutionRuntime *runtime,
                             SDL_AudioDeviceID *audio_device, int audio_output_rate,
                             const char *rom_path, const char *fds_bios_path,
                             const char *studybox_bios_path, size_t *fds_side);
bool frontend_execution_register_commands(FrontendExecutionRuntime *runtime);
void frontend_execution_set_open_handler(FrontendExecutionRuntime *runtime,
                                         FrontendOpenHandler handler, void *userdata);
void frontend_execution_set_reload_handler(FrontendExecutionRuntime *runtime,
                                           FrontendReloadHandler handler, void *userdata);
void frontend_execution_set_restore_handler(FrontendExecutionRuntime *runtime,
                                            FrontendRestoreHandler handler, void *userdata);
void frontend_execution_set_protected_paths(FrontendExecutionRuntime *runtime,
                                            const char *const *paths, size_t count);
void frontend_execution_begin_machine_change(FrontendExecutionRuntime *runtime);
void frontend_execution_end_machine_change(FrontendExecutionRuntime *runtime);
void frontend_execution_end_machine_change_preserving_audio(FrontendExecutionRuntime *runtime);
bool frontend_execution_handle_shortcut(FrontendExecutionRuntime *runtime,
                                        const SDL_KeyboardEvent *event);
bool frontend_execution_handle_shortcut_action(FrontendExecutionRuntime *runtime,
                                               FrontendShortcut shortcut,
                                               bool down, bool repeat);
void frontend_execution_release_host_input(FrontendExecutionRuntime *runtime);
bool frontend_execution_set_speeds(FrontendExecutionRuntime *runtime,
                                   double speed, double fast_forward_speed);
void frontend_execution_set_muted(FrontendExecutionRuntime *runtime, bool muted);
bool frontend_execution_muted(const FrontendExecutionRuntime *runtime);
bool frontend_execution_run_frame(FrontendExecutionRuntime *runtime);
void frontend_execution_sync_debugger(FrontendExecutionRuntime *runtime);
bool frontend_execution_paused(const FrontendExecutionRuntime *runtime);
double frontend_execution_speed(const FrontendExecutionRuntime *runtime);

bool frontend_execution_set_rewind_seconds(FrontendExecutionRuntime *runtime,
                                           unsigned seconds);
unsigned frontend_execution_rewind_seconds(const FrontendExecutionRuntime *runtime);
size_t frontend_execution_rewind_available(const FrontendExecutionRuntime *runtime);
bool frontend_execution_rewind_step(FrontendExecutionRuntime *runtime,
                                    char *error, size_t error_size);
void frontend_execution_clear_timeline(FrontendExecutionRuntime *runtime);
void frontend_execution_shutdown(FrontendExecutionRuntime *runtime);

bool frontend_execution_set_run_ahead(FrontendExecutionRuntime *runtime, unsigned frames);
unsigned frontend_execution_run_ahead(const FrontendExecutionRuntime *runtime);
NesReplayResult frontend_execution_replay_status(const FrontendExecutionRuntime *runtime,
                                                 NesStateResult *state_result);
bool frontend_execution_movie_set_path(FrontendExecutionRuntime *runtime, const char *path,
                                       char *error, size_t error_size);
bool frontend_execution_movie_set_start_kind(FrontendExecutionRuntime *runtime,
                                             NesMovieStartKind start_kind);
bool frontend_execution_movie_record(FrontendExecutionRuntime *runtime,
                                     char *error, size_t error_size);
bool frontend_execution_movie_play(FrontendExecutionRuntime *runtime,
                                   char *error, size_t error_size);
bool frontend_execution_movie_stop(FrontendExecutionRuntime *runtime,
                                   char *error, size_t error_size);
void frontend_execution_movie_progress(const FrontendExecutionRuntime *runtime,
                                       NesMovieProgress *progress);

#endif
