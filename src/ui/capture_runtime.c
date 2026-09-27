/*
 * capture_runtime.c - Capture from the application output and machine controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "capture_runtime.h"
#include "output_guard.h"
#include "frontend_commands.h"
#include "../replay/tas_session.h"
#include "../replay/tas_project.h"
#include "../rom/fds.h"
#include "../system/vs_system.h"
#include "../video/ntsc_composite.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <string.h>

static bool has_image(void *context) {
    NesCaptureRuntime *capture = context;
    return capture->execution && capture->execution->rom_path && capture->execution->rom_path[0];
}

static bool get_frame(void *context, bool displayed, NesCaptureFrame *frame, char *error, size_t error_size) {
    NesCaptureRuntime *capture = context;
    if (!has_image(context) || !frame) {
        if (error && error_size) {
            snprintf(error, error_size, "No image is loaded");
        }
        return false;
    }
    if (capture->video) {
        const uint32_t *pixels = NULL;
        unsigned width = 0, height = 0, stride = 0;
        if (!frontend_video_runtime_pixels(capture->video, displayed, &pixels, &width, &height, &stride)) {
            if (error && error_size) {
                snprintf(error, error_size, "No completed video frame is available");
            }
            return false;
        }
        *frame = (NesCaptureFrame){pixels, width, height, stride};
        return true;
    }
    if (displayed && capture->composite_enabled && *capture->composite_enabled && capture->composite_pixels &&
        ntsc_composite_supported(nes_timing()->region, vs_enabled())) {
        ntsc_composite_filter_frame(ppu.pixel_signal, ppu.completed_video_phase, capture->composite_pixels);
        *frame = (NesCaptureFrame){capture->composite_pixels, NTSC_COMPOSITE_WIDTH, NTSC_COMPOSITE_HEIGHT,
                                   NTSC_COMPOSITE_WIDTH};
    } else {
        *frame = (NesCaptureFrame){vs_video_framebuffer(), vs_video_width(), SCREEN_HEIGHT, vs_video_width()};
    }
    return true;
}

static bool validate_path(void *context, const char *path, char *error, size_t error_size) {
    NesCaptureRuntime *capture = context;
    return frontend_output_path_allowed(path, capture->execution, capture->protected_paths,
                                        capture->protected_path_count, error, error_size);
}

static bool before_machine_change(void *context, char *error, size_t error_size) {
    NesCaptureRuntime *capture = context;
    if (capture->devices && !frontend_devices_finish(capture->devices, error, error_size)) {
        return false;
    }
    NesFileResult result = nes_capture_session_stop(&capture->frontend.session);
    nes_capture_frontend_refresh(&capture->frontend);
    if (result != NES_FILE_OK && error && error_size) {
        snprintf(error, error_size, "%s", capture->frontend.session.error);
    }
    return result == NES_FILE_OK;
}

static bool apply_preferences(void *context, const NesMoviePreferences *preferences, char *error, size_t size) {
    NesCaptureRuntime *capture = context;
    if (!nes_movie_preferences_valid(preferences)) {
        return false;
    }
    NesTasSession *tas = nes_movie_tas(capture->execution->movie);
    NesTasProgress progress;
    nes_tas_session_progress(tas, &progress);
    bool read_change = preferences->read_only != capture->frontend.preferences.read_only;
    bool mode_change = preferences->record_insert != capture->frontend.preferences.record_insert;
    if (progress.active && (read_change || mode_change)) {
        if (progress.frame_in_progress || progress.seeking) {
            if (error && size) {
                snprintf(error, size, "Finish the current frame or seek before changing movie editing defaults");
            }
            return false;
        }
        NesMovieResult result = NES_MOVIE_OK;
        if (mode_change) {
            result = nes_tas_session_set_recording(
                tas, progress.recording, preferences->record_insert ? NES_TAS_RECORD_INSERT : NES_TAS_RECORD_OVERWRITE,
                progress.record_players ? progress.record_players : 3);
        }
        if (result == NES_MOVIE_OK && read_change) {
            result = nes_tas_session_set_read_only(tas, preferences->read_only);
        }
        if (result != NES_MOVIE_OK) {
            if (error && size) {
                snprintf(error, size, "%s", nes_tas_session_error(tas));
            }
            return false;
        }
    }
    capture->execution->movie_start_kind =
        preferences->record_power_on ? NES_MOVIE_START_POWER_ON : NES_MOVIE_START_STATE;
    nes_movie_backup_set_policy(&preferences->backup);
    return true;
}

bool nes_capture_runtime_preferences(NesCaptureRuntime *capture, const NesMoviePreferences *preferences, char *error,
                                     size_t size) {
    if (!capture || !capture->execution || !apply_preferences(capture, preferences, error, size)) {
        return false;
    }
    capture->frontend.preferences = *preferences;
    return true;
}

bool nes_capture_movie_end(FrontendExecutionRuntime *execution, const NesMoviePreferences *preferences, char *error,
                           size_t size) {
    if (!execution || !nes_movie_preferences_valid(preferences)) {
        return false;
    }
    if (preferences->end_behavior == NES_MOVIE_END_STOP) {
        if (!frontend_execution_movie_stop(execution, error, size)) {
            return false;
        }
        snprintf(execution->movie_status, sizeof(execution->movie_status),
                 "Playback completed. The previous session was restored.");
        return true;
    }
    execution_control_set_paused(&execution->execution, true);
    execution->execution.frame_advance_pending = false;
    (void)frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, true);
    frontend_execution_refresh_audio(execution);
    snprintf(execution->movie_status, sizeof(execution->movie_status),
             "End of movie. Stop restores the previous game.");
    return true;
}

static void refresh_overlay(void *context, bool completed) {
    NesCaptureRuntime *capture = context;
    NesCaptureFrontend *f = &capture->frontend;
    NesTasSession *tas = nes_movie_tas(capture->execution->movie);
    NesTasProgress progress;
    nes_tas_session_progress(tas, &progress);
    NesMovieProgress movie;
    nes_movie_progress(capture->execution->movie, &movie);
    bool active = movie.mode != NES_MOVIE_IDLE;
    uint64_t frame = active ? (movie.frame ? movie.frame - 1u : 0u) : ppu.frame_count;
    bool changed = capture->last_movie_active != active || (active && capture->last_movie_frame != frame);
    capture->last_movie_active = active;
    capture->last_movie_frame = frame;
    if (progress.active) {
        const NesTasProject *project = nes_tas_session_project_const(tas);
        uint64_t revision = nes_tas_project_revision(project);
        if (capture->movie_generation != progress.generation || capture->subtitle_revision != revision) {
            if (capture->movie_generation != progress.generation && !progress.frame_in_progress && !progress.seeking) {
                if (!progress.recording) {
                    (void)nes_tas_session_set_read_only(tas, f->preferences.read_only);
                }
                if ((progress.recording || !f->preferences.read_only) &&
                    progress.record_mode !=
                        (f->preferences.record_insert ? NES_TAS_RECORD_INSERT : NES_TAS_RECORD_OVERWRITE)) {
                    (void)nes_tas_session_set_recording(tas, progress.recording,
                                                        f->preferences.record_insert ? NES_TAS_RECORD_INSERT
                                                                                     : NES_TAS_RECORD_OVERWRITE,
                                                        progress.record_players ? progress.record_players : 3);
                }
            }
            const NesFm2Movie *metadata = nes_tas_project_movie(project);
            NesFileResult result = nes_movie_subtitles_load(&capture->subtitles, &metadata->subtitles);
            if (result != NES_FILE_OK) {
                nes_movie_subtitles_free(&capture->subtitles);
                snprintf(f->session.error, sizeof(f->session.error), "Movie subtitles cannot be displayed: %s",
                         nes_file_result_message(result));
            }
            capture->movie_generation = progress.generation;
            capture->subtitle_revision = revision;
            changed = true;
        }
    } else {
        nes_movie_subtitles_free(&capture->subtitles);
        capture->movie_generation = capture->subtitle_revision = 0;
    }
    NesCaptureOverlayState *state = &f->overlay_state;
    state->movie_active = active;
    state->frame = frame;
    if (completed || changed) {
        state->active_players = 0;
        for (unsigned i = 0; i < 6; ++i) {
            state->pads[i] = joypad_player(i)->buttons;
        }
        for (unsigned i = 0; i < 2; ++i) {
            if (joypad_port_device(i) == NES_PORT_GAMEPAD) {
                state->active_players |= 1u << i;
            }
        }
        if (joypad_adapter() == NES_ADAPTER_FOUR_SCORE) {
            state->active_players |= 15;
        }
        if (joypad_adapter() == NES_ADAPTER_FAMICOM_TWO) {
            state->active_players |= 12;
        }
        if (joypad_adapter() == NES_ADAPTER_FAMICOM_FOUR) {
            state->active_players |= 60;
        }
        if (vs_enabled()) {
            state->active_players = 0;
            for (unsigned i = 0; i < 4; ++i) {
                if (vs_input_overlay_buttons(i, &state->pads[i])) {
                    state->active_players |= 1u << i;
                }
            }
        }
    }
    snprintf(state->subtitle, sizeof(state->subtitle), "%s",
             progress.active ? nes_movie_subtitle_at(capture->subtitles, frame, f->preferences.subtitle_duration) : "");
    snprintf(state->status, sizeof(state->status), "%s%s",
             progress.active ? "TAS "
             : active        ? "Movie "
                             : "",
             progress.seeking                                ? "Seeking"
             : frontend_execution_paused(capture->execution) ? "Paused"
             : movie.mode == NES_MOVIE_RECORDING             ? "Recording"
             : active                                        ? "Playback"
                                                             : "Live");
    nes_tas_session_progress(tas, &progress);
    snprintf(f->effective_preferences, sizeof(f->effective_preferences),
             "%s; effective access: %s; recording: %s; overlays use completed emulated frames", state->status,
             progress.active ? (progress.read_only ? "read-only" : "editable") : "default",
             progress.active ? (progress.record_mode == NES_TAS_RECORD_INSERT ? "insert" : "overwrite") : "default");
    if (capture->video && capture->video->texture) {
        NesCaptureFrame source, output;
        char error[128];
        unsigned mask = f->preferences.display_overlays & ~NES_OVERLAY_SUBTITLE;
        if (f->preferences.subtitles) {
            mask |= NES_OVERLAY_SUBTITLE;
        }
        if (get_frame(capture, true, &source, error, sizeof(error)) &&
            nes_capture_overlay_compose(&capture->display_overlay, &source, state, mask, f->preferences.position,
                                        &output) == NES_FILE_OK) {
            (void)SDL_UpdateTexture(capture->video->texture, NULL, output.pixels, (int)(output.stride * 4u));
        }
    }
}

bool nes_capture_runtime_init(NesCaptureRuntime *capture, FrontendExecutionRuntime *execution,
                              const bool *composite_enabled, uint32_t *composite_pixels,
                              const char *const *protected_paths, size_t path_count) {
    if (!capture || !execution || execution->before_machine_change || (path_count && !protected_paths)) {
        return false;
    }
    memset(capture, 0, sizeof(*capture));
    capture->execution = execution;
    capture->composite_enabled = composite_enabled;
    capture->composite_pixels = composite_pixels;
    capture->protected_paths = protected_paths;
    capture->protected_path_count = path_count;
    NesCaptureFrontendHooks hooks = {has_image, get_frame, validate_path, capture};
    if (!nes_capture_frontend_init(&capture->frontend, &hooks)) {
        return false;
    }
    capture->frontend.overlay_context = capture;
    capture->frontend.refresh_overlay = refresh_overlay;
    capture->frontend.apply_preferences = apply_preferences;
    execution->before_machine_change = before_machine_change;
    execution->machine_change_context = capture;
    return true;
}

void nes_capture_runtime_set_video(NesCaptureRuntime *capture, FrontendVideoRuntime *video) {
    if (capture) {
        capture->video = video;
    }
}

NesFileResult nes_capture_runtime_shutdown(NesCaptureRuntime *capture) {
    if (!capture) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (capture->execution && capture->execution->machine_change_context == capture) {
        capture->execution->before_machine_change = NULL;
        capture->execution->machine_change_context = NULL;
    }
    NesFileResult result = nes_capture_frontend_shutdown(&capture->frontend);
    nes_capture_overlay_destroy(&capture->display_overlay);
    nes_movie_subtitles_free(&capture->subtitles);
    capture->execution = NULL;
    return result;
}
