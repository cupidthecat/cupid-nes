/*
 * capture_runtime.h - Capture from the application output and machine controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NES_CAPTURE_RUNTIME_H
#define NES_CAPTURE_RUNTIME_H

#include "capture_frontend.h"
#include "frontend_execution.h"
#include "video_runtime.h"
#include "device_frontend.h"

typedef struct {
    NesCaptureFrontend frontend;
    FrontendExecutionRuntime *execution;
    const bool *composite_enabled;
    uint32_t *composite_pixels;
    FrontendVideoRuntime *video;
    FrontendDeviceRuntime *devices;
    const char *const *protected_paths;
    size_t protected_path_count;
    NesCaptureOverlay display_overlay;
    NesMovieSubtitles *subtitles;
    uint64_t subtitle_revision;
    uint64_t movie_generation;
    uint64_t last_movie_frame;
    bool last_movie_active;
} NesCaptureRuntime;

bool nes_capture_runtime_init(NesCaptureRuntime *capture, FrontendExecutionRuntime *execution,
                              const bool *composite_enabled, uint32_t *composite_pixels,
                              const char *const *protected_paths, size_t path_count);
void nes_capture_runtime_set_video(NesCaptureRuntime *capture, FrontendVideoRuntime *video);
/* Apply saved defaults after initialization; rejects unsafe mid-frame TAS changes. */
bool nes_capture_runtime_preferences(NesCaptureRuntime *capture, const NesMoviePreferences *preferences, char *error,
                                     size_t size);
/* Use from the shared end-of-movie branch. Returns true when handled. */
bool nes_capture_movie_end(FrontendExecutionRuntime *execution, const NesMoviePreferences *preferences, char *error,
                           size_t size);
NesFileResult nes_capture_runtime_shutdown(NesCaptureRuntime *capture);

#endif
