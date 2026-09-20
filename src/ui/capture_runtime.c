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

static bool get_frame(void *context, bool displayed, NesCaptureFrame *frame,
                        char *error, size_t error_size) {
    NesCaptureRuntime *capture = context;
    if (!has_image(context) || !frame) {
        if (error && error_size) snprintf(error, error_size, "No image is loaded");
        return false;
    }
    if (capture->video) {
        const uint32_t *pixels = NULL;
        unsigned width = 0, height = 0, stride = 0;
        if (!frontend_video_runtime_pixels(capture->video, displayed, &pixels,
                                           &width, &height, &stride)) {
            if (error && error_size) snprintf(error, error_size, "No completed video frame is available");
            return false;
        }
        *frame = (NesCaptureFrame){pixels, width, height, stride};
        return true;
    }
    if (displayed && capture->composite_enabled && *capture->composite_enabled
        && capture->composite_pixels && ntsc_composite_supported(nes_timing()->region, vs_enabled())) {
        ntsc_composite_filter_frame(ppu.pixel_signal, ppu.completed_video_phase, capture->composite_pixels);
        *frame = (NesCaptureFrame){capture->composite_pixels, NTSC_COMPOSITE_WIDTH,
                                   NTSC_COMPOSITE_HEIGHT, NTSC_COMPOSITE_WIDTH};
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
    NesFileResult result = nes_capture_session_stop(&capture->frontend.session);
    nes_capture_frontend_refresh(&capture->frontend);
    if (result != NES_FILE_OK && error && error_size)
        snprintf(error, error_size, "%s", capture->frontend.session.error);
    return result == NES_FILE_OK;
}

bool nes_capture_runtime_init(NesCaptureRuntime *capture, FrontendExecutionRuntime *execution,
                               const bool *composite_enabled, uint32_t *composite_pixels,
                               const char *const *protected_paths, size_t path_count) {
    if (!capture || !execution || execution->before_machine_change || (path_count && !protected_paths)) return false;
    memset(capture, 0, sizeof(*capture));
    capture->execution = execution;
    capture->composite_enabled = composite_enabled;
    capture->composite_pixels = composite_pixels;
    capture->protected_paths = protected_paths;
    capture->protected_path_count = path_count;
    NesCaptureFrontendHooks hooks = {has_image, get_frame, validate_path, capture};
    if (!nes_capture_frontend_init(&capture->frontend, &hooks)) return false;
    execution->before_machine_change = before_machine_change;
    execution->machine_change_context = capture;
    return true;
}

void nes_capture_runtime_set_video(NesCaptureRuntime *capture, FrontendVideoRuntime *video) {
    if (capture) capture->video = video;
}

NesFileResult nes_capture_runtime_shutdown(NesCaptureRuntime *capture) {
    if (!capture) return NES_FILE_INVALID_ARGUMENT;
    if (capture->execution && capture->execution->machine_change_context == capture) {
        capture->execution->before_machine_change = NULL;
        capture->execution->machine_change_context = NULL;
    }
    NesFileResult result = nes_capture_frontend_shutdown(&capture->frontend);
    capture->execution = NULL;
    return result;
}
