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

typedef struct {
    NesCaptureFrontend frontend;
    FrontendExecutionRuntime *execution;
    const bool *composite_enabled;
    uint32_t *composite_pixels;
    const char *const *protected_paths;
    size_t protected_path_count;
} NesCaptureRuntime;

bool nes_capture_runtime_init(NesCaptureRuntime *capture, FrontendExecutionRuntime *execution,
                               const bool *composite_enabled, uint32_t *composite_pixels,
                               const char *const *protected_paths, size_t path_count);
NesFileResult nes_capture_runtime_shutdown(NesCaptureRuntime *capture);

#endif
