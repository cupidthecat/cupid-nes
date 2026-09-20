/*
 * capture_frontend.h - Screenshot and recording commands and controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NES_CAPTURE_FRONTEND_H
#define NES_CAPTURE_FRONTEND_H

#include "../capture/capture_session.h"
#include "platform_frontend.h"
#include <SDL2/SDL.h>

enum {
    CAPTURE_COMMAND_SCREENSHOT = 0x1600,
    CAPTURE_COMMAND_AUDIO,
    CAPTURE_COMMAND_VIDEO,
    CAPTURE_COMMAND_STOP,
    CAPTURE_PANEL = 0x1600,
    CAPTURE_PATH_CAPACITY = 1024
};

typedef struct {
    bool (*has_image)(void *context);
    bool (*get_frame)(void *context, bool displayed_output, NesCaptureFrame *frame,
                       char *error, size_t error_size);
    bool (*validate_path)(void *context, const char *path, char *error, size_t error_size);
    void *context;
} NesCaptureFrontendHooks;

typedef struct {
    NesCaptureSession session;
    NesCaptureOptions options;
    NesCaptureFrontendHooks hooks;
    char paths[3][CAPTURE_PATH_CAPACITY];
    void *ui;
} NesCaptureFrontend;

bool nes_capture_frontend_init(NesCaptureFrontend *frontend, const NesCaptureFrontendHooks *hooks);
NesFileResult nes_capture_frontend_shutdown(NesCaptureFrontend *frontend);
void nes_capture_frontend_refresh(NesCaptureFrontend *frontend);
bool nes_capture_frontend_set_path(NesCaptureFrontend *frontend, FrontendSaveFileType type,
                                    const char *path, char *error, size_t error_size);
bool nes_capture_path_allowed(const char *path, const char *const *protected_paths, size_t count,
                               char *error, size_t error_size);
void nes_capture_frontend_begin_frame(NesCaptureFrontend *frontend);
void nes_capture_frontend_end_frame(NesCaptureFrontend *frontend, bool completed);
bool nes_capture_frontend_handle_shortcut(NesCaptureFrontend *frontend,
                                          const SDL_KeyboardEvent *event,
                                          char *error, size_t error_size);

#endif
