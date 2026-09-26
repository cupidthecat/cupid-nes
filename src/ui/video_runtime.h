/*
 * video_runtime.h - Desktop presentation frame selection and rendering
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_VIDEO_RUNTIME_H
#define FRONTEND_VIDEO_RUNTIME_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "settings.h"
#include "hd_pack_frontend.h"
#include "frontend_session.h"
#include "../video/presentation.h"
#include "../video/shader_preset.h"
#include "../hd/hd_builder.h"

typedef struct {
    NesHdRuntime *hd;
    NesHdFrontend *hd_frontend;
    NesPixelFilter *pixel_filter;
    NesShaderPreset *shader;
    NesHdBuilder *builder;
    NesHdFrame builder_frame;
    bool builder_preview;
    char shader_error[512];
    char shader_requested_path[1024];
    char builder_error[256];
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    FrontendSettings *settings;
    NesVideoPresentationFrame frame;
    const uint32_t *source_pixels;
    unsigned source_width;
    unsigned source_height;
    unsigned texture_width;
    unsigned texture_height;
    bool composite;
} FrontendVideoRuntime;

bool frontend_video_runtime_init(FrontendVideoRuntime *runtime, SDL_Renderer *renderer,
                                 FrontendSettings *settings, bool composite,
                                 char *error, size_t error_size);
bool frontend_video_runtime_attach_hd(FrontendVideoRuntime *runtime,
    const FrontendSession *session, char *error, size_t error_size);
void frontend_video_runtime_set_composite(FrontendVideoRuntime *runtime, bool composite);
bool frontend_video_runtime_refresh(FrontendVideoRuntime *runtime,
                                    char *error, size_t error_size);
void frontend_video_runtime_shutdown(FrontendVideoRuntime *runtime);

SDL_Texture *frontend_video_runtime_texture(const FrontendVideoRuntime *runtime);
bool frontend_video_runtime_pixels(const FrontendVideoRuntime *runtime, bool displayed,
                                   const uint32_t **pixels, unsigned *width,
                                   unsigned *height, unsigned *stride);
void frontend_video_runtime_display_size(const FrontendVideoRuntime *runtime,
                                         unsigned *width, unsigned *height);
bool frontend_video_runtime_aim(const FrontendVideoRuntime *runtime, int x, int y,
                                unsigned *side, int *nes_x, int *nes_y);
bool frontend_video_runtime_load_shader(FrontendVideoRuntime *runtime, const char *path,
                                        char *error, size_t error_size);
void frontend_video_runtime_unload_image(FrontendVideoRuntime *runtime);

#endif
