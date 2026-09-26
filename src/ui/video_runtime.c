/*
 * video_runtime.c - Desktop presentation frame selection and rendering
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "video_runtime.h"
#include "app_paths.h"
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../replay/rewind.h"
#include "../system/vs_system.h"
#include "../system/timing.h"
#include <stdio.h>
#include <string.h>

static bool fail(char *error, size_t error_size, const char *message) {
    if (error && error_size && error != message) snprintf(error, error_size, "%s", message ? message : "Video output failed");
    return false;
}

static bool ensure_texture(FrontendVideoRuntime *runtime, unsigned width, unsigned height,
                           char *error, size_t error_size) {
    SDL_ScaleMode mode = runtime->settings->bilinear_interpolation ? SDL_ScaleModeLinear : SDL_ScaleModeNearest;
    if (runtime->texture && runtime->texture_width == width && runtime->texture_height == height) {
        if (SDL_SetTextureScaleMode(runtime->texture, mode) != 0) {
            return fail(error, error_size, SDL_GetError());
        }

        return true;
    }

    SDL_Texture *replacement = SDL_CreateTexture(runtime->renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, (int)width, (int)height);
    if (!replacement) return fail(error, error_size, SDL_GetError());
    if (SDL_SetTextureScaleMode(replacement, mode) != 0) {
        SDL_DestroyTexture(replacement);
        return fail(error, error_size, SDL_GetError());
    }

    if (runtime->texture) SDL_DestroyTexture(runtime->texture);
    runtime->texture = replacement;
    runtime->texture_width = width;
    runtime->texture_height = height;
    return true;
}

static bool source_for_current_frame(FrontendVideoRuntime *runtime,
                                     NesVideoPresentationSource *source,
                                     char *error, size_t error_size) {
    unsigned width = 0, height = 0;
    const uint32_t *future = nes_runahead_presented_frame(&width, &height);
    bool use_future = future != NULL;
    if (!use_future) {
        width = vs_video_width();
        height = 240;
    }
    const uint32_t *pixels = use_future ? future : vs_video_completed_framebuffer();
    if (!pixels || height != 240 || (width != 256 && width != 512))
        return fail(error, error_size, "The completed video frame has invalid dimensions");

    memset(source, 0, sizeof(*source));
    source->pixels = pixels;
    source->screens = width / 256;
    source->region = nes_timing()->region;
    source->vs_system = vs_enabled();
    source->composite = runtime->composite;
    source->ntsc_settings = &runtime->settings->ntsc_picture;
    for (unsigned side = 0; side < source->screens; ++side) {
        source->signals[side] = use_future
            ? nes_runahead_presented_signal(side, &source->phases[side])
            : vs_video_completed_signal(side, &source->phases[side]);
        source->trace[side] = use_future
            ? nes_runahead_presented_trace(side)
            : vs_video_completed_trace(side);
    }
    runtime->source_pixels = pixels;
    runtime->source_width = width;
    runtime->source_height = height;
    return true;
}

bool frontend_video_runtime_init(FrontendVideoRuntime *runtime, SDL_Renderer *renderer,
                                 FrontendSettings *settings, bool composite,
                                 char *error, size_t error_size) {
    if (!runtime || !renderer || !settings) return fail(error, error_size, "Video runtime is incomplete");
    memset(runtime, 0, sizeof(*runtime));
    runtime->renderer = renderer;
    runtime->settings = settings;
    runtime->composite = composite;
    runtime->pixel_filter = nes_pixel_filter_create();
    runtime->shader = nes_shader_create();
    runtime->builder = nes_hd_builder_create();
    if (!runtime->pixel_filter || !runtime->shader || !runtime->builder) {
        frontend_video_runtime_shutdown(runtime);
        return fail(error, error_size, "Could not allocate video presentation tools");
    }
    if (!nes_video_presentation_set(&settings->presentation, error, error_size)
        || !frontend_video_runtime_refresh(runtime, error, error_size)) {
        frontend_video_runtime_shutdown(runtime);
        return false;
    }
    return true;
}

static uint8_t hd_memory(void *context, unsigned side, bool ppu_space, uint16_t address) {
    (void)context;
    return vs_debug_peek_memory(side, ppu_space, address);
}

bool frontend_video_runtime_attach_hd(FrontendVideoRuntime *runtime,
    const FrontendSession *session, char *error, size_t error_size) {
    if (!runtime || !session || !session->active) return false;
    if (!runtime->hd) {
        char root[4096];
        if (!frontend_paths_join(root, sizeof(root), "hd-packs")) return false;
        runtime->hd = nes_hd_runtime_create(root, error, error_size);
        if (!runtime->hd) return false;
        runtime->hd_frontend = nes_hd_frontend_create(runtime->hd, error, error_size);
        if (!runtime->hd_frontend) return false;
        nes_hd_runtime_set_memory_reader(runtime->hd, hd_memory, NULL);
    }
    const FrontendImageResult *image = &session->current_result;
    NesHdGameInfo game = {
        .rom_path = image->archive_member[0] ? image->archive_member : session->current.path,
        .rom_sha1 = image->sha1[0] ? image->sha1 : NULL,
        .chr_rom = chr_rom, .chr_size = chr_size, .chr_is_rom = cart_has_chr_rom()
    };
    if (!nes_hd_runtime_set_game(runtime->hd, &game, error, error_size)) return false;
    if (!nes_hd_builder_set_game(runtime->builder, &game, error, error_size)) return false;
    if (runtime->builder_preview) (void)nes_video_trace_use(NES_VIDEO_TRACE_BUILDER, false);
    runtime->builder_preview = false;
    memset(&runtime->builder_frame, 0, sizeof(runtime->builder_frame));
    nes_hd_frontend_set_capture_source(runtime->hd_frontend, NULL);
    return nes_hd_runtime_discover(runtime->hd, error, error_size)
        && nes_hd_frontend_restore_preferences(runtime->hd_frontend,image->sha1,error,error_size);
}

void frontend_video_runtime_set_composite(FrontendVideoRuntime *runtime, bool composite) {
    if (runtime) runtime->composite = composite;
}

bool frontend_video_runtime_refresh(FrontendVideoRuntime *runtime,
                                    char *error, size_t error_size) {
    if (!runtime || !runtime->renderer || !runtime->settings)
        return fail(error, error_size, "Video runtime is not initialized");
    NesVideoPresentationSource source;
    if (!source_for_current_frame(runtime, &source, error, error_size)) return false;
    if (!nes_video_presentation_render(&source, &runtime->frame, error, error_size)) return false;
    if (runtime->hd) {
        NesHdFrameSource hd_source = {
            .fallback_pixels = runtime->frame.pixels,
            .fallback_width = runtime->frame.width, .fallback_height = runtime->frame.height,
            .screens = source.screens, .trace = {source.trace[0], source.trace[1]},
            .overscan = {runtime->frame.overscan.left, runtime->frame.overscan.right,
                         runtime->frame.overscan.top, runtime->frame.overscan.bottom},
            .show_background = runtime->settings->presentation.show_background,
            .show_sprites = runtime->settings->presentation.show_sprites,
            .allow_pack_overscan = true
        };
        nes_hd_frontend_set_capture_source(runtime->hd_frontend, &hd_source);
        if (runtime->builder_preview) {
            if (!nes_hd_builder_preview(runtime->builder, &hd_source, hd_memory, NULL, &runtime->builder_frame,
                                         runtime->builder_error, sizeof(runtime->builder_error))) {
                memset(&runtime->builder_frame, 0, sizeof(runtime->builder_frame));
            }
        }
        NesHdFrame frame;
        if (!nes_hd_runtime_render(runtime->hd, &hd_source, &frame, error, error_size)) return false;
        if (frame.hd_rendered) {
            runtime->frame.pixels = frame.pixels;
            runtime->frame.width = frame.width;
            runtime->frame.height = frame.height;
            runtime->frame.overscan = (NesVideoOverscan){frame.overscan.left, frame.overscan.right,
                                                        frame.overscan.top, frame.overscan.bottom};
            runtime->frame.filtered = false;
        }
    }
    if (!nes_pixel_filter_render(runtime->pixel_filter, &runtime->settings->pixel_filter,
                                  &runtime->frame, &runtime->frame, error, error_size)) return false;
    if (nes_shader_enabled(runtime->shader)) {
        int width = 0, height = 0;
        SDL_GetRendererOutputSize(runtime->renderer, &width, &height);
        if (!nes_shader_render(runtime->shader, &runtime->frame,
                               width > 0 ? (unsigned)width : runtime->frame.width,
                               height > 0 ? (unsigned)height : runtime->frame.height,
                               &runtime->frame, runtime->shader_error, sizeof(runtime->shader_error))) {
            nes_shader_enable(runtime->shader, false);
        }
    }
    if (!ensure_texture(runtime, runtime->frame.width, runtime->frame.height, error, error_size)) return false;
    if (SDL_UpdateTexture(runtime->texture, NULL, runtime->frame.pixels,
                          (int)(runtime->frame.width * sizeof(uint32_t))) != 0)
        return fail(error, error_size, SDL_GetError());
    if (error && error_size) error[0] = '\0';
    return true;
}

void frontend_video_runtime_shutdown(FrontendVideoRuntime *runtime) {
    if (!runtime) return;
    nes_hd_frontend_destroy(runtime->hd_frontend);
    nes_hd_runtime_destroy(runtime->hd);
    nes_pixel_filter_destroy(runtime->pixel_filter);
    nes_shader_destroy(runtime->shader);
    nes_hd_builder_destroy(runtime->builder);
    if (runtime->builder_preview) (void)nes_video_trace_use(NES_VIDEO_TRACE_BUILDER, false);
    if (runtime->texture) SDL_DestroyTexture(runtime->texture);
    memset(runtime, 0, sizeof(*runtime));
}

SDL_Texture *frontend_video_runtime_texture(const FrontendVideoRuntime *runtime) {
    return runtime ? runtime->texture : NULL;
}

bool frontend_video_runtime_pixels(const FrontendVideoRuntime *runtime, bool displayed,
                                   const uint32_t **pixels, unsigned *width,
                                   unsigned *height, unsigned *stride) {
    if (!runtime || !pixels || !width || !height || !stride) return false;
    if (displayed) {
        if (!runtime->frame.pixels || !runtime->frame.width || !runtime->frame.height) return false;
        *pixels = runtime->frame.pixels;
        *width = *stride = runtime->frame.width;
        *height = runtime->frame.height;
        return true;
    }
    if (!runtime->source_pixels || !runtime->source_width || !runtime->source_height) return false;
    *pixels = runtime->source_pixels;
    *width = *stride = runtime->source_width;
    *height = runtime->source_height;
    return true;
}

void frontend_video_runtime_display_size(const FrontendVideoRuntime *runtime,
                                         unsigned *width, unsigned *height) {
    unsigned w = runtime && runtime->frame.width ? runtime->frame.width : 256;
    unsigned h = runtime && runtime->frame.height ? runtime->frame.height : 240;
    if (runtime && runtime->settings && runtime->settings->aspect_mode == FRONTEND_ASPECT_4_3
        && runtime->frame.screens) {
        w = (unsigned)(((uint64_t)h * 4u * runtime->frame.screens + 2u) / 3u);
    }
    if (width) *width = w;
    if (height) *height = h;
}

bool frontend_video_runtime_aim(const FrontendVideoRuntime *runtime, int x, int y,
                                unsigned *side, int *nes_x, int *nes_y) {
    return runtime && nes_video_presentation_aim(&runtime->frame, x, y, side, nes_x, nes_y);
}

bool frontend_video_runtime_load_shader(FrontendVideoRuntime *runtime, const char *path,
                                        char *error, size_t error_size) {
    if (!runtime || !runtime->shader) return fail(error, error_size, "Video runtime is not initialized");
    if (!path || strlen(path) >= sizeof(runtime->shader_requested_path))
        return fail(error, error_size, "Shader preset path is missing or too long");
    memmove(runtime->shader_requested_path, path, strlen(path) + 1);
    if (!nes_shader_load(runtime->shader, path, runtime->shader_error, sizeof(runtime->shader_error))) {
        nes_shader_enable(runtime->shader, false);
        return fail(error, error_size, runtime->shader_error);
    }
    if (error && error_size) error[0] = 0;
    return true;
}

void frontend_video_runtime_unload_image(FrontendVideoRuntime *runtime) {
    if (!runtime) return;
    nes_hd_frontend_set_capture_source(runtime->hd_frontend, NULL);
    nes_hd_runtime_clear_game(runtime->hd);
    if (runtime->builder_preview) (void)nes_video_trace_use(NES_VIDEO_TRACE_BUILDER, false);
    runtime->builder_preview = false;
    memset(&runtime->builder_frame, 0, sizeof(runtime->builder_frame));
    nes_hd_builder_clear_game(runtime->builder);
    runtime->source_pixels = NULL;
    runtime->source_width = runtime->source_height = 0;
    memset(&runtime->frame, 0, sizeof(runtime->frame));
    if (runtime->texture) SDL_DestroyTexture(runtime->texture);
    runtime->texture = NULL;
    runtime->texture_width = runtime->texture_height = 0;
}
