/*
 * hd_runtime.h - Host-side HD pack lifecycle and presentation API
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_HD_RUNTIME_H
#define CUPID_NES_HD_RUNTIME_H

#include "../video/video_trace.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_HD_PACK_NAME_MAX = 160,
    NES_HD_STATUS_MAX = 256,
    NES_HD_MAX_CANDIDATES = 256
};

typedef struct NesHdRuntime NesHdRuntime;

typedef struct {
    unsigned left, right, top, bottom;
} NesHdOverscan;

typedef struct {
    const char *rom_path;
    /* Optional canonical 40-hex SHA-1 of the loaded game image. When omitted,
     * name-based discovery remains available and supportedRom checks are unknown. */
    const char *rom_sha1;
    const uint8_t *chr_rom;
    size_t chr_size;
    bool chr_is_rom;
} NesHdGameInfo;

typedef struct {
    const char *path;          /* Owned by the runtime until the next discovery. */
    const char *name;          /* Display name derived from the candidate path. */
    bool archive;
    bool installed;
    bool compatible;
    bool active;
} NesHdPackCandidateInfo;

typedef struct {
    bool game_attached;
    bool pack_loaded;
    bool enabled;
    bool trace_ready;
    bool audio_active;
    bool alternate_audio_registers;
    size_t candidate_count;
    const char *active_path;   /* Owned by the runtime. */
    const char *status;        /* Owned by the runtime. */
} NesHdRuntimeInfo;

typedef uint8_t (*NesHdMemoryReader)(void *context, unsigned side,
                                    bool ppu_space, uint16_t address);

typedef struct {
    /* Presentation output to retain when video_trace is not complete yet. This
     * should normally be the result from nes_video_presentation_render(). */
    const uint32_t *fallback_pixels;
    unsigned fallback_width;
    unsigned fallback_height;
    unsigned screens;
    const NesVideoTraceFrame *trace[2];
    NesHdOverscan overscan;
    bool show_background;
    bool show_sprites;
    bool allow_pack_overscan;
} NesHdFrameSource;

typedef struct {
    const uint32_t *pixels;    /* Owned by the runtime until the next render. */
    unsigned width;
    unsigned height;
    unsigned screens;
    unsigned scale;
    NesHdOverscan overscan;
    bool hd_rendered;
    bool trace_ready;
} NesHdFrame;

NesHdRuntime *nes_hd_runtime_create(const char *pack_root,
                                    char *error, size_t error_size);
void nes_hd_runtime_destroy(NesHdRuntime *runtime);
/* Restoring machine state discards host audio voices from the old timeline. */
void nes_hd_runtime_reset_audio(NesHdRuntime *runtime);

bool nes_hd_runtime_set_game(NesHdRuntime *runtime, const NesHdGameInfo *game,
                             char *error, size_t error_size);
void nes_hd_runtime_clear_game(NesHdRuntime *runtime);
void nes_hd_runtime_set_memory_reader(NesHdRuntime *runtime,
                                      NesHdMemoryReader reader, void *context);
/* Additional output paths that export/capture must never replace, such as
 * firmware or battery-save files. The runtime copies all supplied strings. */
bool nes_hd_runtime_set_protected_paths(NesHdRuntime *runtime,
                                        const char *const *paths, size_t count,
                                        char *error, size_t error_size);

bool nes_hd_runtime_discover(NesHdRuntime *runtime,
                             char *error, size_t error_size);
size_t nes_hd_runtime_candidate_count(const NesHdRuntime *runtime);
bool nes_hd_runtime_candidate_at(const NesHdRuntime *runtime, size_t index,
                                 NesHdPackCandidateInfo *info);

/* Loading/switching is transactional: a failed candidate keeps the current
 * pack and enabled state. Directories contain hires.txt; ZIPs stay in-memory. */
bool nes_hd_runtime_load(NesHdRuntime *runtime, const char *path,
                         char *error, size_t error_size);
bool nes_hd_runtime_switch(NesHdRuntime *runtime, size_t candidate_index,
                           char *error, size_t error_size);
bool nes_hd_runtime_enable(NesHdRuntime *runtime, bool enabled,
                           char *error, size_t error_size);

/* Installs a fully validated self-contained ZIP below pack_root/<game>/.
 * No extracted asset is written. The installed candidate becomes active. */
bool nes_hd_runtime_install(NesHdRuntime *runtime, const char *source_path,
                            const char *install_name,
                            char *installed_path, size_t installed_path_size,
                            char *error, size_t error_size);
bool nes_hd_runtime_export(const NesHdRuntime *runtime, const char *zip_path,
                           char *error, size_t error_size);
/* Build a new version-109 pack from one paired set of completed presentation
 * traces. This does not require a loaded pack and never changes the active
 * pack, trace-user state, or emulated hardware. */
bool nes_hd_runtime_capture(const NesHdRuntime *runtime,
                            const NesHdFrameSource *source,
                            const char *zip_path,
                            char *error, size_t error_size);

bool nes_hd_runtime_render(NesHdRuntime *runtime, const NesHdFrameSource *source,
                           NesHdFrame *frame, char *error, size_t error_size);
bool nes_hd_runtime_info(const NesHdRuntime *runtime, NesHdRuntimeInfo *info);

#ifdef __cplusplus
}
#endif

#endif
