/*
 * fm2.h - FCEUX FM2/FM3 movie container codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REPLAY_FM2_H
#define CUPID_REPLAY_FM2_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_FM2_MD5_SIZE = 16,
    NES_FM2_GUID_SIZE = 16,
    NES_FM3_PROJECT_MODULE_COUNT = 6,
    NES_FM2_DIAGNOSTIC_MESSAGE_SIZE = 160
};

typedef enum {
    NES_FM2_OK = 0,
    NES_FM2_INVALID_ARGUMENT,
    NES_FM2_OUT_OF_MEMORY,
    NES_FM2_LIMIT,
    NES_FM2_TRUNCATED,
    NES_FM2_CORRUPT,
    NES_FM2_UNSUPPORTED
} NesFm2Result;

typedef enum { NES_FM2_TEXT = 0, NES_FM2_BINARY, NES_FM3_PROJECT } NesFm2Container;

typedef enum { NES_FM2_PORT_NONE = 0, NES_FM2_PORT_GAMEPAD = 1, NES_FM2_PORT_ZAPPER = 2 } NesFm2Port;

typedef enum { NES_FM2_START_POWER_ON = 0, NES_FM2_START_SAVERAM, NES_FM2_START_SAVESTATE } NesFm2StartKind;

typedef enum {
    NES_FM2_COMMAND_RESET = 1u << 0,
    NES_FM2_COMMAND_POWER = 1u << 1,
    NES_FM2_COMMAND_FDS_INSERT = 1u << 2,
    NES_FM2_COMMAND_FDS_SELECT = 1u << 3,
    NES_FM2_COMMAND_VS_COIN_1 = 1u << 4,
    NES_FM2_COMMAND_VS_COIN_2 = 1u << 5,
    NES_FM2_COMMAND_VS_SERVICE = 1u << 6
} NesFm2Command;

typedef enum {
    NES_FM3_MODULE_MARKERS = 0,
    NES_FM3_MODULE_BOOKMARKS,
    NES_FM3_MODULE_GREENZONE,
    NES_FM3_MODULE_HISTORY,
    NES_FM3_MODULE_PIANO_ROLL,
    NES_FM3_MODULE_SELECTION
} NesFm3ProjectModuleKind;

typedef struct {
    size_t max_file_bytes;
    size_t max_frames;
    size_t max_text_bytes;
    size_t max_blob_bytes;
    size_t max_comments;
    size_t max_subtitles;
    size_t max_extensions;
    size_t max_project_bytes;
} NesFm2Limits;

typedef struct {
    NesFm2Result result;
    size_t byte_offset;
    size_t frame_index;
    char message[NES_FM2_DIAGNOSTIC_MESSAGE_SIZE];
} NesFm2Diagnostic;

typedef struct {
    uint8_t *data;
    size_t size;
} NesFm2Blob;

typedef struct {
    char **items;
    size_t count;
} NesFm2StringList;

typedef struct {
    char *key;
    char *value;
} NesFm2HeaderField;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t button;
    uint8_t bogo;
    uint64_t zaphit;
} NesFm2Zapper;

typedef struct {
    uint8_t commands;
    uint8_t pads[4];
    NesFm2Zapper zappers[2];
} NesFm2Frame;

typedef struct {
    uint8_t *data;
    size_t size;
} NesFm3ProjectModule;

typedef struct {
    NesFm2Container original_container;
    uint32_t version;
    uint32_t emu_version;
    uint32_t rerecord_count;
    bool pal;
    char *rom_filename;
    uint8_t rom_md5[NES_FM2_MD5_SIZE];
    uint8_t guid[NES_FM2_GUID_SIZE];
    bool fourscore;
    bool microphone;
    NesFm2Port ports[3];
    bool fds;
    bool new_ppu;
    uint32_t ram_init_option;
    uint32_t ram_init_seed;
    NesFm2StringList comments;
    NesFm2StringList subtitles;
    NesFm2Blob savestate;
    NesFm2Blob saveram;
    NesFm2HeaderField *extensions;
    size_t extension_count;
    NesFm2Frame *frames;
    size_t frame_count;
    bool explicit_length;

    bool project_present;
    bool project_timeline_valid;
    uint32_t project_version;
    uint32_t project_saved_modules;
    NesFm3ProjectModule project_modules[NES_FM3_PROJECT_MODULE_COUNT];
} NesFm2Movie;

NesFm2Limits nes_fm2_default_limits(void);
void nes_fm2_movie_init(NesFm2Movie *movie);
void nes_fm2_movie_free(NesFm2Movie *movie);

/* Clone and parse are transactional for an initialized destination: success
 * replaces it, while failure leaves its previous contents untouched. */
NesFm2Result nes_fm2_movie_clone(const NesFm2Movie *source, NesFm2Movie *out, NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2_parse(const uint8_t *data, size_t size, const NesFm2Limits *limits, NesFm2Movie *out,
                           NesFm2Diagnostic *diagnostic);

NesFm2Result nes_fm2_measure(const NesFm2Movie *movie, NesFm2Container container, size_t *size,
                             NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2_export(const NesFm2Movie *movie, NesFm2Container container, uint8_t *destination, size_t capacity,
                            size_t *written, NesFm2Diagnostic *diagnostic);

NesFm2StartKind nes_fm2_start_kind(const NesFm2Movie *movie);
/* FM3 module slices can be preserved byte-for-byte only while their timeline
 * still matches the input log. Call this after editing frame order or content;
 * FM3 export will then reject the stale project data until it is rebuilt. */
void nes_fm2_invalidate_project_timeline(NesFm2Movie *movie);
void nes_fm2_discard_project(NesFm2Movie *movie);

const char *nes_fm2_result_string(NesFm2Result result);

#ifdef __cplusplus
}
#endif
#endif
