/*
 * tas_project_fm3.c - Safe FM3 project metadata export
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_project_fm3_internal.h"

#include "../third_party/miniz/miniz.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t fm3_markers_id[FM3_MARKERS_ID_SIZE] = "MARKERS";
static const uint8_t fm3_bookmarks_id[FM3_BOOKMARKS_ID_SIZE] = "BOOKMARKS";
static const uint8_t fm3_greenzone_skip_id[FM3_GREENZONE_ID_SIZE] = "GREENZONX";
static const uint8_t fm3_history_skip_id[FM3_HISTORY_ID_SIZE] = "HISTORX";
static const uint8_t fm3_piano_roll_skip_id[FM3_PIANO_ROLL_ID_SIZE] = "PIANO_ROLX";
static const uint8_t fm3_selection_id[FM3_SELECTION_ID_SIZE] = "SELECTION";

typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
    NesTasResult result;
} Fm3Buffer;

typedef struct {
    NesFm3ProjectModule module;
    bool owned;
} Fm3BuiltModule;

static void fm3_diagnostic(NesFm2Diagnostic *diagnostic, NesFm2Result result, const char *message) {
    if (!diagnostic) {
        return;
    }
    diagnostic->result = result;
    diagnostic->byte_offset = 0;
    diagnostic->frame_index = 0;
    if (!message) {
        message = "";
    }
    size_t length = strlen(message);
    if (length >= sizeof(diagnostic->message)) {
        length = sizeof(diagnostic->message) - 1;
    }
    memcpy(diagnostic->message, message, length);
    diagnostic->message[length] = '\0';
}

static NesTasResult fm3_from_codec(NesFm2Result result) {
    switch (result) {
    case NES_FM2_OK:
        return NES_TAS_OK;
    case NES_FM2_INVALID_ARGUMENT:
        return NES_TAS_INVALID_ARGUMENT;
    case NES_FM2_OUT_OF_MEMORY:
        return NES_TAS_OUT_OF_MEMORY;
    case NES_FM2_LIMIT:
        return NES_TAS_RANGE_ERROR;
    case NES_FM2_UNSUPPORTED:
        return NES_TAS_UNSUPPORTED;
    case NES_FM2_TRUNCATED:
    case NES_FM2_CORRUPT:
    default:
        return NES_TAS_FORMAT_ERROR;
    }
}

static bool fm3_buffer_reserve(Fm3Buffer *buffer, size_t bytes) {
    if (buffer->result != NES_TAS_OK) {
        return false;
    }
    if (bytes > SIZE_MAX - buffer->size) {
        buffer->result = NES_TAS_RANGE_ERROR;
        return false;
    }
    size_t required = buffer->size + bytes;
    if (required <= buffer->capacity) {
        return true;
    }
    size_t capacity = buffer->capacity ? buffer->capacity : 256;
    while (capacity < required) {
        if (capacity > SIZE_MAX / 2) {
            capacity = required;
            break;
        }
        capacity *= 2;
    }
    uint8_t *data = (uint8_t *)realloc(buffer->data, capacity);
    if (!data) {
        buffer->result = NES_TAS_OUT_OF_MEMORY;
        return false;
    }
    buffer->data = data;
    buffer->capacity = capacity;
    return true;
}

static void fm3_buffer_bytes(Fm3Buffer *buffer, const void *data, size_t size) {
    if (!fm3_buffer_reserve(buffer, size)) {
        return;
    }
    if (size) {
        memcpy(buffer->data + buffer->size, data, size);
    }
    buffer->size += size;
}

static void fm3_buffer_u32(Fm3Buffer *buffer, uint32_t value) {
    uint8_t bytes[4] = {(uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    fm3_buffer_bytes(buffer, bytes, sizeof(bytes));
}

static void fm3_buffer_u8(Fm3Buffer *buffer, uint8_t value) {
    fm3_buffer_bytes(buffer, &value, 1);
}

static NesTasResult fm3_buffer_compressed(Fm3Buffer *buffer, const uint8_t *data, size_t size) {
    if (!buffer || (!data && size)) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (size > (size_t)((mz_ulong)-1)) {
        return NES_TAS_RANGE_ERROR;
    }
    uint8_t empty = 0;
    mz_ulong bound = mz_compressBound((mz_ulong)size);
    if ((size_t)bound > UINT32_MAX) {
        return NES_TAS_RANGE_ERROR;
    }
    uint8_t *compressed = (uint8_t *)malloc(bound ? (size_t)bound : 1);
    if (!compressed) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    mz_ulong compressed_size = bound;
    int zresult = mz_compress(compressed, &compressed_size, size ? data : &empty, (mz_ulong)size);
    if (zresult != MZ_OK || !compressed_size || compressed_size > UINT32_MAX) {
        free(compressed);
        return zresult == MZ_MEM_ERROR ? NES_TAS_OUT_OF_MEMORY : NES_TAS_FORMAT_ERROR;
    }
    fm3_buffer_u32(buffer, (uint32_t)compressed_size);
    fm3_buffer_bytes(buffer, compressed, (size_t)compressed_size);
    free(compressed);
    return buffer->result;
}

static NesTasResult fm3_buffer_lag_log(Fm3Buffer *buffer, const TasTimeline *timeline) {
    if (!buffer || !timeline) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = timeline->movie.frame_count;
    size_t count = 0;
    for (size_t frame = frames; frame > 0; --frame) {
        if (timeline->lag && timeline->lag[frame - 1] != NES_TAS_LAG_UNKNOWN) {
            count = frame;
            break;
        }
    }
    if (count > UINT32_MAX) {
        return NES_TAS_RANGE_ERROR;
    }
    fm3_buffer_u32(buffer, (uint32_t)count);
    if (!count || buffer->result != NES_TAS_OK) {
        return buffer->result;
    }

    uint8_t *raw = (uint8_t *)malloc(count);
    if (!raw) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    for (size_t frame = 0; frame < count; ++frame) {
        uint8_t value = timeline->lag ? timeline->lag[frame] : NES_TAS_LAG_UNKNOWN;
        switch (value) {
        case NES_TAS_LAG_UNKNOWN:
            raw[frame] = 2;
            break;
        case NES_TAS_LAG_NO:
            raw[frame] = 0;
            break;
        case NES_TAS_LAG_YES:
            raw[frame] = 1;
            break;
        default:
            free(raw);
            return NES_TAS_FORMAT_ERROR;
        }
    }
    if (count > (size_t)((mz_ulong)-1)) {
        free(raw);
        return NES_TAS_RANGE_ERROR;
    }
    mz_ulong bound = mz_compressBound((mz_ulong)count);
    if ((size_t)bound > UINT32_MAX) {
        free(raw);
        return NES_TAS_RANGE_ERROR;
    }
    uint8_t *compressed = (uint8_t *)malloc(bound ? (size_t)bound : 1);
    if (!compressed) {
        free(raw);
        return NES_TAS_OUT_OF_MEMORY;
    }
    mz_ulong compressed_size = bound;
    int zresult = mz_compress(compressed, &compressed_size, raw, (mz_ulong)count);
    free(raw);
    if (zresult != MZ_OK || !compressed_size || compressed_size > UINT32_MAX) {
        free(compressed);
        return zresult == MZ_MEM_ERROR ? NES_TAS_OUT_OF_MEMORY : NES_TAS_FORMAT_ERROR;
    }
    fm3_buffer_u32(buffer, (uint32_t)compressed_size);
    fm3_buffer_bytes(buffer, compressed, (size_t)compressed_size);
    free(compressed);
    return buffer->result;
}

static NesTasResult fm3_build_greenzone(const NesTasProject *project, Fm3BuiltModule *built) {
    Fm3Buffer buffer = {0};
    buffer.result = NES_TAS_OK;
    fm3_buffer_bytes(&buffer, fm3_greenzone_skip_id, sizeof(fm3_greenzone_skip_id));
    NesTasResult result = fm3_buffer_lag_log(&buffer, &project->state.timeline);
    if (result == NES_TAS_OK) {
        /* FCEUX's compact/no-greenzone record stores the playback cursor after
         * LagLog. Cursor zero deliberately avoids serializing or applying any
         * foreign emulator savestate. */
        fm3_buffer_u32(&buffer, 0);
        result = buffer.result;
    }
    if (result != NES_TAS_OK) {
        free(buffer.data);
        return result;
    }
    built->module.data = buffer.data;
    built->module.size = buffer.size;
    built->owned = true;
    return NES_TAS_OK;
}

static NesTasResult fm3_buffer_markers_body(Fm3Buffer *buffer, const TasTimeline *timeline);

static NesTasResult fm3_buffer_input_log(Fm3Buffer *buffer, const TasTimeline *timeline) {
    if (!buffer || !timeline) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = timeline->movie.frame_count;
    uint8_t input_type = 0;
    unsigned joypads = 0;
    if (frames > UINT32_MAX || !fm3_movie_input_type(&timeline->movie, &input_type, &joypads)) {
        return NES_TAS_UNSUPPORTED;
    }
    if (frames && joypads > SIZE_MAX / frames) {
        return NES_TAS_RANGE_ERROR;
    }
    size_t joystick_size = frames * joypads;
    uint8_t *joysticks = joystick_size ? (uint8_t *)malloc(joystick_size) : NULL;
    uint8_t *commands = frames ? (uint8_t *)malloc(frames) : NULL;
    if ((joystick_size && !joysticks) || (frames && !commands)) {
        free(joysticks);
        free(commands);
        return NES_TAS_OUT_OF_MEMORY;
    }
    for (size_t frame = 0; frame < frames; ++frame) {
        commands[frame] = timeline->movie.frames[frame].commands;
        for (unsigned joy = 0; joy < joypads; ++joy) {
            joysticks[frame * joypads + joy] = timeline->movie.frames[frame].pads[joy];
        }
    }
    fm3_buffer_u32(buffer, (uint32_t)frames);
    fm3_buffer_u8(buffer, input_type);
    NesTasResult result = fm3_buffer_compressed(buffer, joysticks, joystick_size);
    if (result == NES_TAS_OK) {
        result = fm3_buffer_compressed(buffer, commands, frames);
    }
    free(joysticks);
    free(commands);
    if (result != NES_TAS_OK) {
        return result;
    }
    fm3_buffer_u8(buffer, 0); /* no Hot Changes metadata */
    return buffer->result;
}

static NesTasResult fm3_buffer_snapshot(Fm3Buffer *buffer, const TasBookmark *bookmark) {
    if (!buffer || !bookmark || !bookmark->occupied) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = bookmark->timeline.movie.frame_count;
    if (bookmark->key_frame > frames || bookmark->key_frame > INT32_MAX) {
        return NES_TAS_FORMAT_ERROR;
    }
    fm3_buffer_u32(buffer, (uint32_t)bookmark->key_frame);
    fm3_buffer_u32(buffer, 0);
    fm3_buffer_u32(buffer, frames ? (uint32_t)(frames - 1) : 0);
    fm3_buffer_u32(buffer, 0);
    fm3_buffer_u32(buffer, 0);
    fm3_buffer_u32(buffer, 0);
    const char *name = bookmark->name ? bookmark->name : "";
    size_t name_size = strlen(name);
    if (name_size >= 100) {
        return NES_TAS_RANGE_ERROR;
    }
    fm3_buffer_u8(buffer, (uint8_t)name_size);
    fm3_buffer_bytes(buffer, name, name_size);
    NesTasResult result = fm3_buffer_input_log(buffer, &bookmark->timeline);
    if (result == NES_TAS_OK) {
        result = fm3_buffer_lag_log(buffer, &bookmark->timeline);
    }
    if (result == NES_TAS_OK) {
        result = fm3_buffer_markers_body(buffer, &bookmark->timeline);
    }
    return result == NES_TAS_OK ? buffer->result : result;
}

static NesTasResult fm3_build_bookmarks(const NesTasProject *project, Fm3BuiltModule *built) {
    static const uint8_t timestamp[9] = "00:00:00";
    Fm3Buffer buffer = {0};
    buffer.result = NES_TAS_OK;
    fm3_buffer_bytes(&buffer, fm3_bookmarks_id, sizeof(fm3_bookmarks_id));
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        const TasBookmark *bookmark = &project->state.bookmarks[slot];
        fm3_buffer_u8(&buffer, bookmark->occupied ? 1 : 0);
        if (!bookmark->occupied) {
            continue;
        }
        NesTasResult result = fm3_buffer_snapshot(&buffer, bookmark);
        if (result != NES_TAS_OK) {
            free(buffer.data);
            return result;
        }
        /* Foreign emulator savestates and screenshots are deliberately not
         * propagated. FCEUX stores both as length-prefixed optional blobs. */
        fm3_buffer_u32(&buffer, 0);
        fm3_buffer_u32(&buffer, 0);
    }

    int current = project->state.current_branch;
    if (!fm3_bookmark_relations_valid(project->state.bookmarks, current)) {
        free(buffer.data);
        return NES_TAS_FORMAT_ERROR;
    }
    fm3_buffer_bytes(&buffer, timestamp, sizeof(timestamp));
    fm3_buffer_u32(&buffer, (uint32_t)(int32_t)current);
    fm3_buffer_u8(&buffer, 0);
    fm3_buffer_bytes(&buffer, timestamp, sizeof(timestamp));
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        int parent = project->state.bookmarks[slot].parent_slot;
        if (parent < -1 || parent >= NES_TAS_BOOKMARK_COUNT || parent == (int)slot ||
            (project->state.bookmarks[slot].occupied && parent >= 0 && !project->state.bookmarks[parent].occupied)) {
            free(buffer.data);
            return NES_TAS_FORMAT_ERROR;
        }
        fm3_buffer_u32(&buffer, (uint32_t)(int32_t)parent);
    }
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        fm3_buffer_u8(&buffer, 0xFF); /* no cached branch timeline */
    }
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT * NES_TAS_BOOKMARK_COUNT; ++i) {
        fm3_buffer_u32(&buffer, UINT32_C(0xFFFFFFFE)); /* FIRST_DIFFERENCE_UNKNOWN */
    }

    if (buffer.result != NES_TAS_OK) {
        NesTasResult result = buffer.result;
        free(buffer.data);
        return result;
    }
    built->module.data = buffer.data;
    built->module.size = buffer.size;
    built->owned = true;
    return NES_TAS_OK;
}

static void fm3_buffer_note(Fm3Buffer *buffer, const char *note) {
    if (buffer->result != NES_TAS_OK) {
        return;
    }
    if (!note) {
        note = "";
    }
    size_t length = strlen(note);
    if (length > NES_TAS_MARKER_NOTE_MAX || length == UINT32_MAX) {
        buffer->result = NES_TAS_RANGE_ERROR;
        return;
    }
    fm3_buffer_u32(buffer, (uint32_t)(length + 1));
    fm3_buffer_bytes(buffer, note, length + 1);
}

static NesTasResult fm3_buffer_markers_body(Fm3Buffer *buffer, const TasTimeline *timeline) {
    if (!buffer || !timeline) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = timeline->movie.frame_count;
    if (frames > INT32_MAX || frames > SIZE_MAX / 4 || timeline->marker_count > INT32_MAX - 1) {
        return NES_TAS_RANGE_ERROR;
    }
    size_t raw_size = frames * 4;
    uint8_t *raw = raw_size ? (uint8_t *)calloc(raw_size, 1) : NULL;
    if (raw_size && !raw) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        const TasMarker *marker = &timeline->markers[i];
        if (marker->frame >= frames || i + 1 > INT32_MAX) {
            free(raw);
            return NES_TAS_FORMAT_ERROR;
        }
        uint32_t marker_id = (uint32_t)(i + 1);
        uint8_t *slot = raw + marker->frame * 4;
        slot[0] = (uint8_t)marker_id;
        slot[1] = (uint8_t)(marker_id >> 8);
        slot[2] = (uint8_t)(marker_id >> 16);
        slot[3] = (uint8_t)(marker_id >> 24);
    }
    fm3_buffer_u32(buffer, (uint32_t)frames);
    NesTasResult result = fm3_buffer_compressed(buffer, raw, raw_size);
    free(raw);
    if (result != NES_TAS_OK) {
        return result;
    }
    fm3_buffer_u32(buffer, (uint32_t)timeline->marker_count + 1);
    fm3_buffer_note(buffer, timeline->intro_note);
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        fm3_buffer_note(buffer, timeline->markers[i].note);
    }
    return buffer->result;
}

static NesTasResult fm3_build_markers(const NesTasProject *project, Fm3BuiltModule *built) {
    Fm3Buffer buffer = {0};
    buffer.result = NES_TAS_OK;
    fm3_buffer_bytes(&buffer, fm3_markers_id, sizeof(fm3_markers_id));
    NesTasResult result = fm3_buffer_markers_body(&buffer, &project->state.timeline);
    if (result != NES_TAS_OK) {
        free(buffer.data);
        return result;
    }
    built->module.data = buffer.data;
    built->module.size = buffer.size;
    built->owned = true;
    return NES_TAS_OK;
}

static NesTasResult fm3_build_selection(const NesTasProject *project, Fm3BuiltModule *built) {
    size_t frames = project->state.timeline.movie.frame_count;
    if (frames > INT32_MAX) {
        return NES_TAS_RANGE_ERROR;
    }
    size_t selected = 0;
    for (size_t i = 0; i < frames; ++i) {
        if (project->state.selection && project->state.selection[i]) {
            selected++;
        }
    }
    if (selected > INT32_MAX || selected > (SIZE_MAX - 26) / 4) {
        return NES_TAS_RANGE_ERROR;
    }
    size_t size = 26 + selected * 4;
    uint8_t *data = (uint8_t *)malloc(size);
    if (!data) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    Fm3Buffer buffer = {data, 0, size, NES_TAS_OK};
    fm3_buffer_bytes(&buffer, fm3_selection_id, sizeof(fm3_selection_id));
    fm3_buffer_u32(&buffer, 0);
    fm3_buffer_u32(&buffer, 1);
    fm3_buffer_u32(&buffer, (uint32_t)selected);
    for (size_t frame = 0; frame < frames; ++frame) {
        if (project->state.selection && project->state.selection[frame]) {
            fm3_buffer_u32(&buffer, (uint32_t)frame);
        }
    }
    fm3_buffer_u32(&buffer, 0);
    if (buffer.result != NES_TAS_OK || buffer.size != size) {
        NesTasResult result = buffer.result == NES_TAS_OK ? NES_TAS_FORMAT_ERROR : buffer.result;
        free(data);
        return result;
    }
    built->module.data = data;
    built->module.size = size;
    built->owned = true;
    return NES_TAS_OK;
}

static void fm3_use_static_module(Fm3BuiltModule *built, const uint8_t *data, size_t size) {
    built->module.data = (uint8_t *)data;
    built->module.size = size;
    built->owned = false;
}

static void fm3_built_modules_free(Fm3BuiltModule modules[NES_FM3_PROJECT_MODULE_COUNT]) {
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        if (modules[i].owned) {
            free(modules[i].module.data);
        }
    }
}

static NesTasResult fm3_prepare_modules(const NesTasProject *project,
                                        Fm3BuiltModule modules[NES_FM3_PROJECT_MODULE_COUNT], uint32_t *saved_modules) {
    memset(modules, 0, sizeof(*modules) * NES_FM3_PROJECT_MODULE_COUNT);
    *saved_modules = 0;
    uint32_t dirty = project->fm3_dirty_modules;
    if (dirty & ~TAS_FM3_ALL_MODULES) {
        return NES_TAS_FORMAT_ERROR;
    }

    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        bool source_usable = project->source_project_present && (dirty & TAS_FM3_MODULE_BIT(i)) == 0;
        if (i == NES_FM3_MODULE_HISTORY && dirty != 0) {
            source_usable = false;
        }
        if (source_usable) {
            modules[i].module = project->source_modules[i];
            if (project->source_saved_modules & TAS_FM3_MODULE_BIT(i)) {
                *saved_modules |= TAS_FM3_MODULE_BIT(i);
            }
            continue;
        }

        NesTasResult result = NES_TAS_OK;
        switch (i) {
        case NES_FM3_MODULE_MARKERS:
            result = fm3_build_markers(project, &modules[i]);
            if (result == NES_TAS_OK) {
                *saved_modules |= TAS_FM3_MODULE_BIT(i);
            }
            break;
        case NES_FM3_MODULE_BOOKMARKS:
            result = fm3_build_bookmarks(project, &modules[i]);
            if (result == NES_TAS_OK) {
                *saved_modules |= TAS_FM3_MODULE_BIT(i);
            }
            break;
        case NES_FM3_MODULE_GREENZONE:
            result = fm3_build_greenzone(project, &modules[i]);
            break;
        case NES_FM3_MODULE_HISTORY:
            fm3_use_static_module(&modules[i], fm3_history_skip_id, sizeof(fm3_history_skip_id));
            break;
        case NES_FM3_MODULE_PIANO_ROLL:
            fm3_use_static_module(&modules[i], fm3_piano_roll_skip_id, sizeof(fm3_piano_roll_skip_id));
            break;
        case NES_FM3_MODULE_SELECTION:
            result = fm3_build_selection(project, &modules[i]);
            if (result == NES_TAS_OK) {
                *saved_modules |= TAS_FM3_MODULE_BIT(i);
            }
            break;
        default:
            result = NES_TAS_FORMAT_ERROR;
            break;
        }
        if (result != NES_TAS_OK) {
            fm3_built_modules_free(modules);
            return result;
        }
    }
    return NES_TAS_OK;
}

NesTasResult tas_project_write_fm3(const NesTasProject *project, uint8_t *destination, size_t capacity, size_t *written,
                                   NesFm2Diagnostic *diagnostic, bool measure_only, size_t *measured) {
    if (!project || project->edit_active || (measure_only ? !measured : !written)) {
        fm3_diagnostic(diagnostic, NES_FM2_INVALID_ARGUMENT, "invalid TAS project FM3 export arguments");
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (!measure_only && capacity && !destination) {
        fm3_diagnostic(diagnostic, NES_FM2_INVALID_ARGUMENT, "FM3 export destination is null");
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (project->source_project_present && project->source_project_version != 3) {
        fm3_diagnostic(diagnostic, NES_FM2_UNSUPPORTED, "unsupported source FM3 project version");
        return NES_TAS_UNSUPPORTED;
    }

    Fm3BuiltModule modules[NES_FM3_PROJECT_MODULE_COUNT];
    uint32_t saved_modules = 0;
    NesTasResult result = fm3_prepare_modules(project, modules, &saved_modules);
    if (result != NES_TAS_OK) {
        NesFm2Result fm2_result = result == NES_TAS_OUT_OF_MEMORY ? NES_FM2_OUT_OF_MEMORY
                                  : result == NES_TAS_RANGE_ERROR ? NES_FM2_LIMIT
                                                                  : NES_FM2_CORRUPT;
        fm3_diagnostic(diagnostic, fm2_result, "unable to build safe FM3 project modules");
        return result;
    }

    NesFm2Movie movie = project->state.timeline.movie;
    movie.project_present = true;
    movie.project_timeline_valid = true;
    movie.project_version = 3;
    movie.project_saved_modules = saved_modules;
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        movie.project_modules[i] = modules[i].module;
    }

    NesFm2Result encoded;
    if (measure_only) {
        encoded = nes_fm2_measure(&movie, NES_FM3_PROJECT, measured, diagnostic);
    } else {
        encoded = nes_fm2_export(&movie, NES_FM3_PROJECT, destination, capacity, written, diagnostic);
    }
    fm3_built_modules_free(modules);
    return fm3_from_codec(encoded);
}

NesTasResult nes_tas_project_measure_fm3(const NesTasProject *project, size_t *size, NesFm2Diagnostic *diagnostic) {
    return tas_project_write_fm3(project, NULL, 0, NULL, diagnostic, true, size);
}

NesTasResult nes_tas_project_export_fm3(const NesTasProject *project, uint8_t *destination, size_t capacity,
                                        size_t *written, NesFm2Diagnostic *diagnostic) {
    return tas_project_write_fm3(project, destination, capacity, written, diagnostic, false, NULL);
}
