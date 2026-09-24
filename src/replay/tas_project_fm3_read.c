/*
 * tas_project_fm3_read.c - Bounded FM3 metadata and branch import
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
static const uint8_t fm3_bookmarks_skip_id[FM3_BOOKMARKS_ID_SIZE] = "BOOKMARKX";
static const uint8_t fm3_greenzone_id[FM3_GREENZONE_ID_SIZE] = "GREENZONE";
static const uint8_t fm3_greenzone_skip_id[FM3_GREENZONE_ID_SIZE] = "GREENZONX";
static const uint8_t fm3_selection_id[FM3_SELECTION_ID_SIZE] = "SELECTION";

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t offset;
} Fm3Reader;

typedef struct {
    size_t remaining;
} Fm3DecodeBudget;

typedef struct {
    TasMarker *markers;
    size_t marker_count;
    char *intro_note;
} Fm3MarkersImport;

typedef struct {
    TasBookmark bookmarks[NES_TAS_BOOKMARK_COUNT];
    int current_branch;
} Fm3BookmarksImport;

static bool fm3_reader_bytes(Fm3Reader *reader, size_t size, const uint8_t **out) {
    if (!reader || reader->offset > reader->size || size > reader->size - reader->offset) {
        return false;
    }
    if (out) {
        *out = reader->data + reader->offset;
    }
    reader->offset += size;
    return true;
}

static bool fm3_reader_u32(Fm3Reader *reader, uint32_t *value) {
    const uint8_t *data = NULL;
    if (!value || !fm3_reader_bytes(reader, 4, &data)) {
        return false;
    }
    *value = (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
    return true;
}

static bool fm3_reader_u8(Fm3Reader *reader, uint8_t *value) {
    const uint8_t *data = NULL;
    if (!value || !fm3_reader_bytes(reader, 1, &data)) {
        return false;
    }
    *value = data[0];
    return true;
}

static bool fm3_reader_skip_u32_array(Fm3Reader *reader, uint32_t count) {
    size_t item_count = count;
    if (item_count > SIZE_MAX / 4) {
        return false;
    }
    return fm3_reader_bytes(reader, item_count * 4, NULL);
}

static bool fm3_size_add(size_t *total, size_t value) {
    if (!total || value > SIZE_MAX - *total) {
        return false;
    }
    *total += value;
    return true;
}

static bool fm3_size_mul(size_t left, size_t right, size_t *out) {
    if (!out || (left && right > SIZE_MAX / left)) {
        return false;
    }
    *out = left * right;
    return true;
}

static NesTasResult fm3_budget_take(Fm3DecodeBudget *budget, size_t bytes) {
    if (!budget) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (bytes > budget->remaining) {
        return NES_TAS_RANGE_ERROR;
    }
    budget->remaining -= bytes;
    return NES_TAS_OK;
}

static NesTasResult fm3_budget_movie_metadata(Fm3DecodeBudget *budget, const NesFm2Movie *movie) {
    if (!budget || !movie) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t bytes = 0;
    size_t array_bytes = 0;

    if (movie->rom_filename) {
        size_t length = strlen(movie->rom_filename);
        if (length == SIZE_MAX || !fm3_size_add(&bytes, length + 1)) {
            return NES_TAS_RANGE_ERROR;
        }
    }
    if (movie->comments.count) {
        if (!movie->comments.items ||
            !fm3_size_mul(movie->comments.count, sizeof(*movie->comments.items), &array_bytes) ||
            !fm3_size_add(&bytes, array_bytes)) {
            return NES_TAS_FORMAT_ERROR;
        }
        for (size_t i = 0; i < movie->comments.count; ++i) {
            if (!movie->comments.items[i]) {
                return NES_TAS_FORMAT_ERROR;
            }
            size_t length = strlen(movie->comments.items[i]);
            if (length == SIZE_MAX || !fm3_size_add(&bytes, length + 1)) {
                return NES_TAS_RANGE_ERROR;
            }
        }
    }
    if (movie->subtitles.count) {
        if (!movie->subtitles.items ||
            !fm3_size_mul(movie->subtitles.count, sizeof(*movie->subtitles.items), &array_bytes) ||
            !fm3_size_add(&bytes, array_bytes)) {
            return NES_TAS_FORMAT_ERROR;
        }
        for (size_t i = 0; i < movie->subtitles.count; ++i) {
            if (!movie->subtitles.items[i]) {
                return NES_TAS_FORMAT_ERROR;
            }
            size_t length = strlen(movie->subtitles.items[i]);
            if (length == SIZE_MAX || !fm3_size_add(&bytes, length + 1)) {
                return NES_TAS_RANGE_ERROR;
            }
        }
    }
    if ((movie->savestate.size && !movie->savestate.data) || (movie->saveram.size && !movie->saveram.data)) {
        return NES_TAS_FORMAT_ERROR;
    }
    if (!fm3_size_add(&bytes, movie->savestate.size) || !fm3_size_add(&bytes, movie->saveram.size)) {
        return NES_TAS_RANGE_ERROR;
    }

    if (movie->extension_count) {
        if (!movie->extensions || !fm3_size_mul(movie->extension_count, sizeof(*movie->extensions), &array_bytes) ||
            !fm3_size_add(&bytes, array_bytes)) {
            return NES_TAS_FORMAT_ERROR;
        }
        for (size_t i = 0; i < movie->extension_count; ++i) {
            if (!movie->extensions[i].key || !movie->extensions[i].value) {
                return NES_TAS_FORMAT_ERROR;
            }
            size_t key_length = strlen(movie->extensions[i].key);
            size_t value_length = strlen(movie->extensions[i].value);
            if (key_length == SIZE_MAX || value_length == SIZE_MAX || !fm3_size_add(&bytes, key_length + 1) ||
                !fm3_size_add(&bytes, value_length + 1)) {
                return NES_TAS_RANGE_ERROR;
            }
        }
    }

    /* tas_timeline_from_movie() also creates its default intro note. Snapshot
     * frames and FM3 modules are stripped from the metadata view before cloning. */
    if (!fm3_size_add(&bytes, sizeof("Power on"))) {
        return NES_TAS_RANGE_ERROR;
    }
    return fm3_budget_take(budget, bytes);
}

static char *fm3_reader_note(Fm3Reader *reader, NesTasResult *result, Fm3DecodeBudget *budget) {
    uint32_t length = 0;
    const uint8_t *data = NULL;
    if (!fm3_reader_u32(reader, &length) || length == 0 || length > FM3_NOTE_BYTES_MAX ||
        !fm3_reader_bytes(reader, length, &data) || data[length - 1] != 0 ||
        (length > 1 && memchr(data, 0, length - 1) != NULL)) {
        *result = NES_TAS_FORMAT_ERROR;
        return NULL;
    }
    *result = fm3_budget_take(budget, length);
    if (*result != NES_TAS_OK) {
        return NULL;
    }
    char *copy = (char *)malloc(length);
    if (!copy) {
        *result = NES_TAS_OUT_OF_MEMORY;
        return NULL;
    }
    memcpy(copy, data, length);
    return copy;
}

static void fm3_markers_import_free(Fm3MarkersImport *imported) {
    if (!imported) {
        return;
    }
    for (size_t i = 0; i < imported->marker_count; ++i) {
        free(imported->markers[i].note);
    }
    free(imported->markers);
    free(imported->intro_note);
    memset(imported, 0, sizeof(*imported));
}

static uint32_t fm3_read_u32le(const uint8_t *data) {
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static NesTasResult fm3_parse_markers_body(Fm3Reader *reader, size_t frames, Fm3MarkersImport *out,
                                           Fm3DecodeBudget *budget) {
    if (!reader || !out || !budget) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    uint32_t frame_count = 0;
    uint32_t compressed_size = 0;
    const uint8_t *compressed = NULL;
    uint32_t note_count = 0;

    if (!fm3_reader_u32(reader, &frame_count) || frame_count != frames || frame_count > INT32_MAX ||
        !fm3_reader_u32(reader, &compressed_size) || compressed_size == 0 ||
        !fm3_reader_bytes(reader, compressed_size, &compressed)) {
        return NES_TAS_FORMAT_ERROR;
    }

    if (frames > SIZE_MAX / 4) {
        return NES_TAS_RANGE_ERROR;
    }
    size_t raw_size = frames * 4;
    NesTasResult result = fm3_budget_take(budget, raw_size);
    if (result != NES_TAS_OK) {
        return result;
    }
    uint8_t empty = 0;
    uint8_t *raw = raw_size ? (uint8_t *)malloc(raw_size) : NULL;
    if (raw_size && !raw) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    mz_ulong unpacked = (mz_ulong)raw_size;
    if ((size_t)unpacked != raw_size) {
        free(raw);
        return NES_TAS_RANGE_ERROR;
    }
    int zresult = mz_uncompress(raw_size ? raw : &empty, &unpacked, compressed, (mz_ulong)compressed_size);
    if (zresult != MZ_OK || unpacked != (mz_ulong)raw_size) {
        free(raw);
        return NES_TAS_FORMAT_ERROR;
    }

    size_t marker_count = 0;
    uint32_t expected_id = 1;
    for (size_t frame = 0; frame < frames; ++frame) {
        uint32_t marker_id = fm3_read_u32le(raw + frame * 4);
        if (!marker_id) {
            continue;
        }
        if (marker_id != expected_id || marker_id > INT32_MAX) {
            free(raw);
            return NES_TAS_FORMAT_ERROR;
        }
        marker_count++;
        expected_id++;
    }

    if (marker_count > SIZE_MAX / sizeof(*out->markers)) {
        free(raw);
        return NES_TAS_RANGE_ERROR;
    }
    if (marker_count) {
        size_t marker_bytes = 0;
        if (!fm3_size_mul(marker_count, sizeof(*out->markers), &marker_bytes)) {
            free(raw);
            return NES_TAS_RANGE_ERROR;
        }
        result = fm3_budget_take(budget, marker_bytes);
        if (result != NES_TAS_OK) {
            free(raw);
            return result;
        }
        out->markers = (TasMarker *)calloc(marker_count, sizeof(*out->markers));
        if (!out->markers) {
            free(raw);
            return NES_TAS_OUT_OF_MEMORY;
        }
    }
    out->marker_count = marker_count;
    size_t marker_index = 0;
    for (size_t frame = 0; frame < frames; ++frame) {
        if (fm3_read_u32le(raw + frame * 4) != 0) {
            out->markers[marker_index++].frame = frame;
        }
    }
    free(raw);

    if (!fm3_reader_u32(reader, &note_count) || note_count != marker_count + 1 || note_count > INT32_MAX) {
        fm3_markers_import_free(out);
        return NES_TAS_FORMAT_ERROR;
    }
    result = NES_TAS_OK;
    out->intro_note = fm3_reader_note(reader, &result, budget);
    if (!out->intro_note) {
        fm3_markers_import_free(out);
        return result;
    }
    for (size_t i = 0; i < marker_count; ++i) {
        out->markers[i].note = fm3_reader_note(reader, &result, budget);
        if (!out->markers[i].note) {
            fm3_markers_import_free(out);
            return result;
        }
    }
    return NES_TAS_OK;
}

static NesTasResult fm3_parse_markers(const NesFm3ProjectModule *module, size_t frames, Fm3MarkersImport *out,
                                      Fm3DecodeBudget *budget) {
    if (!module || !out || !budget || (!module->data && module->size)) {
        return NES_TAS_FORMAT_ERROR;
    }
    Fm3Reader reader = {module->data, module->size, 0};
    const uint8_t *id = NULL;
    if (!fm3_reader_bytes(&reader, FM3_MARKERS_ID_SIZE, &id) ||
        memcmp(id, fm3_markers_id, sizeof(fm3_markers_id)) != 0) {
        return NES_TAS_FORMAT_ERROR;
    }
    NesTasResult result = fm3_parse_markers_body(&reader, frames, out, budget);
    if (result != NES_TAS_OK) {
        return result;
    }
    if (reader.offset != reader.size) {
        fm3_markers_import_free(out);
        return NES_TAS_FORMAT_ERROR;
    }
    return NES_TAS_OK;
}

static NesTasResult fm3_parse_selection(const NesFm3ProjectModule *module, size_t frames, uint8_t **selection_out,
                                        Fm3DecodeBudget *budget) {
    if (!module || !selection_out || !budget || (!module->data && module->size)) {
        return NES_TAS_FORMAT_ERROR;
    }
    *selection_out = NULL;
    Fm3Reader reader = {module->data, module->size, 0};
    const uint8_t *id = NULL;
    uint32_t cursor = 0;
    uint32_t total = 0;
    if (!fm3_reader_bytes(&reader, FM3_SELECTION_ID_SIZE, &id) ||
        memcmp(id, fm3_selection_id, sizeof(fm3_selection_id)) != 0 || !fm3_reader_u32(&reader, &cursor) ||
        !fm3_reader_u32(&reader, &total) || total == 0 || total > INT32_MAX || cursor >= total) {
        return NES_TAS_FORMAT_ERROR;
    }

    size_t remaining = reader.size - reader.offset;
    if ((size_t)total > (remaining >= 4 ? (remaining - 4) / 4 : 0)) {
        return NES_TAS_FORMAT_ERROR;
    }

    NesTasResult budget_result = fm3_budget_take(budget, frames);
    if (budget_result != NES_TAS_OK) {
        return budget_result;
    }
    uint8_t *selection = frames ? (uint8_t *)calloc(frames, 1) : NULL;
    if (frames && !selection) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    for (uint32_t i = 0; i < total; ++i) {
        uint32_t count = 0;
        if (!fm3_reader_u32(&reader, &count)) {
            goto format_error;
        }
        if (i != cursor) {
            if (!fm3_reader_skip_u32_array(&reader, count)) {
                goto format_error;
            }
            continue;
        }
        uint32_t previous = 0;
        for (uint32_t j = 0; j < count; ++j) {
            uint32_t frame = 0;
            if (!fm3_reader_u32(&reader, &frame) || frame >= frames || frame > INT32_MAX || (j && frame <= previous)) {
                goto format_error;
            }
            selection[frame] = 1;
            previous = frame;
        }
    }

    uint32_t clipboard_count = 0;
    if (!fm3_reader_u32(&reader, &clipboard_count) || !fm3_reader_skip_u32_array(&reader, clipboard_count) ||
        reader.offset != reader.size) {
        goto format_error;
    }
    *selection_out = selection;
    return NES_TAS_OK;

format_error:
    free(selection);
    return NES_TAS_FORMAT_ERROR;
}

static NesTasResult fm3_parse_lag_log(Fm3Reader *reader, size_t frames, uint8_t **lag_out, Fm3DecodeBudget *budget) {
    if (!reader || !lag_out || !budget) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    *lag_out = NULL;
    uint32_t count = 0;
    if (!fm3_reader_u32(reader, &count) || count > frames || count > INT32_MAX) {
        return NES_TAS_FORMAT_ERROR;
    }
    size_t allocation_bytes = 0;
    if (!fm3_size_add(&allocation_bytes, frames) || !fm3_size_add(&allocation_bytes, (size_t)count)) {
        return NES_TAS_RANGE_ERROR;
    }
    NesTasResult result = fm3_budget_take(budget, allocation_bytes);
    if (result != NES_TAS_OK) {
        return result;
    }
    uint8_t *lag = frames ? (uint8_t *)calloc(frames, 1) : NULL;
    if (frames && !lag) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    if (count) {
        uint32_t compressed_size = 0;
        const uint8_t *compressed = NULL;
        if (!fm3_reader_u32(reader, &compressed_size) || !compressed_size ||
            !fm3_reader_bytes(reader, compressed_size, &compressed)) {
            free(lag);
            return NES_TAS_FORMAT_ERROR;
        }
        uint8_t *raw = (uint8_t *)malloc(count);
        if (!raw) {
            free(lag);
            return NES_TAS_OUT_OF_MEMORY;
        }
        mz_ulong unpacked = count;
        int zresult = mz_uncompress(raw, &unpacked, compressed, compressed_size);
        if (zresult != MZ_OK || unpacked != count) {
            free(raw);
            free(lag);
            return NES_TAS_FORMAT_ERROR;
        }
        for (uint32_t frame = 0; frame < count; ++frame) {
            switch (raw[frame]) {
            case 0:
                lag[frame] = NES_TAS_LAG_NO;
                break;
            case 1:
                lag[frame] = NES_TAS_LAG_YES;
                break;
            case 2:
                lag[frame] = NES_TAS_LAG_UNKNOWN;
                break;
            default:
                free(raw);
                free(lag);
                return NES_TAS_FORMAT_ERROR;
            }
        }
        free(raw);
    }
    *lag_out = lag;
    return NES_TAS_OK;
}

static NesTasResult fm3_parse_greenzone(const NesFm3ProjectModule *module, size_t frames, uint8_t **lag_out,
                                        Fm3DecodeBudget *budget) {
    if (!module || !lag_out || !budget || (!module->data && module->size)) {
        return NES_TAS_FORMAT_ERROR;
    }
    *lag_out = NULL;
    Fm3Reader reader = {module->data, module->size, 0};
    const uint8_t *id = NULL;
    if (!fm3_reader_bytes(&reader, FM3_GREENZONE_ID_SIZE, &id)) {
        return NES_TAS_FORMAT_ERROR;
    }
    bool skipped = memcmp(id, fm3_greenzone_skip_id, sizeof(fm3_greenzone_skip_id)) == 0;
    if (!skipped && memcmp(id, fm3_greenzone_id, sizeof(fm3_greenzone_id)) != 0) {
        return NES_TAS_FORMAT_ERROR;
    }

    uint8_t *lag = NULL;
    NesTasResult result = fm3_parse_lag_log(&reader, frames, &lag, budget);
    if (result != NES_TAS_OK) {
        return result;
    }

    uint32_t cursor = 0;
    if (skipped) {
        if (!fm3_reader_u32(&reader, &cursor) || cursor > frames) {
            goto format_error;
        }
        if (cursor) {
            uint32_t state_size = 0;
            if (!fm3_reader_u32(&reader, &state_size) || !fm3_reader_bytes(&reader, state_size, NULL)) {
                goto format_error;
            }
        }
    } else {
        uint32_t greenzone_size = 0;
        if (!fm3_reader_u32(&reader, &greenzone_size) || greenzone_size > frames || !fm3_reader_u32(&reader, &cursor) ||
            cursor > frames) {
            goto format_error;
        }
        int32_t previous = -1;
        bool terminated = false;
        while (reader.offset < reader.size) {
            uint32_t encoded_frame = 0;
            if (!fm3_reader_u32(&reader, &encoded_frame)) {
                goto format_error;
            }
            int32_t frame = (int32_t)encoded_frame;
            if (frame == -1) {
                terminated = true;
                break;
            }
            if (frame < 0 || (uint32_t)frame >= greenzone_size || frame <= previous) {
                goto format_error;
            }
            uint32_t state_size = 0;
            if (!fm3_reader_u32(&reader, &state_size) || !fm3_reader_bytes(&reader, state_size, NULL)) {
                goto format_error;
            }
            previous = frame;
        }
        if (!terminated) {
            goto format_error;
        }
    }
    if (reader.offset != reader.size) {
        goto format_error;
    }
    *lag_out = lag;
    return NES_TAS_OK;

format_error:
    free(lag);
    return NES_TAS_FORMAT_ERROR;
}

static NesTasResult fm3_decompress_exact(const uint8_t *compressed, uint32_t compressed_size, uint8_t *destination,
                                         size_t expected_size) {
    if ((!compressed && compressed_size) || (!destination && expected_size)) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    uint8_t empty = 0;
    mz_ulong unpacked = (mz_ulong)expected_size;
    if ((size_t)unpacked != expected_size) {
        return NES_TAS_RANGE_ERROR;
    }
    int zresult = mz_uncompress(expected_size ? destination : &empty, &unpacked, compressed, compressed_size);
    return zresult == MZ_OK && unpacked == (mz_ulong)expected_size ? NES_TAS_OK : NES_TAS_FORMAT_ERROR;
}

static NesTasResult fm3_parse_input_log(Fm3Reader *reader, const NesFm2Movie *source, NesFm2Frame **frames_out,
                                        size_t *frame_count_out, Fm3DecodeBudget *budget) {
    if (!reader || !source || !frames_out || !frame_count_out || !budget) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    *frames_out = NULL;
    *frame_count_out = 0;
    uint32_t frame_count32 = 0;
    uint8_t input_type = 0;
    uint8_t expected_type = 0;
    unsigned joypads = 0;
    NesFm2Limits limits = nes_fm2_default_limits();
    if (!fm3_reader_u32(reader, &frame_count32) || frame_count32 > limits.max_frames || frame_count32 > INT32_MAX ||
        !fm3_reader_u8(reader, &input_type) || !fm3_movie_input_type(source, &expected_type, &joypads) ||
        input_type != expected_type) {
        return NES_TAS_FORMAT_ERROR;
    }

    size_t frame_count = frame_count32;
    size_t joystick_bytes = 0;
    size_t frame_bytes = 0;
    size_t allocation_bytes = 0;
    if (!fm3_size_mul(frame_count, joypads, &joystick_bytes) ||
        !fm3_size_mul(frame_count, sizeof(NesFm2Frame), &frame_bytes) ||
        !fm3_size_add(&allocation_bytes, joystick_bytes) || !fm3_size_add(&allocation_bytes, frame_count) ||
        !fm3_size_add(&allocation_bytes, frame_bytes)) {
        return NES_TAS_RANGE_ERROR;
    }
    NesTasResult result = fm3_budget_take(budget, allocation_bytes);
    if (result != NES_TAS_OK) {
        return result;
    }
    uint32_t joystick_compressed_size = 0;
    const uint8_t *joystick_compressed = NULL;
    uint32_t command_compressed_size = 0;
    const uint8_t *command_compressed = NULL;
    if (!fm3_reader_u32(reader, &joystick_compressed_size) || !joystick_compressed_size ||
        !fm3_reader_bytes(reader, joystick_compressed_size, &joystick_compressed) ||
        !fm3_reader_u32(reader, &command_compressed_size) || !command_compressed_size ||
        !fm3_reader_bytes(reader, command_compressed_size, &command_compressed)) {
        return NES_TAS_FORMAT_ERROR;
    }

    uint8_t *joysticks = joystick_bytes ? (uint8_t *)malloc(joystick_bytes) : NULL;
    uint8_t *commands = frame_count ? (uint8_t *)malloc(frame_count) : NULL;
    if ((joystick_bytes && !joysticks) || (frame_count && !commands)) {
        free(joysticks);
        free(commands);
        return NES_TAS_OUT_OF_MEMORY;
    }
    result = fm3_decompress_exact(joystick_compressed, joystick_compressed_size, joysticks, joystick_bytes);
    if (result == NES_TAS_OK) {
        result = fm3_decompress_exact(command_compressed, command_compressed_size, commands, frame_count);
    }
    if (result != NES_TAS_OK) {
        free(joysticks);
        free(commands);
        return result;
    }

    uint8_t hot_changes = 0;
    if (!fm3_reader_u8(reader, &hot_changes) || hot_changes > 1) {
        free(joysticks);
        free(commands);
        return NES_TAS_FORMAT_ERROR;
    }
    if (hot_changes) {
        size_t hot_size = 0;
        if (!fm3_size_mul(joystick_bytes, 4, &hot_size)) {
            free(joysticks);
            free(commands);
            return NES_TAS_RANGE_ERROR;
        }
        uint32_t hot_compressed_size = 0;
        const uint8_t *hot_compressed = NULL;
        if (!fm3_reader_u32(reader, &hot_compressed_size) || !hot_compressed_size ||
            !fm3_reader_bytes(reader, hot_compressed_size, &hot_compressed)) {
            free(joysticks);
            free(commands);
            return NES_TAS_FORMAT_ERROR;
        }
        result = fm3_budget_take(budget, hot_size);
        if (result != NES_TAS_OK) {
            free(joysticks);
            free(commands);
            return result;
        }
        uint8_t *hot = hot_size ? (uint8_t *)malloc(hot_size) : NULL;
        if (hot_size && !hot) {
            free(joysticks);
            free(commands);
            return NES_TAS_OUT_OF_MEMORY;
        }
        result = fm3_decompress_exact(hot_compressed, hot_compressed_size, hot, hot_size);
        free(hot);
        if (result != NES_TAS_OK) {
            free(joysticks);
            free(commands);
            return result;
        }
    }

    NesFm2Frame *frames = frame_count ? (NesFm2Frame *)calloc(frame_count, sizeof(*frames)) : NULL;
    if (frame_count && !frames) {
        free(joysticks);
        free(commands);
        return NES_TAS_OUT_OF_MEMORY;
    }
    for (size_t frame = 0; frame < frame_count; ++frame) {
        frames[frame].commands = commands[frame];
        for (unsigned joy = 0; joy < joypads; ++joy) {
            frames[frame].pads[joy] = joysticks[frame * joypads + joy];
        }
    }
    free(joysticks);
    free(commands);
    *frames_out = frames;
    *frame_count_out = frame_count;
    return NES_TAS_OK;
}

static void fm3_bookmarks_import_init(Fm3BookmarksImport *imported) {
    memset(imported, 0, sizeof(*imported));
    imported->current_branch = -1;
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        tas_bookmark_init(&imported->bookmarks[slot]);
    }
}

static void fm3_bookmarks_import_free(Fm3BookmarksImport *imported) {
    if (!imported) {
        return;
    }
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        tas_bookmark_free(&imported->bookmarks[slot]);
    }
    imported->current_branch = -1;
}

static NesTasResult fm3_parse_snapshot(Fm3Reader *reader, const NesFm2Movie *source, TasBookmark *bookmark,
                                       Fm3DecodeBudget *budget) {
    if (!reader || !source || !bookmark || !budget) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    uint32_t values[6];
    for (unsigned i = 0; i < 6; ++i) {
        if (!fm3_reader_u32(reader, &values[i])) {
            return NES_TAS_FORMAT_ERROR;
        }
    }
    uint8_t description_size = 0;
    const uint8_t *description = NULL;
    if (!fm3_reader_u8(reader, &description_size) || description_size >= 100 ||
        !fm3_reader_bytes(reader, description_size, &description) ||
        (description_size && memchr(description, 0, description_size))) {
        return NES_TAS_FORMAT_ERROR;
    }

    NesFm2Frame *frames = NULL;
    size_t frame_count = 0;
    NesTasResult result = fm3_parse_input_log(reader, source, &frames, &frame_count, budget);
    if (result != NES_TAS_OK) {
        return result;
    }

    uint8_t *lag = NULL;
    result = fm3_parse_lag_log(reader, frame_count, &lag, budget);
    if (result != NES_TAS_OK) {
        free(frames);
        return result;
    }
    Fm3MarkersImport markers = {0};
    result = fm3_parse_markers_body(reader, frame_count, &markers, budget);
    if (result != NES_TAS_OK) {
        free(frames);
        free(lag);
        return result;
    }
    if (values[0] > frame_count || values[0] > INT32_MAX) {
        free(frames);
        free(lag);
        fm3_markers_import_free(&markers);
        return NES_TAS_FORMAT_ERROR;
    }

    TasTimeline timeline;
    tas_timeline_init(&timeline);
    result = fm3_budget_movie_metadata(budget, source);
    if (result != NES_TAS_OK) {
        free(frames);
        free(lag);
        fm3_markers_import_free(&markers);
        return result;
    }
    NesFm2Movie metadata = *source;
    metadata.frames = NULL;
    metadata.frame_count = 0;
    metadata.project_present = false;
    metadata.project_timeline_valid = false;
    metadata.project_version = 0;
    metadata.project_saved_modules = 0;
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        metadata.project_modules[i].data = NULL;
        metadata.project_modules[i].size = 0;
    }
    result = tas_timeline_from_movie(&timeline, &metadata);
    if (result != NES_TAS_OK) {
        free(frames);
        free(lag);
        fm3_markers_import_free(&markers);
        return result;
    }
    free(timeline.movie.frames);
    timeline.movie.frames = frames;
    timeline.movie.frame_count = frame_count;
    free(timeline.lag);
    timeline.lag = lag;
    for (size_t i = 0; i < timeline.marker_count; ++i) {
        free(timeline.markers[i].note);
    }
    free(timeline.markers);
    free(timeline.intro_note);
    timeline.markers = markers.markers;
    timeline.marker_count = markers.marker_count;
    timeline.marker_capacity = markers.marker_count;
    timeline.intro_note = markers.intro_note;
    memset(&markers, 0, sizeof(markers));

    size_t name_size = description_size < NES_TAS_BOOKMARK_NAME_MAX ? description_size : NES_TAS_BOOKMARK_NAME_MAX;
    result = fm3_budget_take(budget, name_size + 1);
    if (result != NES_TAS_OK) {
        tas_timeline_free(&timeline);
        return result;
    }
    char *name = (char *)malloc(name_size + 1);
    if (!name) {
        tas_timeline_free(&timeline);
        return NES_TAS_OUT_OF_MEMORY;
    }
    if (name_size) {
        memcpy(name, description, name_size);
    }
    name[name_size] = '\0';

    bookmark->occupied = true;
    bookmark->key_frame = values[0];
    bookmark->name = name;
    bookmark->timeline = timeline;
    bookmark->parent_slot = -1;
    return NES_TAS_OK;
}

static NesTasResult fm3_parse_bookmarks(const NesFm3ProjectModule *module, const NesFm2Movie *source,
                                        Fm3BookmarksImport *out, Fm3DecodeBudget *budget) {
    if (!module || !source || !out || !budget || (!module->data && module->size)) {
        return NES_TAS_FORMAT_ERROR;
    }
    Fm3Reader reader = {module->data, module->size, 0};
    const uint8_t *id = NULL;
    if (!fm3_reader_bytes(&reader, FM3_BOOKMARKS_ID_SIZE, &id)) {
        goto format_error;
    }
    if (memcmp(id, fm3_bookmarks_skip_id, sizeof(fm3_bookmarks_skip_id)) == 0) {
        if (reader.offset != reader.size) {
            goto format_error;
        }
        return NES_TAS_OK;
    }
    if (memcmp(id, fm3_bookmarks_id, sizeof(fm3_bookmarks_id)) != 0) {
        goto format_error;
    }

    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        uint8_t occupied = 0;
        if (!fm3_reader_u8(&reader, &occupied) || occupied > 1) {
            goto format_error;
        }
        if (!occupied) {
            continue;
        }
        NesTasResult result = fm3_parse_snapshot(&reader, source, &out->bookmarks[slot], budget);
        if (result != NES_TAS_OK) {
            fm3_bookmarks_import_free(out);
            return result;
        }
        uint32_t state_size = 0;
        uint32_t screenshot_size = 0;
        if (!fm3_reader_u32(&reader, &state_size) || !fm3_reader_bytes(&reader, state_size, NULL) ||
            !fm3_reader_u32(&reader, &screenshot_size) || !fm3_reader_bytes(&reader, screenshot_size, NULL)) {
            goto format_error;
        }
    }

    const uint8_t *timestamp = NULL;
    uint32_t current_branch = 0;
    uint8_t changes = 0;
    if (!fm3_reader_bytes(&reader, 9, &timestamp) || timestamp[8] != 0 || !fm3_reader_u32(&reader, &current_branch) ||
        !fm3_reader_u8(&reader, &changes) || changes > 1 || !fm3_reader_bytes(&reader, 9, &timestamp) ||
        timestamp[8] != 0) {
        goto format_error;
    }
    int32_t branch = (int32_t)current_branch;
    if (branch < -1 || branch >= NES_TAS_BOOKMARK_COUNT) {
        goto format_error;
    }
    out->current_branch = branch;

    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        uint32_t encoded_parent = 0;
        if (!fm3_reader_u32(&reader, &encoded_parent)) {
            goto format_error;
        }
        int32_t parent = (int32_t)encoded_parent;
        if (parent < -1 || parent >= NES_TAS_BOOKMARK_COUNT || parent == (int32_t)slot) {
            goto format_error;
        }
        out->bookmarks[slot].parent_slot = parent;
    }
    const uint8_t *cached = NULL;
    if (!fm3_reader_bytes(&reader, NES_TAS_BOOKMARK_COUNT, &cached)) {
        goto format_error;
    }
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        if (cached[slot] != 0xFF && cached[slot] >= NES_TAS_BOOKMARK_COUNT) {
            goto format_error;
        }
    }
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT * NES_TAS_BOOKMARK_COUNT; ++i) {
        uint32_t ignored = 0;
        if (!fm3_reader_u32(&reader, &ignored)) {
            goto format_error;
        }
    }
    if (reader.offset != reader.size) {
        goto format_error;
    }
    if (!fm3_bookmark_relations_valid(out->bookmarks, out->current_branch)) {
        goto format_error;
    }
    return NES_TAS_OK;

format_error:
    fm3_bookmarks_import_free(out);
    return NES_TAS_FORMAT_ERROR;
}

NesTasResult tas_import_fm3_metadata(NesTasProject *project, const NesFm2Movie *source) {
    if (!project || !source) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (!source->project_present) {
        return NES_TAS_OK;
    }
    if (source->project_version != 3 || (source->project_saved_modules & ~TAS_FM3_ALL_MODULES)) {
        return NES_TAS_UNSUPPORTED;
    }

    Fm3MarkersImport markers = {0};
    Fm3BookmarksImport bookmarks;
    fm3_bookmarks_import_init(&bookmarks);
    uint8_t *selection = NULL;
    uint8_t *lag = NULL;
    Fm3DecodeBudget budget = {FM3_DECODE_BUDGET_BYTES};
    bool have_markers = (source->project_saved_modules & TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS)) != 0;
    bool have_selection = (source->project_saved_modules & TAS_FM3_MODULE_BIT(NES_FM3_MODULE_SELECTION)) != 0;
    NesTasResult result = NES_TAS_OK;

    if (have_markers) {
        result =
            fm3_parse_markers(&source->project_modules[NES_FM3_MODULE_MARKERS], source->frame_count, &markers, &budget);
        if (result != NES_TAS_OK) {
            goto done;
        }
    }
    if (have_selection) {
        result = fm3_parse_selection(&source->project_modules[NES_FM3_MODULE_SELECTION], source->frame_count,
                                     &selection, &budget);
        if (result != NES_TAS_OK) {
            goto done;
        }
    }
    result =
        fm3_parse_greenzone(&source->project_modules[NES_FM3_MODULE_GREENZONE], source->frame_count, &lag, &budget);
    if (result != NES_TAS_OK) {
        goto done;
    }
    result = fm3_parse_bookmarks(&source->project_modules[NES_FM3_MODULE_BOOKMARKS], source, &bookmarks, &budget);
    if (result != NES_TAS_OK) {
        goto done;
    }

    if (have_markers) {
        TasTimeline *timeline = &project->state.timeline;
        for (size_t i = 0; i < timeline->marker_count; ++i) {
            free(timeline->markers[i].note);
        }
        free(timeline->markers);
        free(timeline->intro_note);
        timeline->markers = markers.markers;
        timeline->marker_count = markers.marker_count;
        timeline->marker_capacity = markers.marker_count;
        timeline->intro_note = markers.intro_note;
        memset(&markers, 0, sizeof(markers));
    }
    if (have_selection) {
        free(project->state.selection);
        project->state.selection = selection;
        selection = NULL;
    }
    free(project->state.timeline.lag);
    project->state.timeline.lag = lag;
    lag = NULL;
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        tas_bookmark_free(&project->state.bookmarks[slot]);
        project->state.bookmarks[slot] = bookmarks.bookmarks[slot];
        tas_bookmark_init(&bookmarks.bookmarks[slot]);
    }
    project->state.current_branch = bookmarks.current_branch;

done:
    fm3_markers_import_free(&markers);
    fm3_bookmarks_import_free(&bookmarks);
    free(selection);
    free(lag);
    return result;
}
