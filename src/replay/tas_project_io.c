/*
 * tas_project_io.c - native editable TAS project persistence
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_project_internal.h"

#include "../third_party/miniz/miniz.h"
#include "../util/file_io.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

enum {
    CTAS_MAX_FILE_BYTES = 1024u * 1024u * 1024u,
    CTAS_MAX_DECODED_BYTES = 512u * 1024u * 1024u,
    CTAS_FRAME_BYTES = 29
};

typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
    NesTasResult result;
} CtasWriter;

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t offset;
    size_t allocation_budget;
    NesTasResult result;
} CtasReader;

static NesTasResult ctas_from_fm2(NesFm2Result result) {
    switch (result) {
        case NES_FM2_OK: return NES_TAS_OK;
        case NES_FM2_INVALID_ARGUMENT: return NES_TAS_INVALID_ARGUMENT;
        case NES_FM2_OUT_OF_MEMORY: return NES_TAS_OUT_OF_MEMORY;
        case NES_FM2_LIMIT: return NES_TAS_RANGE_ERROR;
        case NES_FM2_UNSUPPORTED: return NES_TAS_UNSUPPORTED;
        case NES_FM2_TRUNCATED:
        case NES_FM2_CORRUPT:
        default: return NES_TAS_FORMAT_ERROR;
    }
}

static NesTasResult ctas_from_file(NesFileResult result) {
    switch (result) {
        case NES_FILE_OK: return NES_TAS_OK;
        case NES_FILE_INVALID_ARGUMENT: return NES_TAS_INVALID_ARGUMENT;
        case NES_FILE_OUT_OF_MEMORY: return NES_TAS_OUT_OF_MEMORY;
        case NES_FILE_TOO_LARGE: return NES_TAS_RANGE_ERROR;
        case NES_FILE_NOT_FOUND:
        case NES_FILE_IO_ERROR:
        default: return NES_TAS_IO_ERROR;
    }
}

static bool ctas_writer_reserve(CtasWriter *writer, size_t bytes) {
    if (writer->result != NES_TAS_OK) return false;
    if (writer->size > CTAS_MAX_FILE_BYTES || bytes > CTAS_MAX_FILE_BYTES - writer->size) {
        writer->result = NES_TAS_RANGE_ERROR;
        return false;
    }
    size_t required = writer->size + bytes;
    if (required <= writer->capacity) return true;

    size_t capacity = writer->capacity ? writer->capacity : 4096;
    while (capacity < required) {
        capacity = capacity <= CTAS_MAX_FILE_BYTES / 2 ? capacity * 2 : required;
    }
    uint8_t *data = (uint8_t *)realloc(writer->data, capacity);
    if (!data) {
        writer->result = NES_TAS_OUT_OF_MEMORY;
        return false;
    }
    writer->data = data;
    writer->capacity = capacity;
    return true;
}

static void ctas_write_bytes(CtasWriter *writer, const void *data, size_t size) {
    if (!ctas_writer_reserve(writer, size)) return;
    if (size) memcpy(writer->data + writer->size, data, size);
    writer->size += size;
}

static void ctas_write_u8(CtasWriter *writer, uint8_t value) {
    ctas_write_bytes(writer, &value, 1);
}

static void ctas_write_u32(CtasWriter *writer, uint32_t value) {
    uint8_t bytes[4] = {
        (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)
    };
    ctas_write_bytes(writer, bytes, sizeof(bytes));
}

static void ctas_write_i32(CtasWriter *writer, int32_t value) {
    ctas_write_u32(writer, (uint32_t)value);
}

static void ctas_write_u64(CtasWriter *writer, uint64_t value) {
    uint8_t bytes[8];
    for (unsigned i = 0; i < 8; ++i) bytes[i] = (uint8_t)(value >> (i * 8));
    ctas_write_bytes(writer, bytes, sizeof(bytes));
}

static void ctas_write_string(CtasWriter *writer, const char *text, size_t limit) {
    if (!text) text = "";
    size_t length = strlen(text);
    if (length > limit || length > UINT32_MAX) {
        writer->result = NES_TAS_RANGE_ERROR;
        return;
    }
    ctas_write_u32(writer, (uint32_t)length);
    ctas_write_bytes(writer, text, length);
}

static const uint8_t *ctas_read_bytes(CtasReader *reader, size_t size) {
    if (reader->result != NES_TAS_OK) return NULL;
    if (reader->offset > reader->size || size > reader->size - reader->offset) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return NULL;
    }
    const uint8_t *data = reader->data + reader->offset;
    reader->offset += size;
    return data;
}

static bool ctas_reader_charge(CtasReader *reader, size_t bytes) {
    if (reader->result != NES_TAS_OK) return false;
    if (bytes > reader->allocation_budget) {
        reader->result = NES_TAS_RANGE_ERROR;
        return false;
    }
    reader->allocation_budget -= bytes;
    return true;
}

static bool ctas_reader_charge_array(CtasReader *reader, size_t count, size_t item_size) {
    if (item_size && count > SIZE_MAX / item_size) {
        reader->result = NES_TAS_RANGE_ERROR;
        return false;
    }
    return ctas_reader_charge(reader, count * item_size);
}

static uint8_t ctas_read_u8(CtasReader *reader) {
    const uint8_t *data = ctas_read_bytes(reader, 1);
    return data ? data[0] : 0;
}

static uint32_t ctas_read_u32(CtasReader *reader) {
    const uint8_t *data = ctas_read_bytes(reader, 4);
    if (!data) return 0;
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16
        | (uint32_t)data[3] << 24;
}

static int32_t ctas_read_i32(CtasReader *reader) {
    return (int32_t)ctas_read_u32(reader);
}

static uint64_t ctas_read_u64(CtasReader *reader) {
    const uint8_t *data = ctas_read_bytes(reader, 8);
    if (!data) return 0;
    uint64_t value = 0;
    for (unsigned i = 0; i < 8; ++i) value |= (uint64_t)data[i] << (i * 8);
    return value;
}

static char *ctas_read_string(CtasReader *reader, size_t limit) {
    uint32_t length = ctas_read_u32(reader);
    if (reader->result != NES_TAS_OK) return NULL;
    if (length > limit) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return NULL;
    }
    const uint8_t *data = ctas_read_bytes(reader, length);
    if (!data && length) return NULL;
    if (length && memchr(data, '\0', length)) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return NULL;
    }
    if (!ctas_reader_charge(reader, (size_t)length + 1)) return NULL;
    char *text = (char *)malloc((size_t)length + 1);
    if (!text) {
        reader->result = NES_TAS_OUT_OF_MEMORY;
        return NULL;
    }
    if (length) memcpy(text, data, length);
    text[length] = '\0';
    return text;
}

static void ctas_write_frame(CtasWriter *writer, const NesFm2Frame *frame) {
    ctas_write_u8(writer, frame->commands);
    ctas_write_bytes(writer, frame->pads, sizeof(frame->pads));
    for (unsigned port = 0; port < 2; ++port) {
        const NesFm2Zapper *zapper = &frame->zappers[port];
        ctas_write_u8(writer, zapper->x);
        ctas_write_u8(writer, zapper->y);
        ctas_write_u8(writer, zapper->button);
        ctas_write_u8(writer, zapper->bogo);
        ctas_write_u64(writer, zapper->zaphit);
    }
}

static void ctas_read_frame(CtasReader *reader, NesFm2Frame *frame) {
    memset(frame, 0, sizeof(*frame));
    frame->commands = ctas_read_u8(reader);
    const uint8_t *pads = ctas_read_bytes(reader, sizeof(frame->pads));
    if (pads) memcpy(frame->pads, pads, sizeof(frame->pads));
    for (unsigned port = 0; port < 2; ++port) {
        NesFm2Zapper *zapper = &frame->zappers[port];
        zapper->x = ctas_read_u8(reader);
        zapper->y = ctas_read_u8(reader);
        zapper->button = ctas_read_u8(reader);
        zapper->bogo = ctas_read_u8(reader);
        zapper->zaphit = ctas_read_u64(reader);
    }
}

static void ctas_write_movie(CtasWriter *writer, const NesFm2Movie *movie) {
    if (writer->result != NES_TAS_OK) return;
    size_t measured = 0;
    NesFm2Diagnostic diagnostic;
    NesFm2Result fm2 = nes_fm2_measure(movie, NES_FM2_BINARY, &measured, &diagnostic);
    if (fm2 != NES_FM2_OK) {
        writer->result = ctas_from_fm2(fm2);
        return;
    }
    ctas_write_u64(writer, measured);
    if (!ctas_writer_reserve(writer, measured)) return;
    size_t written = 0;
    fm2 = nes_fm2_export(movie, NES_FM2_BINARY, writer->data + writer->size,
                         writer->capacity - writer->size, &written, &diagnostic);
    if (fm2 != NES_FM2_OK || written != measured) {
        writer->result = fm2 == NES_FM2_OK ? NES_TAS_FORMAT_ERROR : ctas_from_fm2(fm2);
        return;
    }
    writer->size += written;
}

static void ctas_read_movie(CtasReader *reader, NesFm2Movie *movie) {
    uint64_t encoded_size = ctas_read_u64(reader);
    if (reader->result != NES_TAS_OK) return;
    if (encoded_size > SIZE_MAX) {
        reader->result = NES_TAS_RANGE_ERROR;
        return;
    }
    const uint8_t *encoded = ctas_read_bytes(reader, (size_t)encoded_size);
    if (!encoded && encoded_size) return;
    if (!ctas_reader_charge(reader, (size_t)encoded_size)) return;
    NesFm2Limits limits = nes_fm2_default_limits();
    size_t budget_frames = reader->allocation_budget / sizeof(NesFm2Frame);
    if (budget_frames < limits.max_frames) limits.max_frames = budget_frames ? budget_frames : 1;
    NesFm2Diagnostic diagnostic;
    NesFm2Result fm2 = nes_fm2_parse(encoded, (size_t)encoded_size, &limits, movie, &diagnostic);
    if (fm2 != NES_FM2_OK) {
        if (fm2 == NES_FM2_OUT_OF_MEMORY) reader->result = NES_TAS_OUT_OF_MEMORY;
        else if (fm2 == NES_FM2_LIMIT) reader->result = NES_TAS_RANGE_ERROR;
        else reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    if (!ctas_reader_charge_array(reader, movie->frame_count, sizeof(NesFm2Frame))) return;
    if (movie->project_present) reader->result = NES_TAS_FORMAT_ERROR;
}

static void ctas_write_timeline(CtasWriter *writer, const TasTimeline *timeline) {
    ctas_write_movie(writer, &timeline->movie);
    ctas_write_u64(writer, timeline->movie.frame_count);
    ctas_write_bytes(writer, timeline->lag, timeline->movie.frame_count);
    ctas_write_string(writer, timeline->intro_note, NES_TAS_MARKER_NOTE_MAX);
    ctas_write_u64(writer, timeline->marker_count);
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        ctas_write_u64(writer, timeline->markers[i].frame);
        ctas_write_string(writer, timeline->markers[i].note, NES_TAS_MARKER_NOTE_MAX);
    }
}

static void ctas_read_timeline(CtasReader *reader, TasTimeline *timeline) {
    ctas_read_movie(reader, &timeline->movie);
    if (reader->result != NES_TAS_OK) return;
    uint64_t lag_count = ctas_read_u64(reader);
    if (lag_count != timeline->movie.frame_count) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    size_t frames = timeline->movie.frame_count;
    if (frames) {
        if (reader->offset > reader->size || frames > reader->size - reader->offset) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        if (!ctas_reader_charge(reader, frames)) return;
        timeline->lag = (uint8_t *)malloc(frames);
        if (!timeline->lag) {
            reader->result = NES_TAS_OUT_OF_MEMORY;
            return;
        }
        const uint8_t *lag = ctas_read_bytes(reader, frames);
        if (!lag) return;
        memcpy(timeline->lag, lag, frames);
        for (size_t i = 0; i < frames; ++i) {
            if (timeline->lag[i] > NES_TAS_LAG_YES) {
                reader->result = NES_TAS_FORMAT_ERROR;
                return;
            }
        }
    }
    timeline->intro_note = ctas_read_string(reader, NES_TAS_MARKER_NOTE_MAX);
    if (!timeline->intro_note) return;
    uint64_t marker_count = ctas_read_u64(reader);
    if (reader->result != NES_TAS_OK) return;
    if (marker_count > frames || marker_count > SIZE_MAX / sizeof(*timeline->markers)) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    if (marker_count) {
        size_t remaining = reader->offset <= reader->size ? reader->size - reader->offset : 0;
        if (marker_count > remaining / 12) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        if (!ctas_reader_charge_array(reader, (size_t)marker_count,
                                      sizeof(*timeline->markers)))
            return;
        timeline->markers = (TasMarker *)calloc((size_t)marker_count, sizeof(*timeline->markers));
        if (!timeline->markers) {
            reader->result = NES_TAS_OUT_OF_MEMORY;
            return;
        }
        timeline->marker_capacity = (size_t)marker_count;
    }
    size_t previous = 0;
    for (size_t i = 0; i < (size_t)marker_count; ++i) {
        uint64_t frame = ctas_read_u64(reader);
        if (frame >= frames || frame > SIZE_MAX || (i && frame <= previous)) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        timeline->markers[i].frame = (size_t)frame;
        timeline->markers[i].note = ctas_read_string(reader, NES_TAS_MARKER_NOTE_MAX);
        if (!timeline->markers[i].note) return;
        timeline->marker_count++;
        previous = (size_t)frame;
    }
}

static void ctas_write_bookmark(CtasWriter *writer, const TasBookmark *bookmark) {
    ctas_write_u8(writer, bookmark->occupied ? 1 : 0);
    if (!bookmark->occupied) return;
    ctas_write_u64(writer, bookmark->key_frame);
    ctas_write_i32(writer, bookmark->parent_slot);
    ctas_write_string(writer, bookmark->name, NES_TAS_BOOKMARK_NAME_MAX);
    ctas_write_timeline(writer, &bookmark->timeline);
}

static void ctas_read_bookmark(CtasReader *reader, TasBookmark *bookmark) {
    uint8_t occupied = ctas_read_u8(reader);
    if (occupied > 1 || reader->result != NES_TAS_OK) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    if (!occupied) return;
    bookmark->occupied = true;
    uint64_t key_frame = ctas_read_u64(reader);
    int32_t parent = ctas_read_i32(reader);
    if (key_frame > SIZE_MAX || parent < -1 || parent >= NES_TAS_BOOKMARK_COUNT) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    bookmark->key_frame = (size_t)key_frame;
    bookmark->parent_slot = parent;
    bookmark->name = ctas_read_string(reader, NES_TAS_BOOKMARK_NAME_MAX);
    if (!bookmark->name) return;
    ctas_read_timeline(reader, &bookmark->timeline);
    if (reader->result == NES_TAS_OK && bookmark->key_frame > bookmark->timeline.movie.frame_count)
        reader->result = NES_TAS_FORMAT_ERROR;
}

static void ctas_write_state(CtasWriter *writer, const TasProjectState *state) {
    ctas_write_timeline(writer, &state->timeline);
    ctas_write_u64(writer, state->timeline.movie.frame_count);
    ctas_write_bytes(writer, state->selection, state->timeline.movie.frame_count);
    ctas_write_i32(writer, state->current_branch);
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot)
        ctas_write_bookmark(writer, &state->bookmarks[slot]);
}

static void ctas_read_state(CtasReader *reader, TasProjectState *state) {
    ctas_read_timeline(reader, &state->timeline);
    if (reader->result != NES_TAS_OK) return;
    size_t frames = state->timeline.movie.frame_count;
    uint64_t selection_count = ctas_read_u64(reader);
    if (selection_count != frames) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    if (frames) {
        if (reader->offset > reader->size || frames > reader->size - reader->offset) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        if (!ctas_reader_charge(reader, frames)) return;
        state->selection = (uint8_t *)malloc(frames);
        if (!state->selection) {
            reader->result = NES_TAS_OUT_OF_MEMORY;
            return;
        }
        const uint8_t *selection = ctas_read_bytes(reader, frames);
        if (!selection) return;
        memcpy(state->selection, selection, frames);
        for (size_t i = 0; i < frames; ++i) {
            if (state->selection[i] > 1) {
                reader->result = NES_TAS_FORMAT_ERROR;
                return;
            }
        }
    }
    state->current_branch = ctas_read_i32(reader);
    if (state->current_branch < -1 || state->current_branch >= NES_TAS_BOOKMARK_COUNT) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        ctas_read_bookmark(reader, &state->bookmarks[slot]);
        if (reader->result != NES_TAS_OK) return;
    }
    if (state->current_branch >= 0 && !state->bookmarks[state->current_branch].occupied)
        reader->result = NES_TAS_FORMAT_ERROR;
}

static void ctas_write_clipboard(CtasWriter *writer, const TasClipboard *clipboard) {
    ctas_write_u64(writer, clipboard->span);
    ctas_write_u64(writer, clipboard->count);
    for (size_t i = 0; i < clipboard->span; ++i) {
        ctas_write_u8(writer, clipboard->present[i] ? 1 : 0);
        if (clipboard->present[i]) ctas_write_frame(writer, &clipboard->frames[i]);
    }
}

static void ctas_read_clipboard(CtasReader *reader, TasClipboard *clipboard) {
    uint64_t span64 = ctas_read_u64(reader);
    uint64_t count64 = ctas_read_u64(reader);
    if (reader->result != NES_TAS_OK) return;
    size_t remaining = reader->offset <= reader->size ? reader->size - reader->offset : 0;
    NesFm2Limits limits = nes_fm2_default_limits();
    if (span64 > SIZE_MAX || count64 > span64 || span64 > SIZE_MAX / sizeof(*clipboard->frames) ||
        span64 > limits.max_frames || span64 > remaining ||
        count64 > (remaining - (size_t)span64) / CTAS_FRAME_BYTES) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    clipboard->span = (size_t)span64;
    clipboard->count = (size_t)count64;
    if (!clipboard->span) return;
    if (!ctas_reader_charge_array(reader, clipboard->span,
                                  sizeof(*clipboard->frames) + 1))
        return;
    clipboard->frames = (NesFm2Frame *)calloc(clipboard->span, sizeof(*clipboard->frames));
    clipboard->present = (uint8_t *)calloc(clipboard->span, 1);
    if (!clipboard->frames || !clipboard->present) {
        reader->result = NES_TAS_OUT_OF_MEMORY;
        return;
    }
    size_t count = 0;
    for (size_t i = 0; i < clipboard->span; ++i) {
        uint8_t present = ctas_read_u8(reader);
        if (reader->result != NES_TAS_OK || present > 1) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        clipboard->present[i] = present;
        if (present) {
            ctas_read_frame(reader, &clipboard->frames[i]);
            if (reader->result != NES_TAS_OK) return;
            count++;
        }
    }
    if (reader->result == NES_TAS_OK && count != clipboard->count)
        reader->result = NES_TAS_FORMAT_ERROR;
}

static void ctas_write_history(CtasWriter *writer, const TasHistoryEntry *entry, size_t count) {
    ctas_write_u64(writer, count);
    for (; entry; entry = entry->next) {
        ctas_write_u64(writer, entry->first_changed == SIZE_MAX ? UINT64_MAX : entry->first_changed);
        ctas_write_state(writer, &entry->state);
    }
}

static void ctas_read_history(CtasReader *reader, TasHistoryEntry **head, size_t *count,
                              size_t *history_bytes) {
    uint64_t count64 = ctas_read_u64(reader);
    if (reader->result != NES_TAS_OK) return;
    if (count64 > SIZE_MAX || count64 > CTAS_MAX_FILE_BYTES / sizeof(TasHistoryEntry)) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    size_t remaining = reader->offset <= reader->size ? reader->size - reader->offset : 0;
    if (count64 > remaining / 8) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    TasHistoryEntry **tail = head;
    for (size_t i = 0; i < (size_t)count64; ++i) {
        if (!ctas_reader_charge(reader, sizeof(TasHistoryEntry))) return;
        TasHistoryEntry *entry = (TasHistoryEntry *)calloc(1, sizeof(*entry));
        if (!entry) {
            reader->result = NES_TAS_OUT_OF_MEMORY;
            return;
        }
        tas_state_init(&entry->state);
        uint64_t first = ctas_read_u64(reader);
        if (first != UINT64_MAX && first > SIZE_MAX) reader->result = NES_TAS_FORMAT_ERROR;
        entry->first_changed = first == UINT64_MAX ? SIZE_MAX : (size_t)first;
        ctas_read_state(reader, &entry->state);
        if (reader->result != NES_TAS_OK) {
            tas_state_free(&entry->state);
            free(entry);
            return;
        }
        if (entry->first_changed != SIZE_MAX
            && entry->first_changed > entry->state.timeline.movie.frame_count) {
            tas_state_free(&entry->state);
            free(entry);
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        entry->bytes = tas_state_estimated_bytes(&entry->state);
        if (entry->bytes > SIZE_MAX - *history_bytes) {
            tas_state_free(&entry->state);
            free(entry);
            reader->result = NES_TAS_RANGE_ERROR;
            return;
        }
        *history_bytes += entry->bytes;
        *tail = entry;
        tail = &entry->next;
        (*count)++;
    }
}

static void ctas_write_source_modules(CtasWriter *writer, const NesTasProject *project) {
    ctas_write_u8(writer, project->source_project_present ? 1 : 0);
    ctas_write_u32(writer, project->source_project_version);
    ctas_write_u32(writer, project->source_saved_modules);
    ctas_write_u32(writer, project->fm3_dirty_modules);
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        ctas_write_u64(writer, project->source_modules[i].size);
        ctas_write_bytes(writer, project->source_modules[i].data, project->source_modules[i].size);
    }
}

static void ctas_read_source_modules(CtasReader *reader, NesTasProject *project) {
    uint8_t present = ctas_read_u8(reader);
    project->source_project_version = ctas_read_u32(reader);
    project->source_saved_modules = ctas_read_u32(reader);
    project->fm3_dirty_modules = ctas_read_u32(reader);
    if (present > 1 || (project->source_saved_modules & ~TAS_FM3_ALL_MODULES)
        || (project->fm3_dirty_modules & ~TAS_FM3_ALL_MODULES)) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    project->source_project_present = present != 0;
    if (project->source_project_present && project->source_project_version != 3) {
        reader->result = NES_TAS_FORMAT_ERROR;
        return;
    }
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        uint64_t size64 = ctas_read_u64(reader);
        if (size64 > SIZE_MAX) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        size_t size = (size_t)size64;
        const uint8_t *data = ctas_read_bytes(reader, size);
        if (!data && size) return;
        if (size) {
            if (!ctas_reader_charge(reader, size)) return;
            project->source_modules[i].data = (uint8_t *)malloc(size);
            if (!project->source_modules[i].data) {
                reader->result = NES_TAS_OUT_OF_MEMORY;
                return;
            }
            memcpy(project->source_modules[i].data, data, size);
            project->source_modules[i].size = size;
        }
    }
    if (!project->source_project_present) {
        if (project->source_saved_modules || project->source_project_version) {
            reader->result = NES_TAS_FORMAT_ERROR;
            return;
        }
        for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
            if (project->source_modules[i].size) {
                reader->result = NES_TAS_FORMAT_ERROR;
                return;
            }
        }
    }
}

NesTasResult nes_tas_project_save(const NesTasProject *project, const char *path) {
    if (!project || !path || project->edit_active) return NES_TAS_INVALID_ARGUMENT;
    CtasWriter writer = {0};
    writer.result = NES_TAS_OK;
    ctas_write_bytes(&writer, NES_CTAS_MAGIC, NES_CTAS_MAGIC_SIZE);
    ctas_write_u32(&writer, NES_CTAS_VERSION);
    ctas_write_u64(&writer, project->max_history_entries);
    ctas_write_u64(&writer, project->max_history_bytes);
    ctas_write_source_modules(&writer, project);
    ctas_write_state(&writer, &project->state);
    ctas_write_clipboard(&writer, &project->clipboard);
    ctas_write_history(&writer, project->undo, project->undo_count);
    ctas_write_history(&writer, project->redo, project->redo_count);
    if (writer.result == NES_TAS_OK) {
        uint32_t checksum = (uint32_t)mz_crc32(MZ_CRC32_INIT, writer.data, writer.size);
        ctas_write_u32(&writer, checksum);
    }
    if (writer.result != NES_TAS_OK) {
        NesTasResult result = writer.result;
        free(writer.data);
        return result;
    }
    NesTasResult result = ctas_from_file(nes_file_write_atomic(path, writer.data, writer.size));
    free(writer.data);
    return result;
}

NesTasResult nes_tas_project_load(const char *path, NesTasProject **out) {
    if (!path || !out) return NES_TAS_INVALID_ARGUMENT;
    uint8_t *data = NULL;
    size_t size = 0;
    NesFileResult file = nes_file_read_all(path, CTAS_MAX_FILE_BYTES, &data, &size);
    if (file != NES_FILE_OK) return ctas_from_file(file);
    if (size < NES_CTAS_MAGIC_SIZE + 4 + 4) {
        free(data);
        return NES_TAS_FORMAT_ERROR;
    }
    uint32_t stored_crc = (uint32_t)data[size - 4] | (uint32_t)data[size - 3] << 8
        | (uint32_t)data[size - 2] << 16 | (uint32_t)data[size - 1] << 24;
    uint32_t actual_crc = (uint32_t)mz_crc32(MZ_CRC32_INIT, data, size - 4);
    if (stored_crc != actual_crc) {
        free(data);
        return NES_TAS_FORMAT_ERROR;
    }

    CtasReader reader = {data, size - 4, 0, CTAS_MAX_DECODED_BYTES, NES_TAS_OK};
    const uint8_t *magic = ctas_read_bytes(&reader, NES_CTAS_MAGIC_SIZE);
    uint32_t version = ctas_read_u32(&reader);
    if (!magic || memcmp(magic, NES_CTAS_MAGIC, NES_CTAS_MAGIC_SIZE) != 0
        || version != NES_CTAS_VERSION) reader.result = NES_TAS_FORMAT_ERROR;

    NesTasProject *project = NULL;
    if (reader.result == NES_TAS_OK) {
        if (ctas_reader_charge(&reader, sizeof(NesTasProject))) {
            project = (NesTasProject *)calloc(1, sizeof(*project));
            if (!project) reader.result = NES_TAS_OUT_OF_MEMORY;
        }
    }
    if (project) {
        tas_state_init(&project->state);
        project->state.current_branch = -1;
        uint64_t max_entries = ctas_read_u64(&reader);
        uint64_t max_bytes = ctas_read_u64(&reader);
        if (max_entries > SIZE_MAX || max_bytes > SIZE_MAX) reader.result = NES_TAS_FORMAT_ERROR;
        project->max_history_entries = (size_t)max_entries;
        project->max_history_bytes = (size_t)max_bytes;
        ctas_read_source_modules(&reader, project);
        ctas_read_state(&reader, &project->state);
        ctas_read_clipboard(&reader, &project->clipboard);
        ctas_read_history(&reader, &project->undo, &project->undo_count,
                          &project->history_bytes);
        ctas_read_history(&reader, &project->redo, &project->redo_count,
                          &project->history_bytes);
        if (reader.result == NES_TAS_OK && reader.offset != reader.size)
            reader.result = NES_TAS_FORMAT_ERROR;
        if (reader.result == NES_TAS_OK
            && (project->undo_count > project->max_history_entries
                || project->redo_count > project->max_history_entries
                || project->history_bytes > project->max_history_bytes))
            reader.result = NES_TAS_FORMAT_ERROR;
    }
    free(data);
    if (reader.result != NES_TAS_OK) {
        nes_tas_project_destroy(project);
        return reader.result;
    }
    *out = project;
    return NES_TAS_OK;
}
