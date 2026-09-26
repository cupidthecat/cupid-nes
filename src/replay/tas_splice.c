/*
 * tas_splice.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Transactional TAS range splicing. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_splice.h"
#include "tas_project_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static NesTasResult result(NesTasResult value, const char *message, char *error, size_t capacity) {
    if (error && capacity) {
        snprintf(error, capacity, "%s", message ? message : value == NES_TAS_OK ? "" : nes_tas_result_string(value));
    }
    return value;
}

static bool same_blob(const NesFm2Blob *a, const NesFm2Blob *b) {
    return a->size == b->size && (!a->size || (a->data && b->data && !memcmp(a->data, b->data, a->size)));
}

static size_t field_count(const NesFm2Movie *movie, const NesFm2HeaderField *field) {
    size_t count = 0;
    for (size_t i = 0; i < movie->extension_count; ++i) {
        const NesFm2HeaderField *other = &movie->extensions[i];
        if (!strcmp(field->key, other->key) && !strcmp(field->value, other->value)) {
            ++count;
        }
    }
    return count;
}

static const char *compatible(const NesFm2Movie *a, const NesFm2Movie *b) {
    if (memcmp(a->rom_md5, b->rom_md5, NES_FM2_MD5_SIZE)) {
        return "Source and destination ROM checksums differ.";
    }
    if (a->pal != b->pal) {
        return "Source and destination regions differ.";
    }
    if (a->fourscore != b->fourscore || a->microphone != b->microphone ||
        memcmp(a->ports, b->ports, sizeof(a->ports))) {
        return "Source and destination controller layouts differ.";
    }
    if (a->fds != b->fds || a->new_ppu != b->new_ppu || a->ram_init_option != b->ram_init_option ||
        a->ram_init_seed != b->ram_init_seed) {
        return "Source and destination startup profiles differ.";
    }
    if (nes_fm2_start_kind(a) != nes_fm2_start_kind(b) || !same_blob(&a->savestate, &b->savestate) ||
        !same_blob(&a->saveram, &b->saveram)) {
        return "Source and destination startup memory differs.";
    }
    if (a->extension_count != b->extension_count) {
        return "Source and destination header extensions differ.";
    }
    for (size_t i = 0; i < a->extension_count; ++i) {
        if (field_count(a, &a->extensions[i]) != field_count(b, &a->extensions[i])) {
            return "Source and destination header extensions differ.";
        }
    }
    return NULL;
}

static bool commands_allowed(const NesFm2Movie *movie, uint8_t allowed) {
    if (!movie->fds) {
        allowed &= (uint8_t)~(NES_FM2_COMMAND_FDS_INSERT | NES_FM2_COMMAND_FDS_SELECT);
    }
    for (size_t i = 0; i < movie->frame_count; ++i) {
        if (movie->frames[i].commands & (uint8_t)~allowed) {
            return false;
        }
    }
    return true;
}

NesTasResult nes_tas_splice_preview(const NesTasProject *destination, const NesTasProject *source,
                                    const NesTasSpliceOptions *options, NesTasSplicePreview *out, char *error,
                                    size_t capacity) {
    if (!destination || !source || !options || !out || (unsigned)options->mode > NES_TAS_SPLICE_EXTRACT ||
        (options->allowed_commands & 0x80u)) {
        return result(NES_TAS_INVALID_ARGUMENT, "Invalid splice options.", error, capacity);
    }
    if (nes_tas_edit_active(destination) || nes_tas_edit_active(source)) {
        return result(NES_TAS_INVALID_ARGUMENT, "Finish the current grouped edit before splicing.", error, capacity);
    }
    const NesFm2Movie *a = nes_tas_project_movie(destination), *b = nes_tas_project_movie(source);
    const char *reason = compatible(a, b);
    if (reason) {
        return result(NES_TAS_UNSUPPORTED, reason, error, capacity);
    }
    if (!commands_allowed(a, options->allowed_commands) || !commands_allowed(b, options->allowed_commands)) {
        return result(NES_TAS_UNSUPPORTED, "Movie commands are unavailable on the destination hardware.", error,
                      capacity);
    }
    if (options->source_first > b->frame_count || !options->source_count ||
        options->source_count > b->frame_count - options->source_first) {
        return result(NES_TAS_RANGE_ERROR, "Choose a nonempty source range inside the movie.", error, capacity);
    }
    NesTasSplicePreview preview = {.source_frames = b->frame_count,
                                   .destination_frames = a->frame_count,
                                   .first = options->destination_first,
                                   .inserted = options->source_count};
    if (options->mode == NES_TAS_SPLICE_APPEND) {
        preview.first = a->frame_count;
    } else if (options->mode == NES_TAS_SPLICE_REPLACE) {
        preview.removed = options->destination_count;
    } else if (options->mode == NES_TAS_SPLICE_EXTRACT) {
        preview.first = 0;
        preview.removed = a->frame_count;
    }
    if (preview.first > a->frame_count || preview.removed > a->frame_count - preview.first) {
        return result(NES_TAS_RANGE_ERROR, "Destination range is outside the current timeline.", error, capacity);
    }
    size_t retained = a->frame_count - preview.removed;
    size_t limit = nes_fm2_default_limits().max_frames;
    if (limit > SIZE_MAX / sizeof(NesFm2Frame)) {
        limit = SIZE_MAX / sizeof(NesFm2Frame);
    }
    if (retained > limit || preview.inserted > limit - retained) {
        return result(NES_TAS_RANGE_ERROR, "The resulting movie exceeds the frame limit.", error, capacity);
    }
    preview.resulting_frames = retained + preview.inserted;
    for (size_t i = 0; i < destination->state.timeline.marker_count; ++i) {
        size_t frame = destination->state.timeline.markers[i].frame;
        if (frame < preview.first || frame >= preview.first + preview.removed) {
            ++preview.resulting_markers;
        }
    }
    if (options->copy_markers) {
        for (size_t i = 0; i < source->state.timeline.marker_count; ++i) {
            size_t frame = source->state.timeline.markers[i].frame;
            if (frame >= options->source_first && frame - options->source_first < options->source_count) {
                ++preview.imported_markers;
            }
        }
    }
    if (preview.imported_markers > SIZE_MAX - preview.resulting_markers ||
        preview.resulting_markers + preview.imported_markers > SIZE_MAX / sizeof(TasMarker)) {
        return result(NES_TAS_RANGE_ERROR, "The resulting marker count is too large.", error, capacity);
    }
    preview.resulting_markers += preview.imported_markers;
    *out = preview;
    return result(NES_TAS_OK, NULL, error, capacity);
}

typedef struct {
    NesFm2Frame *frames;
    uint8_t *lag;
    uint8_t *selection;
    TasMarker *markers;
    size_t marker_count;
} PreparedSplice;

static void prepared_free(PreparedSplice *prepared) {
    free(prepared->frames);
    free(prepared->lag);
    free(prepared->selection);
    for (size_t i = 0; i < prepared->marker_count; ++i) {
        free(prepared->markers[i].note);
    }
    free(prepared->markers);
}

static bool copy_marker(PreparedSplice *prepared, const TasMarker *marker, size_t frame) {
    size_t size = strlen(marker->note) + 1;
    char *note = tas_edit_malloc(size);
    if (!note) {
        return false;
    }
    memcpy(note, marker->note, size);
    prepared->markers[prepared->marker_count++] = (TasMarker){frame, note};
    return true;
}

static bool prepare(PreparedSplice *prepared, const NesTasProject *destination, const NesTasProject *source,
                    const NesTasSpliceOptions *options, const NesTasSplicePreview *preview) {
    const TasTimeline *a = &destination->state.timeline, *b = &source->state.timeline;
    size_t count = preview->resulting_frames;
    prepared->frames = tas_edit_malloc(count * sizeof(*prepared->frames));
    prepared->lag = tas_edit_malloc(count);
    prepared->selection = tas_edit_malloc(count);
    if (preview->resulting_markers) {
        prepared->markers = tas_edit_malloc(preview->resulting_markers * sizeof(*prepared->markers));
    }
    if (!prepared->frames || !prepared->lag || !prepared->selection ||
        (preview->resulting_markers && !prepared->markers)) {
        return false;
    }
    size_t first = preview->first, after = first + preview->removed;
    if (first) {
        memcpy(prepared->frames, a->movie.frames, first * sizeof(*prepared->frames));
        memcpy(prepared->lag, a->lag, first);
        memcpy(prepared->selection, destination->state.selection, first);
    }
    memcpy(prepared->frames + first, b->movie.frames + options->source_first,
           preview->inserted * sizeof(*prepared->frames));
    memset(prepared->selection + first, 1, preview->inserted);
    size_t tail = a->movie.frame_count - after;
    if (tail) {
        memcpy(prepared->frames + first + preview->inserted, a->movie.frames + after, tail * sizeof(*prepared->frames));
        memcpy(prepared->selection + first + preview->inserted, destination->state.selection + after, tail);
    }
    memset(prepared->lag + first, NES_TAS_LAG_UNKNOWN, count - first);
    for (size_t i = 0; i < a->marker_count; ++i) {
        if (a->markers[i].frame < first && !copy_marker(prepared, &a->markers[i], a->markers[i].frame)) {
            return false;
        }
    }
    if (options->copy_markers) {
        for (size_t i = 0; i < b->marker_count; ++i) {
            size_t frame = b->markers[i].frame;
            if (frame >= options->source_first && frame - options->source_first < preview->inserted &&
                !copy_marker(prepared, &b->markers[i], first + frame - options->source_first)) {
                return false;
            }
        }
    }
    for (size_t i = 0; i < a->marker_count; ++i) {
        if (a->markers[i].frame >= after &&
            !copy_marker(prepared, &a->markers[i], a->markers[i].frame - preview->removed + preview->inserted)) {
            return false;
        }
    }
    return true;
}

static size_t first_input_change(const TasTimeline *before, const PreparedSplice *after,
                                 const NesTasSplicePreview *preview) {
    if (before->movie.frame_count != preview->resulting_frames) {
        return preview->first;
    }
    for (size_t i = preview->first; i < preview->resulting_frames; ++i) {
        if (!tas_frame_equal(&before->movie.frames[i], &after->frames[i])) {
            return i;
        }
    }
    return SIZE_MAX;
}

static bool same_markers(const TasTimeline *before, const PreparedSplice *after) {
    if (before->marker_count != after->marker_count) {
        return false;
    }
    for (size_t i = 0; i < before->marker_count; ++i) {
        if (before->markers[i].frame != after->markers[i].frame ||
            strcmp(before->markers[i].note, after->markers[i].note)) {
            return false;
        }
    }
    return true;
}

static bool navigation_changes(const TasProjectState *state, const NesTasSplicePreview *preview) {
    for (size_t i = 0; i < state->navigation_count; ++i) {
        size_t before = state->navigation[i].frame, after = before;
        if (after >= preview->first + preview->removed) {
            after -= preview->removed;
        } else if (after >= preview->first) {
            after = preview->first;
        }
        if (after >= preview->first) {
            after += preview->inserted;
        }
        if (before != after) {
            return true;
        }
    }
    return false;
}

NesTasResult nes_tas_splice_apply(NesTasProject *destination, const NesTasProject *source,
                                  const NesTasSpliceOptions *options, char *error, size_t capacity) {
    NesTasSplicePreview preview;
    NesTasResult checked = nes_tas_splice_preview(destination, source, options, &preview, error, capacity);
    if (checked != NES_TAS_OK) {
        return checked;
    }
    /* Finish every fallible allocation before changing any live project data.
     * This also makes overlapping ranges in the same project safe. */
    PreparedSplice prepared = {0};
    if (!prepare(&prepared, destination, source, options, &preview)) {
        prepared_free(&prepared);
        return result(NES_TAS_OUT_OF_MEMORY, NULL, error, capacity);
    }
    TasTimeline *timeline = &destination->state.timeline;
    size_t changed = first_input_change(timeline, &prepared, &preview);
    bool markers_changed = !same_markers(timeline, &prepared);
    bool selection_changed = timeline->movie.frame_count != preview.resulting_frames ||
                             memcmp(destination->state.selection, prepared.selection, preview.resulting_frames);
    if (changed == SIZE_MAX && !markers_changed && !selection_changed &&
        !navigation_changes(&destination->state, &preview)) {
        prepared_free(&prepared);
        return result(NES_TAS_OK, NULL, error, capacity);
    }
    /* Equal input retains measured lag and cached playback. A changed selection
     * or marker alone must not turn a compatible replacement into a replay. */
    size_t preserved = changed == SIZE_MAX ? preview.resulting_frames : changed;
    if (preserved) {
        memcpy(prepared.lag, timeline->lag, preserved);
    }
    NesTasResult begun = nes_tas_edit_begin(destination);
    if (begun != NES_TAS_OK) {
        prepared_free(&prepared);
        return result(begun, NULL, error, capacity);
    }
    free(timeline->movie.frames);
    free(timeline->lag);
    free(destination->state.selection);
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        free(timeline->markers[i].note);
    }
    free(timeline->markers);
    timeline->movie.frames = prepared.frames;
    timeline->movie.frame_count = preview.resulting_frames;
    timeline->lag = prepared.lag;
    destination->state.selection = prepared.selection;
    timeline->markers = prepared.markers;
    timeline->marker_count = timeline->marker_capacity = prepared.marker_count;
    tas_navigation_delete(&destination->state, preview.first, preview.removed);
    tas_navigation_insert(&destination->state, preview.first, preview.inserted);
    uint32_t dirty = changed != SIZE_MAX ? TAS_FM3_ALL_MODULES : 0;
    if (markers_changed) {
        dirty |= TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS);
        tas_project_set_branch_changed(destination, true);
    }
    if (selection_changed) {
        dirty |= TAS_FM3_MODULE_BIT(NES_FM3_MODULE_SELECTION);
    }
    tas_project_mark_change(destination, changed, dirty);
    return result(nes_tas_edit_end(destination), NULL, error, capacity);
}
