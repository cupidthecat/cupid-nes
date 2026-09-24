/*
 * tas_project_edit.c - TAS input timeline and clipboard editing
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_project_internal.h"

#include <stdlib.h>
#include <string.h>

static size_t tas_fail_after = SIZE_MAX;
static size_t tas_allocations;

void tas_project_test_fail_alloc_after(size_t successful_allocations) {
    tas_fail_after = successful_allocations;
    tas_allocations = 0;
}

void tas_project_test_reset_alloc_fail(void) {
    tas_fail_after = SIZE_MAX;
    tas_allocations = 0;
}

static void *tas_edit_malloc(size_t size) {
    if (tas_allocations >= tas_fail_after) return NULL;
    tas_allocations++;
    return malloc(size ? size : 1);
}

static void *tas_edit_calloc(size_t count, size_t size) {
    if (tas_allocations >= tas_fail_after) return NULL;
    tas_allocations++;
    return calloc(count ? count : 1, size ? size : 1);
}

static void invalidate_lag_from(NesTasProject *project, size_t first) {
    size_t frames = project->state.timeline.movie.frame_count;
    if (first < frames) memset(project->state.timeline.lag + first, NES_TAS_LAG_UNKNOWN,
                               frames - first);
    tas_project_mark_dirty(project, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_GREENZONE));
}

NesTasResult tas_project_resize_insert(NesTasProject *project, size_t first, size_t count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t old_count = project->state.timeline.movie.frame_count;
    if (first > old_count) return NES_TAS_RANGE_ERROR;
    if (!count) return NES_TAS_OK;
    if (count > SIZE_MAX - old_count) return NES_TAS_RANGE_ERROR;
    size_t new_count = old_count + count;
    if (new_count > SIZE_MAX / sizeof(NesFm2Frame)) return NES_TAS_RANGE_ERROR;

    NesFm2Frame *frames = tas_edit_calloc(new_count, sizeof(*frames));
    uint8_t *lag = (uint8_t *)tas_edit_calloc(new_count, 1);
    uint8_t *selection = (uint8_t *)tas_edit_calloc(new_count, 1);
    if (!frames || !lag || !selection) {
        free(frames);
        free(lag);
        free(selection);
        return NES_TAS_OUT_OF_MEMORY;
    }

    if (first) {
        memcpy(frames, project->state.timeline.movie.frames, first * sizeof(*frames));
        memcpy(lag, project->state.timeline.lag, first);
        memcpy(selection, project->state.selection, first);
    }
    size_t tail = old_count - first;
    if (tail) {
        memcpy(frames + first + count, project->state.timeline.movie.frames + first,
               tail * sizeof(*frames));
        memcpy(lag + first + count, project->state.timeline.lag + first, tail);
        memcpy(selection + first + count, project->state.selection + first, tail);
    }

    free(project->state.timeline.movie.frames);
    free(project->state.timeline.lag);
    free(project->state.selection);
    project->state.timeline.movie.frames = frames;
    project->state.timeline.movie.frame_count = new_count;
    project->state.timeline.lag = lag;
    project->state.selection = selection;
    for (size_t i = 0; i < project->state.timeline.marker_count; ++i) {
        if (project->state.timeline.markers[i].frame >= first)
            project->state.timeline.markers[i].frame += count;
    }
    return NES_TAS_OK;
}

NesTasResult tas_project_resize_delete(NesTasProject *project, size_t first, size_t count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t old_count = project->state.timeline.movie.frame_count;
    if (first > old_count || count > old_count - first) return NES_TAS_RANGE_ERROR;
    if (!count) return NES_TAS_OK;
    size_t new_count = old_count - count;

    NesFm2Frame *frames = NULL;
    uint8_t *lag = NULL;
    uint8_t *selection = NULL;
    if (new_count) {
        frames = (NesFm2Frame *)tas_edit_malloc(new_count * sizeof(*frames));
        lag = (uint8_t *)tas_edit_malloc(new_count);
        selection = (uint8_t *)tas_edit_malloc(new_count);
        if (!frames || !lag || !selection) {
            free(frames);
            free(lag);
            free(selection);
            return NES_TAS_OUT_OF_MEMORY;
        }
        if (first) {
            memcpy(frames, project->state.timeline.movie.frames, first * sizeof(*frames));
            memcpy(lag, project->state.timeline.lag, first);
            memcpy(selection, project->state.selection, first);
        }
        size_t tail = old_count - first - count;
        if (tail) {
            memcpy(frames + first, project->state.timeline.movie.frames + first + count,
                   tail * sizeof(*frames));
            memcpy(lag + first, project->state.timeline.lag + first + count, tail);
            memcpy(selection + first, project->state.selection + first + count, tail);
        }
    }

    free(project->state.timeline.movie.frames);
    free(project->state.timeline.lag);
    free(project->state.selection);
    project->state.timeline.movie.frames = frames;
    project->state.timeline.movie.frame_count = new_count;
    project->state.timeline.lag = lag;
    project->state.selection = selection;

    size_t out = 0;
    size_t end = first + count;
    for (size_t i = 0; i < project->state.timeline.marker_count; ++i) {
        TasMarker marker = project->state.timeline.markers[i];
        if (marker.frame >= first && marker.frame < end) {
            free(marker.note);
            continue;
        }
        if (marker.frame >= end) marker.frame -= count;
        project->state.timeline.markers[out++] = marker;
    }
    project->state.timeline.marker_count = out;
    return NES_TAS_OK;
}

bool nes_tas_clipboard_available(const NesTasProject *project) {
    return project && project->clipboard.count != 0;
}

NesTasResult nes_tas_copy_selection(NesTasProject *project) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t first = 0, last = 0;
    if (!nes_tas_selection_bounds(project, &first, &last)) return NES_TAS_EMPTY_SELECTION;
    size_t span = last - first + 1;
    if (span > SIZE_MAX / sizeof(NesFm2Frame)) return NES_TAS_RANGE_ERROR;

    TasClipboard replacement = {0};
    replacement.frames = (NesFm2Frame *)tas_edit_malloc(span * sizeof(*replacement.frames));
    replacement.present = (uint8_t *)tas_edit_calloc(span, 1);
    if (!replacement.frames || !replacement.present) {
        tas_clipboard_free(&replacement);
        return NES_TAS_OUT_OF_MEMORY;
    }
    replacement.span = span;
    for (size_t i = 0; i < span; ++i) {
        size_t frame = first + i;
        if (!project->state.selection[frame]) continue;
        replacement.frames[i] = project->state.timeline.movie.frames[frame];
        replacement.present[i] = 1;
        replacement.count++;
    }
    tas_clipboard_free(&project->clipboard);
    project->clipboard = replacement;
    return NES_TAS_OK;
}

NesTasResult nes_tas_cut_selection(NesTasProject *project) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    NesTasResult copied = nes_tas_copy_selection(project);
    if (copied != NES_TAS_OK) return copied;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    size_t first_changed = SIZE_MAX;
    NesFm2Frame zero = {0};
    size_t frames = project->state.timeline.movie.frame_count;
    for (size_t i = 0; i < frames; ++i) {
        if (!project->state.selection[i]
            || tas_frame_equal(&project->state.timeline.movie.frames[i], &zero)) continue;
        project->state.timeline.movie.frames[i] = zero;
        if (i < first_changed) first_changed = i;
    }
    if (first_changed != SIZE_MAX) {
        invalidate_lag_from(project, first_changed);
        tas_project_mark_change(project, first_changed, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_paste(NesTasProject *project, size_t first, NesTasPasteMode mode) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    if (mode != NES_TAS_PASTE_OVERWRITE && mode != NES_TAS_PASTE_INSERT)
        return NES_TAS_INVALID_ARGUMENT;
    if (!project->clipboard.count) return NES_TAS_EMPTY_CLIPBOARD;
    size_t frames = project->state.timeline.movie.frame_count;
    if (first > frames || project->clipboard.span > SIZE_MAX - first)
        return NES_TAS_RANGE_ERROR;
    if (mode == NES_TAS_PASTE_OVERWRITE && first + project->clipboard.span > frames)
        return NES_TAS_RANGE_ERROR;

    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    if (mode == NES_TAS_PASTE_INSERT) {
        result = tas_project_resize_insert(project, first, project->clipboard.span);
        if (result != NES_TAS_OK) return tas_project_auto_finish(project, owned, result);
    }
    size_t first_changed = SIZE_MAX;
    for (size_t i = 0; i < project->clipboard.span; ++i) {
        if (!project->clipboard.present[i]) continue;
        size_t frame = first + i;
        NesFm2Frame *target = &project->state.timeline.movie.frames[frame];
        if (!tas_frame_equal(target, &project->clipboard.frames[i])) {
            *target = project->clipboard.frames[i];
            if (frame < first_changed) first_changed = frame;
        }
    }
    if (mode == NES_TAS_PASTE_INSERT && first_changed == SIZE_MAX) first_changed = first;
    if (first_changed != SIZE_MAX) {
        invalidate_lag_from(project, first_changed);
        tas_project_mark_change(project, first_changed, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_insert_frames(NesTasProject *project, size_t first, size_t count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    if (!count) return first <= project->state.timeline.movie.frame_count
        ? NES_TAS_OK : NES_TAS_RANGE_ERROR;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    result = tas_project_resize_insert(project, first, count);
    if (result == NES_TAS_OK && count) {
        invalidate_lag_from(project, first);
        tas_project_mark_change(project, first, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, result);
}

NesTasResult nes_tas_delete_frames(NesTasProject *project, size_t first, size_t count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    if (!count) return first <= project->state.timeline.movie.frame_count
        ? NES_TAS_OK : NES_TAS_RANGE_ERROR;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    result = tas_project_resize_delete(project, first, count);
    if (result == NES_TAS_OK && count) {
        invalidate_lag_from(project, first);
        tas_project_mark_change(project, first, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, result);
}

NesTasResult nes_tas_clone_frames(NesTasProject *project, size_t first, size_t count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t frames = project->state.timeline.movie.frame_count;
    if (!count || first >= frames || count > frames - first) return NES_TAS_RANGE_ERROR;
    if (count > SIZE_MAX / sizeof(NesFm2Frame)) return NES_TAS_RANGE_ERROR;
    NesFm2Frame *copy = (NesFm2Frame *)tas_edit_malloc(count * sizeof(*copy));
    if (!copy) return NES_TAS_OUT_OF_MEMORY;
    memcpy(copy, project->state.timeline.movie.frames + first, count * sizeof(*copy));

    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result == NES_TAS_OK) result = tas_project_resize_insert(project, first + count, count);
    if (result == NES_TAS_OK) {
        memcpy(project->state.timeline.movie.frames + first + count, copy, count * sizeof(*copy));
        invalidate_lag_from(project, first + count);
        tas_project_mark_change(project, first + count, TAS_FM3_ALL_MODULES);
    }
    free(copy);
    return tas_project_auto_finish(project, owned, result);
}

NesTasResult nes_tas_delete_selection(NesTasProject *project) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t first = 0, last = 0;
    if (!nes_tas_selection_bounds(project, &first, &last)) return NES_TAS_EMPTY_SELECTION;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    size_t cursor = last + 1;
    while (cursor > first) {
        size_t end = cursor;
        while (end > first && !project->state.selection[end - 1]) end--;
        if (end == first && !project->state.selection[first]) break;
        size_t start = end;
        while (start > first && project->state.selection[start - 1]) start--;
        if (start == end && project->state.selection[first]) start = first;
        result = tas_project_resize_delete(project, start, end - start);
        if (result != NES_TAS_OK) return tas_project_auto_finish(project, owned, result);
        cursor = start;
    }
    invalidate_lag_from(project, first);
    tas_project_mark_change(project, first, TAS_FM3_ALL_MODULES);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_clone_selection(NesTasProject *project) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t first = 0, last = 0;
    if (!nes_tas_selection_bounds(project, &first, &last)) return NES_TAS_EMPTY_SELECTION;
    size_t selected = nes_tas_selection_count(project);
    if (!selected || selected > SIZE_MAX / sizeof(NesFm2Frame)) return NES_TAS_RANGE_ERROR;

    NesFm2Frame *copies = (NesFm2Frame *)tas_edit_malloc(selected * sizeof(*copies));
    size_t *positions = (size_t *)tas_edit_malloc(selected * sizeof(*positions));
    if (!copies || !positions) {
        free(copies);
        free(positions);
        return NES_TAS_OUT_OF_MEMORY;
    }
    size_t n = 0;
    for (size_t i = first; i <= last; ++i) {
        if (!project->state.selection[i]) continue;
        copies[n] = project->state.timeline.movie.frames[i];
        positions[n++] = i;
    }

    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) goto done;
    for (size_t index = selected; index > 0;) {
        size_t group_end = index;
        size_t group_start = index - 1;
        while (group_start > 0 && positions[group_start - 1] + 1 == positions[group_start]) group_start--;
        size_t count = group_end - group_start;
        size_t at = positions[group_start];
        result = tas_project_resize_insert(project, at, count);
        if (result != NES_TAS_OK) break;
        memcpy(project->state.timeline.movie.frames + at, copies + group_start,
               count * sizeof(*copies));
        index = group_start;
    }
    if (result == NES_TAS_OK) {
        size_t new_frames = project->state.timeline.movie.frame_count;
        memset(project->state.selection, 0, new_frames);
        size_t shift = selected;
        size_t run = 1;
        for (size_t index = selected; index > 0; --index) {
            if (index > 1 && positions[index - 2] + 1 == positions[index - 1]) {
                run++;
                continue;
            }
            size_t at = positions[index - 1] + shift;
            for (size_t j = 0; j < run; ++j) project->state.selection[at + j] = 1;
            shift -= run;
            run = 1;
        }
        invalidate_lag_from(project, first);
        tas_project_mark_change(project, first, TAS_FM3_ALL_MODULES);
    }
    result = tas_project_auto_finish(project, owned, result);

done:
    free(copies);
    free(positions);
    return result;
}

NesTasResult nes_tas_truncate(NesTasProject *project, size_t frame_count) {
    if (!project) return NES_TAS_INVALID_ARGUMENT;
    size_t current = project->state.timeline.movie.frame_count;
    if (frame_count > current) return NES_TAS_RANGE_ERROR;
    if (frame_count == current) return NES_TAS_OK;
    return nes_tas_delete_frames(project, frame_count, current - frame_count);
}

NesTasResult nes_tas_set_frame(NesTasProject *project, size_t frame,
                               const NesFm2Frame *input) {
    if (!project || !input) return NES_TAS_INVALID_ARGUMENT;
    if (frame >= project->state.timeline.movie.frame_count) return NES_TAS_RANGE_ERROR;
    if (tas_frame_equal(&project->state.timeline.movie.frames[frame], input)) return NES_TAS_OK;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    project->state.timeline.movie.frames[frame] = *input;
    invalidate_lag_from(project, frame);
    tas_project_mark_change(project, frame, TAS_FM3_ALL_MODULES);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_record_frame(NesTasProject *project, size_t frame,
                                  const NesFm2Frame *input, bool insert) {
    if (!project || !input) return NES_TAS_INVALID_ARGUMENT;
    size_t frames = project->state.timeline.movie.frame_count;
    if (frame > frames || (insert && frame > frames)) return NES_TAS_RANGE_ERROR;
    if (!insert && frame < frames && tas_frame_equal(&project->state.timeline.movie.frames[frame], input))
        return NES_TAS_OK;

    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    bool append = frame == frames;
    if (insert || append) result = tas_project_resize_insert(project, frame, 1);
    if (result == NES_TAS_OK) {
        project->state.timeline.movie.frames[frame] = *input;
        invalidate_lag_from(project, frame);
        tas_project_mark_change(project, frame, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, result);
}

NesTasResult nes_tas_paint_button(NesTasProject *project, size_t first, size_t last,
                                  unsigned controller, uint8_t button_mask, bool pressed) {
    if (!project || controller >= 4) return NES_TAS_INVALID_ARGUMENT;
    size_t frames = project->state.timeline.movie.frame_count;
    if (first > last || last >= frames) return NES_TAS_RANGE_ERROR;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    size_t changed = SIZE_MAX;
    for (size_t i = first; i <= last; ++i) {
        uint8_t old = project->state.timeline.movie.frames[i].pads[controller];
        uint8_t value = pressed ? (uint8_t)(old | button_mask) : (uint8_t)(old & ~button_mask);
        if (old == value) continue;
        project->state.timeline.movie.frames[i].pads[controller] = value;
        if (changed == SIZE_MAX) changed = i;
    }
    if (changed != SIZE_MAX) {
        invalidate_lag_from(project, changed);
        tas_project_mark_change(project, changed, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_apply_pattern(NesTasProject *project, size_t first, size_t last,
                                   unsigned controller, uint8_t button_mask,
                                   const uint8_t *pattern, size_t pattern_length,
                                   bool skip_known_lag) {
    if (!project || !pattern || !pattern_length || controller >= 4)
        return NES_TAS_INVALID_ARGUMENT;
    size_t frames = project->state.timeline.movie.frame_count;
    if (first > last || last >= frames) return NES_TAS_RANGE_ERROR;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) return result;
    size_t changed = SIZE_MAX, pattern_index = 0;
    for (size_t i = first; i <= last; ++i) {
        if (skip_known_lag && project->state.timeline.lag[i] == NES_TAS_LAG_YES) continue;
        bool pressed = pattern[pattern_index++ % pattern_length] != 0;
        uint8_t old = project->state.timeline.movie.frames[i].pads[controller];
        uint8_t value = pressed ? (uint8_t)(old | button_mask) : (uint8_t)(old & ~button_mask);
        if (old == value) continue;
        project->state.timeline.movie.frames[i].pads[controller] = value;
        if (changed == SIZE_MAX) changed = i;
    }
    if (changed != SIZE_MAX) {
        invalidate_lag_from(project, changed);
        tas_project_mark_change(project, changed, TAS_FM3_ALL_MODULES);
    }
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}
