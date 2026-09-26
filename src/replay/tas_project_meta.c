/*
 * tas_project_meta.c - TAS markers, notes, and branch bookmarks
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_project_internal.h"

#include <stdlib.h>
#include <string.h>

static size_t marker_lower_bound(const TasTimeline *timeline, size_t frame) {
    size_t low = 0, high = timeline->marker_count;
    while (low < high) {
        size_t middle = low + (high - low) / 2;
        if (timeline->markers[middle].frame < frame) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    return low;
}

static NesTasResult marker_reserve(TasTimeline *timeline, size_t count) {
    if (count <= timeline->marker_capacity) {
        return NES_TAS_OK;
    }
    size_t capacity = timeline->marker_capacity ? timeline->marker_capacity : 8;
    while (capacity < count) {
        if (capacity > SIZE_MAX / 2) {
            return NES_TAS_RANGE_ERROR;
        }
        capacity *= 2;
    }
    if (capacity > SIZE_MAX / sizeof(*timeline->markers)) {
        return NES_TAS_RANGE_ERROR;
    }
    TasMarker *markers = (TasMarker *)realloc(timeline->markers, capacity * sizeof(*markers));
    if (!markers) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    timeline->markers = markers;
    timeline->marker_capacity = capacity;
    return NES_TAS_OK;
}

size_t nes_tas_marker_count(const NesTasProject *project) {
    return project ? project->state.timeline.marker_count : 0;
}

bool nes_tas_marker(const NesTasProject *project, size_t index, NesTasMarkerView *out) {
    if (!project || !out || index >= project->state.timeline.marker_count) {
        return false;
    }
    const TasMarker *marker = &project->state.timeline.markers[index];
    out->frame = marker->frame;
    out->note = marker->note ? marker->note : "";
    return true;
}

NesTasResult nes_tas_marker_set(NesTasProject *project, size_t frame, const char *note) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (frame >= project->state.timeline.movie.frame_count) {
        return NES_TAS_RANGE_ERROR;
    }
    char *copy = tas_strdup_limit(note, NES_TAS_MARKER_NOTE_MAX);
    if (!copy) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    TasTimeline *timeline = &project->state.timeline;
    size_t index = marker_lower_bound(timeline, frame);
    if (index < timeline->marker_count && timeline->markers[index].frame == frame &&
        strcmp(timeline->markers[index].note ? timeline->markers[index].note : "", copy) == 0) {
        free(copy);
        return NES_TAS_OK;
    }
    bool replacing = index < timeline->marker_count && timeline->markers[index].frame == frame;
    NesTasResult result = NES_TAS_OK;
    if (!replacing) {
        result = marker_reserve(timeline, timeline->marker_count + 1);
        if (result != NES_TAS_OK) {
            free(copy);
            return result;
        }
    }
    bool owned = false;
    result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        free(copy);
        return result;
    }
    if (replacing) {
        free(timeline->markers[index].note);
        timeline->markers[index].note = copy;
    } else {
        memmove(timeline->markers + index + 1, timeline->markers + index,
                (timeline->marker_count - index) * sizeof(*timeline->markers));
        timeline->markers[index].frame = frame;
        timeline->markers[index].note = copy;
        timeline->marker_count++;
    }
    tas_project_set_branch_changed(project, true);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_marker_remove(NesTasProject *project, size_t frame) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (frame >= project->state.timeline.movie.frame_count) {
        return NES_TAS_RANGE_ERROR;
    }
    TasTimeline *timeline = &project->state.timeline;
    size_t index = marker_lower_bound(timeline, frame);
    if (index >= timeline->marker_count || timeline->markers[index].frame != frame) {
        return NES_TAS_OK;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    free(timeline->markers[index].note);
    timeline->marker_count--;
    memmove(timeline->markers + index, timeline->markers + index + 1,
            (timeline->marker_count - index) * sizeof(*timeline->markers));
    tas_project_set_branch_changed(project, true);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_marker_move(NesTasProject *project, size_t from_frame, size_t to_frame) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = project->state.timeline.movie.frame_count;
    if (from_frame >= frames || to_frame >= frames) {
        return NES_TAS_RANGE_ERROR;
    }
    if (from_frame == to_frame) {
        return NES_TAS_OK;
    }
    TasTimeline *timeline = &project->state.timeline;
    size_t from = marker_lower_bound(timeline, from_frame);
    size_t to = marker_lower_bound(timeline, to_frame);
    if (from >= timeline->marker_count || timeline->markers[from].frame != from_frame) {
        return NES_TAS_RANGE_ERROR;
    }
    if (to < timeline->marker_count && timeline->markers[to].frame == to_frame) {
        return NES_TAS_RANGE_ERROR;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    TasMarker moving = timeline->markers[from];
    memmove(timeline->markers + from, timeline->markers + from + 1,
            (timeline->marker_count - from - 1) * sizeof(*timeline->markers));
    timeline->marker_count--;
    to = marker_lower_bound(timeline, to_frame);
    memmove(timeline->markers + to + 1, timeline->markers + to,
            (timeline->marker_count - to) * sizeof(*timeline->markers));
    moving.frame = to_frame;
    timeline->markers[to] = moving;
    timeline->marker_count++;
    tas_project_set_branch_changed(project, true);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

const char *nes_tas_intro_note(const NesTasProject *project) {
    return project && project->state.timeline.intro_note ? project->state.timeline.intro_note : "";
}

NesTasResult nes_tas_set_intro_note(NesTasProject *project, const char *note) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    char *copy = tas_strdup_limit(note, NES_TAS_MARKER_NOTE_MAX);
    if (!copy) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    const char *old = project->state.timeline.intro_note ? project->state.timeline.intro_note : "";
    if (strcmp(old, copy) == 0) {
        free(copy);
        return NES_TAS_OK;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        free(copy);
        return result;
    }
    free(project->state.timeline.intro_note);
    project->state.timeline.intro_note = copy;
    tas_project_set_branch_changed(project, true);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_MARKERS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

bool nes_tas_bookmark(const NesTasProject *project, unsigned slot, NesTasBookmarkView *out) {
    if (!project || !out || slot >= NES_TAS_BOOKMARK_COUNT) {
        return false;
    }
    const TasBookmark *bookmark = &project->state.bookmarks[slot];
    out->occupied = bookmark->occupied;
    out->key_frame = bookmark->key_frame;
    out->name = bookmark->name ? bookmark->name : "";
    out->parent_slot = bookmark->parent_slot;
    return true;
}

static size_t timeline_first_difference(const TasTimeline *left, const TasTimeline *right);

static bool tas_bookmark_branch_reaches(const NesTasProject *project, int branch, unsigned target) {
    for (unsigned depth = 0; branch >= 0 && depth < NES_TAS_BOOKMARK_COUNT; ++depth) {
        if ((unsigned)branch == target) {
            return true;
        }
        if (branch >= NES_TAS_BOOKMARK_COUNT || !project->state.bookmarks[branch].occupied) {
            return false;
        }
        branch = project->state.bookmarks[branch].parent_slot;
    }
    return false;
}

static int tas_bookmark_parent_for_key_frame(const NesTasProject *project, unsigned slot, size_t key_frame) {
    int branch = project->state.current_branch;
    unsigned seen = 0;
    while (branch >= 0 && branch < NES_TAS_BOOKMARK_COUNT) {
        unsigned bit = 1u << (unsigned)branch;
        if ((seen & bit) || !project->state.bookmarks[branch].occupied) {
            return -1;
        }
        seen |= bit;
        if ((unsigned)branch != slot && project->state.bookmarks[branch].key_frame <= key_frame &&
            !tas_bookmark_branch_reaches(project, branch, slot) &&
            timeline_first_difference(&project->state.bookmarks[branch].timeline, &project->state.timeline) >=
                project->state.bookmarks[branch].key_frame) {
            return branch;
        }
        branch = project->state.bookmarks[branch].parent_slot;
    }
    return -1;
}

NesTasResult nes_tas_bookmark_set(NesTasProject *project, unsigned slot, size_t key_frame, const char *name) {
    if (!project || slot >= NES_TAS_BOOKMARK_COUNT) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (key_frame > project->state.timeline.movie.frame_count) {
        return NES_TAS_RANGE_ERROR;
    }
    TasBookmark replacement;
    tas_bookmark_init(&replacement);
    replacement.occupied = true;
    replacement.key_frame = key_frame;
    replacement.parent_slot = tas_bookmark_parent_for_key_frame(project, slot, key_frame);
    replacement.name = tas_strdup_limit(name, NES_TAS_BOOKMARK_NAME_MAX);
    if (!replacement.name) {
        tas_bookmark_free(&replacement);
        return NES_TAS_OUT_OF_MEMORY;
    }
    NesTasResult result = tas_timeline_clone(&project->state.timeline, &replacement.timeline);
    if (result != NES_TAS_OK) {
        tas_bookmark_free(&replacement);
        return result;
    }
    bool owned = false;
    result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        tas_bookmark_free(&replacement);
        return result;
    }
    int previous_parent = project->state.bookmarks[slot].parent_slot;
    for (unsigned child = 0; child < NES_TAS_BOOKMARK_COUNT; ++child) {
        TasBookmark *bookmark = &project->state.bookmarks[child];
        if (bookmark->occupied && bookmark->parent_slot == (int)slot &&
            (bookmark->key_frame < key_frame ||
             timeline_first_difference(&bookmark->timeline, &replacement.timeline) < key_frame)) {
            bookmark->parent_slot = previous_parent;
        }
    }
    tas_bookmark_free(&project->state.bookmarks[slot]);
    project->state.bookmarks[slot] = replacement;
    tas_bookmark_init(&replacement);
    project->state.current_branch = (int)slot;
    tas_project_set_branch_changed(project, false);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_BOOKMARKS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_bookmark_clear(NesTasProject *project, unsigned slot) {
    if (!project || slot >= NES_TAS_BOOKMARK_COUNT) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (!project->state.bookmarks[slot].occupied) {
        return NES_TAS_OK;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    int parent = project->state.bookmarks[slot].parent_slot;
    for (unsigned child = 0; child < NES_TAS_BOOKMARK_COUNT; ++child) {
        if (project->state.bookmarks[child].occupied && project->state.bookmarks[child].parent_slot == (int)slot) {
            project->state.bookmarks[child].parent_slot = parent;
        }
    }
    tas_bookmark_free(&project->state.bookmarks[slot]);
    if (project->state.current_branch == (int)slot) {
        project->state.current_branch = parent;
        tas_project_set_branch_changed(project, true);
    }
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_BOOKMARKS));
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

static size_t timeline_first_difference(const TasTimeline *left, const TasTimeline *right) {
    size_t shared =
        left->movie.frame_count < right->movie.frame_count ? left->movie.frame_count : right->movie.frame_count;
    for (size_t i = 0; i < shared; ++i) {
        if (!tas_frame_equal(&left->movie.frames[i], &right->movie.frames[i])) {
            return i;
        }
    }
    return left->movie.frame_count == right->movie.frame_count ? SIZE_MAX : shared;
}

NesTasResult nes_tas_bookmark_deploy(NesTasProject *project, unsigned slot) {
    if (!project || slot >= NES_TAS_BOOKMARK_COUNT) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    const TasBookmark *bookmark = &project->state.bookmarks[slot];
    if (!bookmark->occupied) {
        return NES_TAS_EMPTY_BOOKMARK;
    }
    TasTimeline replacement;
    tas_timeline_init(&replacement);
    NesTasResult result = tas_timeline_clone(&bookmark->timeline, &replacement);
    if (result != NES_TAS_OK) {
        return result;
    }
    if (replacement.movie.rerecord_count < project->state.timeline.movie.rerecord_count) {
        replacement.movie.rerecord_count = project->state.timeline.movie.rerecord_count;
    }
    size_t frames = replacement.movie.frame_count;
    uint8_t *selection = frames ? (uint8_t *)calloc(frames, 1) : NULL;
    if (frames && !selection) {
        tas_timeline_free(&replacement);
        return NES_TAS_OUT_OF_MEMORY;
    }
    size_t first_changed = timeline_first_difference(&project->state.timeline, &replacement);
    bool owned = false;
    result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        free(selection);
        tas_timeline_free(&replacement);
        return result;
    }
    tas_timeline_free(&project->state.timeline);
    project->state.timeline = replacement;
    tas_timeline_init(&replacement);
    tas_navigation_clamp(&project->state);
    free(project->state.selection);
    project->state.selection = selection;
    project->state.current_branch = (int)slot;
    tas_project_mark_change(project, first_changed, TAS_FM3_ALL_MODULES);
    tas_project_set_branch_changed(project, false);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

int nes_tas_current_branch(const NesTasProject *project) {
    return project ? project->state.current_branch : -1;
}
