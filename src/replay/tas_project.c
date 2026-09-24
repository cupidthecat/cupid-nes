/*
 * tas_project.c - TAS project ownership, history, and transactions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "tas_project_internal.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

static NesTasResult tas_from_fm2(NesFm2Result result) {
    switch (result) {
    case NES_FM2_OK:
        return NES_TAS_OK;
    case NES_FM2_INVALID_ARGUMENT:
        return NES_TAS_INVALID_ARGUMENT;
    case NES_FM2_OUT_OF_MEMORY:
        return NES_TAS_OUT_OF_MEMORY;
    case NES_FM2_LIMIT:
        return NES_TAS_RANGE_ERROR;
    case NES_FM2_TRUNCATED:
    case NES_FM2_CORRUPT:
        return NES_TAS_FORMAT_ERROR;
    case NES_FM2_UNSUPPORTED:
        return NES_TAS_UNSUPPORTED;
    default:
        return NES_TAS_FORMAT_ERROR;
    }
}

char *tas_strdup_limit(const char *text, size_t max_length) {
    if (!text) {
        text = "";
    }
    size_t length = strlen(text);
    if (length > max_length) {
        length = max_length;
    }
    char *copy = (char *)malloc(length + 1);
    if (!copy) {
        return NULL;
    }
    memcpy(copy, text, length);
    copy[length] = '\0';
    return copy;
}

bool tas_frame_equal(const NesFm2Frame *left, const NesFm2Frame *right) {
    if (!left || !right || left->commands != right->commands ||
        memcmp(left->pads, right->pads, sizeof(left->pads)) != 0) {
        return false;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const NesFm2Zapper *a = &left->zappers[i];
        const NesFm2Zapper *b = &right->zappers[i];
        if (a->x != b->x || a->y != b->y || a->button != b->button || a->bogo != b->bogo || a->zaphit != b->zaphit) {
            return false;
        }
    }
    return true;
}

static bool tas_timeline_equal(const TasTimeline *left, const TasTimeline *right) {
    if (left->movie.frame_count != right->movie.frame_count ||
        left->movie.rerecord_count != right->movie.rerecord_count || left->marker_count != right->marker_count ||
        strcmp(left->intro_note ? left->intro_note : "", right->intro_note ? right->intro_note : "") != 0) {
        return false;
    }
    for (size_t i = 0; i < left->movie.frame_count; ++i) {
        if (!tas_frame_equal(&left->movie.frames[i], &right->movie.frames[i])) {
            return false;
        }
        if (left->lag[i] != right->lag[i]) {
            return false;
        }
    }
    for (size_t i = 0; i < left->marker_count; ++i) {
        if (left->markers[i].frame != right->markers[i].frame ||
            strcmp(left->markers[i].note ? left->markers[i].note : "",
                   right->markers[i].note ? right->markers[i].note : "") != 0) {
            return false;
        }
    }
    return true;
}

static bool tas_bookmark_equal(const TasBookmark *left, const TasBookmark *right) {
    if (left->occupied != right->occupied) {
        return false;
    }
    if (!left->occupied) {
        return true;
    }
    return left->key_frame == right->key_frame && left->parent_slot == right->parent_slot &&
           strcmp(left->name ? left->name : "", right->name ? right->name : "") == 0 &&
           tas_timeline_equal(&left->timeline, &right->timeline);
}

static bool tas_state_equal(const TasProjectState *left, const TasProjectState *right) {
    if (!tas_timeline_equal(&left->timeline, &right->timeline) || left->current_branch != right->current_branch ||
        left->current_branch_changed != right->current_branch_changed) {
        return false;
    }
    if (left->timeline.movie.frame_count &&
        memcmp(left->selection, right->selection, left->timeline.movie.frame_count) != 0) {
        return false;
    }
    for (size_t i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        if (!tas_bookmark_equal(&left->bookmarks[i], &right->bookmarks[i])) {
            return false;
        }
    }
    return true;
}

void tas_timeline_init(TasTimeline *timeline) {
    if (!timeline) {
        return;
    }
    memset(timeline, 0, sizeof(*timeline));
    nes_fm2_movie_init(&timeline->movie);
}

void tas_timeline_free(TasTimeline *timeline) {
    if (!timeline) {
        return;
    }
    nes_fm2_movie_free(&timeline->movie);
    free(timeline->lag);
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        free(timeline->markers[i].note);
    }
    free(timeline->markers);
    free(timeline->intro_note);
    tas_timeline_init(timeline);
}

NesTasResult tas_timeline_from_movie(TasTimeline *timeline, const NesFm2Movie *movie) {
    if (!timeline || !movie) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasTimeline replacement;
    tas_timeline_init(&replacement);
    NesFm2Result cloned = nes_fm2_movie_clone(movie, &replacement.movie, NULL);
    if (cloned != NES_FM2_OK) {
        tas_timeline_free(&replacement);
        return tas_from_fm2(cloned);
    }
    nes_fm2_discard_project(&replacement.movie);
    if (replacement.movie.frame_count) {
        replacement.lag = (uint8_t *)calloc(replacement.movie.frame_count, 1);
        if (!replacement.lag) {
            tas_timeline_free(&replacement);
            return NES_TAS_OUT_OF_MEMORY;
        }
    }
    replacement.intro_note = tas_strdup_limit("Power on", NES_TAS_MARKER_NOTE_MAX);
    if (!replacement.intro_note) {
        tas_timeline_free(&replacement);
        return NES_TAS_OUT_OF_MEMORY;
    }
    tas_timeline_free(timeline);
    *timeline = replacement;
    return NES_TAS_OK;
}

NesTasResult tas_timeline_clone(const TasTimeline *source, TasTimeline *out) {
    if (!source || !out) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasTimeline replacement;
    tas_timeline_init(&replacement);
    NesFm2Result cloned = nes_fm2_movie_clone(&source->movie, &replacement.movie, NULL);
    if (cloned != NES_FM2_OK) {
        tas_timeline_free(&replacement);
        return tas_from_fm2(cloned);
    }
    size_t frames = source->movie.frame_count;
    if (frames) {
        replacement.lag = (uint8_t *)malloc(frames);
        if (!replacement.lag) {
            goto oom;
        }
        memcpy(replacement.lag, source->lag, frames);
    }
    if (source->marker_count) {
        replacement.markers = (TasMarker *)calloc(source->marker_count, sizeof(*replacement.markers));
        if (!replacement.markers) {
            goto oom;
        }
        replacement.marker_capacity = source->marker_count;
        for (size_t i = 0; i < source->marker_count; ++i) {
            replacement.markers[i].frame = source->markers[i].frame;
            replacement.markers[i].note = tas_strdup_limit(source->markers[i].note, NES_TAS_MARKER_NOTE_MAX);
            if (!replacement.markers[i].note) {
                goto oom;
            }
            replacement.marker_count++;
        }
    }
    replacement.intro_note = tas_strdup_limit(source->intro_note, NES_TAS_MARKER_NOTE_MAX);
    if (!replacement.intro_note) {
        goto oom;
    }
    tas_timeline_free(out);
    *out = replacement;
    return NES_TAS_OK;

oom:
    tas_timeline_free(&replacement);
    return NES_TAS_OUT_OF_MEMORY;
}

void tas_bookmark_init(TasBookmark *bookmark) {
    if (!bookmark) {
        return;
    }
    memset(bookmark, 0, sizeof(*bookmark));
    bookmark->parent_slot = -1;
    tas_timeline_init(&bookmark->timeline);
}

void tas_bookmark_free(TasBookmark *bookmark) {
    if (!bookmark) {
        return;
    }
    free(bookmark->name);
    tas_timeline_free(&bookmark->timeline);
    tas_bookmark_init(bookmark);
}

NesTasResult tas_bookmark_clone(const TasBookmark *source, TasBookmark *out) {
    if (!source || !out) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasBookmark replacement;
    tas_bookmark_init(&replacement);
    replacement.occupied = source->occupied;
    replacement.key_frame = source->key_frame;
    replacement.parent_slot = source->parent_slot;
    if (source->name) {
        replacement.name = tas_strdup_limit(source->name, NES_TAS_BOOKMARK_NAME_MAX);
        if (!replacement.name) {
            goto oom;
        }
    }
    if (source->occupied) {
        NesTasResult result = tas_timeline_clone(&source->timeline, &replacement.timeline);
        if (result != NES_TAS_OK) {
            tas_bookmark_free(&replacement);
            return result;
        }
    }
    tas_bookmark_free(out);
    *out = replacement;
    return NES_TAS_OK;

oom:
    tas_bookmark_free(&replacement);
    return NES_TAS_OUT_OF_MEMORY;
}

void tas_state_init(TasProjectState *state) {
    if (!state) {
        return;
    }
    memset(state, 0, sizeof(*state));
    tas_timeline_init(&state->timeline);
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        tas_bookmark_init(&state->bookmarks[i]);
    }
    state->current_branch = -1;
}

void tas_state_free(TasProjectState *state) {
    if (!state) {
        return;
    }
    tas_timeline_free(&state->timeline);
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        tas_bookmark_free(&state->bookmarks[i]);
    }
    free(state->selection);
    tas_state_init(state);
}

NesTasResult tas_state_clone(const TasProjectState *source, TasProjectState *out) {
    if (!source || !out) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasProjectState replacement;
    tas_state_init(&replacement);
    NesTasResult result = tas_timeline_clone(&source->timeline, &replacement.timeline);
    if (result != NES_TAS_OK) {
        goto fail;
    }
    size_t frames = source->timeline.movie.frame_count;
    if (frames) {
        replacement.selection = (uint8_t *)malloc(frames);
        if (!replacement.selection) {
            result = NES_TAS_OUT_OF_MEMORY;
            goto fail;
        }
        memcpy(replacement.selection, source->selection, frames);
    }
    replacement.current_branch = source->current_branch;
    replacement.current_branch_changed = source->current_branch_changed;
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        result = tas_bookmark_clone(&source->bookmarks[i], &replacement.bookmarks[i]);
        if (result != NES_TAS_OK) {
            goto fail;
        }
    }
    tas_state_free(out);
    *out = replacement;
    return NES_TAS_OK;

fail:
    tas_state_free(&replacement);
    return result;
}

static size_t tas_timeline_estimated_bytes(const TasTimeline *timeline) {
    if (!timeline) {
        return 0;
    }
    size_t total = sizeof(*timeline);
    size_t frames = timeline->movie.frame_count;
    if (frames <= (SIZE_MAX - total) / (sizeof(NesFm2Frame) + 1)) {
        total += frames * (sizeof(NesFm2Frame) + 1);
    }
    for (size_t i = 0; i < timeline->marker_count; ++i) {
        size_t note = timeline->markers[i].note ? strlen(timeline->markers[i].note) + 1 : 1;
        if (total <= SIZE_MAX - sizeof(TasMarker) - note) {
            total += sizeof(TasMarker) + note;
        }
    }
    if (timeline->intro_note && total <= SIZE_MAX - strlen(timeline->intro_note) - 1) {
        total += strlen(timeline->intro_note) + 1;
    }
    return total;
}

size_t tas_state_estimated_bytes(const TasProjectState *state) {
    if (!state) {
        return 0;
    }
    size_t total = sizeof(*state) + state->timeline.movie.frame_count;
    size_t timeline = tas_timeline_estimated_bytes(&state->timeline);
    if (total <= SIZE_MAX - timeline) {
        total += timeline;
    }
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        const TasBookmark *bookmark = &state->bookmarks[i];
        if (!bookmark->occupied) {
            continue;
        }
        size_t bytes = tas_timeline_estimated_bytes(&bookmark->timeline);
        if (bookmark->name && bytes <= SIZE_MAX - strlen(bookmark->name) - 1) {
            bytes += strlen(bookmark->name) + 1;
        }
        if (total <= SIZE_MAX - bytes) {
            total += bytes;
        }
    }
    return total;
}

void tas_clipboard_free(TasClipboard *clipboard) {
    if (!clipboard) {
        return;
    }
    free(clipboard->frames);
    free(clipboard->present);
    memset(clipboard, 0, sizeof(*clipboard));
}

NesTasResult tas_clipboard_clone(const TasClipboard *source, TasClipboard *out) {
    if (!source || !out) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasClipboard replacement = {0};
    replacement.span = source->span;
    replacement.count = source->count;
    if (source->span) {
        if (source->span > SIZE_MAX / sizeof(*replacement.frames)) {
            return NES_TAS_RANGE_ERROR;
        }
        replacement.frames = (NesFm2Frame *)malloc(source->span * sizeof(*replacement.frames));
        replacement.present = (uint8_t *)malloc(source->span);
        if (!replacement.frames || !replacement.present) {
            tas_clipboard_free(&replacement);
            return NES_TAS_OUT_OF_MEMORY;
        }
        memcpy(replacement.frames, source->frames, source->span * sizeof(*replacement.frames));
        memcpy(replacement.present, source->present, source->span);
    }
    tas_clipboard_free(out);
    *out = replacement;
    return NES_TAS_OK;
}

static void tas_source_modules_free(NesTasProject *project) {
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        free(project->source_modules[i].data);
        project->source_modules[i].data = NULL;
        project->source_modules[i].size = 0;
    }
}

static NesTasResult tas_source_modules_copy(NesTasProject *project, const NesFm2Movie *movie) {
    project->source_project_present = movie->project_present;
    project->source_project_version = movie->project_version;
    project->source_saved_modules = movie->project_saved_modules;
    if (!movie->project_present) {
        return NES_TAS_OK;
    }
    for (unsigned i = 0; i < NES_FM3_PROJECT_MODULE_COUNT; ++i) {
        size_t size = movie->project_modules[i].size;
        if (!size) {
            continue;
        }
        project->source_modules[i].data = (uint8_t *)malloc(size);
        if (!project->source_modules[i].data) {
            tas_source_modules_free(project);
            return NES_TAS_OUT_OF_MEMORY;
        }
        memcpy(project->source_modules[i].data, movie->project_modules[i].data, size);
        project->source_modules[i].size = size;
    }
    return NES_TAS_OK;
}

static void tas_history_entry_free(TasHistoryEntry *entry) {
    if (!entry) {
        return;
    }
    tas_state_free(&entry->state);
    free(entry);
}

static void tas_history_list_clear(TasHistoryEntry **head, size_t *count, size_t *bytes) {
    TasHistoryEntry *entry = head ? *head : NULL;
    while (entry) {
        TasHistoryEntry *next = entry->next;
        if (bytes && *bytes >= entry->bytes) {
            *bytes -= entry->bytes;
        }
        tas_history_entry_free(entry);
        entry = next;
    }
    if (head) {
        *head = NULL;
    }
    if (count) {
        *count = 0;
    }
}

static void tas_history_remove_tail(NesTasProject *project, TasHistoryEntry **head, size_t *count) {
    if (!project || !head || !*head || !count || !*count) {
        return;
    }
    TasHistoryEntry *previous = NULL;
    TasHistoryEntry *entry = *head;
    while (entry->next) {
        previous = entry;
        entry = entry->next;
    }
    if (previous) {
        previous->next = NULL;
    } else {
        *head = NULL;
    }
    if (project->history_bytes >= entry->bytes) {
        project->history_bytes -= entry->bytes;
    } else {
        project->history_bytes = 0;
    }
    (*count)--;
    tas_history_entry_free(entry);
}

static void tas_history_trim(NesTasProject *project) {
    if (!project) {
        return;
    }
    while (project->undo_count > project->max_history_entries) {
        tas_history_remove_tail(project, &project->undo, &project->undo_count);
    }
    while (project->redo_count > project->max_history_entries) {
        tas_history_remove_tail(project, &project->redo, &project->redo_count);
    }
    while (project->history_bytes > project->max_history_bytes) {
        if (project->undo_count) {
            tas_history_remove_tail(project, &project->undo, &project->undo_count);
        } else if (project->redo_count) {
            tas_history_remove_tail(project, &project->redo, &project->redo_count);
        } else {
            break;
        }
    }
}

static void tas_history_clear_redo(NesTasProject *project) {
    tas_history_list_clear(&project->redo, &project->redo_count, &project->history_bytes);
}

static void tas_history_push_undo(NesTasProject *project, TasHistoryEntry *entry) {
    if (!project || !entry) {
        return;
    }
    tas_history_clear_redo(project);
    if (!project->max_history_entries || !project->max_history_bytes || entry->bytes > project->max_history_bytes) {
        tas_history_entry_free(entry);
        return;
    }
    entry->next = project->undo;
    project->undo = entry;
    project->undo_count++;
    project->history_bytes += entry->bytes;
    tas_history_trim(project);
}

static void tas_revision_event(NesTasProject *project, size_t first_changed) {
    if (!project) {
        return;
    }
    if (project->revision != UINT64_MAX) {
        project->revision++;
    }
    project->journal[project->journal_head].revision = project->revision;
    project->journal[project->journal_head].first_changed = first_changed;
    project->journal_head = (project->journal_head + 1) % TAS_CHANGE_JOURNAL_CAPACITY;
    if (project->journal_count < TAS_CHANGE_JOURNAL_CAPACITY) {
        project->journal_count++;
    }
}

void tas_project_mark_dirty(NesTasProject *project, uint32_t dirty_modules) {
    if (project) {
        project->fm3_dirty_modules |= dirty_modules;
    }
}

void tas_project_set_branch_changed(NesTasProject *project, bool changed) {
    if (!project || project->state.current_branch_changed == changed) {
        return;
    }
    project->state.current_branch_changed = changed;
    tas_project_mark_dirty(project, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_BOOKMARKS));
}

void tas_project_mark_change(NesTasProject *project, size_t first_changed, uint32_t dirty_modules) {
    if (!project) {
        return;
    }
    if (first_changed != SIZE_MAX) {
        tas_project_set_branch_changed(project, true);
    }
    tas_project_mark_dirty(project, dirty_modules);
    if (first_changed != SIZE_MAX) {
        nes_fm2_invalidate_project_timeline(&project->state.timeline.movie);
    }
    if (project->edit_active) {
        project->edit_changed = true;
        if (first_changed < project->edit_first_changed) {
            project->edit_first_changed = first_changed;
        }
    }
    tas_revision_event(project, first_changed);
}

NesTasResult tas_project_auto_begin(NesTasProject *project, bool *owned) {
    if (!project || !owned) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (project->edit_active) {
        *owned = false;
        return NES_TAS_OK;
    }
    NesTasResult result = nes_tas_edit_begin(project);
    *owned = result == NES_TAS_OK;
    return result;
}

NesTasResult tas_project_auto_finish(NesTasProject *project, bool owned, NesTasResult result) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (!owned) {
        return result;
    }
    if (result != NES_TAS_OK) {
        nes_tas_edit_cancel(project);
        return result;
    }
    return nes_tas_edit_end(project);
}

NesTasResult nes_tas_project_create_checked(const NesFm2Movie *movie, NesTasProject **out) {
    if (!movie || !out) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    NesTasProject *project = (NesTasProject *)calloc(1, sizeof(*project));
    if (!project) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    tas_state_init(&project->state);
    project->state.current_branch = -1;
    project->max_history_entries = TAS_DEFAULT_HISTORY_ENTRIES;
    project->max_history_bytes = NES_TAS_DEFAULT_HISTORY_BYTES;

    NesTasResult created = tas_source_modules_copy(project, movie);
    if (created == NES_TAS_OK) {
        created = tas_timeline_from_movie(&project->state.timeline, movie);
    }
    if (created != NES_TAS_OK) {
        nes_tas_project_destroy(project);
        return created;
    }
    if (movie->frame_count) {
        project->state.selection = (uint8_t *)calloc(movie->frame_count, 1);
        if (!project->state.selection) {
            nes_tas_project_destroy(project);
            return NES_TAS_OUT_OF_MEMORY;
        }
    }
    NesTasResult imported = tas_import_fm3_metadata(project, movie);
    if (imported != NES_TAS_OK) {
        nes_tas_project_destroy(project);
        return imported;
    }
    *out = project;
    return NES_TAS_OK;
}

NesTasProject *nes_tas_project_create(const NesFm2Movie *movie) {
    NesTasProject *project = NULL;
    (void)nes_tas_project_create_checked(movie, &project);
    return project;
}

void nes_tas_project_destroy(NesTasProject *project) {
    if (!project) {
        return;
    }
    tas_state_free(&project->state);
    tas_clipboard_free(&project->clipboard);
    tas_history_list_clear(&project->undo, &project->undo_count, &project->history_bytes);
    tas_history_list_clear(&project->redo, &project->redo_count, &project->history_bytes);
    tas_history_entry_free(project->edit_history_entry);
    tas_source_modules_free(project);
    free(project);
}

const NesFm2Movie *nes_tas_project_movie(const NesTasProject *project) {
    return project ? &project->state.timeline.movie : NULL;
}

size_t nes_tas_project_frame_count(const NesTasProject *project) {
    return project ? project->state.timeline.movie.frame_count : 0;
}

const NesFm2Frame *nes_tas_project_frame(const NesTasProject *project, size_t frame) {
    if (!project || frame >= project->state.timeline.movie.frame_count) {
        return NULL;
    }
    return &project->state.timeline.movie.frames[frame];
}

uint64_t nes_tas_project_revision(const NesTasProject *project) {
    return project ? project->revision : 0;
}

NesTasChange nes_tas_project_change_since(const NesTasProject *project, uint64_t revision) {
    NesTasChange change = {0, SIZE_MAX};
    if (!project) {
        return change;
    }
    change.revision = project->revision;
    if (revision >= project->revision) {
        return change;
    }
    if (!project->journal_count) {
        change.first_changed_frame = 0;
        return change;
    }
    size_t oldest_index =
        (project->journal_head + TAS_CHANGE_JOURNAL_CAPACITY - project->journal_count) % TAS_CHANGE_JOURNAL_CAPACITY;
    uint64_t oldest_revision = project->journal[oldest_index].revision;
    if (oldest_revision > 0 && revision < oldest_revision - 1) {
        change.first_changed_frame = 0;
        return change;
    }
    for (size_t i = 0; i < project->journal_count; ++i) {
        size_t index = (oldest_index + i) % TAS_CHANGE_JOURNAL_CAPACITY;
        const TasJournalEntry *entry = &project->journal[index];
        if (entry->revision > revision && entry->first_changed < change.first_changed_frame) {
            change.first_changed_frame = entry->first_changed;
        }
    }
    return change;
}

bool nes_tas_edit_active(const NesTasProject *project) {
    return project && project->edit_active;
}

NesTasResult nes_tas_edit_begin(NesTasProject *project) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (project->edit_active) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasHistoryEntry *entry = (TasHistoryEntry *)calloc(1, sizeof(*entry));
    if (!entry) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    tas_state_init(&entry->state);
    NesTasResult result = tas_state_clone(&project->state, &entry->state);
    if (result != NES_TAS_OK) {
        tas_history_entry_free(entry);
        return result;
    }
    entry->bytes = tas_state_estimated_bytes(&entry->state);
    entry->first_changed = SIZE_MAX;
    project->edit_history_entry = entry;
    project->edit_old_dirty_modules = project->fm3_dirty_modules;
    project->edit_active = true;
    project->edit_changed = false;
    project->edit_first_changed = SIZE_MAX;
    return NES_TAS_OK;
}

NesTasResult nes_tas_edit_end(NesTasProject *project) {
    if (!project || !project->edit_active) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasHistoryEntry *entry = project->edit_history_entry;
    project->edit_history_entry = NULL;
    project->edit_active = false;
    if (project->edit_changed && !tas_state_equal(&entry->state, &project->state)) {
        entry->first_changed = project->edit_first_changed;
        tas_history_push_undo(project, entry);
    } else {
        if (project->edit_changed) {
            project->fm3_dirty_modules = project->edit_old_dirty_modules;
        }
        tas_history_entry_free(entry);
    }
    project->edit_changed = false;
    project->edit_first_changed = SIZE_MAX;
    return NES_TAS_OK;
}

void nes_tas_edit_cancel(NesTasProject *project) {
    if (!project || !project->edit_active) {
        return;
    }
    TasHistoryEntry *entry = project->edit_history_entry;
    bool changed = project->edit_changed;
    size_t first_changed = project->edit_first_changed;
    uint32_t old_dirty = project->edit_old_dirty_modules;
    project->edit_history_entry = NULL;
    project->edit_active = false;
    project->edit_changed = false;
    project->edit_first_changed = SIZE_MAX;
    if (changed && entry) {
        TasProjectState modified = project->state;
        project->state = entry->state;
        tas_state_init(&entry->state);
        tas_state_free(&modified);
        project->fm3_dirty_modules = old_dirty;
        tas_revision_event(project, first_changed);
    }
    tas_history_entry_free(entry);
}

void nes_tas_project_set_history_limit(NesTasProject *project, size_t max_entries, size_t max_bytes) {
    if (!project) {
        return;
    }
    project->max_history_entries = max_entries;
    project->max_history_bytes = max_bytes;
    tas_history_trim(project);
}

bool nes_tas_project_can_undo(const NesTasProject *project) {
    return project && !project->edit_active && project->undo != NULL;
}

bool nes_tas_project_can_redo(const NesTasProject *project) {
    return project && !project->edit_active && project->redo != NULL;
}

static NesTasResult tas_history_jump(NesTasProject *project, bool undo) {
    if (!project || project->edit_active) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasHistoryEntry **source = undo ? &project->undo : &project->redo;
    TasHistoryEntry **target = undo ? &project->redo : &project->undo;
    size_t *source_count = undo ? &project->undo_count : &project->redo_count;
    size_t *target_count = undo ? &project->redo_count : &project->undo_count;
    if (!*source) {
        return NES_TAS_RANGE_ERROR;
    }

    TasHistoryEntry *current = (TasHistoryEntry *)calloc(1, sizeof(*current));
    if (!current) {
        return NES_TAS_OUT_OF_MEMORY;
    }
    tas_state_init(&current->state);
    current->state = project->state;
    tas_state_init(&project->state);
    current->bytes = tas_state_estimated_bytes(&current->state);

    TasHistoryEntry *entry = *source;
    *source = entry->next;
    (*source_count)--;
    if (project->history_bytes >= entry->bytes) {
        project->history_bytes -= entry->bytes;
    } else {
        project->history_bytes = 0;
    }

    project->state = entry->state;
    tas_state_init(&entry->state);
    current->first_changed = entry->first_changed;
    current->next = *target;
    *target = current;
    (*target_count)++;
    project->history_bytes += current->bytes;
    size_t first_changed = entry->first_changed;
    tas_history_entry_free(entry);
    project->fm3_dirty_modules |= TAS_FM3_ALL_MODULES;
    if (first_changed != SIZE_MAX) {
        nes_fm2_invalidate_project_timeline(&project->state.timeline.movie);
    }
    tas_revision_event(project, first_changed);
    tas_history_trim(project);
    return NES_TAS_OK;
}

NesTasResult nes_tas_project_undo(NesTasProject *project) {
    return tas_history_jump(project, true);
}

NesTasResult nes_tas_project_redo(NesTasProject *project) {
    return tas_history_jump(project, false);
}

void nes_tas_selection_clear(NesTasProject *project) {
    if (!project || !project->state.selection) {
        return;
    }
    size_t frames = project->state.timeline.movie.frame_count;
    bool changed = false;
    for (size_t i = 0; i < frames; ++i) {
        if (project->state.selection[i]) {
            changed = true;
            break;
        }
    }
    if (!changed) {
        return;
    }
    memset(project->state.selection, 0, frames);
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_SELECTION));
}

NesTasResult nes_tas_selection_set(NesTasProject *project, size_t frame, bool selected) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (frame >= project->state.timeline.movie.frame_count) {
        return NES_TAS_RANGE_ERROR;
    }
    uint8_t value = selected ? 1u : 0u;
    if (project->state.selection[frame] == value) {
        return NES_TAS_OK;
    }
    project->state.selection[frame] = value;
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_SELECTION));
    return NES_TAS_OK;
}

NesTasResult nes_tas_selection_set_range(NesTasProject *project, size_t first, size_t last, bool selected) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    size_t frames = project->state.timeline.movie.frame_count;
    if (first > last || last >= frames) {
        return NES_TAS_RANGE_ERROR;
    }
    uint8_t value = selected ? 1u : 0u;
    bool changed = false;
    for (size_t i = first; i <= last; ++i) {
        if (project->state.selection[i] != value) {
            project->state.selection[i] = value;
            changed = true;
        }
    }
    if (changed) {
        tas_project_mark_change(project, SIZE_MAX, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_SELECTION));
    }
    return NES_TAS_OK;
}

bool nes_tas_selection_contains(const NesTasProject *project, size_t frame) {
    return project && frame < project->state.timeline.movie.frame_count && project->state.selection[frame] != 0;
}

size_t nes_tas_selection_count(const NesTasProject *project) {
    if (!project) {
        return 0;
    }
    size_t count = 0;
    for (size_t i = 0; i < project->state.timeline.movie.frame_count; ++i) {
        if (project->state.selection[i]) {
            count++;
        }
    }
    return count;
}

bool nes_tas_selection_bounds(const NesTasProject *project, size_t *first, size_t *last) {
    if (!project) {
        return false;
    }
    size_t frames = project->state.timeline.movie.frame_count;
    size_t begin = SIZE_MAX, end = 0;
    for (size_t i = 0; i < frames; ++i) {
        if (!project->state.selection[i]) {
            continue;
        }
        if (begin == SIZE_MAX) {
            begin = i;
        }
        end = i;
    }
    if (begin == SIZE_MAX) {
        return false;
    }
    if (first) {
        *first = begin;
    }
    if (last) {
        *last = end;
    }
    return true;
}

NesTasLagState nes_tas_lag(const NesTasProject *project, size_t frame) {
    if (!project || frame >= project->state.timeline.movie.frame_count) {
        return NES_TAS_LAG_UNKNOWN;
    }
    uint8_t value = project->state.timeline.lag[frame];
    return value <= NES_TAS_LAG_YES ? (NesTasLagState)value : NES_TAS_LAG_UNKNOWN;
}

NesTasResult nes_tas_set_lag(NesTasProject *project, size_t frame, NesTasLagState state) {
    if (!project || state < NES_TAS_LAG_UNKNOWN || state > NES_TAS_LAG_YES) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (frame >= project->state.timeline.movie.frame_count) {
        return NES_TAS_RANGE_ERROR;
    }
    if (project->state.timeline.lag[frame] == (uint8_t)state) {
        return NES_TAS_OK;
    }
    project->state.timeline.lag[frame] = (uint8_t)state;
    tas_project_mark_dirty(project, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_GREENZONE));
    if (project->edit_active) {
        project->edit_changed = true;
    }
    return NES_TAS_OK;
}

void nes_tas_invalidate_lag(NesTasProject *project, size_t first) {
    if (!project) {
        return;
    }
    size_t frames = project->state.timeline.movie.frame_count;
    if (first >= frames) {
        return;
    }
    bool changed = false;
    for (size_t i = first; i < frames; ++i) {
        if (project->state.timeline.lag[i] != NES_TAS_LAG_UNKNOWN) {
            project->state.timeline.lag[i] = NES_TAS_LAG_UNKNOWN;
            changed = true;
        }
    }
    if (changed) {
        tas_project_mark_dirty(project, TAS_FM3_MODULE_BIT(NES_FM3_MODULE_GREENZONE));
        if (project->edit_active) {
            project->edit_changed = true;
        }
    }
}

uint64_t nes_tas_rerecord_count(const NesTasProject *project) {
    return project ? project->state.timeline.movie.rerecord_count : 0;
}

NesTasResult nes_tas_increment_rerecord(NesTasProject *project) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (project->state.timeline.movie.rerecord_count == UINT32_MAX) {
        return NES_TAS_RANGE_ERROR;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    project->state.timeline.movie.rerecord_count++;
    tas_project_mark_change(project, SIZE_MAX, TAS_FM3_ALL_MODULES);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

const char *nes_tas_result_string(NesTasResult result) {
    switch (result) {
    case NES_TAS_OK:
        return "ok";
    case NES_TAS_INVALID_ARGUMENT:
        return "invalid argument";
    case NES_TAS_OUT_OF_MEMORY:
        return "out of memory";
    case NES_TAS_RANGE_ERROR:
        return "value or frame is out of range";
    case NES_TAS_EMPTY_SELECTION:
        return "selection is empty";
    case NES_TAS_EMPTY_CLIPBOARD:
        return "clipboard is empty";
    case NES_TAS_EMPTY_BOOKMARK:
        return "bookmark is empty";
    case NES_TAS_FORMAT_ERROR:
        return "invalid TAS project format";
    case NES_TAS_IO_ERROR:
        return "TAS project I/O failed";
    case NES_TAS_UNSUPPORTED:
        return "unsupported TAS project data";
    default:
        return "unknown TAS project result";
    }
}
