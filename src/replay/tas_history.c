/*
 * tas_history.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Retained TAS history views. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_project_internal.h"

#include <stdio.h>
#include <string.h>

static const char *note(const char *value) {
    return value ? value : "";
}

static void affected(NesTasHistoryView *view, size_t frame) {
    if (frame < view->first_frame) {
        view->first_frame = frame;
    }
    if (view->last_frame == SIZE_MAX || frame > view->last_frame) {
        view->last_frame = frame;
    }
}

static void describe(NesTasHistoryView *view, const char *text) {
    size_t used = strlen(view->description);
    snprintf(view->description + used, sizeof(view->description) - used, "%s%s", used ? ", " : "", text);
}

void tas_history_describe(const TasProjectState *before, const TasProjectState *after, size_t first_changed,
                          NesTasHistoryView *out) {
    memset(out, 0, sizeof(*out));
    out->first_frame = out->last_frame = SIZE_MAX;
    const TasTimeline *a = &before->timeline;
    const TasTimeline *b = &after->timeline;
    size_t common = a->movie.frame_count < b->movie.frame_count ? a->movie.frame_count : b->movie.frame_count;
    bool input = false, lag = false, selection = false;
    size_t selection_first = SIZE_MAX, selection_last = SIZE_MAX;
    size_t lag_first = SIZE_MAX, lag_last = SIZE_MAX;
    for (size_t i = 0; i < common; ++i) {
        if (!tas_frame_equal(&a->movie.frames[i], &b->movie.frames[i])) {
            input = true;
            affected(out, i);
        }
        if (a->lag[i] != b->lag[i]) {
            lag = true;
            if (lag_first == SIZE_MAX) {
                lag_first = i;
            }
            lag_last = i;
        }
        if (before->selection[i] != after->selection[i]) {
            selection = true;
            if (selection_first == SIZE_MAX) {
                selection_first = i;
            }
            selection_last = i;
        }
    }
    bool length = a->movie.frame_count != b->movie.frame_count;
    if (length) {
        affected(out, first_changed < common ? first_changed : common);
        affected(out, (a->movie.frame_count > b->movie.frame_count ? a->movie.frame_count : b->movie.frame_count) - 1);
    }
    unsigned changed_branches = 0;
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        if (!tas_bookmark_equal(&before->bookmarks[i], &after->bookmarks[i])) {
            changed_branches |= 1u << i;
        }
    }
    char label[64];
    bool deployed = after->current_branch >= 0 && !changed_branches &&
                    (before->current_branch != after->current_branch ||
                     (before->current_branch_changed && !after->current_branch_changed));
    if (deployed) {
        snprintf(label, sizeof(label), "Deploy branch %u", ((unsigned)after->current_branch + 1) % 10);
        describe(out, label);
    } else if (b->movie.rerecord_count > a->movie.rerecord_count && (input || length)) {
        describe(out, "Record take");
    } else if (length) {
        bool insert = b->movie.frame_count > a->movie.frame_count;
        size_t count =
            insert ? b->movie.frame_count - a->movie.frame_count : a->movie.frame_count - b->movie.frame_count;
        snprintf(label, sizeof(label), "%s %zu frame%s", insert ? "Insert" : "Delete", count, count == 1 ? "" : "s");
        describe(out, label);
    } else if (input) {
        describe(out, "Edit input");
    }
    bool markers = false;
    size_t ai = 0, bi = 0;
    while (ai < a->marker_count || bi < b->marker_count) {
        const TasMarker *am = ai < a->marker_count ? &a->markers[ai] : NULL;
        const TasMarker *bm = bi < b->marker_count ? &b->markers[bi] : NULL;
        if (am && bm && am->frame == bm->frame) {
            if (strcmp(note(am->note), note(bm->note))) {
                markers = true;
                affected(out, am->frame);
            }
            ++ai;
            ++bi;
        } else if (am && (!bm || am->frame < bm->frame)) {
            markers = true;
            affected(out, am->frame);
            ++ai;
        } else {
            markers = true;
            affected(out, bm->frame);
            ++bi;
        }
    }
    if (markers) {
        describe(out, "Edit markers");
    }
    if (!tas_navigation_equal(before, after)) {
        describe(out, "Edit navigation bookmarks");
        size_t count =
            before->navigation_count > after->navigation_count ? before->navigation_count : after->navigation_count;
        for (size_t i = 0; i < count; ++i) {
            const TasNavigationBookmark *an = i < before->navigation_count ? &before->navigation[i] : NULL;
            const TasNavigationBookmark *bn = i < after->navigation_count ? &after->navigation[i] : NULL;
            if (an && bn && an->frame == bn->frame && !strcmp(an->name, bn->name) && !strcmp(an->note, bn->note)) {
                continue;
            }
            if (an) {
                affected(out, an->frame);
            }
            if (bn) {
                affected(out, bn->frame);
            }
        }
    }
    if (strcmp(note(a->intro_note), note(b->intro_note))) {
        describe(out, "Intro note");
    }
    for (unsigned i = 0; i < NES_TAS_BOOKMARK_COUNT; ++i) {
        const TasBookmark *ab = &before->bookmarks[i];
        const TasBookmark *bb = &after->bookmarks[i];
        if (changed_branches & (1u << i)) {
            snprintf(label, sizeof(label), "%s branch %u", bb->occupied ? "Store" : "Clear", (i + 1) % 10);
            describe(out, label);
            affected(out, bb->occupied ? bb->key_frame : ab->key_frame);
        }
    }
    if (lag && !input && !length && first_changed != SIZE_MAX) {
        describe(out, "Lag annotation");
        affected(out, lag_first);
        affected(out, lag_last);
    }
    if (!out->description[0]) {
        if (a->movie.rerecord_count != b->movie.rerecord_count) {
            describe(out, "Rerecord count");
        } else if (selection) {
            describe(out, "Selection");
            affected(out, selection_first);
            affected(out, selection_last);
        } else {
            describe(out, "Project metadata");
        }
    }
}

void tas_history_describe_all(NesTasProject *project) {
    if (!project) {
        return;
    }
    const TasProjectState *after = &project->state;
    for (TasHistoryEntry *entry = project->undo; entry; entry = entry->next) {
        tas_history_describe(&entry->state, after, entry->first_changed, &entry->summary);
        after = &entry->state;
    }
    const TasProjectState *before = &project->state;
    for (TasHistoryEntry *entry = project->redo; entry; entry = entry->next) {
        tas_history_describe(before, &entry->state, entry->first_changed, &entry->summary);
        before = &entry->state;
    }
}

void nes_tas_project_history_info(const NesTasProject *project, NesTasHistoryInfo *out) {
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (project) {
        out->position = project->undo_count;
        out->operations = project->undo_count + project->redo_count;
        out->bytes = project->history_bytes;
        out->entry_limit = project->max_history_entries;
        out->byte_limit = project->max_history_bytes;
        out->edit_active = project->edit_active;
    }
}

bool nes_tas_project_history_entry(const NesTasProject *project, size_t position, NesTasHistoryView *out) {
    if (!project || !out || position > project->undo_count + project->redo_count) {
        return false;
    }
    NesTasHistoryView view = {0};
    view.first_frame = view.last_frame = SIZE_MAX;
    if (!position) {
        snprintf(view.description, sizeof(view.description), "Earliest retained state");
    } else {
        bool undone = position > project->undo_count;
        const TasHistoryEntry *entry = undone ? project->redo : project->undo;
        size_t distance = undone ? position - project->undo_count - 1 : project->undo_count - position;
        for (size_t i = 0; entry && i < distance; ++i) {
            entry = entry->next;
        }
        if (!entry) {
            return false;
        }
        view = entry->summary;
    }
    view.position = position;
    view.applied = position <= project->undo_count;
    view.current = position == project->undo_count;
    *out = view;
    return true;
}
