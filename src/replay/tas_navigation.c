/*
 * tas_navigation.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Lightweight TAS navigation bookmarks. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_project_internal.h"

#include <stdlib.h>
#include <string.h>

static bool navigation_text(TasNavigationBookmark *entry, const char *name, const char *note) {
    if (!name || !*name || strlen(name) > NES_TAS_BOOKMARK_NAME_MAX ||
        (note && strlen(note) > NES_TAS_NAVIGATION_NOTE_MAX)) {
        return false;
    }
    memset(entry, 0, sizeof(*entry));
    memcpy(entry->name, name, strlen(name) + 1);
    if (note) {
        memcpy(entry->note, note, strlen(note) + 1);
    }
    return true;
}

bool tas_navigation_equal(const TasProjectState *left, const TasProjectState *right) {
    if (left->navigation_count != right->navigation_count) {
        return false;
    }
    for (size_t i = 0; i < left->navigation_count; ++i) {
        const TasNavigationBookmark *a = &left->navigation[i], *b = &right->navigation[i];
        if (a->frame != b->frame || strcmp(a->name, b->name) || strcmp(a->note, b->note)) {
            return false;
        }
    }
    return true;
}

size_t nes_tas_navigation_count(const NesTasProject *project) {
    return project ? project->state.navigation_count : 0;
}

bool nes_tas_navigation(const NesTasProject *project, size_t index, NesTasNavigationView *out) {
    if (!project || !out || index >= project->state.navigation_count) {
        return false;
    }
    const TasNavigationBookmark *entry = &project->state.navigation[index];
    *out = (NesTasNavigationView){entry->frame, entry->name, entry->note};
    return true;
}

NesTasResult nes_tas_navigation_add(NesTasProject *project, size_t frame, const char *name, const char *note,
                                    size_t *index) {
    TasNavigationBookmark entry;
    if (!project || !navigation_text(&entry, name, note)) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    TasProjectState *state = &project->state;
    if (frame > state->timeline.movie.frame_count || state->navigation_count >= NES_TAS_NAVIGATION_LIMIT) {
        return NES_TAS_RANGE_ERROR;
    }
    entry.frame = frame;
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    if (state->navigation_count == state->navigation_capacity) {
        size_t capacity = state->navigation_capacity ? state->navigation_capacity * 2 : 8;
        if (capacity > NES_TAS_NAVIGATION_LIMIT) {
            capacity = NES_TAS_NAVIGATION_LIMIT;
        }
        TasNavigationBookmark *replacement = tas_edit_malloc(capacity * sizeof(*replacement));
        if (!replacement) {
            return tas_project_auto_finish(project, owned, NES_TAS_OUT_OF_MEMORY);
        }
        if (state->navigation_count) {
            memcpy(replacement, state->navigation, state->navigation_count * sizeof(*replacement));
        }
        free(state->navigation);
        state->navigation = replacement;
        state->navigation_capacity = capacity;
    }
    size_t at = 0;
    while (at < state->navigation_count && state->navigation[at].frame <= frame) {
        ++at;
    }
    memmove(state->navigation + at + 1, state->navigation + at,
            (state->navigation_count - at) * sizeof(*state->navigation));
    state->navigation[at] = entry;
    ++state->navigation_count;
    tas_project_mark_change(project, SIZE_MAX, 0);
    result = tas_project_auto_finish(project, owned, NES_TAS_OK);
    if (result == NES_TAS_OK && index) {
        *index = at;
    }
    return result;
}

NesTasResult nes_tas_navigation_rename(NesTasProject *project, size_t index, const char *name, const char *note) {
    TasNavigationBookmark entry;
    if (!project || !navigation_text(&entry, name, note)) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (index >= project->state.navigation_count) {
        return NES_TAS_RANGE_ERROR;
    }
    entry.frame = project->state.navigation[index].frame;
    if (!strcmp(entry.name, project->state.navigation[index].name) &&
        !strcmp(entry.note, project->state.navigation[index].note)) {
        return NES_TAS_OK;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    project->state.navigation[index] = entry;
    tas_project_mark_change(project, SIZE_MAX, 0);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

NesTasResult nes_tas_navigation_remove(NesTasProject *project, size_t index) {
    if (!project) {
        return NES_TAS_INVALID_ARGUMENT;
    }
    if (index >= project->state.navigation_count) {
        return NES_TAS_RANGE_ERROR;
    }
    bool owned = false;
    NesTasResult result = tas_project_auto_begin(project, &owned);
    if (result != NES_TAS_OK) {
        return result;
    }
    TasProjectState *state = &project->state;
    memmove(state->navigation + index, state->navigation + index + 1,
            (state->navigation_count - index - 1) * sizeof(*state->navigation));
    if (!--state->navigation_count) {
        free(state->navigation);
        state->navigation = NULL;
        state->navigation_capacity = 0;
    }
    tas_project_mark_change(project, SIZE_MAX, 0);
    return tas_project_auto_finish(project, owned, NES_TAS_OK);
}

bool nes_tas_navigation_neighbor(const NesTasProject *project, size_t frame, bool next, size_t *index) {
    if (!project || !index) {
        return false;
    }
    size_t found = SIZE_MAX;
    for (size_t i = 0; i < project->state.navigation_count; ++i) {
        size_t candidate = project->state.navigation[i].frame;
        if (next && candidate > frame) {
            found = i;
            break;
        }
        if (!next && candidate < frame) {
            found = i;
        }
    }
    if (found == SIZE_MAX) {
        return false;
    }
    *index = found;
    return true;
}

void tas_navigation_insert(TasProjectState *state, size_t first, size_t count) {
    for (size_t i = 0; i < state->navigation_count; ++i) {
        if (state->navigation[i].frame >= first) {
            state->navigation[i].frame += count;
        }
    }
}

void tas_navigation_delete(TasProjectState *state, size_t first, size_t count) {
    for (size_t i = 0; i < state->navigation_count; ++i) {
        size_t *frame = &state->navigation[i].frame;
        if (*frame >= first + count) {
            *frame -= count;
        } else if (*frame >= first) {
            *frame = first;
        }
    }
}

void tas_navigation_clamp(TasProjectState *state) {
    for (size_t i = 0; i < state->navigation_count; ++i) {
        if (state->navigation[i].frame > state->timeline.movie.frame_count) {
            state->navigation[i].frame = state->timeline.movie.frame_count;
        }
    }
}
