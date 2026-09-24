/*
 * tas_project_fm3_internal.h - Shared FM3 input and branch validation
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_TAS_PROJECT_FM3_INTERNAL_H
#define CUPID_TAS_PROJECT_FM3_INTERNAL_H

#include "tas_project_internal.h"

enum {
    FM3_MARKERS_ID_SIZE = 8,
    FM3_BOOKMARKS_ID_SIZE = 10,
    FM3_GREENZONE_ID_SIZE = 10,
    FM3_HISTORY_ID_SIZE = 8,
    FM3_PIANO_ROLL_ID_SIZE = 11,
    FM3_SELECTION_ID_SIZE = 10,
    FM3_NOTE_BYTES_MAX = NES_TAS_MARKER_NOTE_MAX + 1,
    FM3_DECODE_BUDGET_BYTES = 256u * 1024u * 1024u
};

static inline bool fm3_movie_input_type(const NesFm2Movie *movie, uint8_t *type, unsigned *joypads) {
    if (!movie || !type || !joypads) {
        return false;
    }
    if (movie->fourscore) {
        *type = 2;
        *joypads = 4;
        return movie->ports[0] == NES_FM2_PORT_GAMEPAD && movie->ports[1] == NES_FM2_PORT_GAMEPAD;
    }
    if (movie->ports[0] != NES_FM2_PORT_GAMEPAD) {
        return false;
    }
    if (movie->ports[1] == NES_FM2_PORT_GAMEPAD) {
        *type = 1;
        *joypads = 2;
        return true;
    }
    if (movie->ports[1] == NES_FM2_PORT_NONE) {
        *type = 0;
        *joypads = 1;
        return true;
    }
    return false;
}

static inline bool fm3_bookmark_relations_valid(const TasBookmark *bookmarks, int current_branch) {
    if (current_branch < -1 || current_branch >= NES_TAS_BOOKMARK_COUNT ||
        (current_branch >= 0 && !bookmarks[current_branch].occupied)) {
        return false;
    }
    for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
        if (!bookmarks[slot].occupied) {
            continue;
        }
        unsigned seen = 0;
        int branch = (int)slot;
        while (branch >= 0) {
            if (branch >= NES_TAS_BOOKMARK_COUNT || !bookmarks[branch].occupied) {
                return false;
            }
            unsigned bit = 1u << (unsigned)branch;
            if (seen & bit) {
                return false;
            }
            seen |= bit;
            int parent = bookmarks[branch].parent_slot;
            if (parent < -1 || parent >= NES_TAS_BOOKMARK_COUNT || parent == branch) {
                return false;
            }
            if (parent >= 0 &&
                (!bookmarks[parent].occupied || bookmarks[parent].key_frame > bookmarks[branch].key_frame)) {
                return false;
            }
            branch = parent;
        }
    }
    return true;
}

#endif
