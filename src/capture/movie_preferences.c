/* movie_preferences.c - Validated application configuration fields
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "movie_preferences.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

static const char *const keys[NES_MOVIE_PREFERENCE_COUNT] = {"movie.end",
                                                             "movie.read_only",
                                                             "movie.record_insert",
                                                             "movie.record_power_on",
                                                             "movie.subtitles",
                                                             "movie.subtitle_duration",
                                                             "movie.display_overlays",
                                                             "movie.capture_overlays",
                                                             "movie.overlay_position",
                                                             "movie.backups",
                                                             "movie.backup_count",
                                                             "movie.backup_directory"};

void nes_movie_preferences_defaults(NesMoviePreferences *p) {
    if (!p) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->read_only = true;
    p->subtitles = true;
    p->subtitle_duration = 180;
    nes_movie_backup_defaults(&p->backup);
}

bool nes_movie_preferences_valid(const NesMoviePreferences *p) {
    return p && (unsigned)p->end_behavior <= NES_MOVIE_END_STOP && p->subtitle_duration >= 1 &&
           p->subtitle_duration <= 36000 && !(p->display_overlays & ~NES_OVERLAY_ALL) &&
           !(p->capture_overlays & ~NES_OVERLAY_ALL) && (unsigned)p->position <= NES_OVERLAY_BOTTOM_RIGHT &&
           nes_movie_backup_valid(&p->backup) && !strchr(p->backup.directory, '\n') &&
           !strchr(p->backup.directory, '\r');
}

const char *nes_movie_preferences_key(unsigned index) {
    return index < NES_MOVIE_PREFERENCE_COUNT ? keys[index] : NULL;
}

static int key_index(const char *key) {
    if (key) {
        for (unsigned i = 0; i < NES_MOVIE_PREFERENCE_COUNT; ++i) {
            if (!strcmp(key, keys[i])) {
                return (int)i;
            }
        }
    }
    return -1;
}

bool nes_movie_preferences_get(const NesMoviePreferences *p, const char *key, char *value, size_t capacity) {
    if (!nes_movie_preferences_valid(p) || !value || !capacity) {
        return false;
    }
    unsigned number;
    switch (key_index(key)) {
    case 0:
        number = (unsigned)p->end_behavior;
        break;
    case 1:
        number = p->read_only;
        break;
    case 2:
        number = p->record_insert;
        break;
    case 3:
        number = p->record_power_on;
        break;
    case 4:
        number = p->subtitles;
        break;
    case 5:
        number = p->subtitle_duration;
        break;
    case 6:
        number = p->display_overlays;
        break;
    case 7:
        number = p->capture_overlays;
        break;
    case 8:
        number = (unsigned)p->position;
        break;
    case 9:
        number = p->backup.enabled;
        break;
    case 10:
        number = p->backup.retained;
        break;
    case 11: {
        size_t length = strlen(p->backup.directory);
        if (length >= capacity) {
            return false;
        }
        memcpy(value, p->backup.directory, length + 1u);
        return true;
    }
    default:
        return false;
    }
    int length = snprintf(value, capacity, "%u", number);
    return length >= 0 && (size_t)length < capacity;
}

bool nes_movie_preferences_set(NesMoviePreferences *p, const char *key, const char *value) {
    if (!p || !value) {
        return false;
    }
    int index = key_index(key);
    if (index < 0) {
        return false;
    }
    NesMoviePreferences next = *p;
    if (index == 11) {
        size_t length = strlen(value);
        if (length >= sizeof(next.backup.directory)) {
            return false;
        }
        memcpy(next.backup.directory, value, length + 1u);
    } else {
        if (*value < '0' || *value > '9') {
            return false;
        }
        errno = 0;
        char *end;
        unsigned long number = strtoul(value, &end, 10);
        if (errno || *end || number > 36000) {
            return false;
        }
        if ((index <= 4 || index == 9) && number > 1) {
            return false;
        }
        switch (index) {
        case 0:
            next.end_behavior = (NesMovieEndBehavior)number;
            break;
        case 1:
            next.read_only = number != 0;
            break;
        case 2:
            next.record_insert = number != 0;
            break;
        case 3:
            next.record_power_on = number != 0;
            break;
        case 4:
            next.subtitles = number != 0;
            break;
        case 5:
            next.subtitle_duration = (unsigned)number;
            break;
        case 6:
            next.display_overlays = (unsigned)number;
            break;
        case 7:
            next.capture_overlays = (unsigned)number;
            break;
        case 8:
            next.position = (NesOverlayPosition)number;
            break;
        case 9:
            next.backup.enabled = number != 0;
            break;
        case 10:
            next.backup.retained = (unsigned)number;
            break;
        }
    }
    if (!nes_movie_preferences_valid(&next)) {
        return false;
    }
    *p = next;
    return true;
}
