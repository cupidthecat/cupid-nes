/* movie_subtitles.c - Frame-addressed subtitle lookup through seeks and pause
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "capture_overlay.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint64_t frame;
    size_t order;
    char *text;
} Subtitle;

struct NesMovieSubtitles {
    Subtitle *entries;
    size_t count;
};

static int compare(const void *a, const void *b) {
    const Subtitle *left = a, *right = b;
    if (left->frame != right->frame) {
        return left->frame < right->frame ? -1 : 1;
    }
    return left->order < right->order ? -1 : left->order > right->order;
}

void nes_movie_subtitles_free(NesMovieSubtitles **track) {
    if (!track || !*track) {
        return;
    }
    for (size_t i = 0; i < (*track)->count; ++i) {
        free((*track)->entries[i].text);
    }
    free((*track)->entries);
    free(*track);
    *track = NULL;
}

static bool valid_utf8(const unsigned char *p) {
    while (*p) {
        unsigned c = *p++;
        if (c < 128) {
            if (c < 32 || c == 127) {
                return false;
            }
            continue;
        }
        unsigned n, minimum;
        if (c >= 0xC2 && c <= 0xDF) {
            n = 1;
            minimum = 0x80;
            c &= 31;
        } else if (c >= 0xE0 && c <= 0xEF) {
            n = 2;
            minimum = 0x800;
            c &= 15;
        } else if (c >= 0xF0 && c <= 0xF4) {
            n = 3;
            minimum = 0x10000;
            c &= 7;
        } else {
            return false;
        }
        while (n--) {
            unsigned b = *p++;
            if ((b & 0xC0) != 0x80) {
                return false;
            }
            c = c * 64 + (b & 63);
        }
        if (c < minimum || c > 0x10FFFF || (c >= 0xD800 && c <= 0xDFFF)) {
            return false;
        }
    }
    return true;
}

NesFileResult nes_movie_subtitles_load(NesMovieSubtitles **track, const NesFm2StringList *records) {
    if (!track || !records || (records->count && !records->items) || records->count > 100000) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesMovieSubtitles *next = calloc(1, sizeof(*next));
    if (!next) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    next->entries = calloc(records->count ? records->count : 1, sizeof(*next->entries));
    NesFileResult result = next->entries ? NES_FILE_OK : NES_FILE_OUT_OF_MEMORY;
    for (size_t i = 0; result == NES_FILE_OK && i < records->count; ++i) {
        const char *p = records->items[i];
        if (!p || *p < '0' || *p > '9') {
            result = NES_FILE_INVALID_ARGUMENT;
            break;
        }
        uint64_t frame = 0;
        while (*p >= '0' && *p <= '9') {
            unsigned digit = (unsigned)(*p++ - '0');
            if (frame > (UINT64_MAX - digit) / 10u) {
                result = NES_FILE_TOO_LARGE;
                break;
            }
            frame = frame * 10u + digit;
        }
        if (result != NES_FILE_OK) {
            break;
        }
        if (*p != ' ' && *p != '\t') {
            result = NES_FILE_INVALID_ARGUMENT;
            break;
        }
        ++p;
        size_t length = 0;
        while (length < NES_OVERLAY_TEXT && p[length]) {
            ++length;
        }
        if (length == NES_OVERLAY_TEXT || !valid_utf8((const unsigned char *)p)) {
            result = NES_FILE_INVALID_ARGUMENT;
            break;
        }
        char *text = malloc(length + 1u);
        if (!text) {
            result = NES_FILE_OUT_OF_MEMORY;
            break;
        }
        memcpy(text, p, length + 1u);
        next->entries[next->count++] = (Subtitle){frame, i, text};
    }
    if (result != NES_FILE_OK) {
        nes_movie_subtitles_free(&next);
        return result;
    }
    qsort(next->entries, next->count, sizeof(*next->entries), compare);
    nes_movie_subtitles_free(track);
    *track = next;
    return NES_FILE_OK;
}

const char *nes_movie_subtitle_at(const NesMovieSubtitles *track, uint64_t frame, unsigned duration) {
    if (!track || !duration) {
        return "";
    }
    size_t low = 0, high = track->count;
    while (low < high) {
        size_t mid = low + (high - low) / 2;
        if (track->entries[mid].frame <= frame) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    if (!low || frame - track->entries[low - 1].frame >= duration) {
        return "";
    }
    return track->entries[low - 1].text;
}
