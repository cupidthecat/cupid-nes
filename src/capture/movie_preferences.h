/* movie_preferences.h - Persistent movie and TAS defaults
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_MOVIE_PREFERENCES_H
#define NES_MOVIE_PREFERENCES_H
#include "movie_backup.h"
#include "capture_overlay.h"

typedef enum { NES_MOVIE_END_PAUSE, NES_MOVIE_END_STOP } NesMovieEndBehavior;

typedef struct {
    NesMovieEndBehavior end_behavior;
    bool read_only;
    bool record_insert;
    bool record_power_on;
    bool subtitles;
    unsigned subtitle_duration;
    unsigned display_overlays;
    unsigned capture_overlays;
    NesOverlayPosition position;
    NesMovieBackupOptions backup;
} NesMoviePreferences;

enum { NES_MOVIE_PREFERENCE_COUNT = 12 };

void nes_movie_preferences_defaults(NesMoviePreferences *preferences);
bool nes_movie_preferences_valid(const NesMoviePreferences *preferences);
const char *nes_movie_preferences_key(unsigned index);
bool nes_movie_preferences_get(const NesMoviePreferences *preferences, const char *key, char *value, size_t capacity);
/* Unknown keys or malformed values leave preferences unchanged. */
bool nes_movie_preferences_set(NesMoviePreferences *preferences, const char *key, const char *value);
#endif
