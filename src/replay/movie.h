/*
 * movie.h - Deterministic input movie recording and playback
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REPLAY_MOVIE_H
#define CUPID_REPLAY_MOVIE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct NesMovieSession NesMovieSession;

typedef enum {
    NES_MOVIE_IDLE = 0,
    NES_MOVIE_RECORDING,
    NES_MOVIE_PLAYBACK
} NesMovieMode;

typedef enum {
    NES_MOVIE_START_STATE = 0,
    NES_MOVIE_START_POWER_ON
} NesMovieStartKind;

typedef enum {
    NES_MOVIE_OK = 0,
    NES_MOVIE_COMPLETE,
    NES_MOVIE_INVALID_ARGUMENT,
    NES_MOVIE_NO_IMAGE,
    NES_MOVIE_CONFLICT,
    NES_MOVIE_UNSUPPORTED_HOST_STATE,
    NES_MOVIE_OUT_OF_MEMORY,
    NES_MOVIE_IO_ERROR,
    NES_MOVIE_FORMAT_ERROR,
    NES_MOVIE_VERSION_ERROR,
    NES_MOVIE_INCOMPATIBLE,
    NES_MOVIE_CORRUPT,
    NES_MOVIE_STATE_ERROR,
    NES_MOVIE_EVENT_ERROR,
    NES_MOVIE_LIMIT_REACHED
} NesMovieResult;

typedef struct {
    NesMovieMode mode;
    NesMovieStartKind start_kind;
    uint64_t frame;
    uint64_t total_frames;
    size_t event_count;
    const char *path;
    NesMovieResult last_result;
} NesMovieProgress;

/* A session owns an isolated replay timeline. Starting either mode captures
 * the live machine so stop can restore the user's pre-movie progress. */
NesMovieSession *nes_movie_create(void);
void nes_movie_destroy(NesMovieSession *movie);

/* Recording writes the completed movie atomically when stopped. Playback
 * parses, bounds-checks, verifies compatibility, and prepares the embedded
 * starting state before it changes the active machine. */
NesMovieResult nes_movie_record_start(NesMovieSession *movie, const char *path);
NesMovieResult nes_movie_record_start_power_on(NesMovieSession *movie, const char *path);
NesMovieResult nes_movie_play_start(NesMovieSession *movie, const char *path);
NesMovieResult nes_movie_stop(NesMovieSession *movie);
NesMovieResult nes_movie_set_path(NesMovieSession *movie, const char *path);

/* Call once immediately before each authoritative emulation frame. Recording
 * commits host actions collected since the previous boundary. Playback injects
 * the actions assigned to this frame. NES_MOVIE_COMPLETE means no frame should
 * be executed; stop the movie to restore the live session. Repeated calls while
 * an interrupted frame is in progress do not inject or record twice. */
NesMovieResult nes_movie_frame_boundary(NesMovieSession *movie);
NesMovieResult nes_movie_frame_complete(NesMovieSession *movie, bool completed);

NesMovieMode nes_movie_mode(const NesMovieSession *movie);
void nes_movie_progress(const NesMovieSession *movie, NesMovieProgress *progress);
const char *nes_movie_result_string(NesMovieResult result);

#ifdef __cplusplus
}
#endif

#endif
