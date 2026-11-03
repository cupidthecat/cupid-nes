/*
 * history_frontend.h - Visual rewind timeline controls
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HISTORY_FRONTEND_H
#define CUPID_HISTORY_FRONTEND_H
#include "frontend_execution.h"
typedef struct FrontendHistory FrontendHistory;
enum {
    HISTORY_PANEL = 0x2700, HISTORY_OPEN, HISTORY_POSITION, HISTORY_PREVIOUS, HISTORY_NEXT,
    HISTORY_PLAY, HISTORY_SAVE_STATE, HISTORY_RESUME, HISTORY_CLOSE, HISTORY_EXPORT,
    HISTORY_FIRST, HISTORY_LAST, HISTORY_RANGE_FIRST, HISTORY_RANGE_LAST, HISTORY_VOLUME,
    HISTORY_BOOKMARK_SET, HISTORY_BOOKMARK_JUMP
};
enum { HISTORY_BOOKMARKS = 10 };
typedef struct {
    bool open, playing, can_export;
    size_t index, count, range_first, range_last;
    size_t bookmarks[HISTORY_BOOKMARKS]; /* SIZE_MAX means empty. */
    unsigned width, volume;
    double fps;
    uint64_t revision;
    const uint32_t *pixels;
    const char *status;
} FrontendHistoryState;
/* Native GTK and SDL share one controller and identical snapshot semantics. */
void frontend_history_use_native(FrontendHistory *history);
bool frontend_history_state(FrontendHistory *history, FrontendHistoryState *state);
bool frontend_history_action(FrontendHistory *history, unsigned id, const char *value,
                              int selected, char *error, size_t error_size);
FrontendHistory *frontend_history_create(FrontendExecutionRuntime *execution);
void frontend_history_destroy(FrontendHistory *history);
/* Close before replacing/resetting the session or changing its rewind limits. */
void frontend_history_close(FrontendHistory *history);
bool frontend_history_event(FrontendHistory *history, const SDL_Event *event);
void frontend_history_tick(FrontendHistory *history);
#endif
