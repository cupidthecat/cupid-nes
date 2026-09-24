/* Desktop TAS editor internals. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_TAS_INTERNAL_H
#define CUPID_DESKTOP_TAS_INTERNAL_H

#include "desktop_internal.h"
#include "desktop_tas.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "tas_frontend.h"
#include "../replay/tas_project.h"
#include "../replay/tas_session.h"

enum {
    TAS_GRID_LAG = 0,
    TAS_GRID_COMMAND_BASE = 1,
    TAS_GRID_COMMAND_COUNT = 7,
    TAS_GRID_PAD_BASE = 16,
    TAS_GRID_PAD_COLUMNS = 8,

    TAS_EDIT_SEEK = 0x7410,
    TAS_EDIT_MARKER = 0x7411,
    TAS_EDIT_BRANCH = 0x7412,
    TAS_EDIT_INSERT = 0x7413
};

typedef enum {
    TAS_ACTION_OPEN = 1,
    TAS_ACTION_NEW,
    TAS_ACTION_SAVE,
    TAS_ACTION_SAVE_AS,
    TAS_ACTION_STOP,
    TAS_ACTION_DISCARD,
    TAS_ACTION_PAUSE,
    TAS_ACTION_FRAME,
    TAS_ACTION_BACK,
    TAS_ACTION_SEEK,
    TAS_ACTION_READ_ONLY,
    TAS_ACTION_RECORD,
    TAS_ACTION_RECORD_MODE,
    TAS_ACTION_SKIP_LAG,
    TAS_ACTION_COPY,
    TAS_ACTION_CUT,
    TAS_ACTION_PASTE,
    TAS_ACTION_PASTE_INSERT,
    TAS_ACTION_INSERT,
    TAS_ACTION_DELETE,
    TAS_ACTION_CLONE,
    TAS_ACTION_TRUNCATE,
    TAS_ACTION_UNDO,
    TAS_ACTION_REDO,
    TAS_ACTION_PATTERN,
    TAS_ACTION_PATTERN_NEXT,
    TAS_ACTION_HOLD,
    TAS_ACTION_AUTOFIRE,
    TAS_ACTION_MARKER,
    TAS_ACTION_BRANCH_STORE,
    TAS_ACTION_BRANCH_LOAD,
    TAS_ACTION_BRANCH_CLEAR,
    TAS_ACTION_EDIT_PLAYER_BASE = 40,
    TAS_ACTION_RECORD_PLAYER_BASE = 50,
    TAS_ACTION_BRANCH_SLOT_BASE = 70,
    TAS_ACTION_EXPORT = 80,
    TAS_ACTION_SCRIPT,
    TAS_ACTION_FOLLOW = 82,
    TAS_ACTION_SEEK_CURSOR,
    TAS_ACTION_CURSOR_PLAYBACK,
    TAS_ACTION_MARKER_PREV,
    TAS_ACTION_MARKER_NEXT,
    TAS_ACTION_SELECT_BETWEEN_MARKERS,
    TAS_ACTION_INSERT_MANY,
    TAS_ACTION_SELECT_ALL,
    TAS_ACTION_SELECT_NONE,
    TAS_ACTION_SIDEBAR_BRANCHES,
    TAS_ACTION_SIDEBAR_MARKERS,
    TAS_ACTION_SIDEBAR_INPUT,
    TAS_ACTION_MARKER_REMOVE,
    TAS_ACTION_BRANCH_JUMP,
    TAS_ACTION_HEADER_BASE = 200,
    TAS_ACTION_ACTIVE_BUTTON_BASE = 300,
    TAS_ACTION_MARKER_GOTO_BASE = 1000
} DesktopTasAction;

struct DesktopTasEditor {
    size_t scroll;
    size_t cursor;
    size_t anchor;
    size_t visible_rows;
    int column;
    unsigned edit_player;
    unsigned branch_slot;
    unsigned pattern;
    unsigned sidebar_tab;
    uint8_t active_button;
    bool anchor_valid;
    bool painting;
    bool paint_pressed;
    unsigned paint_player;
    uint8_t paint_mask;
    size_t paint_last;
    bool dragging_scroll;
    bool follow_playback;
    bool follow_initialized;
    size_t last_playback_frame;
};

DesktopTasEditor *desktop_tas_editor(FrontendDesktopUi *ui);
NesTasSession *desktop_tas_session(FrontendDesktopUi *ui);
const NesTasProject *desktop_tas_project_const(const FrontendDesktopUi *ui);
NesTasProject *desktop_tas_project_writable(FrontendDesktopUi *ui, bool report);
void desktop_tas_status(FrontendDesktopUi *ui, const char *text);
void desktop_tas_keep_cursor_visible(FrontendDesktopUi *ui);
void desktop_tas_follow_playback(FrontendDesktopUi *ui, const NesTasProgress *progress);
void desktop_tas_after_model_edit(FrontendDesktopUi *ui, NesTasResult result, const char *success);

void desktop_tas_action(FrontendDesktopUi *ui, unsigned action);
void desktop_tas_select_row(FrontendDesktopUi *ui, size_t row, SDL_Keymod modifiers);
void desktop_tas_cell_down(FrontendDesktopUi *ui, size_t row, int column);
void desktop_tas_cell_drag(FrontendDesktopUi *ui, size_t row, int column);
void desktop_tas_finish_paint(FrontendDesktopUi *ui);
void desktop_tas_scroll_to(FrontendDesktopUi *ui, int y);
bool desktop_tas_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *event);

#endif
