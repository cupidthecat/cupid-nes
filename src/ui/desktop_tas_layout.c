/*
 * desktop_tas_layout.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS timeline and playback controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"
#include "../system/vs_system.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const Clay_Color ink = {232, 237, 247, 255};
static const Clay_Color muted = {153, 168, 192, 255};
static const Clay_Color accent = {183, 196, 255, 255};
static const Clay_Color surface = {26, 31, 43, 255};
static const Clay_Color raised = {39, 47, 64, 255};
static const Clay_Color selected = {58, 70, 104, 255};

static void text(FrontendDesktopUi *ui, const char *value, unsigned size, Clay_Color color) {
    CLAY_TEXT(desktop_clay_string(ui->clay, value),
              CLAY_TEXT_CONFIG({.fontSize = (uint16_t)size, .textColor = color, .wrapMode = CLAY_TEXT_WRAP_NONE}));
}

static void button(FrontendDesktopUi *ui, const char *label, unsigned action, bool checked, bool enabled) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_ACTION : HIT_NONE, (int)action, 0);
    CLAY(id, {.layout = {.sizing = {.height = CLAY_SIZING_FIXED(25)},
                         .padding = {7, 7, 4, 4},
                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = checked ? selected : raised,
              .border = {.color = checked ? accent : (Clay_Color){73, 87, 115, 255},
                         .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}}}) {
        text(ui, label, 12, enabled ? ink : muted);
    }
}

static void spacer(void) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
    }
}

static void file_toolbar(FrontendDesktopUi *ui, const NesTasProgress *progress, bool writable) {
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        text(ui, "TAS Editor", 18, ink);
        button(ui, "Open", TAS_ACTION_OPEN, false, !progress->active);
        button(ui, "New", TAS_ACTION_NEW, false, !progress->active);
        button(ui, "Save", TAS_ACTION_SAVE, false, progress->active && !progress->frame_in_progress);
        button(ui, "Save as", TAS_ACTION_SAVE_AS, false, progress->active && !progress->frame_in_progress);
        button(ui, "Export", TAS_ACTION_EXPORT, false, progress->active && !progress->frame_in_progress);
        button(ui, "Script", TAS_ACTION_SCRIPT, false, writable);
        button(ui, "Splicer", TAS_ACTION_SPLICE_OPEN, ui->tas_editor->sidebar_tab == 6, progress->active);
        button(ui, "Stop", TAS_ACTION_STOP, false, progress->active);
        button(ui, "Discard edits", TAS_ACTION_DISCARD, false, progress->active && progress->dirty);
        spacer();
        text(ui, progress->dirty ? "Unsaved" : "", 12, accent);
    }
}

static void playback_toolbar(FrontendDesktopUi *ui, const NesTasProgress *progress) {
    DesktopTasEditor *editor = ui->tas_editor;
    bool active = progress->active;
    bool paused = ui->execution && frontend_execution_paused(ui->execution);
    bool boundary = active && !progress->frame_in_progress && !progress->seeking;
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        button(ui, "Back", TAS_ACTION_BACK, false, active && progress->frame > 0);
        button(ui, "Frame", TAS_ACTION_FRAME, false, active);
        button(ui, paused ? "Play" : "Pause", TAS_ACTION_PAUSE, !paused, active);
        button(ui, "Go to", TAS_ACTION_SEEK, false, active);
        button(ui, "Seek cursor", TAS_ACTION_SEEK_CURSOR, false, active);
        button(ui, "Cursor = play", TAS_ACTION_CURSOR_PLAYBACK, false, active);
        button(ui, "Follow", TAS_ACTION_FOLLOW, editor->follow_playback, active);
        button(ui, "Read-only", TAS_ACTION_READ_ONLY, progress->read_only, boundary);
        button(ui, "Record", TAS_ACTION_RECORD, progress->recording, boundary && !progress->read_only);
        button(ui, progress->record_mode == NES_TAS_RECORD_INSERT ? "Insert take" : "Overwrite take",
               TAS_ACTION_RECORD_MODE, progress->record_mode == NES_TAS_RECORD_INSERT,
               boundary && !progress->read_only);
        button(ui, "Skip lag", TAS_ACTION_SKIP_LAG, ui->execution && ui->execution->tas_skip_lag, active);
    }
}

static void fixed_label(FrontendDesktopUi *ui, const char *label, float width, Clay_Color color) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(21)},
                             .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}}}) {
        text(ui, label, 11, color);
    }
}

static void clipped_label(FrontendDesktopUi *ui, const char *label, float width, Clay_Color color) {
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(21)},
                             .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        text(ui, label, 11, color);
    }
}

static void header_button(FrontendDesktopUi *ui, const char *label, unsigned action, bool checked, bool enabled,
                          float width) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_ACTION : HIT_NONE, (int)action, 0);
    CLAY(id,
         {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(21)},
                     .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
          .backgroundColor = checked ? selected : (Clay_Color){0},
          .border = {.color = checked ? accent : (Clay_Color){59, 68, 89, 255}, .width = {.right = 1, .bottom = 1}}}) {
        text(ui, label, 11, checked ? accent : enabled ? muted : (Clay_Color){91, 101, 121, 255});
    }
}

static void wide_button(FrontendDesktopUi *ui, const char *label, unsigned action, bool checked, bool enabled,
                        float width) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_ACTION : HIT_NONE, (int)action, 0);
    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(23)},
                         .padding = {6, 6, 3, 3},
                         .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = checked ? selected : raised,
              .border = {.color = checked ? accent : (Clay_Color){73, 87, 115, 255},
                         .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}},
              .clip = {.horizontal = true, .vertical = true}}) {
        text(ui, label, 11, enabled ? ink : muted);
    }
}

static void cell(FrontendDesktopUi *ui, int row, int column, const char *value, float width, bool on, bool enabled,
                 bool focused) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_CELL : HIT_NONE, row, column);
    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(21)},
                         .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = on        ? selected
                                 : focused ? (Clay_Color){44, 52, 72, 255}
                                           : (Clay_Color){0},
              .border = {.color = focused ? accent : (Clay_Color){59, 68, 89, 255},
                         .width = {.left = focused ? 1 : 0, .right = 1, .top = focused ? 1 : 0, .bottom = 1}}}) {
        text(ui, value, 11, on || focused ? accent : muted);
    }
}

static const char *marker_at(const NesTasProject *project, size_t frame) {
    for (size_t i = 0; i < nes_tas_marker_count(project); ++i) {
        NesTasMarkerView marker;
        if (!nes_tas_marker(project, i, &marker)) {
            continue;
        }
        if (marker.frame == frame) {
            return marker.note ? marker.note : "";
        }
        if (marker.frame > frame) {
            break;
        }
    }
    return "";
}

static void sidebar_tabs(FrontendDesktopUi *ui) {
    unsigned tab = ui->tas_editor->sidebar_tab;
    CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
        button(ui, "Branches", TAS_ACTION_SIDEBAR_BRANCHES, tab == 0, true);
        button(ui, "Markers", TAS_ACTION_SIDEBAR_MARKERS, tab == 1, true);
        button(ui, "Input", TAS_ACTION_SIDEBAR_INPUT, tab == 2, true);
    }
    CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
        button(ui, "Cache", TAS_ACTION_SIDEBAR_CACHE, tab == 3, true);
        button(ui, "History", TAS_ACTION_SIDEBAR_HISTORY, tab == 4, true);
        button(ui, "Bookmarks", TAS_ACTION_SIDEBAR_NAVIGATION, tab == 5, true);
    }
}

static void branches_sidebar(FrontendDesktopUi *ui, const NesTasProject *project, bool writable, float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    NesTasBookmarkView selected_branch = {0};
    if (project) {
        (void)nes_tas_bookmark(project, editor->branch_slot, &selected_branch);
    }
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Store", TAS_ACTION_BRANCH_STORE, false, writable);
            button(ui, "Load", TAS_ACTION_BRANCH_LOAD, false, writable && selected_branch.occupied);
            button(ui, "Jump", TAS_ACTION_BRANCH_JUMP, false, project && selected_branch.occupied);
            button(ui, "Clear", TAS_ACTION_BRANCH_CLEAR, false, writable && selected_branch.occupied);
        }
        text(ui, "Slot   Frame / Parent / Name", 11, muted);
        int current_branch = project ? nes_tas_current_branch(project) : -1;
        for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
            NesTasBookmarkView branch = {0};
            if (project) {
                (void)nes_tas_bookmark(project, slot, &branch);
            }
            char slot_label[8];
            char info[128];
            snprintf(slot_label, sizeof(slot_label), "%u%s", slot == 9 ? 0 : slot + 1, branch.occupied ? "*" : "");
            if (branch.occupied) {
                char parent[16];
                if (branch.parent_slot >= 0) {
                    unsigned parent_slot = (unsigned)branch.parent_slot;
                    snprintf(parent, sizeof(parent), "%u", parent_slot == 9 ? 0 : parent_slot + 1);
                } else {
                    snprintf(parent, sizeof(parent), "-");
                }
                snprintf(info, sizeof(info), "%sF%zu  P%s  %s", current_branch == (int)slot ? "> " : "",
                         branch.key_frame, parent, branch.name && *branch.name ? branch.name : "(unnamed)");
            } else {
                snprintf(info, sizeof(info), "empty");
            }
            CLAY_AUTO_ID({.layout = {.childGap = 4, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
                button(ui, slot_label, TAS_ACTION_BRANCH_SLOT_BASE + slot, editor->branch_slot == slot,
                       project != NULL);
                clipped_label(ui, info, width - 46, current_branch == (int)slot ? accent : muted);
            }
        }
    }
}

static size_t marker_window_start(const NesTasProject *project, size_t cursor, size_t limit) {
    size_t count = nes_tas_marker_count(project);
    size_t nearest = 0;
    for (; nearest < count; ++nearest) {
        NesTasMarkerView marker;
        if (nes_tas_marker(project, nearest, &marker) && marker.frame >= cursor) {
            break;
        }
    }
    size_t start = nearest > limit / 2 ? nearest - limit / 2 : 0;
    if (count > limit && start > count - limit) {
        start = count - limit;
    }
    return start;
}

static void markers_sidebar(FrontendDesktopUi *ui, const NesTasProject *project, bool writable, float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    bool frames = project && nes_tas_project_frame_count(project) != 0;
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Prev", TAS_ACTION_MARKER_PREV, false, project != NULL);
            button(ui, "Next", TAS_ACTION_MARKER_NEXT, false, project != NULL);
            button(ui, "Set / edit", TAS_ACTION_MARKER, false, writable && frames);
            button(ui, "Remove", TAS_ACTION_MARKER_REMOVE, false,
                   writable && frames && *marker_at(project, editor->cursor));
        }
        button(ui, "Select between markers", TAS_ACTION_SELECT_BETWEEN_MARKERS, false, writable && frames);
        size_t count = project ? nes_tas_marker_count(project) : 0;
        if (!count) {
            text(ui, "No markers in this project.", 11, muted);
        } else {
            text(ui, "Frame   Note", 11, muted);
            const size_t visible_markers = 10;
            size_t first = marker_window_start(project, editor->cursor, visible_markers);
            size_t last = first + visible_markers < count ? first + visible_markers : count;
            for (size_t i = first; i < last; ++i) {
                NesTasMarkerView marker;
                if (!nes_tas_marker(project, i, &marker)) {
                    continue;
                }
                char label[160];
                snprintf(label, sizeof(label), "%zu   %s", marker.frame,
                         marker.note && *marker.note ? marker.note : "(no note)");
                bool direct = i <= (size_t)(INT_MAX - TAS_ACTION_MARKER_GOTO_BASE);
                wide_button(ui, label, direct ? TAS_ACTION_MARKER_GOTO_BASE + (unsigned)i : TAS_ACTION_MARKER_GOTO_BASE,
                            marker.frame == editor->cursor, direct, width);
            }
        }
    }
}

static void input_sidebar(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress,
                          bool writable) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesFm2Movie *movie = project ? nes_tas_project_movie(project) : NULL;
    unsigned players = movie && movie->fourscore ? 4 : 2;
    static const char *const button_names[] = {"A", "B", "Se", "St", "U", "D", "L", "R"};
    static const char *const visual_names[] = {"R", "L", "D", "U", "St", "Se", "B", "A"};
    static const unsigned visual_bits[] = {7, 6, 5, 4, 3, 2, 1, 0};
    static const char *const patterns[] = {"Pattern 1/0", "Pattern 0/1", "Pattern 11/00", "Pattern 111/0"};
    bool frames = project && nes_tas_project_frame_count(project) != 0;
    bool selection = project && nes_tas_selection_count(project) != 0;
    bool clipboard = project && nes_tas_clipboard_available(project);
    bool record_ready = progress->active && !progress->read_only && !progress->frame_in_progress && !progress->seeking;
    unsigned active_bit = 0;
    while (active_bit < 8 && !(editor->active_button & (1u << active_bit))) {
        ++active_bit;
    }
    if (active_bit == 8) {
        active_bit = 0;
    }
    char label[48];
    NesTasSession *session = desktop_tas_session(ui);
    uint8_t held = session ? nes_tas_session_hold(session, editor->edit_player) : 0;
    uint8_t autofire = session ? nes_tas_session_autofire(session, editor->edit_player) : 0;
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        CLAY_AUTO_ID({.layout = {.childGap = 4, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
            text(ui, "Edit", 11, muted);
            for (unsigned player = 0; player < players; ++player) {
                snprintf(label, sizeof(label), "P%u", player + 1);
                button(ui, label, TAS_ACTION_EDIT_PLAYER_BASE + player, editor->edit_player == player,
                       progress->active);
            }
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
            text(ui, "Record", 11, muted);
            for (unsigned player = 0; player < players; ++player) {
                snprintf(label, sizeof(label), "P%u", player + 1);
                button(ui, label, TAS_ACTION_RECORD_PLAYER_BASE + player,
                       (progress->record_players & (1u << player)) != 0, record_ready);
            }
        }
        CLAY_AUTO_ID({.layout = {.childGap = 3}}) {
            for (unsigned i = 0; i < 8; ++i) {
                unsigned bit = visual_bits[i];
                header_button(ui, visual_names[i], TAS_ACTION_ACTIVE_BUTTON_BASE + bit,
                              editor->active_button == (1u << bit), progress->active, 26);
            }
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, patterns[editor->pattern % 4], TAS_ACTION_PATTERN_NEXT, false, progress->active);
            button(ui, "Apply", TAS_ACTION_PATTERN, false, writable && frames);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            snprintf(label, sizeof(label), "Hold %s", button_names[active_bit]);
            button(ui, label, TAS_ACTION_HOLD, (held & editor->active_button) != 0, progress->active);
            snprintf(label, sizeof(label), "Auto %s", button_names[active_bit]);
            button(ui, label, TAS_ACTION_AUTOFIRE, (autofire & editor->active_button) != 0, progress->active);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Undo", TAS_ACTION_UNDO, false, writable && nes_tas_project_can_undo(project));
            button(ui, "Redo", TAS_ACTION_REDO, false, writable && nes_tas_project_can_redo(project));
            button(ui, "Copy", TAS_ACTION_COPY, false, writable && selection);
            button(ui, "Cut", TAS_ACTION_CUT, false, writable && selection);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Paste", TAS_ACTION_PASTE, false, writable && clipboard);
            button(ui, "Paste insert", TAS_ACTION_PASTE_INSERT, false, writable && clipboard);
            button(ui, "Insert", TAS_ACTION_INSERT, false, writable);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Insert many", TAS_ACTION_INSERT_MANY, false, writable);
            button(ui, "Delete", TAS_ACTION_DELETE, false, writable && frames);
            button(ui, "Clone", TAS_ACTION_CLONE, false, writable && frames);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Trim", TAS_ACTION_TRUNCATE, false, writable && frames);
            button(ui, "All", TAS_ACTION_SELECT_ALL, false, writable && frames);
            button(ui, "None", TAS_ACTION_SELECT_NONE, false, writable && selection);
            button(ui, "Between", TAS_ACTION_SELECT_BETWEEN_MARKERS, false, writable && frames);
        }
    }
}

static void cache_sidebar(FrontendDesktopUi *ui, const NesTasProgress *progress) {
    NesTasSession *session = desktop_tas_session(ui);
    NesTasCacheInfo cache;
    nes_tas_session_cache_info(session, &cache);
    char label[128];
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        text(ui, "C = cached boundary; S = startup", 11, muted);
        text(ui, "Other boundaries need replay.", 11, muted);
        snprintf(label, sizeof(label), "%zu states / %.2f MiB + startup", cache.checkpoint_count,
                 (double)cache.checkpoint_bytes / (1024.0 * 1024.0));
        text(ui, label, 11, ink);
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            snprintf(label, sizeof(label), "Budget %zu MiB", cache.byte_limit / (1024u * 1024u));
            button(ui, label, TAS_ACTION_CACHE_MB, false, progress->active);
            snprintf(label, sizeof(label), "Every %u", cache.interval);
            button(ui, label, TAS_ACTION_CACHE_INTERVAL, false, progress->active);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Clear all", TAS_ACTION_CACHE_CLEAR, false, progress->active);
            button(ui, "Clear after cursor", TAS_ACTION_CACHE_CLEAR_AFTER, false, progress->active);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Previous state", TAS_ACTION_CACHE_PREV, false, progress->active);
            button(ui, "Next state", TAS_ACTION_CACHE_NEXT, false, progress->active);
        }
        NesTasCacheEntry nearest;
        if (nes_tas_session_cache_before(session, ui->tas_editor->cursor, &nearest)) {
            snprintf(label, sizeof(label), "Cursor %zu: restore %zu", ui->tas_editor->cursor, nearest.frame);
            text(ui, label, 11, ink);
            snprintf(label, sizeof(label), "Then replay %zu frames", ui->tas_editor->cursor - nearest.frame);
            text(ui, label, 11, muted);
            snprintf(label, sizeof(label), "State %zu KiB / lag %zu", nearest.bytes / 1024u, nearest.lag_count);
            text(ui, label, 11, muted);
        }
        text(ui, "Recent states (capture order)", 11, muted);
        size_t first = cache.checkpoint_count > 8 ? cache.checkpoint_count - 8 : 0;
        for (size_t i = first; i < cache.checkpoint_count; ++i) {
            NesTasCacheEntry entry;
            if (nes_tas_session_cache_entry(session, i, &entry)) {
                snprintf(label, sizeof(label), "C %zu / %zu KiB / lag %zu", entry.frame, entry.bytes / 1024u,
                         entry.lag_count);
                text(ui, label, 11, (Clay_Color){131, 219, 165, 255});
            }
        }
    }
}

static void history_sidebar(FrontendDesktopUi *ui, const NesTasProject *project, bool writable, float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    NesTasHistoryInfo info;
    nes_tas_project_history_info(project, &info);
    uint64_t revision = nes_tas_project_revision(project);
    if (!editor->history_initialized || editor->history_project != project || editor->history_revision != revision ||
        editor->history_snapshot.operations != info.operations || editor->history_snapshot.position != info.position) {
        editor->history_selection = info.position;
        editor->history_scroll = (info.position / TAS_HISTORY_VISIBLE) * TAS_HISTORY_VISIBLE;
    }
    editor->history_initialized = true;
    editor->history_project = project;
    editor->history_revision = revision;
    editor->history_snapshot = info;
    char label[192];
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        snprintf(label, sizeof(label), "Position %zu of %zu retained edits", info.position, info.operations);
        text(ui, label, 11, ink);
        snprintf(label, sizeof(label), "Snapshots %.2f / %.2f MiB", (double)info.bytes / (1024.0 * 1024.0),
                 (double)info.byte_limit / (1024.0 * 1024.0));
        text(ui, label, 11, muted);
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Older", TAS_ACTION_HISTORY_OLDER, false, project && editor->history_scroll != 0);
            button(ui, "Newer", TAS_ACTION_HISTORY_NEWER, false,
                   project && info.operations - editor->history_scroll >= TAS_HISTORY_VISIBLE);
            button(ui, "Current", TAS_ACTION_HISTORY_CURRENT, false, project != NULL);
        }
        for (unsigned row = 0; project && row < TAS_HISTORY_VISIBLE; ++row) {
            size_t position = editor->history_scroll + row;
            NesTasHistoryView view;
            if (!nes_tas_project_history_entry(project, position, &view)) {
                break;
            }
            snprintf(label, sizeof(label), "%s%zu %s",
                     view.current   ? "> "
                     : view.applied ? "  "
                                    : "+ ",
                     position, view.description);
            wide_button(ui, label, TAS_ACTION_HISTORY_SELECT_BASE + row, editor->history_selection == position, true,
                        width);
        }
        NesTasHistoryView selected_view;
        if (project && nes_tas_project_history_entry(project, editor->history_selection, &selected_view)) {
            /* A wrapping detail label keeps long grouped descriptions readable. */
            CLAY_TEXT(desktop_clay_string(ui->clay, selected_view.description),
                      CLAY_TEXT_CONFIG({.fontSize = 11, .textColor = ink, .wrapMode = CLAY_TEXT_WRAP_WORDS}));
            if (selected_view.first_frame == SIZE_MAX) {
                text(ui, "No affected frame range", 11, muted);
            } else {
                snprintf(label, sizeof(label), "Frames %zu to %zu", selected_view.first_frame,
                         selected_view.last_frame);
                text(ui, label, 11, muted);
            }
        }
        button(ui, "Restore selected position", TAS_ACTION_HISTORY_RESTORE, false,
               writable && !info.edit_active && editor->history_selection != info.position);
        if (info.edit_active) {
            text(ui, "Finish the current take or edit first.", 11, muted);
        }
    }
}

static void navigation_details(FrontendDesktopUi *ui, const NesTasNavigationView *entry, float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    char details[NES_TAS_BOOKMARK_NAME_MAX + NES_TAS_NAVIGATION_NOTE_MAX + 32];
    snprintf(details, sizeof(details), "Name:\n%s\nNote:\n%s", entry->name, *entry->note ? entry->note : "(empty)");
    size_t starts[sizeof(details)], lengths[sizeof(details)];
    size_t lines = 0, offset = 0;
    for (;;) {
        size_t remaining = strcspn(details + offset, "\n");
        size_t take = desktop_clay_text_fit(ui->clay, details + offset, remaining, 11, width - 4);
        if (remaining && !take) {
            text(ui, "Widen the window to view details.", 11, muted);
            return;
        }
        if (take < remaining) {
            /* Keep words together when possible, and preserve every byte when
             * a long token needs a break at a measured UTF-8 boundary. */
            for (size_t at = take; at; --at) {
                if (details[offset + at - 1] == ' ') {
                    take = at;
                    break;
                }
            }
        }
        starts[lines] = offset;
        lengths[lines++] = take;
        offset += take;
        if (take == remaining) {
            if (!details[offset]) {
                break;
            }
            ++offset;
        }
    }
    editor->navigation_detail_pages = (lines + TAS_NAVIGATION_DETAIL_LINES - 1) / TAS_NAVIGATION_DETAIL_LINES;
    if (editor->navigation_detail_page >= editor->navigation_detail_pages) {
        editor->navigation_detail_page = editor->navigation_detail_pages - 1;
    }
    char label[96];
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width)},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .childGap = 4}}) {
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "List", TAS_ACTION_NAVIGATION_LIST, false, true);
            button(ui, "Earlier", TAS_ACTION_NAVIGATION_DETAIL_PREV, false, editor->navigation_detail_page != 0);
            button(ui, "Later", TAS_ACTION_NAVIGATION_DETAIL_NEXT, false,
                   editor->navigation_detail_page + 1 < editor->navigation_detail_pages);
        }
        snprintf(label, sizeof(label), "Boundary %zu", entry->frame);
        text(ui, label, 11, accent);
        snprintf(label, sizeof(label), "Details page %zu / %zu", editor->navigation_detail_page + 1,
                 editor->navigation_detail_pages);
        text(ui, label, 11, muted);
        size_t first = editor->navigation_detail_page * TAS_NAVIGATION_DETAIL_LINES;
        for (size_t row = first; row < lines && row - first < TAS_NAVIGATION_DETAIL_LINES; ++row) {
            char line[sizeof(details)];
            memcpy(line, details + starts[row], lengths[row]);
            line[lengths[row]] = '\0';
            CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(15)}}}) {
                text(ui, line, 11, ink);
            }
        }
    }
}

static void navigation_sidebar(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress,
                               bool writable, float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    desktop_tas_navigation_observe(ui);
    bool ready = progress->active && !progress->recording && !progress->frame_in_progress && !progress->seeking &&
                 !nes_tas_edit_active(project);
    bool editing = ready && writable;
    bool adding = editing && editor->navigation_count < NES_TAS_NAVIGATION_LIMIT;
    NesTasNavigationView selected_entry;
    bool has_selection = nes_tas_navigation(project, editor->navigation_selection, &selected_entry);
    if (has_selection && editor->navigation_details) {
        navigation_details(ui, &selected_entry, width);
        return;
    }
    size_t neighbor;
    char label[160];
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .childGap = 4}}) {
        snprintf(label, sizeof(label), "%zu bookmarks / playback %zu", editor->navigation_count, progress->frame);
        text(ui, label, 11, ink);
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "At cursor", TAS_ACTION_NAVIGATION_CURSOR, false, adding);
            button(ui, "At playback", TAS_ACTION_NAVIGATION_PLAYBACK, false, adding);
            button(ui, "Details", TAS_ACTION_NAVIGATION_DETAILS, false, has_selection);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Rename", TAS_ACTION_NAVIGATION_NAME, false, editing && has_selection);
            button(ui, "Note", TAS_ACTION_NAVIGATION_NOTE, false, editing && has_selection);
            button(ui, "Delete", TAS_ACTION_NAVIGATION_REMOVE, false, editing && has_selection);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Previous", TAS_ACTION_NAVIGATION_PREV, false,
                   ready && nes_tas_navigation_neighbor(project, progress->frame, false, &neighbor));
            button(ui, "Next", TAS_ACTION_NAVIGATION_NEXT, false,
                   ready && nes_tas_navigation_neighbor(project, progress->frame, true, &neighbor));
            button(ui, "Go to", TAS_ACTION_NAVIGATION_GOTO, false, ready && has_selection);
        }
        CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
            button(ui, "Earlier page", TAS_ACTION_NAVIGATION_OLDER, false, editor->navigation_scroll != 0);
            button(ui, "Later page", TAS_ACTION_NAVIGATION_NEWER, false,
                   editor->navigation_scroll + TAS_NAVIGATION_VISIBLE < editor->navigation_count);
        }
        for (unsigned row = 0; row < TAS_NAVIGATION_VISIBLE; ++row) {
            size_t index = editor->navigation_scroll + row;
            NesTasNavigationView entry;
            if (!nes_tas_navigation(project, index, &entry)) {
                break;
            }
            snprintf(label, sizeof(label), "%zu  %s", entry.frame, entry.name);
            wide_button(ui, label, TAS_ACTION_NAVIGATION_SELECT_BASE + row, editor->navigation_selection == index, true,
                        width);
        }
        if (has_selection) {
            snprintf(label, sizeof(label), "Boundary %zu%s", selected_entry.frame,
                     selected_entry.frame == nes_tas_project_frame_count(project) ? " (end of movie)" : "");
            text(ui, label, 11, accent);
            text(ui, "Details shows the full name and note.", 11, muted);
        } else {
            text(ui, editor->navigation_count ? "Select a bookmark to view its note." : "No bookmarks in this project.",
                 11, muted);
        }
    }
}

static void sidebar(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress, bool writable,
                    float width) {
    DesktopTasEditor *editor = ui->tas_editor;
    if (editor->sidebar_tab > 6) {
        editor->sidebar_tab = 0;
    }
    float content_width = width - 12;
    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_GROW()},
                             .layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .padding = {6, 6, 6, 6},
                             .childGap = 6},
                  .backgroundColor = surface,
                  .border = {.color = {59, 68, 89, 255}, .width = {.left = 1, .right = 1, .top = 1, .bottom = 1}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        sidebar_tabs(ui);
        if (editor->sidebar_tab == 1) {
            markers_sidebar(ui, project, writable, content_width);
        } else if (editor->sidebar_tab == 2) {
            input_sidebar(ui, project, progress, writable);
        } else if (editor->sidebar_tab == 3) {
            cache_sidebar(ui, progress);
        } else if (editor->sidebar_tab == 4) {
            history_sidebar(ui, project, writable, content_width);
        } else if (editor->sidebar_tab == 5) {
            navigation_sidebar(ui, project, progress, writable, content_width);
        } else if (editor->sidebar_tab == 6) {
            desktop_tas_splice_layout(ui, progress, writable, content_width);
        } else {
            branches_sidebar(ui, project, writable, content_width);
        }
    }
}

static bool command_visible(const NesFm2Movie *movie, unsigned command) {
    if (command < 2) {
        return true;
    }
    if (command < 4) {
        return movie->fds;
    }
    return vs_enabled();
}

static void timeline(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress, float width,
                     unsigned visible, bool writable) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    unsigned first_player = movie->fourscore ? (editor->edit_player / 2) * 2 : 0;
    static const char *const pad_names[] = {"A", "B", "Se", "St", "U", "D", "L", "R"};
    static const char *const commands[] = {"R", "P", "I", "S", "C1", "C2", "Sv"};
    const float frame_width = 82, cell_width = 22, lag_width = 25;
    unsigned command_count = 0;
    for (unsigned command = 0; command < 7; ++command) {
        if (command_visible(movie, command)) {
            ++command_count;
        }
    }
    float pad_widths[2];
    for (unsigned n = 0; n < 2; ++n) {
        unsigned player = first_player + n;
        pad_widths[n] = movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD ? cell_width * 8 : 104;
    }
    float note_width =
        width - frame_width - lag_width - cell_width * command_count - pad_widths[0] - pad_widths[1] - 12;
    if (note_width < 0) {
        note_width = 0;
    }
    char value[128];
    size_t frames = nes_tas_project_frame_count(project);
    editor->visible_rows = visible;
    size_t max_scroll = frames > visible ? frames - visible : 0;
    if (editor->scroll > max_scroll) {
        editor->scroll = max_scroll;
    }
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM},
                  .backgroundColor = surface,
                  .clip = {.horizontal = true, .vertical = true}}) {
        CLAY_AUTO_ID({.layout = {.childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
            fixed_label(ui, "Frame", frame_width, ink);
            fixed_label(ui, "Lag", lag_width, ink);
            fixed_label(ui, "Cmd", cell_width * command_count, ink);
            for (unsigned n = 0; n < 2; ++n) {
                snprintf(value, sizeof(value), "Player %u", first_player + n + 1);
                fixed_label(ui, value, pad_widths[n], ink);
            }
            fixed_label(ui, "Marker", note_width, ink);
        }
        CLAY_AUTO_ID({.layout = {.childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
            fixed_label(ui, "", frame_width + lag_width, muted);
            for (unsigned command = 0; command < 7; ++command) {
                if (command_visible(movie, command)) {
                    fixed_label(ui, commands[command], cell_width, muted);
                }
            }
            for (unsigned n = 0; n < 2; ++n) {
                unsigned player = first_player + n;
                if (movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD) {
                    for (unsigned display = 0; display < 8; ++display) {
                        unsigned bit = 7 - display;
                        header_button(ui, pad_names[bit], TAS_ACTION_HEADER_BASE + player * 8 + bit,
                                      editor->edit_player == player && editor->active_button == (1u << bit),
                                      progress->active, cell_width);
                    }
                } else {
                    fixed_label(ui, movie->ports[player] == NES_FM2_PORT_ZAPPER ? "Zapper X,Y:B" : "Disconnected",
                                pad_widths[n], muted);
                }
            }
        }
        CLAY_AUTO_ID({.layout = {.childGap = 0}}) {
            CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
                for (unsigned row = 0; row < visible && editor->scroll + row < frames; ++row) {
                    size_t index = editor->scroll + row;
                    const NesFm2Frame *frame = nes_tas_project_frame(project, index);
                    bool selected_row = nes_tas_selection_contains(project, index);
                    CLAY_AUTO_ID({.layout = {.childGap = 0},
                                  .backgroundColor = selected_row ? (Clay_Color){39, 48, 72, 255}
                                                     : row & 1    ? (Clay_Color){30, 36, 49, 255}
                                                                  : surface}) {
                        Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_TAS_ROW, (int)row, 0);
                        CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(frame_width),
                                                        .height = CLAY_SIZING_FIXED(21)},
                                             .padding = {5, 5, 0, 0},
                                             .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                                  .backgroundColor = editor->cursor == index ? selected : (Clay_Color){0}}) {
                            NesTasCacheEntry cached;
                            bool saved = nes_tas_session_cache_before(desktop_tas_session(ui), index, &cached) &&
                                         cached.frame == index;
                            snprintf(value, sizeof(value), "%s%s%zu", progress->frame == index ? ">" : "",
                                     saved ? cached.initial ? "S " : "C " : "", index);
                            text(ui, value, 11,
                                 progress->frame == index ? accent
                                 : saved                  ? (Clay_Color){131, 219, 165, 255}
                                                          : ink);
                        }
                        NesTasLagState lag = nes_tas_lag(project, index);
                        cell(ui, (int)row, TAS_GRID_LAG,
                             lag == NES_TAS_LAG_YES  ? "L"
                             : lag == NES_TAS_LAG_NO ? "."
                                                     : "?",
                             lag_width, lag == NES_TAS_LAG_YES, writable,
                             editor->cursor == index && editor->column == TAS_GRID_LAG);
                        for (unsigned command = 0; command < 7; ++command) {
                            if (!command_visible(movie, command)) {
                                continue;
                            }
                            bool set = (frame->commands & (1u << command)) != 0;
                            cell(ui, (int)row, TAS_GRID_COMMAND_BASE + (int)command, set ? commands[command] : "",
                                 cell_width, set, writable,
                                 editor->cursor == index && editor->column == TAS_GRID_COMMAND_BASE + (int)command);
                        }
                        for (unsigned n = 0; n < 2; ++n) {
                            unsigned player = first_player + n;
                            if (movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD) {
                                for (unsigned display = 0; display < 8; ++display) {
                                    unsigned bit = 7 - display;
                                    bool set = (frame->pads[player] & (1u << bit)) != 0;
                                    cell(ui, (int)row, TAS_GRID_PAD_BASE + (int)(player * 8 + bit),
                                         set ? pad_names[bit] : "", cell_width, set, writable,
                                         editor->cursor == index &&
                                             editor->column == TAS_GRID_PAD_BASE + (int)(player * 8 + bit));
                                }
                            } else {
                                const NesFm2Zapper *zapper = &frame->zappers[player];
                                if (movie->ports[player] == NES_FM2_PORT_ZAPPER) {
                                    snprintf(value, sizeof(value), "%u,%u:%u", zapper->x, zapper->y, zapper->button);
                                } else {
                                    value[0] = '\0';
                                }
                                fixed_label(ui, value, pad_widths[n], muted);
                            }
                        }
                        clipped_label(ui, marker_at(project, index), note_width, ink);
                    }
                }
            }
            Clay_ElementId scroll = desktop_clay_hit(ui->clay, HIT_TAS_SCROLL, 0, 0);
            float height = visible * 21.0f;
            float thumb = frames > visible ? height * visible / frames : height;
            if (thumb < 16) {
                thumb = 16;
            }
            float offset = max_scroll ? (height - thumb) * editor->scroll / max_scroll : 0;
            CLAY(scroll, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(12), .height = CLAY_SIZING_FIXED(height)},
                                     .layoutDirection = CLAY_TOP_TO_BOTTOM,
                                     .padding = {2, 2, 0, 0}},
                          .backgroundColor = raised}) {
                if (offset > 0) {
                    CLAY_AUTO_ID({.layout = {.sizing = {.height = CLAY_SIZING_FIXED(offset)}}}) {
                    }
                }
                CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW(), .height = CLAY_SIZING_FIXED(thumb)}},
                              .backgroundColor = muted}) {
                }
            }
        }
    }
}

void desktop_tas_layout(FrontendDesktopUi *ui, float width, float height) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    if (!editor) {
        return;
    }
    const NesTasProject *project = desktop_tas_project_const(ui);
    NesTasProgress progress = {0};
    NesTasSession *session = desktop_tas_session(ui);
    if (session) {
        nes_tas_session_progress(session, &progress);
    }
    bool writable = desktop_tas_project_writable(ui, false) != NULL;
    float timeline_height = height > 80 ? height - 80 : 63;
    unsigned rows = timeline_height > 42 ? (unsigned)((timeline_height - 42) / 21) : 1;
    if (rows > 50) {
        rows = 50;
    }
    if (!rows) {
        rows = 1;
    }
    editor->visible_rows = rows;
    desktop_tas_follow_playback(ui, &progress);
    const float sidebar_width = 246;
    float timeline_width = width - sidebar_width - 6;
    if (timeline_width < 1) {
        timeline_width = 1;
    }
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .childGap = 5,
                             .sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(height)}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        file_toolbar(ui, &progress, writable);
        playback_toolbar(ui, &progress);
        CLAY_AUTO_ID(
            {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_GROW()}, .childGap = 6}}) {
            if (project && nes_tas_project_frame_count(project)) {
                timeline(ui, project, &progress, timeline_width, rows, writable);
            } else {
                CLAY_AUTO_ID(
                    {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(timeline_width), .height = CLAY_SIZING_GROW()},
                                .padding = {10, 10, 10, 10}},
                     .backgroundColor = surface}) {
                    text(ui,
                         project ? "Insert a frame or enable recording to begin the timeline."
                                 : "Open an FM2, FM3 or CTAS file, or create a project for the loaded game.",
                         13, muted);
                }
            }
            sidebar(ui, project, &progress, writable, sidebar_width);
        }
        char status[256];
        snprintf(status, sizeof(status),
                 "Frame %zu/%zu | Cursor %zu | Selected %zu | Lag %zu | Rerecords %llu | States %zu%s", progress.frame,
                 progress.total_frames, editor->cursor, project ? nes_tas_selection_count(project) : 0,
                 progress.lag_count, (unsigned long long)progress.rerecord_count, progress.checkpoint_count,
                 progress.seeking ? " | Seeking" : "");
        text(ui, status, 11, progress.seeking ? accent : muted);
    }
}
