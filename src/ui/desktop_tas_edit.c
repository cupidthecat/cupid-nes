/*
 * desktop_tas_edit.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Desktop TAS editor actions and gestures. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"
#include "state_frontend.h"
#include "../replay/tas_script.h"
#include "../system/vs_system.h"
#include "../util/file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t patterns[][4] = {{1, 0, 0, 0}, {0, 1, 0, 1}, {1, 1, 0, 0}, {1, 1, 1, 0}};

static const size_t pattern_lengths[] = {2, 2, 4, 4};

static bool session_progress(FrontendDesktopUi *ui, NesTasProgress *progress) {
    NesTasSession *session = desktop_tas_session(ui);
    if (!session || !progress) {
        return false;
    }
    nes_tas_session_progress(session, progress);
    return progress->active;
}

static void report_frontend(FrontendDesktopUi *ui, bool ok, const char *error, const char *success) {
    desktop_tas_status(ui, ok ? success : error && *error ? error : "TAS action failed.");
}

static bool project_path(const char *path) {
    if (!path) {
        return false;
    }
    const char *extension = strrchr(path, '.');
    return extension && (!SDL_strcasecmp(extension, ".ctas"));
}

static bool selected_range(const NesTasProject *project, size_t cursor, size_t *first, size_t *last) {
    if (project && nes_tas_selection_bounds(project, first, last)) {
        return true;
    }
    if (!project || cursor >= nes_tas_project_frame_count(project)) {
        return false;
    }
    *first = *last = cursor;
    return true;
}

static const char *marker_note(const NesTasProject *project, size_t frame) {
    if (!project) {
        return "";
    }
    size_t count = nes_tas_marker_count(project);
    for (size_t i = 0; i < count; ++i) {
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

static void choose_open(FrontendDesktopUi *ui, bool create) {
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0};
    char error[256] = {0};
    bool chosen = create
                      ? frontend_save_file_dialog(FRONTEND_SAVE_TAS_PROJECT, path, sizeof(path), error, sizeof(error))
                      : frontend_open_movie_dialog(path, sizeof(path), error, sizeof(error));
    if (!chosen) {
        if (error[0]) {
            desktop_tas_status(ui, error);
        }
        return;
    }
    bool opened = tas_frontend_open(ui->execution, path, create, error, sizeof(error));
    report_frontend(ui, opened, error, create ? "New TAS project created." : "TAS project opened.");
    if (opened && ui->tas_editor) {
        ui->tas_editor->cursor = ui->tas_editor->scroll = 0;
        ui->tas_editor->anchor_valid = false;
        ui->tas_editor->follow_initialized = false;
        ui->tas_editor->history_initialized = false;
        ui->tas_editor->navigation_initialized = false;
        ui->tas_editor->navigation_edit_control = 0;
    }
}

static void save_as(FrontendDesktopUi *ui, bool force_dialog) {
    NesTasProgress progress;
    if (!session_progress(ui, &progress)) {
        desktop_tas_status(ui, "Open or create a TAS project before saving.");
        return;
    }
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0};
    char error[256] = {0};
    const char *target = progress.path;
    if (force_dialog || !project_path(target)) {
        if (!frontend_save_file_dialog(FRONTEND_SAVE_TAS_PROJECT, path, sizeof(path), error, sizeof(error))) {
            if (error[0]) {
                desktop_tas_status(ui, error);
            }
            return;
        }
        target = path;
    }
    if (!project_path(target)) {
        desktop_tas_status(ui, "Use a .ctas filename to save the editable project, or choose Export for FM2/FM3.");
        return;
    }
    bool saved = tas_frontend_save(ui->execution, target, error, sizeof(error));
    report_frontend(ui, saved, error,
                    project_path(target) ? "TAS project saved." : "Movie exported; the editable project remains open.");
}

static void export_movie(FrontendDesktopUi *ui) {
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0};
    char error[256] = {0};
    if (!frontend_save_file_dialog(FRONTEND_SAVE_TAS_MOVIE, path, sizeof(path), error, sizeof(error))) {
        if (*error) {
            desktop_tas_status(ui, error);
        }
        return;
    }
    bool saved = tas_frontend_save(ui->execution, path, error, sizeof(error));
    report_frontend(ui, saved, error, "Movie exported. Save a CTAS project to retain the complete editing history.");
}

static void panel_toggle(FrontendDesktopUi *ui, unsigned id, int selected, const char *success) {
    char error[256] = {0};
    bool ok = frontend_panel_action(TAS_PANEL, id, NULL, selected, error, sizeof(error));
    report_frontend(ui, ok, error, success);
}

static void edit_player(FrontendDesktopUi *ui, unsigned player) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor || !project) {
        return;
    }
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    unsigned players = movie && movie->fourscore ? 4u : 2u;
    if (player >= players) {
        return;
    }
    editor->edit_player = player;
    editor->column = TAS_GRID_PAD_BASE + (int)(player * TAS_GRID_PAD_COLUMNS);
}

static void record_player(FrontendDesktopUi *ui, unsigned player) {
    NesTasProgress progress;
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!session_progress(ui, &progress) || !project) {
        return;
    }
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    unsigned allowed = movie && movie->fourscore ? 15u : 3u;
    unsigned bit = 1u << player;
    if (!(allowed & bit)) {
        return;
    }
    unsigned players = progress.record_players ^ bit;
    if (!players) {
        desktop_tas_status(ui, "At least one recording player must remain enabled.");
        return;
    }
    panel_toggle(ui, TAS_CONTROL_PLAYERS, (int)players, "Recording player selection updated.");
}

static void copy_or_edit(FrontendDesktopUi *ui, unsigned action) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    if (!editor || !project) {
        return;
    }
    NesTasResult result = NES_TAS_OK;
    const char *success = "TAS edit applied.";
    size_t count = nes_tas_project_frame_count(project);
    if (action == TAS_ACTION_COPY) {
        result = nes_tas_copy_selection(project);
        desktop_tas_status(ui, result == NES_TAS_OK ? "Selected frames copied." : nes_tas_result_string(result));
        return;
    }
    if (action == TAS_ACTION_CUT) {
        result = nes_tas_cut_selection(project);
        success = "Selected frame inputs cut.";
    } else if (action == TAS_ACTION_PASTE || action == TAS_ACTION_PASTE_INSERT) {
        result = nes_tas_paste(project, editor->cursor,
                               action == TAS_ACTION_PASTE_INSERT ? NES_TAS_PASTE_INSERT : NES_TAS_PASTE_OVERWRITE);
        success = action == TAS_ACTION_PASTE_INSERT ? "Clipboard inserted." : "Clipboard pasted.";
    } else if (action == TAS_ACTION_INSERT) {
        result = nes_tas_insert_frames(project, editor->cursor, 1);
        success = "Blank frame inserted.";
    } else if (action == TAS_ACTION_DELETE) {
        result = nes_tas_selection_count(project) ? nes_tas_delete_selection(project)
                 : editor->cursor < count         ? nes_tas_delete_frames(project, editor->cursor, 1)
                                                  : NES_TAS_RANGE_ERROR;
        success = "Frames deleted.";
    } else if (action == TAS_ACTION_CLONE) {
        result = nes_tas_selection_count(project) ? nes_tas_clone_selection(project)
                 : editor->cursor < count         ? nes_tas_clone_frames(project, editor->cursor, 1)
                                                  : NES_TAS_RANGE_ERROR;
        success = "Frames cloned.";
    } else if (action == TAS_ACTION_TRUNCATE) {
        size_t keep = count && editor->cursor < count ? editor->cursor + 1 : count;
        result = nes_tas_truncate(project, keep);
        success = "Timeline truncated after the cursor.";
    } else if (action == TAS_ACTION_UNDO) {
        result = nes_tas_project_undo(project);
        success = "TAS edit undone.";
    } else if (action == TAS_ACTION_REDO) {
        result = nes_tas_project_redo(project);
        success = "TAS edit redone.";
    }
    desktop_tas_after_model_edit(ui, result, success);
}

static void apply_pattern(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    if (!editor || !project) {
        return;
    }
    size_t first, last;
    if (!selected_range(project, editor->cursor, &first, &last)) {
        desktop_tas_status(ui, "Select at least one frame before applying a pattern.");
        return;
    }
    unsigned pattern = editor->pattern % (sizeof(patterns) / sizeof(patterns[0]));
    NesTasResult result =
        nes_tas_apply_pattern(project, first, last, editor->edit_player, editor->active_button, patterns[pattern],
                              pattern_lengths[pattern], ui->execution->tas_skip_lag);
    desktop_tas_after_model_edit(ui, result, "Pattern applied to the selected frames.");
}

static void toggle_hold(FrontendDesktopUi *ui, bool autofire) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasSession *session = desktop_tas_session(ui);
    if (!editor || !session || !nes_tas_session_active(session)) {
        desktop_tas_status(ui, "Open a TAS session before setting recording input helpers.");
        return;
    }
    uint8_t old = autofire ? nes_tas_session_autofire(session, editor->edit_player)
                           : nes_tas_session_hold(session, editor->edit_player);
    uint8_t value = old ^ editor->active_button;
    NesMovieResult result = autofire ? nes_tas_session_set_autofire(session, editor->edit_player, value, 1, 1)
                                     : nes_tas_session_set_hold(session, editor->edit_player, value);
    desktop_tas_status(ui, result == NES_MOVIE_OK ? autofire ? "Autofire input updated." : "Held input updated."
                                                  : nes_tas_session_error(session));
}

static void marker_edit(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor || !project || !desktop_tas_project_writable(ui, true)) {
        return;
    }
    if (editor->cursor >= nes_tas_project_frame_count(project)) {
        desktop_tas_status(ui, "Move the cursor onto a frame before editing its marker.");
        return;
    }
    desktop_start_text_edit(ui, TAS_EDIT_MARKER, marker_note(project, editor->cursor));
}

static void branch_store(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor || !project || !desktop_tas_project_writable(ui, true)) {
        return;
    }
    NesTasBookmarkView bookmark = {0};
    (void)nes_tas_bookmark(project, editor->branch_slot, &bookmark);
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "Branch %u", editor->branch_slot + 1);
    desktop_start_text_edit(ui, TAS_EDIT_BRANCH,
                            bookmark.occupied && bookmark.name && *bookmark.name ? bookmark.name : fallback);
}

static void branch_load(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor || !project) {
        return;
    }
    NesTasBookmarkView bookmark = {0};
    if (!nes_tas_bookmark(project, editor->branch_slot, &bookmark) || !bookmark.occupied) {
        desktop_tas_status(ui, "The selected branch slot is empty.");
        return;
    }
    if (!desktop_tas_project_writable(ui, true)) {
        return;
    }
    char error[256] = {0};
    bool loaded = tas_frontend_bookmark_load(ui->execution, editor->branch_slot, error, sizeof(error));
    report_frontend(ui, loaded, error, "Branch loaded.");
    if (loaded) {
        editor->cursor = bookmark.key_frame;
        editor->anchor_valid = false;
        desktop_tas_keep_cursor_visible(ui);
    }
}

static void branch_clear(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    if (!editor || !project) {
        return;
    }
    desktop_tas_after_model_edit(ui, nes_tas_bookmark_clear(project, editor->branch_slot), "Branch slot cleared.");
}

static void cursor_to_frame(FrontendDesktopUi *ui, size_t frame) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!editor || !frames) {
        return;
    }
    editor->cursor = frame < frames ? frame : frames - 1;
    desktop_tas_keep_cursor_visible(ui);
}

static void seek_to_frame(FrontendDesktopUi *ui, size_t frame, const char *success) {
    if (!ui || !ui->execution) {
        return;
    }
    char error[256] = {0};
    bool sought = tas_frontend_seek(ui->execution, frame, error, sizeof(error));
    report_frontend(ui, sought, error, success);
    if (sought) {
        cursor_to_frame(ui, frame);
    }
}

static void marker_jump(FrontendDesktopUi *ui, bool next) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!editor || !frames) {
        return;
    }
    size_t target = next ? frames - 1 : 0;
    size_t count = nes_tas_marker_count(project);
    if (next) {
        for (size_t i = 0; i < count; ++i) {
            NesTasMarkerView marker;
            if (nes_tas_marker(project, i, &marker) && marker.frame > editor->cursor) {
                target = marker.frame;
                break;
            }
        }
    } else {
        for (size_t i = 0; i < count; ++i) {
            NesTasMarkerView marker;
            if (!nes_tas_marker(project, i, &marker)) {
                continue;
            }
            if (marker.frame >= editor->cursor) {
                break;
            }
            target = marker.frame;
        }
    }
    cursor_to_frame(ui, target);
}

static void select_between_markers(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!editor || !project || !frames) {
        return;
    }
    size_t center = editor->cursor < frames ? editor->cursor : frames - 1;
    size_t first = 0, end = frames;
    size_t markers = nes_tas_marker_count(project);
    for (size_t i = 0; i < markers; ++i) {
        NesTasMarkerView marker;
        if (!nes_tas_marker(project, i, &marker)) {
            continue;
        }
        if (marker.frame <= center) {
            first = marker.frame;
            continue;
        }
        end = marker.frame;
        break;
    }
    nes_tas_selection_clear(project);
    NesTasResult result = nes_tas_selection_set_range(project, first, end - 1, true);
    desktop_tas_status(ui, result == NES_TAS_OK ? "Selected frames between markers." : nes_tas_result_string(result));
}

static void select_all(FrontendDesktopUi *ui, bool selected) {
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!project) {
        return;
    }
    if (!selected) {
        nes_tas_selection_clear(project);
        desktop_tas_status(ui, "Selection cleared.");
        return;
    }
    if (frames) {
        NesTasResult result = nes_tas_selection_set_range(project, 0, frames - 1, true);
        desktop_tas_status(ui, result == NES_TAS_OK ? "All frames selected." : nes_tas_result_string(result));
    }
}

static bool pad_player_available(const NesFm2Movie *movie, unsigned player) {
    return movie && player < 4 && (movie->fourscore || (player < 2 && movie->ports[player] == NES_FM2_PORT_GAMEPAD));
}

static bool command_available(const NesFm2Movie *movie, int column) {
    int command = column - TAS_GRID_COMMAND_BASE;
    if (!movie || command < 0 || command >= TAS_GRID_COMMAND_COUNT) {
        return false;
    }
    return command < 2 || (command < 4 ? movie->fds : vs_enabled());
}

static void header_toggle(FrontendDesktopUi *ui, unsigned action) {
    unsigned offset = action - TAS_ACTION_HEADER_BASE;
    unsigned player = offset / TAS_GRID_PAD_COLUMNS;
    unsigned bit = offset % TAS_GRID_PAD_COLUMNS;
    uint8_t mask = (uint8_t)(1u << bit);
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    const NesFm2Movie *movie = project ? nes_tas_project_movie(project) : NULL;
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!editor || !project || !frames || !pad_player_available(movie, player)) {
        return;
    }

    size_t selected = nes_tas_selection_count(project);
    bool pressed = false;
    if (selected) {
        for (size_t frame = 0; frame < frames; ++frame) {
            if (!nes_tas_selection_contains(project, frame)) {
                continue;
            }
            const NesFm2Frame *input = nes_tas_project_frame(project, frame);
            if (input && !(input->pads[player] & mask)) {
                pressed = true;
                break;
            }
        }
    } else {
        size_t frame = editor->cursor < frames ? editor->cursor : frames - 1;
        const NesFm2Frame *input = nes_tas_project_frame(project, frame);
        pressed = input && !(input->pads[player] & mask);
    }

    NesTasResult result = nes_tas_edit_begin(project);
    if (result != NES_TAS_OK) {
        desktop_tas_status(ui, nes_tas_result_string(result));
        return;
    }
    if (selected) {
        for (size_t frame = 0; frame < frames && result == NES_TAS_OK; ++frame) {
            if (nes_tas_selection_contains(project, frame)) {
                result = nes_tas_paint_button(project, frame, frame, player, mask, pressed);
            }
        }
    } else {
        size_t frame = editor->cursor < frames ? editor->cursor : frames - 1;
        result = nes_tas_paint_button(project, frame, frame, player, mask, pressed);
    }
    if (result != NES_TAS_OK) {
        nes_tas_edit_cancel(project);
        desktop_tas_status(ui, nes_tas_result_string(result));
        return;
    }
    result = nes_tas_edit_end(project);
    editor->edit_player = player;
    editor->active_button = mask;
    editor->column = TAS_GRID_PAD_BASE + (int)(player * TAS_GRID_PAD_COLUMNS + bit);
    desktop_tas_after_model_edit(ui, result, pressed ? "Selected input column set." : "Selected input column cleared.");
}

static void run_script(FrontendDesktopUi *ui) {
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    if (!project) {
        return;
    }
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0}, error[256] = {0};
    if (!frontend_open_file_dialog(FRONTEND_OPEN_SCRIPT, path, sizeof(path), error, sizeof(error))) {
        if (*error) {
            desktop_tas_status(ui, error);
        }
        return;
    }
    uint8_t *source = NULL;
    size_t size = 0;
    NesFileResult file = nes_file_read_all(path, 1024u * 1024u, &source, &size);
    if (file != NES_FILE_OK) {
        desktop_tas_status(ui, nes_file_result_message(file));
        return;
    }
    bool ok = nes_tas_run_script(project, (const char *)source, size, error, sizeof(error));
    free(source);
    if (ok) {
        desktop_tas_after_model_edit(ui, NES_TAS_OK, "TAS Lua script applied as one undoable edit.");
    } else {
        desktop_tas_status(ui, error);
    }
}

static void cache_action(FrontendDesktopUi *ui, unsigned action) {
    NesTasSession *session = desktop_tas_session(ui);
    DesktopTasEditor *editor = ui->tas_editor;
    if (!session || !nes_tas_session_active(session) || !editor) {
        return;
    }
    NesTasCacheInfo cache;
    nes_tas_session_cache_info(session, &cache);
    if (action == TAS_ACTION_CACHE_CLEAR || action == TAS_ACTION_CACHE_CLEAR_AFTER) {
        NesMovieResult result =
            nes_tas_session_clear_cache(session, action == TAS_ACTION_CACHE_CLEAR ? 0 : editor->cursor);
        desktop_tas_status(ui, result != NES_MOVIE_OK             ? nes_tas_session_error(session)
                               : action == TAS_ACTION_CACHE_CLEAR ? "Checkpoint cache cleared; startup state retained."
                                                                  : "Checkpoints after the cursor cleared.");
    } else if (action == TAS_ACTION_CACHE_MB || action == TAS_ACTION_CACHE_INTERVAL) {
        char value[32];
        bool budget = action == TAS_ACTION_CACHE_MB;
        snprintf(value, sizeof(value), "%zu", budget ? cache.byte_limit / (1024u * 1024u) : cache.interval);
        desktop_start_text_edit(ui, budget ? TAS_EDIT_CACHE_MB : TAS_EDIT_CACHE_INTERVAL, value);
    } else {
        size_t target = action == TAS_ACTION_CACHE_PREV ? 0 : SIZE_MAX;
        for (size_t i = 0; i < cache.checkpoint_count; ++i) {
            NesTasCacheEntry entry;
            if (!nes_tas_session_cache_entry(session, i, &entry)) {
                continue;
            }
            if (action == TAS_ACTION_CACHE_PREV && entry.frame < editor->cursor && entry.frame > target) {
                target = entry.frame;
            } else if (action == TAS_ACTION_CACHE_NEXT && entry.frame > editor->cursor && entry.frame < target) {
                target = entry.frame;
            }
        }
        if (target != SIZE_MAX) {
            cursor_to_frame(ui, target);
        } else {
            desktop_tas_status(ui, "No later checkpoint is cached.");
        }
    }
}

static void history_action(FrontendDesktopUi *ui, unsigned action) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    NesTasHistoryInfo info;
    nes_tas_project_history_info(project, &info);
    if (!project || !editor->history_initialized || editor->history_project != project ||
        editor->history_revision != nes_tas_project_revision(project) ||
        editor->history_snapshot.operations != info.operations || editor->history_snapshot.position != info.position) {
        desktop_tas_status(ui, "History changed. Select a position from the refreshed list.");
        return;
    }
    if (action >= TAS_ACTION_HISTORY_SELECT_BASE && action < TAS_ACTION_HISTORY_SELECT_BASE + TAS_HISTORY_VISIBLE) {
        size_t position = editor->history_scroll + action - TAS_ACTION_HISTORY_SELECT_BASE;
        if (position <= info.operations) {
            editor->history_selection = position;
        }
    } else if (action == TAS_ACTION_HISTORY_OLDER) {
        editor->history_scroll =
            editor->history_scroll >= TAS_HISTORY_VISIBLE ? editor->history_scroll - TAS_HISTORY_VISIBLE : 0;
    } else if (action == TAS_ACTION_HISTORY_NEWER) {
        if (info.operations - editor->history_scroll >= TAS_HISTORY_VISIBLE) {
            editor->history_scroll += TAS_HISTORY_VISIBLE;
        }
    } else if (action == TAS_ACTION_HISTORY_CURRENT) {
        editor->history_selection = info.position;
        editor->history_scroll = (info.position / TAS_HISTORY_VISIBLE) * TAS_HISTORY_VISIBLE;
    } else if (action == TAS_ACTION_HISTORY_RESTORE) {
        NesTasProject *writable = desktop_tas_project_writable(ui, true);
        if (writable) {
            NesTasResult result = nes_tas_project_history_seek(writable, editor->history_selection);
            desktop_tas_after_model_edit(ui, result, "Selected history position restored.");
        }
    }
}

void desktop_tas_action(FrontendDesktopUi *ui, unsigned action) {
    desktop_tas_finish_paint(ui);
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProgress progress = {0};
    bool active = session_progress(ui, &progress);
    if (!editor) {
        return;
    }
    if (action >= TAS_ACTION_SPLICE_OPEN && action <= TAS_ACTION_SPLICE_EXTRACT) {
        desktop_tas_splice_action(ui, action);
        return;
    }
    if (action >= TAS_ACTION_HISTORY_SELECT_BASE && action < TAS_ACTION_HISTORY_SELECT_BASE + TAS_HISTORY_VISIBLE) {
        history_action(ui, action);
        return;
    }
    if ((action >= TAS_ACTION_SIDEBAR_NAVIGATION && action <= TAS_ACTION_NAVIGATION_DETAIL_NEXT) ||
        (action >= TAS_ACTION_NAVIGATION_SELECT_BASE &&
         action < TAS_ACTION_NAVIGATION_SELECT_BASE + TAS_NAVIGATION_VISIBLE)) {
        desktop_tas_navigation_action(ui, action);
        return;
    }
    if (action >= TAS_ACTION_HEADER_BASE && action < TAS_ACTION_HEADER_BASE + 4 * TAS_GRID_PAD_COLUMNS) {
        header_toggle(ui, action);
        return;
    }
    if (action >= TAS_ACTION_ACTIVE_BUTTON_BASE && action < TAS_ACTION_ACTIVE_BUTTON_BASE + TAS_GRID_PAD_COLUMNS) {
        const NesTasProject *project = desktop_tas_project_const(ui);
        const NesFm2Movie *movie = project ? nes_tas_project_movie(project) : NULL;
        unsigned bit = action - TAS_ACTION_ACTIVE_BUTTON_BASE;
        if (pad_player_available(movie, editor->edit_player)) {
            editor->active_button = (uint8_t)(1u << bit);
            editor->column = TAS_GRID_PAD_BASE + (int)(editor->edit_player * TAS_GRID_PAD_COLUMNS + bit);
        }
        return;
    }
    if (action >= TAS_ACTION_MARKER_GOTO_BASE) {
        const NesTasProject *project = desktop_tas_project_const(ui);
        NesTasMarkerView marker;
        size_t index = action - TAS_ACTION_MARKER_GOTO_BASE;
        if (project && nes_tas_marker(project, index, &marker)) {
            cursor_to_frame(ui, marker.frame);
        }
        return;
    }
    if (action >= TAS_ACTION_EDIT_PLAYER_BASE && action < TAS_ACTION_EDIT_PLAYER_BASE + 4) {
        edit_player(ui, action - TAS_ACTION_EDIT_PLAYER_BASE);
        return;
    }
    if (action >= TAS_ACTION_RECORD_PLAYER_BASE && action < TAS_ACTION_RECORD_PLAYER_BASE + 4) {
        record_player(ui, action - TAS_ACTION_RECORD_PLAYER_BASE);
        return;
    }
    if (action >= TAS_ACTION_BRANCH_SLOT_BASE && action < TAS_ACTION_BRANCH_SLOT_BASE + NES_TAS_BOOKMARK_COUNT) {
        editor->branch_slot = action - TAS_ACTION_BRANCH_SLOT_BASE;
        return;
    }
    switch (action) {
    case TAS_ACTION_OPEN:
        choose_open(ui, false);
        break;
    case TAS_ACTION_NEW:
        choose_open(ui, true);
        break;
    case TAS_ACTION_SAVE:
        save_as(ui, false);
        break;
    case TAS_ACTION_SAVE_AS:
        save_as(ui, true);
        break;
    case TAS_ACTION_EXPORT:
        export_movie(ui);
        break;
    case TAS_ACTION_SCRIPT:
        run_script(ui);
        break;
    case TAS_ACTION_FOLLOW:
        editor->follow_playback = !editor->follow_playback;
        editor->follow_initialized = false;
        desktop_tas_status(ui, editor->follow_playback ? "Playback follow enabled." : "Playback follow disabled.");
        break;
    case TAS_ACTION_SEEK_CURSOR:
        if (active) {
            seek_to_frame(ui, editor->cursor, "Playback moved to the grid cursor.");
        }
        break;
    case TAS_ACTION_CURSOR_PLAYBACK:
        if (active) {
            cursor_to_frame(ui, progress.frame);
        }
        break;
    case TAS_ACTION_MARKER_PREV:
        marker_jump(ui, false);
        break;
    case TAS_ACTION_MARKER_NEXT:
        marker_jump(ui, true);
        break;
    case TAS_ACTION_SELECT_BETWEEN_MARKERS:
        select_between_markers(ui);
        break;
    case TAS_ACTION_INSERT_MANY:
        if (desktop_tas_project_writable(ui, true)) {
            desktop_start_text_edit(ui, TAS_EDIT_INSERT, "1");
        }
        break;
    case TAS_ACTION_SELECT_ALL:
        select_all(ui, true);
        break;
    case TAS_ACTION_SELECT_NONE:
        select_all(ui, false);
        break;
    case TAS_ACTION_SIDEBAR_BRANCHES:
    case TAS_ACTION_SIDEBAR_MARKERS:
    case TAS_ACTION_SIDEBAR_INPUT:
        editor->sidebar_tab = action - TAS_ACTION_SIDEBAR_BRANCHES;
        break;
    case TAS_ACTION_SIDEBAR_CACHE:
        editor->sidebar_tab = 3;
        break;
    case TAS_ACTION_SIDEBAR_HISTORY:
        editor->sidebar_tab = 4;
        break;
    case TAS_ACTION_HISTORY_OLDER:
    case TAS_ACTION_HISTORY_NEWER:
    case TAS_ACTION_HISTORY_CURRENT:
    case TAS_ACTION_HISTORY_RESTORE:
        history_action(ui, action);
        break;
    case TAS_ACTION_CACHE_CLEAR:
    case TAS_ACTION_CACHE_CLEAR_AFTER:
    case TAS_ACTION_CACHE_MB:
    case TAS_ACTION_CACHE_INTERVAL:
    case TAS_ACTION_CACHE_PREV:
    case TAS_ACTION_CACHE_NEXT:
        cache_action(ui, action);
        break;
    case TAS_ACTION_MARKER_REMOVE: {
        NesTasProject *project = desktop_tas_project_writable(ui, true);
        if (project) {
            desktop_tas_after_model_edit(ui, nes_tas_marker_remove(project, editor->cursor), "Marker removed.");
        }
        break;
    }
    case TAS_ACTION_BRANCH_JUMP: {
        const NesTasProject *project = desktop_tas_project_const(ui);
        NesTasBookmarkView bookmark = {0};
        if (!project || !nes_tas_bookmark(project, editor->branch_slot, &bookmark) || !bookmark.occupied) {
            desktop_tas_status(ui, "The selected branch slot is empty.");
        } else if (active) {
            seek_to_frame(ui, bookmark.key_frame, "Playback moved to the branch keyframe.");
        }
        break;
    }
    case TAS_ACTION_STOP: {
        char error[256] = {0};
        bool stopped = active && frontend_execution_movie_stop(ui->execution, error, sizeof(error));
        report_frontend(ui, stopped, error, "TAS session closed and live game restored.");
        break;
    }
    case TAS_ACTION_DISCARD: {
        char error[256] = {0};
        bool discarded = active && tas_frontend_discard(ui->execution, error, sizeof(error));
        report_frontend(ui, discarded, error, "Unsaved TAS edits discarded and live game restored.");
        break;
    }
    case TAS_ACTION_PAUSE:
        (void)desktop_invoke_command(ui, FRONTEND_COMMAND_PAUSE);
        break;
    case TAS_ACTION_FRAME:
        (void)desktop_invoke_command(ui, FRONTEND_COMMAND_FRAME_ADVANCE);
        break;
    case TAS_ACTION_BACK:
        if (active) {
            char error[256] = {0};
            size_t frame = progress.frame ? progress.frame - 1 : 0;
            bool sought = tas_frontend_seek(ui->execution, frame, error, sizeof(error));
            report_frontend(ui, sought, error, "Moved back one TAS frame.");
            if (sought) {
                editor->cursor = frame;
                desktop_tas_keep_cursor_visible(ui);
            }
        }
        break;
    case TAS_ACTION_SEEK: {
        char frame[32];
        snprintf(frame, sizeof(frame), "%zu", active ? progress.frame : editor->cursor);
        desktop_start_text_edit(ui, TAS_EDIT_SEEK, frame);
        break;
    }
    case TAS_ACTION_READ_ONLY:
        if (active) {
            panel_toggle(ui, TAS_CONTROL_READ_ONLY, !progress.read_only,
                         progress.read_only ? "Read-only mode disabled." : "Read-only mode enabled.");
        }
        break;
    case TAS_ACTION_RECORD:
        if (active) {
            panel_toggle(ui, TAS_CONTROL_RECORD, !progress.recording,
                         progress.recording ? "Recording stopped." : "Recording enabled.");
        }
        break;
    case TAS_ACTION_RECORD_MODE:
        if (active) {
            panel_toggle(ui, TAS_CONTROL_RECORD_MODE,
                         progress.record_mode == NES_TAS_RECORD_INSERT ? NES_TAS_RECORD_OVERWRITE
                                                                       : NES_TAS_RECORD_INSERT,
                         "Recording mode updated.");
        }
        break;
    case TAS_ACTION_SKIP_LAG:
        if (ui->execution) {
            ui->execution->tas_skip_lag = !ui->execution->tas_skip_lag;
        }
        break;
    case TAS_ACTION_COPY:
    case TAS_ACTION_CUT:
    case TAS_ACTION_PASTE:
    case TAS_ACTION_PASTE_INSERT:
    case TAS_ACTION_INSERT:
    case TAS_ACTION_DELETE:
    case TAS_ACTION_CLONE:
    case TAS_ACTION_TRUNCATE:
    case TAS_ACTION_UNDO:
    case TAS_ACTION_REDO:
        copy_or_edit(ui, action);
        break;
    case TAS_ACTION_PATTERN:
        apply_pattern(ui);
        break;
    case TAS_ACTION_PATTERN_NEXT:
        editor->pattern = (editor->pattern + 1) % (sizeof(patterns) / sizeof(patterns[0]));
        break;
    case TAS_ACTION_HOLD:
        toggle_hold(ui, false);
        break;
    case TAS_ACTION_AUTOFIRE:
        toggle_hold(ui, true);
        break;
    case TAS_ACTION_MARKER:
        marker_edit(ui);
        break;
    case TAS_ACTION_BRANCH_STORE:
        branch_store(ui);
        break;
    case TAS_ACTION_BRANCH_LOAD:
        branch_load(ui);
        break;
    case TAS_ACTION_BRANCH_CLEAR:
        branch_clear(ui);
        break;
    default:
        break;
    }
}

void desktop_tas_select_row(FrontendDesktopUi *ui, size_t row, SDL_Keymod modifiers) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *view = desktop_tas_project_const(ui);
    if (!editor || !view || row >= nes_tas_project_frame_count(view)) {
        return;
    }
    editor->cursor = row;
    NesTasProject *project = desktop_tas_project_writable(ui, false);
    if (!project) {
        editor->anchor = row;
        editor->anchor_valid = true;
        desktop_tas_keep_cursor_visible(ui);
        return;
    }
    bool shift = (modifiers & KMOD_SHIFT) != 0;
    bool control = (modifiers & KMOD_CTRL) != 0;
    NesTasResult result = NES_TAS_OK;
    if (shift && editor->anchor_valid) {
        size_t first = editor->anchor < row ? editor->anchor : row;
        size_t last = editor->anchor < row ? row : editor->anchor;
        if (!control) {
            nes_tas_selection_clear(project);
        }
        result = nes_tas_selection_set_range(project, first, last, true);
    } else if (control) {
        result = nes_tas_selection_set(project, row, !nes_tas_selection_contains(project, row));
        editor->anchor = row;
        editor->anchor_valid = true;
    } else {
        nes_tas_selection_clear(project);
        result = nes_tas_selection_set(project, row, true);
        editor->anchor = row;
        editor->anchor_valid = true;
    }
    if (result != NES_TAS_OK) {
        desktop_tas_status(ui, nes_tas_result_string(result));
    }
    desktop_tas_keep_cursor_visible(ui);
}

static bool decode_pad_column(int column, unsigned *player, uint8_t *mask) {
    if (column < TAS_GRID_PAD_BASE || column >= TAS_GRID_PAD_BASE + 4 * TAS_GRID_PAD_COLUMNS) {
        return false;
    }
    unsigned index = (unsigned)(column - TAS_GRID_PAD_BASE);
    *player = index / TAS_GRID_PAD_COLUMNS;
    *mask = (uint8_t)(1u << (index % TAS_GRID_PAD_COLUMNS));
    return true;
}

static uint8_t command_mask(int column) {
    int index = column - TAS_GRID_COMMAND_BASE;
    return index >= 0 && index < TAS_GRID_COMMAND_COUNT ? (uint8_t)(1u << index) : 0;
}

void desktop_tas_cell_down(FrontendDesktopUi *ui, size_t row, int column) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, true);
    if (!editor || !project || row >= nes_tas_project_frame_count(project)) {
        return;
    }
    editor->cursor = row;
    editor->column = column;
    unsigned player;
    uint8_t mask;
    if (decode_pad_column(column, &player, &mask)) {
        const NesFm2Movie *movie = nes_tas_project_movie(project);
        if (!movie->fourscore && (player >= 2 || movie->ports[player] != NES_FM2_PORT_GAMEPAD)) {
            return;
        }
        const NesFm2Frame *frame = nes_tas_project_frame(project, row);
        if (!frame) {
            return;
        }
        editor->edit_player = player;
        editor->active_button = mask;
        NesTasResult begun = nes_tas_edit_begin(project);
        if (begun != NES_TAS_OK) {
            desktop_tas_status(ui, nes_tas_result_string(begun));
            return;
        }
        editor->painting = true;
        editor->paint_player = player;
        editor->paint_mask = mask;
        editor->paint_pressed = (frame->pads[player] & mask) == 0;
        editor->paint_last = row;
        NesTasResult painted = nes_tas_paint_button(project, row, row, player, mask, editor->paint_pressed);
        if (painted != NES_TAS_OK) {
            nes_tas_edit_cancel(project);
            editor->painting = false;
            desktop_tas_status(ui, nes_tas_result_string(painted));
        }
        return;
    }
    if (column == TAS_GRID_LAG) {
        NesTasLagState state = nes_tas_lag(project, row);
        state = state == NES_TAS_LAG_UNKNOWN ? NES_TAS_LAG_NO
                : state == NES_TAS_LAG_NO    ? NES_TAS_LAG_YES
                                             : NES_TAS_LAG_UNKNOWN;
        NesTasResult result = nes_tas_annotate_lag(project, row, state);
        desktop_tas_after_model_edit(ui, result, "Lag annotation updated.");
        return;
    }
    uint8_t command = command_mask(column);
    if (command) {
        const NesFm2Movie *movie = nes_tas_project_movie(project);
        if (!command_available(movie, column)) {
            return;
        }
        const NesFm2Frame *old = nes_tas_project_frame(project, row);
        if (!old) {
            return;
        }
        NesFm2Frame next = *old;
        next.commands ^= command;
        desktop_tas_after_model_edit(ui, nes_tas_set_frame(project, row, &next), "Frame command updated.");
    }
}

void desktop_tas_cell_drag(FrontendDesktopUi *ui, size_t row, int column) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasProject *project = desktop_tas_project_writable(ui, false);
    unsigned player;
    uint8_t mask;
    if (!editor || !project || !editor->painting || !decode_pad_column(column, &player, &mask) ||
        player != editor->paint_player || mask != editor->paint_mask || row >= nes_tas_project_frame_count(project) ||
        row == editor->paint_last) {
        return;
    }
    size_t first = row < editor->paint_last ? row : editor->paint_last;
    size_t last = row < editor->paint_last ? editor->paint_last : row;
    NesTasResult result = nes_tas_paint_button(project, first, last, player, mask, editor->paint_pressed);
    if (result != NES_TAS_OK) {
        nes_tas_edit_cancel(project);
        editor->painting = false;
        desktop_tas_status(ui, nes_tas_result_string(result));
        return;
    }
    editor->paint_last = row;
    editor->cursor = row;
    desktop_tas_keep_cursor_visible(ui);
}

void desktop_tas_finish_paint(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    if (!editor || !editor->painting) {
        return;
    }
    editor->painting = false;
    NesTasProject *project = desktop_tas_project_writable(ui, false);
    if (!project || !nes_tas_edit_active(project)) {
        return;
    }
    NesTasResult result = nes_tas_edit_end(project);
    desktop_tas_after_model_edit(ui, result, "Button paint applied.");
}

void desktop_tas_scroll_to(FrontendDesktopUi *ui, int y) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    SDL_FRect rect;
    if (!editor || !project || !desktop_clay_bounds(ui->clay, HIT_TAS_SCROLL, 0, 0, &rect) || rect.h <= 0) {
        return;
    }
    size_t frames = nes_tas_project_frame_count(project);
    size_t rows = editor->visible_rows ? editor->visible_rows : 1;
    size_t max_scroll = frames > rows ? frames - rows : 0;
    float position = (y - rect.y) / rect.h;
    if (position < 0) {
        position = 0;
    }
    if (position > 1) {
        position = 1;
    }
    editor->scroll = (size_t)(position * (float)max_scroll);
}

static void move_cursor(FrontendDesktopUi *ui, int delta, bool select) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor || !project) {
        return;
    }
    size_t frames = nes_tas_project_frame_count(project);
    if (!frames) {
        return;
    }
    size_t next = editor->cursor;
    if (delta < 0) {
        size_t amount = (size_t)-delta;
        next = next > amount ? next - amount : 0;
    } else if (delta > 0) {
        size_t amount = (size_t)delta;
        next = next + amount < next || next + amount >= frames ? frames - 1 : next + amount;
    }
    desktop_tas_select_row(ui, next, select ? KMOD_SHIFT : KMOD_NONE);
}

static void move_column(FrontendDesktopUi *ui, bool right) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    const NesFm2Movie *movie = project ? nes_tas_project_movie(project) : NULL;
    if (!editor || !movie) {
        return;
    }
    int columns[1 + TAS_GRID_COMMAND_COUNT + 4 * TAS_GRID_PAD_COLUMNS];
    size_t count = 0;
    columns[count++] = TAS_GRID_LAG;
    for (int command = 0; command < TAS_GRID_COMMAND_COUNT; ++command) {
        int column = TAS_GRID_COMMAND_BASE + command;
        if (command_available(movie, column)) {
            columns[count++] = column;
        }
    }
    for (unsigned player = 0; player < 4; ++player) {
        if (!pad_player_available(movie, player)) {
            continue;
        }
        for (int bit = TAS_GRID_PAD_COLUMNS - 1; bit >= 0; --bit) {
            columns[count++] = TAS_GRID_PAD_BASE + (int)(player * TAS_GRID_PAD_COLUMNS) + bit;
        }
    }
    size_t current = count;
    for (size_t i = 0; i < count; ++i) {
        if (columns[i] == editor->column) {
            current = i;
            break;
        }
    }
    if (current == count) {
        current = right ? 0 : count - 1;
    } else if (right && current + 1 < count) {
        ++current;
    } else if (!right && current) {
        --current;
    }
    editor->column = columns[current];
    unsigned player;
    uint8_t mask;
    if (decode_pad_column(editor->column, &player, &mask)) {
        editor->edit_player = player;
        editor->active_button = mask;
    }
}

bool desktop_tas_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *event) {
    if (!ui || !event) {
        return false;
    }
    SDL_Scancode key = event->keysym.scancode;
    SDL_Keymod mod = (SDL_Keymod)event->keysym.mod;
    if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_S) {
        desktop_tas_action(ui, (mod & KMOD_SHIFT) ? TAS_ACTION_SAVE_AS : TAS_ACTION_SAVE);
        return true;
    }
    if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_O) {
        desktop_tas_action(ui, TAS_ACTION_OPEN);
        return true;
    }
    if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_N) {
        desktop_tas_action(ui, TAS_ACTION_NEW);
        return true;
    }
    if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_G) {
        desktop_tas_action(ui, TAS_ACTION_SEEK);
        return true;
    }
    if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_A) {
        desktop_tas_action(ui, (mod & KMOD_SHIFT) ? TAS_ACTION_SELECT_ALL : TAS_ACTION_SELECT_BETWEEN_MARKERS);
        return true;
    }
    if (key == SDL_SCANCODE_ESCAPE) {
        ui->panel_open = false;
        return true;
    }
    if (key == SDL_SCANCODE_UP || key == SDL_SCANCODE_DOWN) {
        move_cursor(ui, key == SDL_SCANCODE_UP ? -1 : 1, (mod & KMOD_SHIFT) != 0);
    } else if (key == SDL_SCANCODE_PAGEUP || key == SDL_SCANCODE_PAGEDOWN) {
        if (mod & KMOD_CTRL) {
            desktop_tas_action(ui, key == SDL_SCANCODE_PAGEUP ? TAS_ACTION_MARKER_PREV : TAS_ACTION_MARKER_NEXT);
        } else {
            int page = (int)(ui->tas_editor && ui->tas_editor->visible_rows ? ui->tas_editor->visible_rows : 12);
            move_cursor(ui, key == SDL_SCANCODE_PAGEUP ? -page : page, (mod & KMOD_SHIFT) != 0);
        }
    } else if (key == SDL_SCANCODE_HOME) {
        DesktopTasEditor *editor = desktop_tas_editor(ui);
        if (editor) {
            desktop_tas_select_row(ui, 0, (mod & KMOD_SHIFT) ? KMOD_SHIFT : KMOD_NONE);
        }
    } else if (key == SDL_SCANCODE_END) {
        const NesTasProject *project = desktop_tas_project_const(ui);
        size_t frames = project ? nes_tas_project_frame_count(project) : 0;
        if (frames) {
            desktop_tas_select_row(ui, frames - 1, (mod & KMOD_SHIFT) ? KMOD_SHIFT : KMOD_NONE);
        }
    } else if (key == SDL_SCANCODE_LEFT || key == SDL_SCANCODE_RIGHT) {
        move_column(ui, key == SDL_SCANCODE_RIGHT);
    } else if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_C) {
        desktop_tas_action(ui, TAS_ACTION_COPY);
    } else if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_X) {
        desktop_tas_action(ui, TAS_ACTION_CUT);
    } else if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_V) {
        desktop_tas_action(ui, (mod & KMOD_SHIFT) ? TAS_ACTION_PASTE_INSERT : TAS_ACTION_PASTE);
    } else if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_Z) {
        desktop_tas_action(ui, (mod & KMOD_SHIFT) ? TAS_ACTION_REDO : TAS_ACTION_UNDO);
    } else if ((mod & KMOD_CTRL) && key == SDL_SCANCODE_Y) {
        desktop_tas_action(ui, TAS_ACTION_REDO);
    } else if (key == SDL_SCANCODE_INSERT) {
        desktop_tas_action(ui, TAS_ACTION_INSERT_MANY);
    } else if (key == SDL_SCANCODE_DELETE) {
        desktop_tas_action(ui, TAS_ACTION_DELETE);
    } else if (key == SDL_SCANCODE_RETURN || key == SDL_SCANCODE_KP_ENTER) {
        desktop_tas_action(ui, TAS_ACTION_SEEK_CURSOR);
    } else if (key == SDL_SCANCODE_SPACE) {
        DesktopTasEditor *editor = desktop_tas_editor(ui);
        if (editor) {
            desktop_tas_cell_down(ui, editor->cursor, editor->column);
        }
        desktop_tas_finish_paint(ui);
    } else if (key == SDL_SCANCODE_F2) {
        desktop_tas_action(ui, TAS_ACTION_MARKER);
    } else if (key == SDL_SCANCODE_F5 && !(mod & (KMOD_CTRL | KMOD_ALT | KMOD_GUI))) {
        desktop_tas_action(ui, TAS_ACTION_BRANCH_STORE);
    } else if (key == SDL_SCANCODE_F6 && !(mod & (KMOD_CTRL | KMOD_ALT | KMOD_GUI))) {
        desktop_tas_action(ui, TAS_ACTION_BRANCH_LOAD);
    } else {
        return false;
    }
    return true;
}
