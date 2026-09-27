/*
 * desktop_tas_navigation.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native TAS navigation bookmarks. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"

#include <stdio.h>

static uint64_t generation(FrontendDesktopUi *ui) {
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    return progress.generation;
}

void desktop_tas_navigation_observe(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    uint64_t revision = nes_tas_project_revision(project);
    size_t count = nes_tas_navigation_count(project);
    if (!editor->navigation_initialized || editor->navigation_project != project ||
        editor->navigation_revision != revision || editor->navigation_count != count ||
        editor->navigation_generation != generation(ui)) {
        editor->navigation_selection = SIZE_MAX;
        editor->navigation_scroll = 0;
        editor->navigation_details = false;
        editor->navigation_detail_page = 0;
        editor->navigation_detail_pages = 0;
    }
    editor->navigation_initialized = true;
    editor->navigation_project = project;
    editor->navigation_revision = revision;
    editor->navigation_generation = generation(ui);
    editor->navigation_count = count;
}

static bool current_list(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    return project && editor->navigation_initialized && editor->navigation_project == project &&
           editor->navigation_generation == generation(ui) &&
           editor->navigation_revision == nes_tas_project_revision(project) &&
           editor->navigation_count == nes_tas_navigation_count(project);
}

static bool ready(FrontendDesktopUi *ui, bool edit) {
    NesTasSession *session = desktop_tas_session(ui);
    NesTasProgress progress = {0};
    nes_tas_session_progress(session, &progress);
    if (!progress.active || progress.recording || progress.frame_in_progress || progress.seeking ||
        nes_tas_edit_active(desktop_tas_project_const(ui))) {
        desktop_tas_status(ui, "Finish the current take, frame or edit before using bookmarks.");
        return false;
    }
    return !edit || desktop_tas_project_writable(ui, true) != NULL;
}

static void select_bookmark(FrontendDesktopUi *ui, size_t index) {
    desktop_tas_navigation_observe(ui);
    ui->tas_editor->navigation_selection = index;
    ui->tas_editor->navigation_detail_page = 0;
    ui->tas_editor->navigation_scroll = (index / TAS_NAVIGATION_VISIBLE) * TAS_NAVIGATION_VISIBLE;
}

static void start_edit(FrontendDesktopUi *ui, unsigned control, size_t index, size_t frame, const char *text) {
    DesktopTasEditor *editor = ui->tas_editor;
    editor->navigation_edit_project = desktop_tas_project_const(ui);
    editor->navigation_edit_revision = nes_tas_project_revision(editor->navigation_edit_project);
    editor->navigation_edit_generation = generation(ui);
    editor->navigation_edit_control = control;
    editor->navigation_edit_index = index;
    editor->navigation_edit_frame = frame;
    desktop_start_text_edit(ui, control, text);
}

static void seek_bookmark(FrontendDesktopUi *ui, size_t index) {
    NesTasNavigationView entry;
    if (!ready(ui, false) || !nes_tas_navigation(desktop_tas_project_const(ui), index, &entry)) {
        return;
    }
    char error[256] = {0};
    size_t frame = entry.frame;
    if (!tas_frontend_seek(ui->execution, frame, error, sizeof(error))) {
        desktop_tas_status(ui, error);
        return;
    }
    /* The grid has input rows only. Playback still seeks the exact boundary,
     * including frame_count, which is displayed in the bookmark details. */
    ui->tas_editor->cursor = frame;
    ui->tas_editor->anchor_valid = false;
    desktop_tas_keep_cursor_visible(ui);
    select_bookmark(ui, index);
    char message[96];
    snprintf(message, sizeof(message), "%s bookmark boundary %zu.",
             nes_tas_session_seeking(desktop_tas_session(ui)) ? "Seeking" : "Playback moved to", frame);
    desktop_tas_status(ui, message);
}

void desktop_tas_navigation_action(FrontendDesktopUi *ui, unsigned action) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (action == TAS_ACTION_SIDEBAR_NAVIGATION) {
        editor->sidebar_tab = 5;
        return;
    }
    if (!current_list(ui)) {
        desktop_tas_status(ui, "Bookmarks changed. Select an entry from the refreshed list.");
        return;
    }
    if (action >= TAS_ACTION_NAVIGATION_SELECT_BASE &&
        action < TAS_ACTION_NAVIGATION_SELECT_BASE + TAS_NAVIGATION_VISIBLE) {
        size_t index = editor->navigation_scroll + action - TAS_ACTION_NAVIGATION_SELECT_BASE;
        if (index < editor->navigation_count) {
            editor->navigation_selection = index;
            editor->navigation_detail_page = 0;
        }
        return;
    }
    if (action >= TAS_ACTION_NAVIGATION_DETAILS && action <= TAS_ACTION_NAVIGATION_DETAIL_NEXT) {
        if (editor->navigation_selection >= editor->navigation_count) {
            return;
        }
        if (action == TAS_ACTION_NAVIGATION_DETAILS) {
            editor->navigation_details = true;
            editor->navigation_detail_page = 0;
        } else if (action == TAS_ACTION_NAVIGATION_LIST) {
            editor->navigation_details = false;
        } else if (editor->navigation_details && action == TAS_ACTION_NAVIGATION_DETAIL_PREV) {
            if (editor->navigation_detail_page) {
                --editor->navigation_detail_page;
            }
        } else if (editor->navigation_details && editor->navigation_detail_page + 1 < editor->navigation_detail_pages) {
            ++editor->navigation_detail_page;
        }
        return;
    }
    if (action == TAS_ACTION_NAVIGATION_OLDER) {
        editor->navigation_scroll = editor->navigation_scroll >= TAS_NAVIGATION_VISIBLE
                                        ? editor->navigation_scroll - TAS_NAVIGATION_VISIBLE
                                        : 0;
        return;
    }
    if (action == TAS_ACTION_NAVIGATION_NEWER) {
        if (editor->navigation_scroll + TAS_NAVIGATION_VISIBLE < editor->navigation_count) {
            editor->navigation_scroll += TAS_NAVIGATION_VISIBLE;
        }
        return;
    }
    NesTasProgress progress = {0};
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    if (action == TAS_ACTION_NAVIGATION_PREV || action == TAS_ACTION_NAVIGATION_NEXT) {
        size_t index;
        if (nes_tas_navigation_neighbor(project, progress.frame, action == TAS_ACTION_NAVIGATION_NEXT, &index)) {
            seek_bookmark(ui, index);
        } else {
            desktop_tas_status(ui, "No bookmark in that direction from playback.");
        }
        return;
    }
    size_t index = editor->navigation_selection;
    if (action == TAS_ACTION_NAVIGATION_GOTO) {
        seek_bookmark(ui, index);
        return;
    }
    if (!ready(ui, true)) {
        return;
    }
    if (action == TAS_ACTION_NAVIGATION_CURSOR || action == TAS_ACTION_NAVIGATION_PLAYBACK) {
        size_t frame = action == TAS_ACTION_NAVIGATION_CURSOR ? editor->cursor : progress.frame;
        char name[64];
        snprintf(name, sizeof(name), "Frame %zu", frame);
        start_edit(ui, TAS_EDIT_NAVIGATION_ADD, SIZE_MAX, frame, name);
        return;
    }
    NesTasNavigationView entry;
    if (!nes_tas_navigation(project, index, &entry)) {
        desktop_tas_status(ui, "Select a bookmark first.");
        return;
    }
    if (action == TAS_ACTION_NAVIGATION_NAME || action == TAS_ACTION_NAVIGATION_NOTE) {
        bool note = action == TAS_ACTION_NAVIGATION_NOTE;
        start_edit(ui, note ? TAS_EDIT_NAVIGATION_NOTE : TAS_EDIT_NAVIGATION_NAME, index, entry.frame,
                   note ? entry.note : entry.name);
    } else if (action == TAS_ACTION_NAVIGATION_REMOVE) {
        NesTasResult result = nes_tas_navigation_remove(desktop_tas_project_writable(ui, false), index);
        desktop_tas_after_model_edit(ui, result, "Bookmark deleted.");
        desktop_tas_navigation_observe(ui);
    }
}

bool desktop_tas_navigation_commit(FrontendDesktopUi *ui, const char *text, char *error, size_t size) {
    DesktopTasEditor *editor = ui->tas_editor;
    NesTasProject *project = desktop_tas_project_writable(ui, false);
    if (!project || nes_tas_edit_active(project)) {
        if (error && size) {
            snprintf(error, size, "Pause playback and finish recording or editing before changing bookmarks.");
        }
        return false;
    }
    if (editor->navigation_edit_project != project || editor->navigation_edit_generation != generation(ui) ||
        editor->navigation_edit_revision != nes_tas_project_revision(project) ||
        editor->navigation_edit_control != ui->edit_control) {
        if (error && size) {
            snprintf(error, size, "The project changed. Cancel this dialog and select the bookmark again.");
        }
        return false;
    }
    NesTasResult result = NES_TAS_INVALID_ARGUMENT;
    size_t index = editor->navigation_edit_index;
    if (ui->edit_control == TAS_EDIT_NAVIGATION_ADD) {
        result = nes_tas_navigation_add(project, editor->navigation_edit_frame, text, "", &index);
    } else {
        NesTasNavigationView entry;
        if (nes_tas_navigation(project, index, &entry)) {
            bool note = ui->edit_control == TAS_EDIT_NAVIGATION_NOTE;
            result = nes_tas_navigation_rename(project, index, note ? entry.name : text, note ? text : entry.note);
        }
    }
    if (result != NES_TAS_OK) {
        if (error && size) {
            snprintf(error, size, "%s", nes_tas_result_string(result));
        }
        return false;
    }
    editor->navigation_edit_control = 0;
    desktop_tas_after_model_edit(ui, NES_TAS_OK, "Bookmark updated.");
    select_bookmark(ui, index);
    return true;
}
