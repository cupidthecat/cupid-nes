/* Desktop TAS piano-roll editor. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"
#include "host_input.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool desktop_tas_panel(unsigned id) {
    return id == TAS_PANEL;
}

const char *desktop_tas_edit_title(unsigned control) {
    switch (control) {
    case TAS_EDIT_SEEK:
        return "Go to frame";
    case TAS_EDIT_MARKER:
        return "Edit marker";
    case TAS_EDIT_BRANCH:
        return "Store branch";
    case TAS_EDIT_INSERT:
        return "Insert frames";
    default:
        return "Edit value";
    }
}

DesktopTasEditor *desktop_tas_editor(FrontendDesktopUi *ui) {
    if (!ui) {
        return NULL;
    }
    if (!ui->tas_editor) {
        ui->tas_editor = (DesktopTasEditor *)calloc(1, sizeof(*ui->tas_editor));
        if (ui->tas_editor) {
            ui->tas_editor->active_button = 1u;
            ui->tas_editor->column = TAS_GRID_PAD_BASE;
            ui->tas_editor->follow_playback = true;
        }
    }
    return ui->tas_editor;
}

NesTasSession *desktop_tas_session(FrontendDesktopUi *ui) {
    if (!ui || !ui->execution || !ui->execution->movie) {
        return NULL;
    }
    return nes_movie_tas(ui->execution->movie);
}

const NesTasProject *desktop_tas_project_const(const FrontendDesktopUi *ui) {
    if (!ui || !ui->execution || !ui->execution->movie) {
        return NULL;
    }
    return nes_tas_session_project_const(nes_movie_tas_const(ui->execution->movie));
}

NesTasProject *desktop_tas_project_writable(FrontendDesktopUi *ui, bool report) {
    NesTasSession *session = desktop_tas_session(ui);
    if (!session || !nes_tas_session_active(session)) {
        if (report) {
            desktop_tas_status(ui, "Open or create a TAS project before editing.");
        }
        return NULL;
    }
    if (!ui->execution || !frontend_execution_paused(ui->execution)) {
        if (report) {
            desktop_tas_status(ui, "Pause TAS playback before editing the timeline.");
        }
        return NULL;
    }
    NesTasProgress current;
    nes_tas_session_progress(session, &current);
    if (current.seeking) {
        if (report) {
            desktop_tas_status(ui, "Finish seeking before editing the timeline.");
        }
        return NULL;
    }
    NesTasProject *project = nes_tas_session_project(session);
    if (!project && report) {
        NesTasProgress progress;
        nes_tas_session_progress(session, &progress);
        if (progress.read_only) {
            desktop_tas_status(ui, "Turn off read-only mode before editing the timeline.");
        } else if (progress.recording) {
            desktop_tas_status(ui, "Stop recording before editing the timeline.");
        } else if (progress.frame_in_progress) {
            desktop_tas_status(ui, "Finish the current frame before editing the timeline.");
        } else {
            desktop_tas_status(ui, "The TAS timeline is temporarily unavailable for editing.");
        }
    }
    return project;
}

void desktop_tas_status(FrontendDesktopUi *ui, const char *text) {
    if (ui && text && *text) {
        desktop_copy_status(ui, text);
    }
}

void desktop_tas_keep_cursor_visible(FrontendDesktopUi *ui) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    const NesTasProject *project = desktop_tas_project_const(ui);
    if (!editor) {
        return;
    }
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!frames) {
        editor->cursor = editor->scroll = 0;
        return;
    }
    if (editor->cursor >= frames) {
        editor->cursor = frames - 1;
    }
    size_t rows = editor->visible_rows ? editor->visible_rows : 1;
    if (editor->cursor < editor->scroll) {
        editor->scroll = editor->cursor;
    } else if (editor->cursor >= editor->scroll + rows) {
        editor->scroll = editor->cursor - rows + 1;
    }
    size_t max_scroll = frames > rows ? frames - rows : 0;
    if (editor->scroll > max_scroll) {
        editor->scroll = max_scroll;
    }
}

void desktop_tas_follow_playback(FrontendDesktopUi *ui, const NesTasProgress *progress) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    if (!editor) {
        return;
    }
    if (!progress || !progress->active) {
        editor->follow_initialized = false;
        return;
    }
    if (!editor->follow_initialized) {
        editor->last_playback_frame = progress->frame;
        editor->follow_initialized = true;
        return;
    }
    if (editor->last_playback_frame == progress->frame) {
        return;
    }
    editor->last_playback_frame = progress->frame;
    if (!editor->follow_playback || editor->painting || editor->dragging_scroll) {
        return;
    }

    const NesTasProject *project = desktop_tas_project_const(ui);
    size_t frames = project ? nes_tas_project_frame_count(project) : 0;
    if (!frames) {
        editor->scroll = 0;
        return;
    }
    size_t frame = progress->frame < frames ? progress->frame : frames - 1;
    size_t rows = editor->visible_rows ? editor->visible_rows : 1;
    if (frame < editor->scroll) {
        editor->scroll = frame;
    } else if (frame >= editor->scroll + rows) {
        editor->scroll = frame - rows + 1;
    }
    size_t max_scroll = frames > rows ? frames - rows : 0;
    if (editor->scroll > max_scroll) {
        editor->scroll = max_scroll;
    }
}

void desktop_tas_after_model_edit(FrontendDesktopUi *ui, NesTasResult result, const char *success) {
    if (result != NES_TAS_OK) {
        desktop_tas_status(ui, nes_tas_result_string(result));
        return;
    }
    char error[256] = {0};
    if (ui && ui->execution && !tas_frontend_changed(ui->execution, error, sizeof(error))) {
        desktop_tas_status(ui, error[0] ? error : "Could not reconcile the edited TAS timeline.");
        return;
    }
    desktop_tas_status(ui, success);
    desktop_tas_keep_cursor_visible(ui);
}

static bool tas_hit_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    if (!editor) {
        return true;
    }
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    int x = 0, y = 0;
    if (event->type == SDL_MOUSEBUTTONDOWN || event->type == SDL_MOUSEBUTTONUP) {
        /* Desktop dispatch already converted button events to layout coordinates. */
        x = event->button.x;
        y = event->button.y;
    } else if (event->type == SDL_MOUSEMOTION) {
        x = (int)(event->motion.x / scale);
        y = (int)(event->motion.y / scale);
    }
    const DesktopHit *hit = desktop_clay_at(ui->clay, (float)x, (float)y);

    if (event->type == SDL_MOUSEBUTTONUP && event->button.button == SDL_BUTTON_LEFT) {
        desktop_tas_finish_paint(ui);
        editor->dragging_scroll = false;
        return true;
    }
    if (event->type == SDL_MOUSEMOTION && (event->motion.state & SDL_BUTTON_LMASK)) {
        if (editor->dragging_scroll) {
            desktop_tas_scroll_to(ui, y);
        } else if (editor->painting && hit && hit->kind == HIT_TAS_CELL && hit->index >= 0) {
            desktop_tas_cell_drag(ui, editor->scroll + (size_t)hit->index, hit->direction);
        }
        return true;
    }
    if (event->type != SDL_MOUSEBUTTONDOWN || event->button.button != SDL_BUTTON_LEFT) {
        return false;
    }
    if (!hit) {
        return true;
    }
    if (hit->kind == HIT_TAS_ACTION) {
        desktop_tas_action(ui, (unsigned)hit->index);
    } else if (hit->kind == HIT_TAS_ROW && hit->index >= 0) {
        desktop_tas_select_row(ui, editor->scroll + (size_t)hit->index, SDL_GetModState());
    } else if (hit->kind == HIT_TAS_CELL && hit->index >= 0) {
        desktop_tas_cell_down(ui, editor->scroll + (size_t)hit->index, hit->direction);
    } else if (hit->kind == HIT_TAS_SCROLL) {
        editor->dragging_scroll = true;
        desktop_tas_scroll_to(ui, y);
    } else {
        return false;
    }
    return true;
}

bool desktop_tas_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui || !event || !desktop_tas_panel(ui->panel_id)) {
        return false;
    }
    if (!desktop_tas_editor(ui)) {
        return true;
    }
    if (event->type == SDL_CONTROLLERBUTTONDOWN || event->type == SDL_CONTROLLERBUTTONUP) {
        NesTasProgress progress = {0};
        NesTasSession *session = desktop_tas_session(ui);
        if (session) {
            nes_tas_session_progress(session, &progress);
        }
        if (progress.active && progress.recording && !ui->parent) {
            frontend_host_input_event(event, ui->settings, ui->execution);
            return true;
        }
        return false;
    }
    if (event->type == SDL_WINDOWEVENT && event->window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
        desktop_tas_finish_paint(ui);
        ui->tas_editor->dragging_scroll = false;
        return false;
    }
    if (event->type == SDL_MOUSEBUTTONDOWN || event->type == SDL_MOUSEBUTTONUP || event->type == SDL_MOUSEMOTION) {
        return tas_hit_event(ui, event);
    }
    if (event->type == SDL_MOUSEWHEEL) {
        if (!event->wheel.y) {
            return true;
        }
        DesktopTasEditor *editor = ui->tas_editor;
        const NesTasProject *project = desktop_tas_project_const(ui);
        size_t frames = project ? nes_tas_project_frame_count(project) : 0;
        int delta = event->wheel.y > 0 ? -3 : 3;
        if (delta < 0 && editor->scroll < (size_t)-delta) {
            editor->scroll = 0;
        } else if (delta < 0) {
            editor->scroll -= (size_t)-delta;
        } else {
            editor->scroll += (size_t)delta;
        }
        size_t rows = editor->visible_rows ? editor->visible_rows : 1;
        size_t max_scroll = frames > rows ? frames - rows : 0;
        if (editor->scroll > max_scroll) {
            editor->scroll = max_scroll;
        }
        return true;
    }
    if (event->type == SDL_KEYDOWN && !event->key.repeat) {
        return desktop_tas_key(ui, &event->key);
    }
    return false;
}

bool desktop_tas_commit(FrontendDesktopUi *ui, const char *text, char *error, size_t size) {
    if (!ui || !text || !desktop_tas_panel(ui->panel_id)) {
        if (error && size) {
            snprintf(error, size, "Invalid TAS edit.");
        }
        return false;
    }
    DesktopTasEditor *editor = desktop_tas_editor(ui);
    NesTasSession *session = desktop_tas_session(ui);
    if (!editor || !session) {
        if (error && size) {
            snprintf(error, size, "No TAS session is active.");
        }
        return false;
    }
    if (ui->edit_control == TAS_EDIT_SEEK) {
        size_t frame;
        if (!tas_frontend_parse_frame(text, &frame)) {
            snprintf(error, size, "Enter a frame number.");
            return false;
        }
        if (!tas_frontend_seek(ui->execution, frame, error, size)) {
            return false;
        }
        editor->cursor = frame;
        desktop_tas_keep_cursor_visible(ui);
        return true;
    }
    NesTasProject *project = desktop_tas_project_writable(ui, false);
    if (!project) {
        snprintf(error, size, "Pause playback and disable read-only/recording before editing.");
        return false;
    }
    NesTasResult result = NES_TAS_OK;
    if (ui->edit_control == TAS_EDIT_INSERT) {
        size_t count;
        if (!tas_frontend_parse_frame(text, &count) || !count || count > 100000) {
            snprintf(error, size, "Enter a whole number from 1 to 100000.");
            return false;
        }
        size_t frames = nes_tas_project_frame_count(project);
        size_t max_frames = SIZE_MAX / sizeof(NesFm2Frame);
        if (frames > max_frames || count > max_frames - frames) {
            snprintf(error, size, "Frame count is too large.");
            return false;
        }
        size_t at = editor->cursor <= frames ? editor->cursor : frames;
        result = nes_tas_insert_frames(project, at, count);
        if (result != NES_TAS_OK) {
            snprintf(error, size, "%s", nes_tas_result_string(result));
            return false;
        }
        editor->cursor = at;
        /* The insertion has committed. Closing prevents a repeated insert if
         * rebuilding playback needs a later retry. */
        desktop_tas_after_model_edit(ui, result, "Blank frames inserted as one edit.");
        return true;
    } else if (ui->edit_control == TAS_EDIT_MARKER) {
        result =
            *text ? nes_tas_marker_set(project, editor->cursor, text) : nes_tas_marker_remove(project, editor->cursor);
    } else if (ui->edit_control == TAS_EDIT_BRANCH) {
        NesMovieResult saved = nes_tas_session_bookmark_set(session, editor->branch_slot, text);
        if (saved != NES_MOVIE_OK) {
            snprintf(error, size, "%s", nes_tas_session_error(session));
            return false;
        }
        return true;
    } else {
        snprintf(error, size, "Unknown TAS text edit.");
        return false;
    }
    if (result != NES_TAS_OK) {
        snprintf(error, size, "%s", nes_tas_result_string(result));
        return false;
    }
    if (!tas_frontend_changed(ui->execution, error, size)) {
        return false;
    }
    return true;
}

void desktop_tas_destroy(FrontendDesktopUi *ui) {
    if (!ui || !ui->tas_editor) {
        return;
    }
    if (ui->tas_editor->painting) {
        NesTasProject *project = desktop_tas_project_writable(ui, false);
        if (project && nes_tas_edit_active(project)) {
            nes_tas_edit_cancel(project);
        }
    }
    free(ui->tas_editor);
    ui->tas_editor = NULL;
}
