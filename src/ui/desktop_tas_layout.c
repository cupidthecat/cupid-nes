/* TAS timeline and playback controls. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_tas_internal.h"
#include "../system/vs_system.h"
#include <stdio.h>

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

static void file_controls(FrontendDesktopUi *ui, const NesTasProgress *progress) {
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        button(ui, "Open", TAS_ACTION_OPEN, false, !progress->active);
        button(ui, "New", TAS_ACTION_NEW, false, !progress->active);
        button(ui, "Save", TAS_ACTION_SAVE, false, progress->active && !progress->frame_in_progress);
        button(ui, "Save as", TAS_ACTION_SAVE_AS, false, progress->active && !progress->frame_in_progress);
        button(ui, "Export", TAS_ACTION_EXPORT, false, progress->active && !progress->frame_in_progress);
        button(ui, "Run TAS Script", TAS_ACTION_SCRIPT, false, desktop_tas_project_writable(ui, false) != NULL);
        button(ui, "Stop", TAS_ACTION_STOP, false, progress->active);
        button(ui, "Discard edits", TAS_ACTION_DISCARD, false, progress->active && progress->dirty);
        spacer();
        text(ui, progress->dirty ? "Unsaved project" : "", 12, accent);
    }
}

static void playback_controls(FrontendDesktopUi *ui, const NesTasProgress *progress) {
    bool active = progress->active;
    bool paused = ui->execution && frontend_execution_paused(ui->execution);
    bool boundary = active && !progress->frame_in_progress && !progress->seeking;
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        button(ui, "Back", TAS_ACTION_BACK, false, active && progress->frame > 0);
        button(ui, "Frame", TAS_ACTION_FRAME, false, active);
        button(ui, paused ? "Play" : "Pause", TAS_ACTION_PAUSE, !paused, active);
        button(ui, "Go to frame", TAS_ACTION_SEEK, false, active);
        button(ui, "Read-only", TAS_ACTION_READ_ONLY, progress->read_only, boundary);
        button(ui, "Record", TAS_ACTION_RECORD, progress->recording, boundary && !progress->read_only);
        button(ui, progress->record_mode == NES_TAS_RECORD_INSERT ? "Insert take" : "Overwrite take",
               TAS_ACTION_RECORD_MODE, progress->record_mode == NES_TAS_RECORD_INSERT,
               boundary && !progress->read_only);
        button(ui, "Skip lag", TAS_ACTION_SKIP_LAG, ui->execution && ui->execution->tas_skip_lag, active);
    }
}

static void edit_controls(FrontendDesktopUi *ui, const NesTasProject *project, bool writable) {
    bool clipboard = project && nes_tas_clipboard_available(project);
    bool frames = project && nes_tas_project_frame_count(project) != 0;
    bool selection = project && nes_tas_selection_count(project) != 0;
    CLAY_AUTO_ID({.layout = {.childGap = 5}}) {
        button(ui, "Undo", TAS_ACTION_UNDO, false, writable && nes_tas_project_can_undo(project));
        button(ui, "Redo", TAS_ACTION_REDO, false, writable && nes_tas_project_can_redo(project));
        button(ui, "Copy", TAS_ACTION_COPY, false, writable && selection);
        button(ui, "Cut", TAS_ACTION_CUT, false, writable && selection);
        button(ui, "Paste", TAS_ACTION_PASTE, false, writable && clipboard);
        button(ui, "Paste insert", TAS_ACTION_PASTE_INSERT, false, writable && clipboard);
        button(ui, "Insert", TAS_ACTION_INSERT, false, writable);
        button(ui, "Delete", TAS_ACTION_DELETE, false, writable && frames);
        button(ui, "Clone", TAS_ACTION_CLONE, false, writable && frames);
        button(ui, "Trim", TAS_ACTION_TRUNCATE, false, writable && frames);
    }
}

static void input_controls(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress,
                           bool writable) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesFm2Movie *movie = project ? nes_tas_project_movie(project) : NULL;
    unsigned players = movie && movie->fourscore ? 4 : 2;
    static const char *const names[] = {"A", "B", "Select", "Start", "Up", "Down", "Left", "Right"};
    static const char *const patterns[] = {"Pattern 1/0", "Pattern 0/1", "Pattern 11/00", "Pattern 111/0"};
    unsigned bit = 0;
    while (bit < 7 && !(editor->active_button & (1u << bit))) {
        ++bit;
    }
    char label[48];
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        text(ui, "Edit", 12, muted);
        for (unsigned i = 0; i < players; ++i) {
            snprintf(label, sizeof(label), "P%u", i + 1);
            button(ui, label, TAS_ACTION_EDIT_PLAYER_BASE + i, editor->edit_player == i, progress->active);
        }
        text(ui, "Record", 12, muted);
        for (unsigned i = 0; i < players; ++i) {
            snprintf(label, sizeof(label), "P%u", i + 1);
            button(ui, label, TAS_ACTION_RECORD_PLAYER_BASE + i, (progress->record_players & (1u << i)) != 0,
                   progress->active && !progress->read_only && !progress->frame_in_progress && !progress->seeking);
        }
        button(ui, patterns[editor->pattern % 4], TAS_ACTION_PATTERN_NEXT, false, progress->active);
        button(ui, "Apply", TAS_ACTION_PATTERN, false, writable && nes_tas_project_frame_count(project));
        snprintf(label, sizeof(label), "Hold %s", names[bit]);
        button(ui, label, TAS_ACTION_HOLD,
               (nes_tas_session_hold(desktop_tas_session(ui), editor->edit_player) & editor->active_button) != 0,
               progress->active);
        snprintf(label, sizeof(label), "Auto %s", names[bit]);
        button(ui, label, TAS_ACTION_AUTOFIRE,
               (nes_tas_session_autofire(desktop_tas_session(ui), editor->edit_player) & editor->active_button) != 0,
               progress->active);
    }
}

static void branch_controls(FrontendDesktopUi *ui, const NesTasProject *project, bool writable) {
    DesktopTasEditor *editor = ui->tas_editor;
    char label[32];
    NesTasBookmarkView current = {0};
    if (project) {
        (void)nes_tas_bookmark(project, editor->branch_slot, &current);
    }
    CLAY_AUTO_ID({.layout = {.childGap = 5, .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
        button(ui, "Marker", TAS_ACTION_MARKER, false, writable && nes_tas_project_frame_count(project));
        text(ui, "Branch", 12, muted);
        for (unsigned slot = 0; slot < NES_TAS_BOOKMARK_COUNT; ++slot) {
            NesTasBookmarkView branch = {0};
            if (project) {
                (void)nes_tas_bookmark(project, slot, &branch);
            }
            snprintf(label, sizeof(label), "%u%s", slot == 9 ? 0 : slot + 1, branch.occupied ? "*" : "");
            button(ui, label, TAS_ACTION_BRANCH_SLOT_BASE + slot, editor->branch_slot == slot, project != NULL);
        }
        button(ui, "Store", TAS_ACTION_BRANCH_STORE, false, writable);
        button(ui, "Load", TAS_ACTION_BRANCH_LOAD, false, writable && current.occupied);
        button(ui, "Clear", TAS_ACTION_BRANCH_CLEAR, false, writable && current.occupied);
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

static void cell(FrontendDesktopUi *ui, int row, int column, const char *value, float width, bool on, bool enabled) {
    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_TAS_CELL : HIT_NONE, row, column);
    CLAY(id, {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(21)},
                         .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
              .backgroundColor = on ? selected : (Clay_Color){0},
              .border = {.color = {59, 68, 89, 255}, .width = {.right = 1, .bottom = 1}}}) {
        text(ui, value, 11, on ? accent : muted);
    }
}

static const char *marker_at(const NesTasProject *project, size_t frame) {
    for (size_t i = 0; i < nes_tas_marker_count(project); ++i) {
        NesTasMarkerView marker;
        if (!nes_tas_marker(project, i, &marker)) {
            continue;
        }
        if (marker.frame == frame) {
            return marker.note;
        }
        if (marker.frame > frame) {
            break;
        }
    }
    return "";
}

static void timeline(FrontendDesktopUi *ui, const NesTasProject *project, const NesTasProgress *progress, float width,
                     unsigned visible, bool writable) {
    DesktopTasEditor *editor = ui->tas_editor;
    const NesFm2Movie *movie = nes_tas_project_movie(project);
    unsigned first_player = movie->fourscore ? (editor->edit_player / 2) * 2 : 0;
    static const char *const pad_names[] = {"A", "B", "Se", "St", "U", "D", "L", "R"};
    static const char *const commands[] = {"R", "P", "I", "S", "C1", "C2", "Sv"};
    const float frame_width = 65, cell_width = 22, lag_width = 25;
    float pad_widths[2];
    for (unsigned n = 0; n < 2; ++n) {
        unsigned player = first_player + n;
        pad_widths[n] = movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD ? cell_width * 8 : 104;
    }
    float note_width = width - frame_width - lag_width - cell_width * 7 - pad_widths[0] - pad_widths[1] - 12;
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
            fixed_label(ui, "Reset / disk / VS", cell_width * 7, ink);
            for (unsigned n = 0; n < 2; ++n) {
                snprintf(value, sizeof(value), "Player %u", first_player + n + 1);
                fixed_label(ui, value, pad_widths[n], ink);
            }
            fixed_label(ui, "Marker", note_width, ink);
        }
        CLAY_AUTO_ID({.layout = {.childAlignment = {.y = CLAY_ALIGN_Y_CENTER}}}) {
            fixed_label(ui, "", frame_width + lag_width, muted);
            for (unsigned command = 0; command < 7; ++command) {
                fixed_label(ui, commands[command], cell_width, muted);
            }
            for (unsigned n = 0; n < 2; ++n) {
                unsigned player = first_player + n;
                if (movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD) {
                    for (unsigned bit = 0; bit < 8; ++bit) {
                        fixed_label(ui, pad_names[bit], cell_width, muted);
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
                            snprintf(value, sizeof(value), "%s%zu", progress->frame == index ? "> " : "", index);
                            text(ui, value, 11, progress->frame == index ? accent : ink);
                        }
                        NesTasLagState lag = nes_tas_lag(project, index);
                        cell(ui, (int)row, TAS_GRID_LAG,
                             lag == NES_TAS_LAG_YES  ? "L"
                             : lag == NES_TAS_LAG_NO ? "."
                                                     : "?",
                             lag_width, lag == NES_TAS_LAG_YES, writable);
                        for (unsigned command = 0; command < 7; ++command) {
                            bool enabled = command < 2 || (command < 4 ? movie->fds : vs_enabled());
                            bool set = (frame->commands & (1u << command)) != 0;
                            cell(ui, (int)row, TAS_GRID_COMMAND_BASE + (int)command, set ? commands[command] : "",
                                 cell_width, set, writable && enabled);
                        }
                        for (unsigned n = 0; n < 2; ++n) {
                            unsigned player = first_player + n;
                            if (movie->fourscore || movie->ports[player] == NES_FM2_PORT_GAMEPAD) {
                                for (unsigned bit = 0; bit < 8; ++bit) {
                                    bool set = (frame->pads[player] & (1u << bit)) != 0;
                                    cell(ui, (int)row, TAS_GRID_PAD_BASE + (int)(player * 8 + bit),
                                         set ? pad_names[bit] : "", cell_width, set, writable);
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
    nes_tas_session_progress(desktop_tas_session(ui), &progress);
    bool writable = desktop_tas_project_writable(ui, false) != NULL;
    unsigned rows = height > 313 ? (unsigned)((height - 313) / 21) : 1;
    if (rows > 50) {
        rows = 50;
    }
    if (!rows) {
        rows = 1;
    }
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .childGap = 6,
                             .sizing = {.width = CLAY_SIZING_FIXED(width), .height = CLAY_SIZING_FIXED(height)}},
                  .clip = {.horizontal = true, .vertical = true}}) {
        text(ui, "TAS Editor", 20, ink);
        file_controls(ui, &progress);
        playback_controls(ui, &progress);
        edit_controls(ui, project, writable);
        input_controls(ui, project, &progress, writable);
        branch_controls(ui, project, writable);
        char status[256];
        snprintf(status, sizeof(status), "Frame %zu / %zu | Lag %zu | Rerecords %llu | Cached states %zu (%.1f MiB)%s",
                 progress.frame, progress.total_frames, progress.lag_count, (unsigned long long)progress.rerecord_count,
                 progress.checkpoint_count, progress.checkpoint_bytes / (1024.0 * 1024.0),
                 progress.seeking ? " | Seeking" : "");
        text(ui, status, 12, progress.seeking ? accent : muted);
        if (project && nes_tas_project_frame_count(project)) {
            timeline(ui, project, &progress, width, rows, writable);
        } else {
            text(ui,
                 project ? "Insert a frame or enable recording to begin the timeline."
                         : "Open an FM2, FM3 or CTAS file, or create a project for the loaded game.",
                 13, muted);
        }
        text(ui, "Drag buttons to paint | Shift/Ctrl select rows | Space toggles a cell | Ctrl+Z undoes | F2 marker",
             11, muted);
        snprintf(status, sizeof(status), "Cursor %zu | Selected %zu | %s", editor->cursor,
                 project ? nes_tas_selection_count(project) : 0, project ? marker_at(project, editor->cursor) : "");
        text(ui, status, 11, ink);
    }
}
