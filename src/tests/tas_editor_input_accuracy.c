/* TAS editor gesture and display-scale regressions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../debugger/debugger.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../replay/tas_project.h"
#include "../replay/tas_session.h"
#include "../system/execution_policy.h"
#include "../ui/desktop_tas_internal.h"
#include "../ui/frontend_panels.h"
#include "../ui/machine_actions.h"
#include "../ui/platform_frontend.h"
#include "../ui/state_frontend.h"
#include "../util/file_io.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "TAS editor input:%d: %s\n", __LINE__, #condition);                                        \
            ++failures;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static bool make_cartridge(BoardImage *image) {
    if (!board_image_create(image, 0, 0x8000, 0x2000, true)) {
        return false;
    }
    uint8_t *prg = image->data + 16;
    prg[0] = 0x4C;
    prg[1] = 0;
    prg[2] = 0x80;
    for (unsigned i = 0x7FFA; i <= 0x7FFE; i += 2) {
        prg[i] = 0;
        prg[i + 1] = 0x80;
    }
    apu_power_on(&apu);
    if (board_image_load(image)) {
        return false;
    }
    debugger_init();
    debugger_resume();
    cpu_use_default_startup_alignment();
    if (!frontend_machine_power_cycle()) {
        return false;
    }
    ppu_begin_frame_render(framebuffer);
    return true;
}

static bool send_key(FrontendDesktopUi *ui, SDL_Scancode key, SDL_Keymod modifiers) {
    SDL_Event event = {.type = SDL_KEYDOWN};
    event.key.windowID = SDL_GetWindowID(ui->window);
    event.key.keysym.scancode = key;
    event.key.keysym.mod = modifiers;
    return frontend_desktop_handle_event(ui, &event);
}

static bool click_action(FrontendDesktopUi *ui, unsigned action) {
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, HIT_TAS_ACTION, (int)action, 0, &bounds)) {
        return false;
    }
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    SDL_Event event = {.type = SDL_MOUSEBUTTONDOWN};
    event.button.windowID = SDL_GetWindowID(ui->window);
    event.button.button = SDL_BUTTON_LEFT;
    event.button.x = (int)((bounds.x + bounds.w / 2) * scale);
    event.button.y = (int)((bounds.y + bounds.h / 2) * scale);
    return frontend_desktop_handle_event(ui, &event);
}

static bool click_row(FrontendDesktopUi *ui, unsigned row, SDL_Keymod modifiers) {
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, HIT_TAS_ROW, (int)row, 0, &bounds)) {
        return false;
    }
    SDL_SetModState(modifiers);
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    SDL_Event event = {.type = SDL_MOUSEBUTTONDOWN};
    event.button.windowID = SDL_GetWindowID(ui->window);
    event.button.button = SDL_BUTTON_LEFT;
    event.button.x = (int)((bounds.x + bounds.w / 2) * scale);
    event.button.y = (int)((bounds.y + bounds.h / 2) * scale);
    bool handled = frontend_desktop_handle_event(ui, &event);
    SDL_SetModState(KMOD_NONE);
    return handled;
}

static bool click_scroll(FrontendDesktopUi *ui, float position) {
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, HIT_TAS_SCROLL, 0, 0, &bounds)) {
        return false;
    }
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    SDL_Event down = {.type = SDL_MOUSEBUTTONDOWN};
    down.button.windowID = SDL_GetWindowID(ui->window);
    down.button.button = SDL_BUTTON_LEFT;
    down.button.x = (int)((bounds.x + bounds.w / 2) * scale);
    down.button.y = (int)((bounds.y + bounds.h * position) * scale);
    if (!frontend_desktop_handle_event(ui, &down)) {
        return false;
    }
    SDL_Event up = down;
    up.type = SDL_MOUSEBUTTONUP;
    return frontend_desktop_handle_event(ui, &up);
}

static const char *script_path;

static bool choose_script(bool save, unsigned type, char *path, size_t path_size, char *error, size_t error_size,
                          void *context) {
    (void)context;
    (void)error;
    (void)error_size;
    if (save || type != FRONTEND_OPEN_SCRIPT || !script_path) {
        return false;
    }
    return snprintf(path, path_size, "%s", script_path) > 0;
}

static bool state_command(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    (*(unsigned *)userdata)++;
    return true;
}

static void render_editor(FrontendDesktopUi *ui) {
    SDL_SetRenderDrawColor(ui->renderer, 24, 28, 39, 255);
    SDL_RenderClear(ui->renderer);
    frontend_desktop_render(ui, 256, 240, "TAS input fixture", "NTSC", "Paused");
}

static void render_tall_tas_layout(FrontendDesktopUi *ui) {
    SDL_SetRenderDrawColor(ui->renderer, 24, 28, 39, 255);
    SDL_RenderClear(ui->renderer);
    desktop_clay_begin(ui->clay, 900, 1500, 1);
    desktop_tas_layout(ui, 900, 1500);
    desktop_clay_end(ui->clay);
}

static void render_hit_capacity_stress(FrontendDesktopUi *ui) {
    desktop_clay_begin(ui->clay, 900, 1500, 1);
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM}}) {
        for (int i = 0; i < 2300; ++i) {
            Clay_ElementId id = desktop_clay_hit(ui->clay, HIT_NONE, i, 0);
            CLAY(id, {.layout = {.sizing = {.height = CLAY_SIZING_FIXED(1)}}}) {
            }
        }
    }
    desktop_clay_end(ui->clay);
}

static bool hit_inside_window(FrontendDesktopUi *ui, int kind, int index, int direction) {
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, kind, index, direction, &bounds)) {
        return false;
    }
    int width, height;
    SDL_GetWindowSize(ui->window, &width, &height);
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    return bounds.w > 0 && bounds.h > 0 && bounds.x >= 0 && bounds.y >= 0 &&
           (bounds.x + bounds.w) * scale <= width + 1 && (bounds.y + bounds.h) * scale <= height + 1;
}

static bool cell_event(FrontendDesktopUi *ui, Uint32 type, unsigned row, unsigned column) {
    SDL_FRect bounds;
    if (!desktop_clay_bounds(ui->clay, HIT_TAS_CELL, (int)row, (int)column, &bounds)) {
        return false;
    }
    int width, height;
    SDL_GetWindowSize(ui->window, &width, &height);
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    if (bounds.w <= 0 || bounds.h <= 0 || bounds.x < 0 || bounds.y < 0 || (bounds.x + bounds.w) * scale > width + 1 ||
        (bounds.y + bounds.h) * scale > height + 1) {
        return false;
    }
    int x = (int)((bounds.x + bounds.w / 2) * scale);
    int y = (int)((bounds.y + bounds.h / 2) * scale);
    SDL_Event event = {.type = type};
    if (type == SDL_MOUSEMOTION) {
        event.motion.windowID = SDL_GetWindowID(ui->window);
        event.motion.state = SDL_BUTTON_LMASK;
        event.motion.x = x;
        event.motion.y = y;
    } else {
        event.button.windowID = SDL_GetWindowID(ui->window);
        event.button.button = SDL_BUTTON_LEFT;
        event.button.x = x;
        event.button.y = y;
    }
    return frontend_desktop_handle_event(ui, &event);
}

static void focus(FrontendDesktopUi *ui, bool gained) {
    SDL_Event event = {.type = SDL_WINDOWEVENT};
    event.window.windowID = SDL_GetWindowID(ui->window);
    event.window.event = gained ? SDL_WINDOWEVENT_FOCUS_GAINED : SDL_WINDOWEVENT_FOCUS_LOST;
    (void)frontend_desktop_handle_event(ui, &event);
}

int test_tas_editor_input_accuracy(void) {
    int failures = 0;
    BoardImage image = {0};
    FrontendExecutionRuntime runtime = {0};
    FrontendDesktopUi ui = {0};
    FrontendSettings settings;
    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    bool runtime_initialized = false, ui_initialized = false, video_initialized = false;
    NesInputAdapter old_adapter = joypad_adapter();
    const char *path = "build/tas-editor-input.ctas";
    const char *lua_path = "build/tas-editor-input.lua";
    char error[256] = {0};
    unsigned save_slot_calls = 0, load_slot_calls = 0, save_file_calls = 0, load_file_calls = 0;
    CHECK(make_cartridge(&image));
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    video_initialized = true;
    window = SDL_CreateWindow("TAS input checks", 0, 0, 900, 760, SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    CHECK(window && renderer);
    frontend_settings_defaults(&settings);
    settings.pause_on_focus_loss = false;
    frontend_commands_reset();
    frontend_panels_reset();
    frontend_execution_init(&runtime, NULL, 0, NULL, NULL, NULL, NULL);
    runtime_initialized = true;
    runtime.settings = &settings;
    CHECK(frontend_execution_register_commands(&runtime));
    FrontendCommandSpec save_slot = {STATE_COMMAND_SAVE_SLOT,        "Test quick save", "File",          "F5",
                                     FRONTEND_COMMAND_NEEDS_SESSION, state_command,     &save_slot_calls};
    FrontendCommandSpec load_slot = {STATE_COMMAND_LOAD_SLOT,        "Test quick load", "File",          "F6",
                                     FRONTEND_COMMAND_NEEDS_SESSION, state_command,     &load_slot_calls};
    FrontendCommandSpec save_file = {STATE_COMMAND_SAVE_FILE,        "Test save file", "File",          "Ctrl+F5",
                                     FRONTEND_COMMAND_NEEDS_SESSION, state_command,    &save_file_calls};
    FrontendCommandSpec load_file = {STATE_COMMAND_LOAD_FILE,        "Test load file", "File",          "Ctrl+F6",
                                     FRONTEND_COMMAND_NEEDS_SESSION, state_command,    &load_file_calls};
    CHECK(frontend_command_register(&save_slot));
    CHECK(frontend_command_register(&load_slot));
    CHECK(frontend_command_register(&save_file));
    CHECK(frontend_command_register(&load_file));
    frontend_command_set_session_active(true);
    frontend_panel_set_session_active(true);
    frontend_desktop_init(&ui, window, renderer, &settings, &runtime, NULL, NULL);
    ui_initialized = true;
    CHECK(frontend_desktop_register_commands(&ui));
    CHECK(joypad_set_adapter(NES_ADAPTER_FOUR_SCORE));
    CHECK(tas_frontend_open(&runtime, path, true, error, sizeof(error)));
    ui.panel_id = TAS_PANEL;
    ui.panel_open = true;
    NesTasSession *session = nes_movie_tas(runtime.movie);
    NesTasProject *project = nes_tas_session_project(session);
    CHECK(project && nes_tas_project_movie(project)->fourscore);
    CHECK(nes_tas_insert_frames(project, 0, 60) == NES_TAS_OK);
    CHECK(tas_frontend_changed(&runtime, error, sizeof(error)));
    DesktopTasEditor *editor = desktop_tas_editor(&ui);
    CHECK(editor);
    editor->cursor = 2;
    editor->column = TAS_GRID_PAD_BASE + 1;
    CHECK(send_key(&ui, SDL_SCANCODE_SPACE, KMOD_NONE));
    CHECK(!nes_tas_edit_active(project) && !editor->painting);
    CHECK(nes_tas_project_frame(project, 2)->pads[0] == 2);
    CHECK(send_key(&ui, SDL_SCANCODE_Z, KMOD_LCTRL));
    CHECK(nes_tas_project_frame(project, 2)->pads[0] == 0);
    CHECK(send_key(&ui, SDL_SCANCODE_Y, KMOD_RCTRL));
    CHECK(nes_tas_project_frame(project, 2)->pads[0] == 2);
    CHECK(send_key(&ui, SDL_SCANCODE_F5, KMOD_NONE));
    CHECK(ui.edit_text_active);
    snprintf(ui.edit_text, sizeof(ui.edit_text), "%s", "keyboard branch");
    CHECK(send_key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE));
    NesTasBookmarkView keyboard_branch = {0};
    CHECK(nes_tas_bookmark(project, 0, &keyboard_branch) && keyboard_branch.occupied);
    CHECK(send_key(&ui, SDL_SCANCODE_F6, KMOD_NONE));
    CHECK(nes_tas_current_branch(project) == 0);
    CHECK(save_slot_calls == 0 && load_slot_calls == 0);
    CHECK(send_key(&ui, SDL_SCANCODE_F5, KMOD_LCTRL));
    CHECK(send_key(&ui, SDL_SCANCODE_F6, KMOD_LCTRL));
    CHECK(save_file_calls == 1 && load_file_calls == 1);

    /* Insert-count editing accepts one bounded positive count and leaves the
     * timeline/cursor untouched for malformed or oversized values. */
    editor->cursor = 6;
    size_t insert_frames_before = nes_tas_project_frame_count(project);
    size_t insert_cursor_before = editor->cursor;
    const char *invalid_insert_counts[] = {"0", "-1", "100001", "18446744073709551615"};
    for (unsigned i = 0; i < sizeof(invalid_insert_counts) / sizeof(invalid_insert_counts[0]); ++i) {
        ui.edit_control = TAS_EDIT_INSERT;
        error[0] = '\0';
        CHECK(!desktop_tas_commit(&ui, invalid_insert_counts[i], error, sizeof(error)));
        CHECK(error[0] != '\0');
        CHECK(nes_tas_project_frame_count(project) == insert_frames_before);
        CHECK(editor->cursor == insert_cursor_before);
    }
    ui.edit_control = TAS_EDIT_INSERT;
    CHECK(desktop_tas_commit(&ui, "3", error, sizeof(error)));
    CHECK(nes_tas_project_frame_count(project) == insert_frames_before + 3);
    CHECK(editor->cursor == insert_cursor_before);
    desktop_tas_action(&ui, TAS_ACTION_UNDO);
    CHECK(nes_tas_project_frame_count(project) == insert_frames_before);
    CHECK(editor->cursor == insert_cursor_before);

    render_editor(&ui);
    CHECK(desktop_clay_bounds(ui.clay, HIT_TAS_ACTION, TAS_ACTION_SCRIPT, 0, &(SDL_FRect){0}));
    CHECK(desktop_clay_bounds(ui.clay, HIT_TAS_ACTION, TAS_ACTION_DISCARD, 0, &(SDL_FRect){0}));
    CHECK(hit_inside_window(&ui, HIT_TAS_ROW, 0, 0));
    CHECK(hit_inside_window(&ui, HIT_TAS_CELL, 0, TAS_GRID_PAD_BASE + 15));
    CHECK(hit_inside_window(&ui, HIT_TAS_SCROLL, 0, 0));
    CHECK(click_row(&ui, 1, KMOD_NONE));
    CHECK(click_row(&ui, 3, KMOD_CTRL));
    CHECK(nes_tas_selection_count(project) == 2);
    CHECK(click_row(&ui, 5, KMOD_NONE));
    CHECK(click_row(&ui, 7, KMOD_SHIFT));
    CHECK(nes_tas_selection_count(project) == 3);
    size_t scroll_before = editor->scroll;
    CHECK(click_scroll(&ui, 0.95f));
    CHECK(editor->scroll >= scroll_before);
    editor->scroll = 0;

    const char script[] = "taseditor.submitinputchange(4, 1, 1); taseditor.applyinputchanges()";
    CHECK(nes_file_write_atomic(lua_path, script, sizeof(script) - 1) == NES_FILE_OK);
    script_path = lua_path;
    frontend_set_file_chooser(choose_script, NULL);
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_SCRIPT));
    CHECK(nes_tas_project_frame(project, 4)->pads[0] == 1);
    frontend_set_file_chooser(NULL, NULL);
    script_path = NULL;

    editor->cursor = 4;
    desktop_tas_action(&ui, TAS_ACTION_SIDEBAR_MARKERS);
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_MARKER));
    CHECK(ui.edit_text_active);
    snprintf(ui.edit_text, sizeof(ui.edit_text), "%s", "scripted marker");
    CHECK(send_key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE));
    CHECK(!ui.edit_text_active && nes_tas_marker_count(project) == 1);

    CHECK(nes_tas_marker_set(project, 10, "middle") == NES_TAS_OK);
    CHECK(nes_tas_marker_set(project, 20, "later") == NES_TAS_OK);

    /* Marker jumps remain navigation in read-only mode and clamp to timeline
     * boundaries when there is no marker in the requested direction. */
    CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
    uint64_t marker_navigation_revision = nes_tas_project_revision(project);
    editor->cursor = 0;
    desktop_tas_action(&ui, TAS_ACTION_MARKER_PREV);
    CHECK(editor->cursor == 0);
    desktop_tas_action(&ui, TAS_ACTION_MARKER_NEXT);
    CHECK(editor->cursor == 4);
    desktop_tas_action(&ui, TAS_ACTION_MARKER_GOTO_BASE + 2);
    CHECK(editor->cursor == 20);
    editor->cursor = nes_tas_project_frame_count(project) - 1;
    desktop_tas_action(&ui, TAS_ACTION_MARKER_NEXT);
    CHECK(editor->cursor == nes_tas_project_frame_count(project) - 1);
    CHECK(nes_tas_project_revision(project) == marker_navigation_revision);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);

    editor->cursor = 1;
    desktop_tas_action(&ui, TAS_ACTION_SELECT_BETWEEN_MARKERS);
    CHECK(nes_tas_selection_count(project) == 4);
    CHECK(nes_tas_selection_contains(project, 0) && nes_tas_selection_contains(project, 3));
    CHECK(!nes_tas_selection_contains(project, 4));
    editor->cursor = nes_tas_project_frame_count(project) - 1;
    desktop_tas_action(&ui, TAS_ACTION_SELECT_BETWEEN_MARKERS);
    CHECK(nes_tas_selection_count(project) == nes_tas_project_frame_count(project) - 20);
    CHECK(nes_tas_selection_contains(project, 20));

    /* A header click follows FCEUX column-set behavior across a sparse
     * selection and records the entire change as one undo item. */
    desktop_tas_action(&ui, TAS_ACTION_SELECT_NONE);
    CHECK(nes_tas_selection_set(project, 1, true) == NES_TAS_OK);
    CHECK(nes_tas_selection_set(project, 3, true) == NES_TAS_OK);
    editor->cursor = 1;
    desktop_tas_action(&ui, TAS_ACTION_SIDEBAR_INPUT);
    render_editor(&ui);
    CHECK(hit_inside_window(&ui, HIT_TAS_ACTION, TAS_ACTION_HEADER_BASE + 7, 0));
    CHECK(click_action(&ui, TAS_ACTION_HEADER_BASE + 7));
    CHECK((nes_tas_project_frame(project, 1)->pads[0] & 0x80u) != 0);
    CHECK((nes_tas_project_frame(project, 2)->pads[0] & 0x80u) == 0);
    CHECK((nes_tas_project_frame(project, 3)->pads[0] & 0x80u) != 0);
    CHECK(send_key(&ui, SDL_SCANCODE_Z, KMOD_LCTRL));
    CHECK((nes_tas_project_frame(project, 1)->pads[0] & 0x80u) == 0);
    CHECK((nes_tas_project_frame(project, 2)->pads[0] & 0x80u) == 0);
    CHECK((nes_tas_project_frame(project, 3)->pads[0] & 0x80u) == 0);
    CHECK(nes_tas_edit_begin(project) == NES_TAS_OK);
    uint8_t grouped_pad = nes_tas_project_frame(project, 1)->pads[0];
    desktop_tas_action(&ui, TAS_ACTION_HEADER_BASE + 6);
    CHECK(nes_tas_edit_active(project));
    CHECK(nes_tas_project_frame(project, 1)->pads[0] == grouped_pad);
    nes_tas_edit_cancel(project);

    /* Sidebar button selection chooses the Hold/Auto target without editing
     * movie input. It remains available while editing itself is locked out. */
    uint8_t selector_pad = nes_tas_project_frame(project, 1)->pads[0];
    CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
    uint64_t selector_revision = nes_tas_project_revision(project);
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_ACTIVE_BUTTON_BASE + 6));
    CHECK(editor->active_button == 0x40u);
    CHECK(editor->column == TAS_GRID_PAD_BASE + 6);
    CHECK(nes_tas_project_frame(project, 1)->pads[0] == selector_pad);
    CHECK(nes_tas_project_revision(project) == selector_revision);
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    selector_revision = nes_tas_project_revision(project);
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_ACTIVE_BUTTON_BASE + 5));
    CHECK(editor->active_button == 0x20u);
    CHECK(editor->column == TAS_GRID_PAD_BASE + 5);
    CHECK(nes_tas_project_frame(project, 1)->pads[0] == selector_pad);
    CHECK(nes_tas_project_revision(project) == selector_revision);
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);

    /* Follow scrolls only after playback advances. It never moves the grid
     * cursor or selection, and painting suppresses the follow adjustment. */
    size_t follow_selection = nes_tas_selection_count(project);
    editor->visible_rows = 5;
    editor->scroll = 0;
    editor->cursor = 7;
    editor->follow_playback = true;
    editor->follow_initialized = false;
    NesTasProgress follow_progress = {.active = true, .frame = 0};
    desktop_tas_follow_playback(&ui, &follow_progress);
    CHECK(editor->scroll == 0 && editor->cursor == 7 && nes_tas_selection_count(project) == follow_selection);
    follow_progress.frame = 20;
    desktop_tas_follow_playback(&ui, &follow_progress);
    CHECK(editor->scroll == 16 && editor->cursor == 7 && nes_tas_selection_count(project) == follow_selection);
    editor->scroll = 3;
    desktop_tas_follow_playback(&ui, &follow_progress);
    CHECK(editor->scroll == 3);
    editor->painting = true;
    follow_progress.frame = 30;
    desktop_tas_follow_playback(&ui, &follow_progress);
    CHECK(editor->scroll == 3 && editor->cursor == 7 && nes_tas_selection_count(project) == follow_selection);
    editor->painting = false;
    follow_progress.frame = 31;
    desktop_tas_follow_playback(&ui, &follow_progress);
    CHECK(editor->scroll == 27 && editor->cursor == 7 && nes_tas_selection_count(project) == follow_selection);
    desktop_tas_action(&ui, TAS_ACTION_SELECT_NONE);

    /* A single drag is one undo item even when the mouse skips intervening rows.
     * Events use physical window coordinates at fractional and integer scales. */
    const float scales[] = {1, 1.5f, 2};
    for (unsigned i = 0; i < sizeof(scales) / sizeof(scales[0]); ++i) {
        SDL_SetWindowSize(window, (int)(900 * scales[i]), (int)(760 * scales[i]));
        ui.ui_scale = scales[i];
        editor->scroll = 0;
        render_editor(&ui);
        CHECK(cell_event(&ui, SDL_MOUSEBUTTONDOWN, 1, TAS_GRID_PAD_BASE));
        CHECK(cell_event(&ui, SDL_MOUSEMOTION, 3, TAS_GRID_PAD_BASE));
        CHECK(cell_event(&ui, SDL_MOUSEBUTTONUP, 3, TAS_GRID_PAD_BASE));
        CHECK(!nes_tas_edit_active(project) && !editor->painting);
        CHECK(nes_tas_project_frame(project, 1)->pads[0] == 1);
        CHECK(nes_tas_project_frame(project, 2)->pads[0] == 3);
        CHECK(nes_tas_project_frame(project, 3)->pads[0] == 1);
        CHECK(send_key(&ui, SDL_SCANCODE_Z, KMOD_LCTRL));
        CHECK(nes_tas_project_frame(project, 1)->pads[0] == 0);
        CHECK(nes_tas_project_frame(project, 2)->pads[0] == 2);
        CHECK(nes_tas_project_frame(project, 3)->pads[0] == 0);
    }

    /* Fifty visible rows with Four Score exceeds the old 512-hit table. The
     * bottom row and P3/P4 cells must remain addressable after that boundary. */
    ui.ui_scale = 1;
    editor->scroll = 0;
    editor->edit_player = 2;
    editor->column = TAS_GRID_PAD_BASE + 2 * TAS_GRID_PAD_COLUMNS;
    render_tall_tas_layout(&ui);
    CHECK(editor->visible_rows == 50);
    CHECK(desktop_clay_bounds(ui.clay, HIT_TAS_ROW, 49, 0, &(SDL_FRect){0}));
    CHECK(
        desktop_clay_bounds(ui.clay, HIT_TAS_CELL, 49, TAS_GRID_PAD_BASE + 2 * TAS_GRID_PAD_COLUMNS, &(SDL_FRect){0}));
    CHECK(desktop_clay_bounds(ui.clay, HIT_TAS_CELL, 49, TAS_GRID_PAD_BASE + 3 * TAS_GRID_PAD_COLUMNS + 7,
                              &(SDL_FRect){0}));
    render_hit_capacity_stress(&ui);
    CHECK(desktop_clay_bounds(ui.clay, HIT_NONE, 2047, 0, &(SDL_FRect){0}));
    CHECK(!desktop_clay_bounds(ui.clay, HIT_NONE, 2048, 0, &(SDL_FRect){0}));
    editor->edit_player = 0;
    editor->column = TAS_GRID_PAD_BASE;
    SDL_SetWindowSize(window, 900, 760);

    render_editor(&ui);
    CHECK(cell_event(&ui, SDL_MOUSEBUTTONDOWN, 1, TAS_GRID_PAD_BASE));
    CHECK(nes_tas_edit_active(project));
    focus(&ui, false);
    CHECK(!nes_tas_edit_active(project) && !editor->painting);
    CHECK(nes_tas_project_frame(project, 1)->pads[0] == 1);
    focus(&ui, true);
    CHECK(send_key(&ui, SDL_SCANCODE_Z, KMOD_LCTRL));
    CHECK(nes_tas_project_frame(project, 1)->pads[0] == 0);
    CHECK(nes_tas_session_set_read_only(session, true) == NES_MOVIE_OK);
    uint64_t revision = nes_tas_project_revision(project);
    desktop_tas_cell_down(&ui, 1, TAS_GRID_PAD_BASE);
    CHECK(nes_tas_project_revision(project) == revision);
    CHECK(!nes_tas_edit_active(project));
    CHECK(nes_tas_session_set_read_only(session, false) == NES_MOVIE_OK);
    CHECK(nes_tas_session_set_recording(session, true, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);
    SDL_Event controller = {.type = SDL_CONTROLLERBUTTONDOWN};
    controller.cbutton.button = SDL_CONTROLLER_BUTTON_A;
    CHECK(desktop_tas_event(&ui, &controller));
    CHECK(nes_tas_session_set_recording(session, false, NES_TAS_RECORD_OVERWRITE, 1) == NES_MOVIE_OK);

    /* Command-line, drop and menu opens share the native panel route. Raising or
     * closing that window must
     * preserve the project and its edited input. */
    ui.native_windows = true;
    CHECK(frontend_desktop_open_panel(&ui, TAS_PANEL));
    FrontendDesktopUi *tool = ui.tools;
    CHECK(tool && tool->parent == &ui && !tool->next && !ui.panel_open);
    CHECK(tool->window != ui.window && tool->renderer != ui.renderer);
    CHECK(tool->panel_open && tool->panel_id == TAS_PANEL);
    Uint32 flags = SDL_GetWindowFlags(tool->window);
    CHECK((flags & SDL_WINDOW_RESIZABLE) && !(flags & SDL_WINDOW_BORDERLESS));
    int width, height, x, y, minimum_width, minimum_height;
    SDL_GetWindowSize(tool->window, &width, &height);
    CHECK(width == 900 && height == 760);
    SDL_GetWindowMinimumSize(tool->window, &minimum_width, &minimum_height);
    CHECK(minimum_width == 900 && minimum_height == 640);
    render_editor(tool);
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_SCRIPT, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_DISCARD, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_ROW, 0, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_SCROLL, 0, 0));
    unsigned rows_before_resize = tool->tas_editor->visible_rows;
    SDL_SetWindowPosition(tool->window, 113, 127);
    SDL_GetWindowPosition(tool->window, &x, &y);
    CHECK(x == 113 && y == 127);
    SDL_SetWindowSize(tool->window, 1080, 860);
    render_editor(tool);
    CHECK(tool->tas_editor->visible_rows > rows_before_resize);
    SDL_SetWindowSize(tool->window, 900, 640);
    tool->tas_editor->cursor = 4;
    desktop_tas_action(tool, TAS_ACTION_SIDEBAR_BRANCHES);
    render_editor(tool);
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_BRANCH_JUMP, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_BRANCH_STORE, 0));
    desktop_tas_action(tool, TAS_ACTION_SIDEBAR_MARKERS);
    render_editor(tool);
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_MARKER_PREV, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_MARKER_REMOVE, 0));
    desktop_tas_action(tool, TAS_ACTION_SIDEBAR_INPUT);
    render_editor(tool);
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_PATTERN, 0));
    CHECK(hit_inside_window(tool, HIT_TAS_ACTION, TAS_ACTION_HEADER_BASE + 7, 0));
    SDL_GetWindowSize(tool->window, &width, &height);
    CHECK(width == 900 && height == 640);
    CHECK(frontend_desktop_open_panel(&ui, TAS_PANEL));
    CHECK(frontend_desktop_open_panel(tool, TAS_PANEL));
    CHECK(ui.tools == tool && !tool->next && !ui.panel_open);
    ui.open_menu = 5;
    ui.menu_depth = 0;
    DesktopMenuItem items[128];
    int menu_count = desktop_menu_items(&ui, items);
    ui.menu_row = -1;
    for (int row = 0; row < menu_count; ++row) {
        if (items[row].kind == 1 && items[row].id == TAS_PANEL) {
            ui.menu_row = row;
            break;
        }
    }
    CHECK(ui.menu_row >= 0);
    CHECK(send_key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE));
    CHECK(ui.open_menu == -1 && ui.tools == tool && !tool->next && !ui.panel_open);
    tool->tas_editor->cursor = 0;
    tool->tas_editor->column = TAS_GRID_PAD_BASE;
    uint8_t before_native_edit = nes_tas_project_frame(project, 0)->pads[0];
    SDL_Event native_key = {.type = SDL_KEYDOWN};
    native_key.key.windowID = SDL_GetWindowID(tool->window);
    native_key.key.keysym.scancode = SDL_SCANCODE_SPACE;
    CHECK(frontend_desktop_handle_event(&ui, &native_key));
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == (uint8_t)(before_native_edit ^ 1));
    SDL_Event close = {.type = SDL_WINDOWEVENT};
    close.window.windowID = SDL_GetWindowID(tool->window);
    close.window.event = SDL_WINDOWEVENT_CLOSE;
    CHECK(frontend_desktop_handle_event(&ui, &close));
    CHECK(!ui.tools && !ui.panel_open && !ui.quit_requested);
    CHECK(nes_tas_session_active(session) && nes_tas_session_project(session) == project);
    CHECK(nes_tas_project_frame(project, 0)->pads[0] == (uint8_t)(before_native_edit ^ 1));
    CHECK(frontend_desktop_open_panel(&ui, TAS_PANEL));
    CHECK(ui.tools && !ui.tools->next && nes_tas_session_project(session) == project);
    close.window.windowID = SDL_GetWindowID(ui.tools->window);
    CHECK(frontend_desktop_handle_event(&ui, &close));
    CHECK(!ui.tools && nes_tas_session_active(session));
    ui.native_windows = false;
    CHECK(frontend_desktop_open_panel(&ui, TAS_PANEL));
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_STOP));
    CHECK(nes_tas_session_active(session));
    render_editor(&ui);
    CHECK(click_action(&ui, TAS_ACTION_DISCARD));
    CHECK(!nes_tas_session_active(session));
cleanup:
    frontend_set_file_chooser(NULL, NULL);
    if (ui_initialized) {
        frontend_desktop_shutdown(&ui);
    }
    if (runtime_initialized) {
        frontend_execution_shutdown(&runtime);
    }
    frontend_commands_reset();
    frontend_panels_reset();
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    if (video_initialized) {
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
    }
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&image);
    (void)joypad_set_adapter(old_adapter);
    (void)nes_file_remove(path);
    (void)nes_file_remove(lua_path);
    printf("TAS editor input: keyboard, drag, scale, native window, focus and read-only checks, %d failures\n", failures);
    return failures;
}
