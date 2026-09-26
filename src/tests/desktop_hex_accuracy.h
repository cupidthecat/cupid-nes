/*
 * desktop_hex_accuracy.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native hex editor navigation, dialogs and window lifetime. */
#include "../ui/hex_frontend.h"
#include "../debugger/memory_editor.h"

static void hex_select(unsigned address, unsigned count) {
    char text[32];
    snprintf(text, sizeof(text), "$%04X", address);
    CHECK(frontend_panel_action(HEX_FRONTEND_PANEL, HEX_ADDRESS, text, 0, NULL, 0));
    snprintf(text, sizeof(text), "%u", count);
    CHECK(frontend_panel_action(HEX_FRONTEND_PANEL, HEX_COUNT, text, 0, NULL, 0));
    CHECK(frontend_panel_action(HEX_FRONTEND_PANEL, HEX_SELECT, NULL, 0, NULL, 0));
}

static void hex_selection(FrontendDesktopUi *tool, unsigned cursor, unsigned count) {
    FrontendPanelControl c;
    CHECK(memory_control(tool, HEX_SELECTION, &c) >= 0 && c.selected == (int)cursor);
    char expected[24];
    snprintf(expected, sizeof(expected), "%u", count);
    CHECK(memory_control(tool, HEX_COUNT, &c) >= 0 && !strcmp(c.value, expected));
}

static void hex_release(FrontendDesktopUi *tool) {
    SDL_Event event = {.type = SDL_MOUSEBUTTONUP};
    event.button.windowID = SDL_GetWindowID(tool->window);
    event.button.button = SDL_BUTTON_LEFT;
    (void)frontend_desktop_handle_event(tool->parent, &event);
    CHECK(!tool->hex_selecting);
}

static void hex_motion(FrontendDesktopUi *tool, unsigned address, int ascii) {
    SDL_FRect bounds;
    bool found = desktop_clay_bounds(tool->clay, HIT_HEX_BYTE, (int)address, ascii, &bounds);
    CHECK(found);
    if (!found) return;
    SDL_Event event = {.type = SDL_MOUSEMOTION};
    event.motion.windowID = SDL_GetWindowID(tool->window);
    event.motion.state = SDL_BUTTON_LMASK;
    event.motion.x = (int)((bounds.x + bounds.w / 2) * tool->ui_scale);
    event.motion.y = (int)((bounds.y + bounds.h / 2) * tool->ui_scale);
    (void)frontend_desktop_handle_event(tool->parent, &event);
}

static void hex_navigation(FrontendDesktopUi *tool) {
    hex_select(0x20, 1);
    render(tool);
    click_control(tool, HIT_HEX_BYTE, 0x20, 0);
    hex_motion(tool, 0x24, 1);
    hex_selection(tool, 0x24, 5);
    hex_release(tool);
    hex_motion(tool, 0x28, 0);
    hex_selection(tool, 0x24, 5);
    SDL_SetModState(KMOD_LSHIFT);
    click_control(tool, HIT_HEX_BYTE, 0x27, 1);
    SDL_SetModState(KMOD_NONE);
    hex_release(tool);
    hex_selection(tool, 0x27, 8);
    click_control(tool, HIT_HEX_BYTE, 0x20, 1);
    hex_release(tool);
    hex_selection(tool, 0x20, 1);

    FrontendBindingProfile *profile = frontend_settings_active_profile(tool->settings);
    FrontendHostBinding previous = profile->shortcuts[FRONTEND_SHORTCUT_PAUSE];
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE] = (FrontendHostBinding){.key = SDL_SCANCODE_RIGHT};
    key(tool, SDL_SCANCODE_RIGHT, KMOD_NONE);
    hex_selection(tool, 0x21, 1);
    CHECK(tool->execution->execution.paused && debugger_is_paused());
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE] = previous;
    key(tool, SDL_SCANCODE_DOWN, KMOD_LSHIFT);
    hex_selection(tool, 0x31, 17);
    key(tool, SDL_SCANCODE_UP, KMOD_NONE);
    hex_selection(tool, 0x21, 1);
    key(tool, SDL_SCANCODE_LEFT, KMOD_NONE);
    hex_selection(tool, 0x20, 1);
    key(tool, SDL_SCANCODE_END, KMOD_NONE);
    hex_selection(tool, 0x2F, 1);
    key(tool, SDL_SCANCODE_HOME, KMOD_NONE);
    hex_selection(tool, 0x20, 1);
    key(tool, SDL_SCANCODE_PAGEDOWN, KMOD_NONE);
    hex_selection(tool, 0x120, 1);
    key(tool, SDL_SCANCODE_PAGEUP, KMOD_NONE);
    hex_selection(tool, 0x20, 1);
    key(tool, SDL_SCANCODE_END, KMOD_LCTRL);
    hex_selection(tool, 0x7FF, 1);
    key(tool, SDL_SCANCODE_RIGHT, KMOD_NONE);
    hex_selection(tool, 0x7FF, 1);
    key(tool, SDL_SCANCODE_HOME, KMOD_RCTRL);
    key(tool, SDL_SCANCODE_LEFT, KMOD_NONE);
    hex_selection(tool, 0, 1);
    key(tool, SDL_SCANCODE_A, KMOD_LCTRL);
    hex_selection(tool, 0x7FF, 0x800);

    hex_select(0x20, 1);
    render(tool);
    click_control(tool, HIT_HEX_BYTE, 0x20, 0);
    focus_event(tool->parent, tool->window, false);
    CHECK(!tool->hex_selecting);
    focus_event(tool->parent, tool->window, true);
    hex_motion(tool, 0x30, 0);
    hex_selection(tool, 0x20, 1);
}

static void hex_dialogs(FrontendDesktopUi *tool) {
    hex_select(0x20, 1);
    render(tool);
    click_control(tool, HIT_HEX_BYTE, 0x20, 0);
    hex_release(tool);
    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(tool->edit_text_active);
    CHECK(SDL_SetClipboardText("43 75 70 69 64") == 0);
    key(tool, SDL_SCANCODE_V, KMOD_LCTRL);
    hex_selection(tool, 0x20, 1);
    CHECK(read_mem(0x20) == 0x20);
    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
    memory_click(tool, HEX_APPLY);
    hex_selection(tool, 0x20, 5);
    CHECK(read_mem(0x20) == 'C' && read_mem(0x24) == 'd');
    key(tool, SDL_SCANCODE_C, KMOD_RCTRL);
    char *copied = SDL_GetClipboardText();
    CHECK(copied && !strcmp(copied, "43 75 70 69 64"));
    SDL_free(copied);
    memory_click(tool, HEX_TEXT_MODE);
    key(tool, SDL_SCANCODE_C, KMOD_LCTRL);
    copied = SDL_GetClipboardText();
    CHECK(copied && !strcmp(copied, "Cupid"));
    SDL_free(copied);
    CHECK(SDL_SetClipboardText("Cat") == 0);
    key(tool, SDL_SCANCODE_V, KMOD_LCTRL);
    CHECK(read_mem(0x20) == 'C' && read_mem(0x21) == 'a' && read_mem(0x22) == 't');
    hex_selection(tool, 0x20, 3);
    memory_click(tool, HEX_TEXT_MODE);

    key(tool, SDL_SCANCODE_G, KMOD_LCTRL);
    CHECK(tool->edit_text_active);
    strcpy(tool->edit_text, "$0040");
    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
    memory_edit(tool, HEX_COUNT, "1");
    memory_click(tool, HEX_SELECT);
    hex_selection(tool, 0x40, 1);
    key(tool, SDL_SCANCODE_F, KMOD_RCTRL);
    CHECK(tool->edit_text_active);
    strcpy(tool->edit_text, "43 44");
    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
    key(tool, SDL_SCANCODE_F3, KMOD_NONE);
    hex_selection(tool, 0x43, 2);

    memory_click(tool, HEX_SPACE);
    CHECK(tool->choice_open);
    key(tool, SDL_SCANCODE_DOWN, KMOD_NONE);
    hex_selection(tool, 0x43, 2);
    key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    CHECK(!tool->choice_open);
    tool->open_menu = 5;
    tool->menu_depth = tool->menu_row = 0;
    render(tool);
    int panel_row = tool->panel_row;
    FrontendBindingProfile *profile = frontend_settings_active_profile(tool->settings);
    FrontendHostBinding previous = profile->shortcuts[FRONTEND_SHORTCUT_PAUSE];
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE] = (FrontendHostBinding){.key = SDL_SCANCODE_DOWN};
    key(tool, SDL_SCANCODE_DOWN, KMOD_NONE);
    hex_selection(tool, 0x43, 2);
    CHECK(tool->open_menu == 5 && tool->menu_row == 1 && tool->panel_row == panel_row);
    CHECK(tool->execution->execution.paused && debugger_is_paused());
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE] = previous;
    tool->open_menu = 5;
    debugger_pause();
    frontend_execution_sync_debugger(tool->execution);
    key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    CHECK(tool->open_menu == -1 && tool->panel_open);
    memory_click(tool, HEX_LIVE);
    CHECK(SDL_SetClipboardText("FF") == 0);
    key(tool, SDL_SCANCODE_V, KMOD_LCTRL);
    CHECK(read_mem(0x43) == 0x43);
    for (unsigned i = 0; i < 50; ++i) {
        key(tool, SDL_SCANCODE_TAB, i < 25 ? KMOD_NONE : KMOD_LSHIFT);
        FrontendPanelControl controls[64];
        FrontendPanelModel model = {.controls = controls, .capacity = 64};
        CHECK(frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, NULL, 0));
        CHECK(tool->panel_row >= 0 && (size_t)tool->panel_row < model.count);
        if (tool->panel_row >= 0 && (size_t)tool->panel_row < model.count)
            CHECK(controls[tool->panel_row].enabled && !controls[tool->panel_row].read_only);
    }
    memory_click(tool, HEX_LIVE);
}

static void hex_files(FrontendDesktopUi *tool) {
    const uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CHECK(nes_file_write_atomic("build/chosen path.bin", data, sizeof(data)) == NES_FILE_OK);
    hex_select(0x80, 1);
    FrontendPanelControl c;
    const unsigned paths[] = {HEX_IMPORT_PATH, HEX_EXPORT_PATH};
    for (unsigned i = 0; i < 2; ++i) {
        CHECK(frontend_panel_action(HEX_FRONTEND_PANEL, paths[i], "build/retained.bin", 0, NULL, 0));
        ChooserProbe probe = {.save = i != 0, .type = i ? FRONTEND_SAVE_MEMORY : FRONTEND_OPEN_MEMORY};
        frontend_set_file_chooser(choose_path, &probe);
        memory_click(tool, paths[i]);
        CHECK(probe.calls == 1);
        CHECK(memory_control(tool, paths[i], &c) >= 0 && !strcmp(c.value, "build/retained.bin"));
        probe.fail = true;
        memory_click(tool, paths[i]);
        CHECK(probe.calls == 2 && strstr(tool->status, "Chooser test failure"));
        CHECK(memory_control(tool, paths[i], &c) >= 0 && !strcmp(c.value, "build/retained.bin"));
        probe.fail = false;
        probe.accept = true;
        memory_click(tool, paths[i]);
        CHECK(probe.calls == 3);
        CHECK(memory_control(tool, paths[i], &c) >= 0 && !strcmp(c.value, "build/chosen path.bin"));
        frontend_set_file_chooser(NULL, NULL);
        memory_click(tool, i ? HEX_EXPORT : HEX_IMPORT);
    }
    CHECK(read_mem(0x80) == 0xDE && read_mem(0x83) == 0xEF);
    hex_selection(tool, 0x80, 4);
    uint8_t *bytes = NULL;
    size_t size = 0;
    CHECK(nes_file_read_all("build/chosen path.bin", 16, &bytes, &size) == NES_FILE_OK);
    CHECK(bytes && size == sizeof(data) && !memcmp(bytes, data, sizeof(data)));
    free(bytes);
    CHECK(nes_file_remove("build/chosen path.bin") == NES_FILE_OK);
}

static void hex_windows(FrontendDesktopUi *ui, DebugFrontend *debug) {
    uint16_t original_pc = cpu.pc;
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    debugger_pause();
    frontend_execution_sync_debugger(ui->execution);
    debug_watch_reset();
    for (uint16_t address = 0; address < 0x800; ++address) write_mem(address, (uint8_t)address);
    menu_activate(ui, 5, HEX_FRONTEND_PANEL);
    FrontendDesktopUi *tool = ui->tools;
    CHECK(tool && tool->panel_id == HEX_FRONTEND_PANEL && tool->parent == ui);
    if (!tool) return;
    CHECK(desktop_open_window(ui, 1, HEX_FRONTEND_PANEL) == tool);
    CHECK(frontend_panel_action(HEX_FRONTEND_PANEL, HEX_SPACE, NULL, DEBUG_MEMORY_RAM, NULL, 0));
    for (unsigned dpi = 0; dpi < 3; ++dpi) {
        float scale = dpi == 0 ? 1 : dpi == 1 ? 1.5f : 2;
        memory_window_size(tool, 900, 820, scale);
        hex_navigation(tool);
        memory_click(tool, HEX_BASELINE);
        write_mem(0x21, 0xA5);
        render(tool);
        CHECK(desktop_clay_text_visible(tool->clay, "A5*"));
        CHECK(desktop_clay_text_visible(tool->clay, "Hex / Memory Editor"));
        const unsigned labels[] = {HEX_ADDRESS, HEX_COUNT, HEX_LIVE, HEX_BASELINE, HEX_BYTES,
                                   HEX_TEXT_MODE, HEX_APPLY, HEX_PATTERN, HEX_WATCH, HEX_IMPORT_PATH,
                                   HEX_IMPORT, HEX_EXPORT_PATH, HEX_EXPORT};
        for (size_t i = 0; i < sizeof(labels) / sizeof(*labels); ++i) {
            FrontendPanelControl c;
            CHECK(memory_control(tool, labels[i], &c) >= 0);
            CHECK(desktop_clay_text_visible(tool->clay, c.label));
        }
        char path[96];
        snprintf(path, sizeof(path), "build/desktop-hex-%u.png", dpi);
        screenshot(tool, path);
        write_mem(0x21, 0x21);
        memory_window_size(tool, 900, 748, scale);
        memory_click(tool, HEX_BYTES);
        CHECK(tool->edit_text_active);
        key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
        snprintf(path, sizeof(path), "build/desktop-hex-boundary-%u.png", dpi);
        screenshot(tool, path);
        memory_window_size(tool, 640, 480, scale);
        memory_edit(tool, HEX_PATTERN, "43 44");
        tool->panel_row = memory_control(tool, HEX_ROW + 2, NULL);
        render(tool);
        CHECK(desktop_clay_text_visible(tool->clay, "$0020"));
        snprintf(path, sizeof(path), "build/desktop-hex-compact-%u.png", dpi);
        screenshot(tool, path);
    }
    memory_window_size(tool, 900, 820, 1);
    hex_dialogs(tool);
    hex_files(tool);
    hex_select(0x80, 3);
    memory_click(tool, HEX_WATCH);
    FrontendDesktopUi *watch = ui->tools;
    CHECK(watch && watch != tool && watch->panel_id == WATCH_FRONTEND_PANEL);
    CHECK(desktop_open_window(ui, 1, HEX_FRONTEND_PANEL) == tool);
    DebugWatchSample first = {0};
    CHECK(debug_watch_at(0, &first) && first.address == 0x80 && first.spec.width == 3);
    hex_select(0x90, 5);
    memory_click(tool, HEX_WATCH);
    CHECK(ui->tools == watch && debugger_watch_count() == 6);
    DebugWatchSample selected = {0};
    CHECK(debug_watch_at(1, &selected) && selected.address == 0x90 && selected.id != first.id);
    FrontendPanelControl c;
    CHECK(watch && memory_control(watch, WATCH_SELECTION, &c) >= 0 && strstr(c.value, "$0090"));
    CHECK(debug_watch_at(0, &selected) && selected.id == first.id);
    frontend_panel_set_session_active(false);
    render(tool);
    CHECK(desktop_clay_contains_text(tool->clay, "unavailable"));
    CHECK(!frontend_panel_action(HEX_FRONTEND_PANEL, HEX_PASTE, NULL, 0, NULL, 0));
    frontend_panel_set_session_active(true);
    debug_frontend_image_changed(debug);
    render(tool);
    CHECK(memory_control(tool, HEX_APPLY, &c) >= 0 && !c.enabled);
    SDL_Event close = {.type = SDL_WINDOWEVENT};
    close.window.windowID = SDL_GetWindowID(tool->window);
    close.window.event = SDL_WINDOWEVENT_CLOSE;
    CHECK(frontend_desktop_handle_event(ui, &close));
    CHECK(frontend_desktop_handle_event(ui, &close));
    CHECK(ui->tools == watch && !ui->quit_requested && debugger_watch_count() == 6);
    desktop_close_windows(ui);
    debug_watch_reset();
    cpu.pc = original_pc;
    debug_frontend_image_changed(debug);
}
