/*
 * desktop_memory_accuracy.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory search windows and event routing. Included after desktop test helpers. */
#include "../ui/memory_search_frontend.h"
#include "../ui/watch_frontend.h"
#include "../debugger/memory_watch.h"

static int memory_control(FrontendDesktopUi *tool, unsigned id, FrontendPanelControl *out) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(tool->panel_id, &model, NULL, 0)) {
        CHECK(false);
        return -1;
    }

    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id == id) {
            if (out) {
                *out = controls[i];
            }

            return (int)i;
        }
    }

    CHECK(false);
    return -1;
}

static void memory_click(FrontendDesktopUi *tool, unsigned id) {
    int index = memory_control(tool, id, NULL);
    if (index >= 0) {
        tool->panel_row = index;
        render(tool);
        click_control(tool, HIT_PANEL, index, 0);
    }
}

static void memory_window_size(FrontendDesktopUi *tool, int width, int height, float scale) {
    tool->ui_scale = scale;
    SDL_SetWindowSize(tool->window, (int)(width * scale), (int)(height * scale));
    SDL_RenderSetLogicalSize(tool->renderer, (int)(width * scale), (int)(height * scale));
    render(tool);
}

static void memory_windows(FrontendDesktopUi *ui) {
    CheatFrontend *editor = cheat_frontend_create("build/");
    MemoryToolsFrontend *memory = memory_tools_create(editor);
    CHECK(editor && memory && cheat_frontend_register_ui(editor) && memory_tools_register(memory));
    if (!editor || !memory) {
        memory_tools_destroy(memory);
        cheat_frontend_destroy(editor);
        return;
    }

    CHECK(cheats_clear() == CHEAT_OK);
    for (unsigned id = MEMORY_SEARCH_PANEL; id <= CHEAT_FINDER_PANEL; ++id) {
        for (uint16_t address = 0; address < 32; ++address) {
            write_mem(address, (uint8_t)(0x40 + address));
        }

        menu_activate(ui, 5, id);
        FrontendDesktopUi *tool = ui->tools;
        CHECK(tool && tool->panel_id == id && tool->parent == ui);
        if (!tool) {
            continue;
        }

        CHECK(desktop_open_window(ui, 1, id) == tool);
        CHECK(frontend_panel_action(id, MEMORY_SEARCH_LAST, "001F", 0, NULL, 0));
        memory_window_size(tool, 900, 820, 1);
        memory_click(tool, MEMORY_SEARCH_START);
        render(tool);
        CHECK(desktop_clay_contains_text(tool->clay, "Current"));
        CHECK(desktop_clay_contains_text(tool->clay, "Initial"));
        CHECK(desktop_clay_contains_text(tool->clay, "32 of 32"));
        memory_click(tool, MEMORY_SEARCH_ROW + 1);
        memory_click(tool, MEMORY_SEARCH_COPY);
        char *copied = SDL_GetClipboardText();
        CHECK(copied && strstr(copied, "0001\t41\t41\t41\t0"));
        SDL_free(copied);
        memory_click(tool, MEMORY_SEARCH_WATCH);
        CHECK(ui->tools && ui->tools != tool && ui->tools->panel_id == WATCH_FRONTEND_PANEL);
        if (ui->tools && ui->tools != tool) {
            FrontendPanelControl selection;
            CHECK(memory_control(ui->tools, WATCH_SELECTION, &selection) >= 0 && strstr(selection.value, "$0001"));
            CHECK(desktop_open_window(ui, 1, id) == tool);
        }
        DebugWatch watch;
        uint32_t sample;
        CHECK(debugger_watch_at(debugger_watch_count() - 1, &watch, &sample));
        CHECK(watch.address == 1 && watch.size == 1 && sample == 0x41);
        key(tool, SDL_SCANCODE_PAGEDOWN, KMOD_NONE);
        FrontendPanelControl row;
        CHECK(memory_control(tool, MEMORY_SEARCH_ROW, &row) >= 0 && !strcmp(row.value, "000C"));
        key(tool, SDL_SCANCODE_PAGEUP, KMOD_NONE);
        CHECK(memory_control(tool, MEMORY_SEARCH_ROW, &row) >= 0 && !strcmp(row.value, "0000"));

        /* Focus must stay on controls that can be operated. In particular, the
         * Memory Search model also contains Cheat Finder's hidden Test action. */
        for (unsigned step = 0; step < 50; ++step) {
            key(tool, SDL_SCANCODE_TAB, KMOD_NONE);
            FrontendPanelControl controls[64];
            FrontendPanelModel model = {.controls = controls, .capacity = 64};
            CHECK(frontend_panel_snapshot(id, &model, NULL, 0));
            CHECK(tool->panel_row >= 0 && (size_t)tool->panel_row < model.count);
            if (tool->panel_row >= 0 && (size_t)tool->panel_row < model.count) {
                CHECK(controls[tool->panel_row].enabled && !controls[tool->panel_row].read_only);
            }
        }

        for (unsigned dpi = 0; dpi < 3; ++dpi) {
            float scale = dpi == 0 ? 1 : dpi == 1 ? 1.5f : 2;
            memory_window_size(tool, 900, 820, scale);
            memory_click(tool, MEMORY_SEARCH_ROW + 1);
            memory_click(tool, MEMORY_SEARCH_DESCRIPTION);
            CHECK(tool->edit_text_active);
            strcpy(tool->edit_text, "Memory window regression");
            key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
            CHECK(!tool->edit_text_active);
            render(tool);
            int send = memory_control(tool, MEMORY_SEARCH_SEND, NULL);
            SDL_FRect bounds;
            CHECK(desktop_clay_bounds(tool->clay, HIT_PANEL, send, 0, &bounds));
            CHECK(bounds.y + bounds.h <= 820 - 48);
            char path[96];
            snprintf(path, sizeof(path), "build/desktop-memory-%u-%u.png", id, dpi);
            screenshot(tool, path);

            /* At the compact layout boundary, lower actions must remain
             * reachable even when the result grid no longer fits. */
            memory_window_size(tool, 900, 748, scale);
            memory_click(tool, MEMORY_SEARCH_DESCRIPTION);
            CHECK(tool->edit_text_active);
            key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
            memory_window_size(tool, 640, 480, scale);
            tool->panel_row = memory_control(tool, MEMORY_SEARCH_ROW + 1, NULL);
            render(tool);
            CHECK(desktop_clay_contains_text(tool->clay, "Current 41"));
            CHECK(desktop_clay_contains_text(tool->clay, "Initial 41"));
            snprintf(path, sizeof(path), "build/desktop-memory-compact-%u-%u.png", id, dpi);
            screenshot(tool, path);
        }

        memory_window_size(tool, 900, 820, 1);
        memory_click(tool, MEMORY_SEARCH_ROW + 1);
        memory_click(tool, MEMORY_SEARCH_SEND);
        CHECK(cheats_count() == 0);
        CHECK(ui->tools && ui->tools != tool && ui->tools->panel_id == CHEATS_FRONTEND_PANEL);
        if (ui->tools && ui->tools != tool) {
            render(ui->tools);
            CHECK(desktop_clay_contains_text(ui->tools->clay, "0001:41"));
        }

        if (id == CHEAT_FINDER_PANEL) {
            memory_click(tool, MEMORY_SEARCH_TEST);
            CHECK(cheats_count() == 1);
            CHECK(cheats_clear() == CHEAT_OK);
        }

        CHECK(frontend_machine_soft_reset());
        render(tool);
        CHECK(memory_control(tool, MEMORY_SEARCH_FILTER, &row) >= 0 && !row.enabled);
        CHECK(desktop_clay_contains_text(tool->clay, "No snapshot"));
        SDL_Event close = {.type = SDL_WINDOWEVENT};
        close.window.windowID = SDL_GetWindowID(tool->window);
        close.window.event = SDL_WINDOWEVENT_CLOSE;
        CHECK(frontend_desktop_handle_event(ui, &close));
        CHECK(frontend_desktop_handle_event(ui, &close));
        desktop_close_windows(ui);
    }

    memory_tools_destroy(memory);
    cheat_frontend_destroy(editor);
    FrontendPanelInfo info;
    CHECK(!frontend_panel_get(MEMORY_SEARCH_PANEL, &info) && !frontend_panel_get(CHEAT_FINDER_PANEL, &info));
}

static void memory_edit(FrontendDesktopUi *tool, unsigned id, const char *value) {
    memory_click(tool, id);
    CHECK(tool->edit_text_active && strlen(value) < sizeof(tool->edit_text));
    snprintf(tool->edit_text, sizeof(tool->edit_text), "%s", value);
    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(!tool->edit_text_active);
}

static void watch_windows(FrontendDesktopUi *ui, DebugFrontend *debug) {
    uint16_t original_pc = cpu.pc;
    debug_watch_reset();
    debug_frontend_image_changed(debug);
    for (uint16_t address = 0; address < 64; ++address) write_mem(address, (uint8_t)address);
    menu_activate(ui, 5, WATCH_FRONTEND_PANEL);
    FrontendDesktopUi *tool = ui->tools;
    CHECK(tool && tool->panel_id == WATCH_FRONTEND_PANEL && tool->parent == ui);
    if (!tool) return;
    CHECK(desktop_open_window(ui, 1, WATCH_FRONTEND_PANEL) == tool);
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_SPACE, NULL, DEBUG_MEMORY_RAM, NULL, 0));
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_WIDTH, NULL, 0, NULL, 0));
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_FORMAT, NULL, DEBUG_WATCH_HEX, NULL, 0));
    memory_window_size(tool, 900, 820, 1);
    memory_edit(tool, WATCH_EXPRESSION, "$0000");
    memory_edit(tool, WATCH_LABEL, "Array");
    memory_edit(tool, WATCH_COUNT, "12");
    memory_click(tool, WATCH_ADD);
    CHECK(debugger_watch_count() == 12);
    FrontendPanelControl row;
    key(tool, SDL_SCANCODE_PAGEDOWN, KMOD_NONE);
    CHECK(memory_control(tool, WATCH_ROW, &row) >= 0 && !strcmp(row.value, "RAM:000A"));
    memory_click(tool, WATCH_ROW + 1);
    DebugWatchSample selected = {0};
    CHECK(debug_watch_at(11, &selected));
    uint32_t selected_id = selected.id;
    memory_click(tool, WATCH_SORT + DEBUG_WATCH_SORT_CURRENT);
    memory_click(tool, WATCH_SORT + DEBUG_WATCH_SORT_CURRENT);
    CHECK(memory_control(tool, WATCH_ROW, &row) >= 0 && row.selected && !strcmp(row.value, "RAM:000B"));
    memory_click(tool, WATCH_MOVE_DOWN);
    CHECK(debug_watch_at(1, &selected) && selected.id == selected_id);

    memory_click(tool, WATCH_WIDTH);
    CHECK(tool->choice_open);
    render(tool);
    click_control(tool, HIT_CHOICE, 3, 0);
    CHECK(!tool->choice_open);
    memory_click(tool, WATCH_FORMAT);
    render(tool);
    click_control(tool, HIT_CHOICE, DEBUG_WATCH_BINARY, 0);
    const char *expression = "$0020 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0 + #0";
    memory_edit(tool, WATCH_EXPRESSION, expression);
    memory_edit(tool, WATCH_LABEL, "Selected value keeps its complete history");
    memory_click(tool, WATCH_UPDATE);
    memory_click(tool, WATCH_PREVIOUS);
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_FREEZE, NULL, 1, NULL, 0));
    write_mem(0x20, 0x31);
    CHECK(memory_control(tool, WATCH_ROW + 1, &row) >= 0 && !strcmp(row.label, "Changed watch"));
    CHECK(debug_watch_at(1, &selected) && selected.current == 0x23222131 && selected.previous == 0x23222120);
    const char *current = "00100011001000100010000100110001";
    const char *previous = "00100011001000100010000100100000";
    for (unsigned dpi = 0; dpi < 3; ++dpi) {
        float scale = dpi == 0 ? 1 : dpi == 1 ? 1.5f : 2;
        memory_window_size(tool, 900, 820, scale);
        CHECK(desktop_clay_text_visible(tool->clay, current));
        CHECK(desktop_clay_text_visible(tool->clay, previous));
        int export_path = memory_control(tool, WATCH_EXPORT_PATH, NULL);
        SDL_FRect bounds;
        CHECK(desktop_clay_bounds(tool->clay, HIT_PANEL, export_path, 0, &bounds));
        CHECK(bounds.y + bounds.h <= 820 - 48);
        char path[96];
        snprintf(path, sizeof(path), "build/desktop-watch-%u.png", dpi);
        screenshot(tool, path);
        memory_window_size(tool, 900, 748, scale);
        memory_click(tool, WATCH_LABEL);
        CHECK(tool->edit_text_active);
        key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
        memory_window_size(tool, 640, 480, scale);
        tool->panel_row = memory_control(tool, WATCH_ROW + 1, NULL);
        render(tool);
        CHECK(desktop_clay_text_visible(tool->clay, current));
        CHECK(desktop_clay_text_visible(tool->clay, previous));
        snprintf(path, sizeof(path), "build/desktop-watch-compact-%u.png", dpi);
        screenshot(tool, path);
    }
    memory_window_size(tool, 900, 820, 1);
    memory_click(tool, WATCH_COPY);
    char *copied = SDL_GetClipboardText();
    CHECK(copied && strstr(copied, "id,space,address,expression,width,format,enabled") && strstr(copied, current));
    SDL_free(copied);
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_EXPORT_PATH, "build/watch-before-choice.csv", 0, NULL, 0));
    ChooserProbe probe = {.save = true, .type = FRONTEND_SAVE_CSV};
    frontend_set_file_chooser(choose_path, &probe);
    memory_click(tool, WATCH_EXPORT_PATH);
    CHECK(probe.calls == 1);
    CHECK(memory_control(tool, WATCH_EXPORT_PATH, &row) >= 0 && !strcmp(row.value, "build/watch-before-choice.csv"));
    probe.fail = true;
    memory_click(tool, WATCH_EXPORT_PATH);
    CHECK(probe.calls == 2 && strstr(tool->status, "Chooser test failure"));
    CHECK(memory_control(tool, WATCH_EXPORT_PATH, &row) >= 0 && !strcmp(row.value, "build/watch-before-choice.csv"));
    probe.fail = false;
    probe.accept = true;
    memory_click(tool, WATCH_EXPORT_PATH);
    CHECK(probe.calls == 3);
    frontend_set_file_chooser(NULL, NULL);
    memory_click(tool, WATCH_EXPORT);
    uint8_t *bytes = NULL;
    size_t size = 0;
    CHECK(nes_file_read_all("build/chosen path.bin", DEBUG_WATCH_EXPORT_LIMIT, &bytes, &size) == NES_FILE_OK);
    CHECK(bytes && size > 16 && !memcmp(bytes, "id,space,address", 16));
    free(bytes);
    CHECK(nes_file_remove("build/chosen path.bin") == NES_FILE_OK);
    memory_click(tool, WATCH_TOGGLE);
    CHECK(debug_watch_at(1, &selected) && selected.id == selected_id && !selected.spec.enabled);
    memory_click(tool, WATCH_TOGGLE);
    memory_edit(tool, WATCH_EXPRESSION, "$FFFF + #1");
    memory_click(tool, WATCH_UPDATE);
    CHECK(debug_watch_at(1, &selected) && selected.address == 0x20 && !strcmp(selected.spec.expression, expression));
    CHECK(frontend_panel_action(WATCH_FRONTEND_PANEL, WATCH_SPACE, NULL, DEBUG_MEMORY_CPU, NULL, 0));
    cpu.pc = 0x20;
    memory_edit(tool, WATCH_EXPRESSION, "PC");
    memory_click(tool, WATCH_UPDATE);
    cpu.pc = 0x4018;
    render(tool);
    CHECK(desktop_clay_text_visible(tool->clay, "no readable mapped backing"));
    screenshot(tool, "build/desktop-watch-error.png");
    cpu.pc = 0x20;
    memory_click(tool, WATCH_REMOVE);
    CHECK(debugger_watch_count() == 11);
    for (size_t i = 0; i < debugger_watch_count(); ++i) {
        CHECK(debug_watch_at(i, &selected) && selected.id != selected_id);
    }
    FrontendDesktopUi *viewer = desktop_open_window(ui, 1, DEBUGGER_VIEWERS_FRONTEND_PANEL);
    CHECK(viewer && viewer != tool);
    SDL_Event close = {.type = SDL_WINDOWEVENT};
    close.window.windowID = SDL_GetWindowID(tool->window);
    close.window.event = SDL_WINDOWEVENT_CLOSE;
    CHECK(frontend_desktop_handle_event(ui, &close));
    CHECK(ui->tools == viewer && !ui->quit_requested && debugger_watch_count() == 11);
    CHECK(frontend_desktop_handle_event(ui, &close));
    desktop_close_windows(ui);
    cpu.pc = original_pc;
    debug_watch_reset();
    debug_frontend_image_changed(debug);
}
