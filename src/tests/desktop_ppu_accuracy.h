/*
 * desktop_ppu_accuracy.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Detached PPU tool interactions. Included after desktop test helpers. */
#include "../debugger/ppu_inspector.h"

static bool ppu_export_chooser(bool save, unsigned type, char *path, size_t size, char *error, size_t error_size,
                               void *context) {
    (void)error;
    (void)error_size;
    CHECK(save && type == FRONTEND_SAVE_PNG);
    snprintf(path, size, "build/ppu-export.png");
    return *(bool *)context;
}

static void ppu_windows(FrontendDesktopUi *ui) {
    for (unsigned id = DEBUG_PPU_PATTERNS; id <= DEBUG_PPU_PALETTE; ++id) {
        menu_activate(ui, 5, id);
        FrontendDesktopUi *tool = ui->tools;
        CHECK(tool && tool->panel_id == id && tool->window != ui->window);
        if (!tool) {
            continue;
        }
        for (unsigned scale = 0; scale < 3; ++scale) {
            tool->ui_scale = scale == 0 ? 1 : scale == 1 ? 1.5f : 2;
            SDL_SetWindowSize(tool->window, (int)(640 * tool->ui_scale), (int)(480 * tool->ui_scale));
            SDL_RenderSetLogicalSize(tool->renderer, (int)(640 * tool->ui_scale), (int)(480 * tool->ui_scale));
            render(tool);
            CHECK(!desktop_clay_contains_text(tool->clay, "Open a game"));
            click_control(tool, HIT_PPU_ACTION, 2, 0); /* Freeze */
            render(tool);
            click_control(tool, HIT_PPU_ACTION, 3, 0); /* Explicit refresh */
            render(tool);
            click_control(tool, HIT_PPU_ACTION, 2, 0);
            render(tool);
            if (id != DEBUG_PPU_REGISTERS && id != DEBUG_PPU_VRAM) {
                SDL_FRect canvas;
                CHECK(desktop_clay_bounds(tool->clay, HIT_PPU_CANVAS, 0, 0, &canvas));
                CHECK(canvas.w > 0 && canvas.h > 0 && canvas.x >= 0 && canvas.y >= 0);
                CHECK(canvas.x + canvas.w <= 640 && canvas.y + canvas.h <= 480);
            }
            if (id == DEBUG_PPU_VRAM) {
                click_control(tool, HIT_PPU_SCROLL, 0, 0);
                render(tool);
                key(tool, SDL_SCANCODE_DOWN, KMOD_NONE);
                render(tool);
                CHECK(desktop_clay_contains_text(tool->clay, "PPU $"));
            }
            if (scale == 0) {
                if (id != DEBUG_PPU_REGISTERS) {
                    click_control(tool, HIT_PPU_ACTION, 6, 0);
                    render(tool);
                    click_control(tool, HIT_PPU_ACTION, 5, 0);
                    render(tool);
                }
                if (id == DEBUG_PPU_PATTERNS || id == DEBUG_PPU_TILE) {
                    click_control(tool, HIT_PPU_ACTION, 27, 0);
                    render(tool);
                    click_control(tool, HIT_PPU_ACTION, 20, 0);
                    render(tool);
                }
                if (id == DEBUG_PPU_PATTERNS) {
                    for (unsigned bank = 0; bank < 3; ++bank) {
                        click_control(tool, HIT_PPU_ACTION, 7, 0);
                        render(tool);
                    }
                    click_control(tool, HIT_PPU_ACTION, 7, 0);
                    render(tool);
                    SDL_FRect tile;
                    CHECK(desktop_clay_bounds(tool->clay, HIT_PPU_CANVAS, 0, 0, &tile));
                    SDL_Event open_tile = {.type = SDL_MOUSEBUTTONDOWN};
                    open_tile.button.button = SDL_BUTTON_LEFT;
                    open_tile.button.clicks = 2;
                    open_tile.button.x = (int)((tile.x + 4) * tool->ui_scale);
                    open_tile.button.y = (int)((tile.y + 4) * tool->ui_scale);
                    CHECK(frontend_desktop_handle_event(tool, &open_tile));
                    CHECK(ui->tools->panel_id == DEBUG_PPU_TILE);
                    render(ui->tools);
                    CHECK(desktop_clay_contains_text(ui->tools->clay, "BG banks"));
                    bool choose = true;
                    frontend_set_file_chooser(ppu_export_chooser, &choose);
                    click_control(tool, HIT_PPU_ACTION, 9, 0);
                    uint8_t *png = NULL;
                    size_t size = 0;
                    CHECK(nes_file_read_all("build/ppu-export.png", 1024 * 1024, &png, &size) == NES_FILE_OK);
                    CHECK(size > 8 && png && !memcmp(png, "\x89PNG\r\n\x1a\n", 8));
                    free(png);
                    choose = false;
                    click_control(tool, HIT_PPU_ACTION, 9, 0);
                    frontend_set_file_chooser(NULL, NULL);
                    (void)nes_file_remove("build/ppu-export.png");
                }
                if (id == DEBUG_PPU_NAMETABLES || id == DEBUG_PPU_SPRITES || id == DEBUG_PPU_VRAM) {
                    click_control(tool, HIT_PPU_ACTION, 8, 0);
                    render(tool);
                    click_control(tool, HIT_PPU_ACTION, 8, 0);
                    render(tool);
                }
                if (id != DEBUG_PPU_REGISTERS && id != DEBUG_PPU_VRAM) {
                    click_control(tool, HIT_PPU_ACTION, 4, 0);
                    render(tool);
                    click_control(tool, HIT_PPU_ACTION, 4, 0);
                    render(tool);
                }
                if (id == DEBUG_PPU_VRAM || id == DEBUG_PPU_TILE) {
                    click_control(tool, HIT_PPU_ACTION, 10, 0);
                    CHECK(tool->edit_text_active);
                    strcpy(tool->edit_text, "bad input");
                    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
                    CHECK(tool->edit_text_active);
                    strcpy(tool->edit_text, id == DEBUG_PPU_TILE ? "0100" : "2000");
                    key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
                    CHECK(!tool->edit_text_active);
                    render(tool);
                }
                char path[128];
                snprintf(path, sizeof(path), "build/ppu-tool-%u.png", id - DEBUG_PPU_PATTERNS);
                screenshot(tool, path);
            }
        }
        frontend_panel_set_session_active(false);
        render(tool);
        CHECK(desktop_clay_contains_text(tool->clay, "Open a game"));
        frontend_panel_set_session_active(true);
        render(tool);
        desktop_close_windows(ui);
    }
    FrontendDesktopUi *tool = desktop_open_window(ui, 1, DEBUG_PPU_PALETTE);
    CHECK(tool);
    if (tool) {
        debugger_pause();
        frontend_execution_sync_debugger(ui->execution);
        render(tool);
        click_control(tool, HIT_PPU_ACTION, 2, 0);
        render(tool);
        DebugPpuSnapshot before, after;
        debugger_get_ppu(&before);
        click_control(tool, HIT_PPU_ACTION, 1, 0);
        render(tool); /* A paint before emulation advances must not lose the requested refresh. */
        CHECK(frontend_execution_run_frame(ui->execution));
        render(tool);
        debugger_get_ppu(&after);
        CHECK(after.ppu_cycles > before.ppu_cycles);
        char frame_label[64];
        snprintf(frame_label, sizeof(frame_label), "Frame %llu", (unsigned long long)after.frame);
        CHECK(desktop_clay_contains_text(tool->clay, frame_label));
        click_control(tool, HIT_PPU_ACTION, 2, 0);
        render(tool);
        uint8_t original = debugger_peek_ppu(0x3F00);
        click_control(tool, HIT_PPU_ACTION, 11, 0);
        CHECK(tool->edit_text_active);
        strcpy(tool->edit_text, "2A");
        key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(!tool->edit_text_active && debugger_peek_ppu(0x3F00) == 0x2A);
        render(tool);
        click_control(tool, HIT_PPU_ACTION, 12, 0);
        CHECK(debugger_peek_ppu(0x3F00) == original);
        render(tool);
        click_control(tool, HIT_PPU_ACTION, 11, 0);
        debugger_reset_session();
        render(tool);
        strcpy(tool->edit_text, "30");
        key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(tool->edit_text_active && debugger_peek_ppu(0x3F00) == original);
        key(tool, SDL_SCANCODE_ESCAPE, KMOD_NONE);
        debugger_resume();
        frontend_execution_sync_debugger(ui->execution);
        if (frontend_execution_paused(ui->execution)) {
            CHECK(frontend_command_invoke(FRONTEND_COMMAND_PAUSE, NULL, 0));
        }
        desktop_close_windows(ui);
    }
}
