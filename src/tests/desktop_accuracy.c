/* Desktop event, layout and device regression checks. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../debugger/debugger.h"
#include "../debugger/lua_runtime.h"
#include "../rom/rom.h"
#include "../ui/debug_frontend.h"
#include "../ui/cheat_frontend.h"
#include "../cheats/cheats.h"
#include "../ui/platform_frontend.h"
#include "../ui/machine_actions.h"
#include "../capture/capture_writer.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../system/execution_policy.h"
#include "../ui/app_paths.h"
#include "../ui/clay_backend.h"
#include "../ui/desktop_ui.h"
#include "../ui/desktop_internal.h"
#include "../ui/host_input.h"
#include "../ui/device_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/palette_tool.h"
#include "../ui/storage_frontend.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "Desktop check %d failed: %s\n", __LINE__, #x);                                            \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static void key(FrontendDesktopUi *ui, SDL_Scancode sc, SDL_Keymod mod) {
    SDL_Event event = {.type = SDL_KEYDOWN};
    event.key.keysym.scancode = sc;
    event.key.keysym.mod = mod;
    CHECK(frontend_desktop_handle_event(ui, &event));
}

static void screenshot(FrontendDesktopUi *ui, const char *path);

static void click_control(FrontendDesktopUi *ui, int kind, int index, int direction) {
    SDL_FRect rect;
    bool found = desktop_clay_bounds(ui->clay, kind, index, direction, &rect);
    CHECK(found);
    if (!found) {
        return;
    }
    int width, height;
    SDL_GetWindowSize(ui->window, &width, &height);
    CHECK(rect.w > 0 && rect.h > 0 && rect.x >= 0 && rect.y >= 0);
    CHECK((rect.x + rect.w) * ui->ui_scale <= width + 1 && (rect.y + rect.h) * ui->ui_scale <= height + 1);
    SDL_Event event = {.type = SDL_MOUSEBUTTONDOWN};
    event.button.button = SDL_BUTTON_LEFT;
    event.button.x = (int)((rect.x + rect.w / 2) * ui->ui_scale);
    event.button.y = (int)((rect.y + rect.h / 2) * ui->ui_scale);
    CHECK(frontend_desktop_handle_event(ui, &event));
}

static void render(FrontendDesktopUi *ui) {
    SDL_SetRenderDrawColor(ui->renderer, 14, 18, 27, 255);
    SDL_RenderClear(ui->renderer);
    frontend_desktop_render(ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
}

static void mouse_controls(FrontendDesktopUi *ui) {
    ppu_palette_reset_default();
    for (int dpi = 0; dpi < 3; ++dpi) {
        ui->ui_scale = dpi == 0 ? 1 : dpi == 1 ? 1.5f : 2;
        SDL_SetWindowSize(ui->window, (int)(640 * ui->ui_scale), (int)(480 * ui->ui_scale));
        SDL_RenderSetLogicalSize(ui->renderer, (int)(640 * ui->ui_scale), (int)(480 * ui->ui_scale));
        render(ui);
        click_control(ui, HIT_COMMAND, FRONTEND_COMMAND_SETTINGS, 0);
        CHECK(ui->settings_open);
        render(ui);
        click_control(ui, HIT_CATEGORY, 1, 0);
        CHECK(ui->settings_category == 1);
        render(ui);
        double previous = ui->staged.speed;
        click_control(ui, HIT_SETTING, 1, 0);
        CHECK(ui->edit_text_active && ui->edit_number);
        snprintf(ui->edit_text, sizeof(ui->edit_text), "%.2f", previous == 1 ? 1.75 : 1.0);
        key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(!ui->edit_text_active && ui->staged.speed != previous);
        render(ui);
        click_control(ui, HIT_SETTINGS_BUTTON, 2, 0);
        CHECK(!ui->settings_open);
        render(ui);
        click_control(ui, HIT_MENU, 6, 0);
        CHECK(ui->open_menu == 6);
        render(ui);
        click_control(ui, HIT_MENU_ROW, 0, 0);
        CHECK(ui->info_open);
        render(ui);
        click_control(ui, HIT_CLOSE, 0, 0);
        CHECK(!ui->info_open);
        render(ui);
        key(ui, SDL_SCANCODE_F7, KMOD_NONE);
        CHECK(palette_tool_is_visible());
        render(ui);
        click_control(ui, HIT_COLOR, 1, 0);
        CHECK(ui->palette_index == 1);
        render(ui);
        uint32_t before[64], after[64];
        ppu_palette_get(before);
        click_control(ui, HIT_CHANNEL, 0, 0);
        ppu_palette_get(after);
        CHECK(before[1] != after[1]);
        render(ui);
        if (dpi == 0) {
            screenshot(ui, "build/desktop-palette.png");
        }
        click_control(ui, HIT_CLOSE, 0, 0);
        CHECK(!palette_tool_is_visible());
        ppu_palette_reset_default();
    }
    ui->ui_scale = 1;
    SDL_SetWindowSize(ui->window, 768, 720);
    SDL_RenderSetLogicalSize(ui->renderer, 768, 720);
    SDL_RenderClear(ui->renderer);
    frontend_desktop_render(ui, 256, 240, NULL, NULL, "Idle");
    screenshot(ui, "build/desktop-startup.png");
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS, NULL, 0));
    ui->settings_category = 1;
    ui->settings_row = 1;
    key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(ui->edit_text_active && ui->edit_select_all);
    SDL_Event text = {.type = SDL_TEXTINPUT};
    strcpy(text.text.text, "1.25");
    CHECK(frontend_desktop_handle_event(ui, &text));
    CHECK(!strcmp(ui->edit_text, "1.25"));
    render(ui);
    screenshot(ui, "build/desktop-text-entry.png");
    click_control(ui, HIT_EDIT_OK, 0, 0);
    CHECK(!ui->edit_text_active && ui->staged.speed == 1.25);
    SDL_Event quit = {.type = SDL_QUIT};
    CHECK(!frontend_desktop_handle_event(ui, &quit));
    render(ui);
    click_control(ui, HIT_SETTINGS_BUTTON, 2, 0);
}

static int panel_choice_selected;
static const char *panel_choices[45];
static char panel_choice_names[45][24];

static bool panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    for (unsigned i = 0; i < 24; ++i) {
        FrontendPanelControl c = {i, FRONTEND_PANEL_ACTION, "Action", NULL, NULL, 0, -1, true, false};
        if (!frontend_panel_add_control(model, &c)) {
            return false;
        }
    }
    for (int i = 0; i < 45; ++i) {
        snprintf(panel_choice_names[i], sizeof(panel_choice_names[i]), "Option %d", i + 1);
        panel_choices[i] = panel_choice_names[i];
    }
    FrontendPanelControl choice = {
        24, FRONTEND_PANEL_CHOICE, "Multiple pages", NULL, panel_choices, 45, panel_choice_selected, true, false};
    if (!frontend_panel_add_control(model, &choice)) {
        return false;
    }
    return true;
}

static bool panel_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)value;
    if (id == 24) {
        panel_choice_selected = selected;
    }
    (void)error;
    (void)size;
    *(unsigned *)context = id;
    return true;
}

static void screenshot(FrontendDesktopUi *ui, const char *path) {
    int width, height;
    CHECK(SDL_GetRendererOutputSize(ui->renderer, &width, &height) == 0);
    uint32_t *pixels = malloc((size_t)width * height * sizeof(*pixels));
    CHECK(pixels);
    if (!pixels) {
        return;
    }
    CHECK(SDL_RenderReadPixels(ui->renderer, NULL, SDL_PIXELFORMAT_ARGB8888, pixels, width * 4) == 0);
    NesCaptureFrame frame = {pixels, (unsigned)width, (unsigned)height, (size_t)width};
    CHECK(nes_capture_png(path, &frame) == NES_FILE_OK);
    free(pixels);
}

static void silence(void *context, Uint8 *stream, int length) {
    (void)context;
    memset(stream, 0, (size_t)length);
}

static void audio_settings(void) {
    SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_AUDIO) == 0);
    SDL_AudioSpec want = {0}, have;
    want.freq = 44100;
    want.format = AUDIO_F32SYS;
    want.channels = 2;
    want.samples = 512;
    want.callback = silence;
    SDL_AudioDeviceID device = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    CHECK(device);
    if (!device) {
        return;
    }
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, &device, have.freq, NULL, NULL, NULL, NULL);
    SDL_PauseAudioDevice(device, 0);
    CHECK(SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PLAYING);
    frontend_execution_begin_machine_change(&execution);
    CHECK(SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PLAYING);
    frontend_execution_begin_machine_change(&execution);
    CHECK(execution.machine_change_depth == 2);
    CHECK(frontend_execution_set_speeds(&execution, 2, 4));
    frontend_execution_set_muted(&execution, true);
    frontend_execution_end_machine_change_preserving_audio(&execution);
    CHECK(execution.machine_change_depth == 1);
    frontend_execution_end_machine_change_preserving_audio(&execution);
    CHECK(execution.machine_change_depth == 0);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.speed = 2;
    settings.muted = true;
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, NULL, NULL, &settings, &execution, NULL, NULL);
    frontend_commands_reset();
    frontend_panels_reset();
    CHECK(frontend_execution_register_commands(&execution));
    CHECK(frontend_desktop_register_commands(&ui));
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS, NULL, 0));
    ui.staged.speed = 1;
    key(&ui, SDL_SCANCODE_TAB, KMOD_NONE);
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(settings.speed == 1 && execution.machine_change_depth == 0 && execution.execution.speed == 1);
    ui.settings_path = "build/missing-settings-directory/settings.ini";
    ui.staged.speed = 3;
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(settings.speed == 1 && execution.machine_change_depth == 0 && execution.execution.speed == 1);
    key(&ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    frontend_execution_set_muted(&execution, false);
    for (int i = 0; i < 6; ++i) {
        frontend_execution_suspend(&execution, FRONTEND_SUSPEND_UI, true);
        frontend_execution_suspend(&execution, FRONTEND_SUSPEND_FOCUS, true);
        frontend_execution_suspend(&execution, FRONTEND_SUSPEND_UI, false);
        CHECK(SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PAUSED);
        frontend_execution_suspend(&execution, FRONTEND_SUSPEND_FOCUS, false);
        CHECK(SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PLAYING);
        CHECK(frontend_execution_set_speeds(&execution, 2, 4));
        CHECK(apu.sample_rate == have.freq / 2.0);
        CHECK(frontend_execution_set_speeds(&execution, 1, 4));
        CHECK(apu.sample_rate == have.freq);
    }
    frontend_desktop_shutdown(&ui);
    frontend_execution_shutdown(&execution);
    SDL_CloseAudioDevice(device);
    apu_audio_shutdown_state(&apu);
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    frontend_commands_reset();
    frontend_panels_reset();
}

static void storage_settings(void) {
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    strcpy(settings.game_database_path, "build/custom-database.txt");
    strcpy(settings.movie_file_path, "build/movie.cpm");
    settings.disable_database_corrections = true;
    settings.rewind_step_frames = 3;
    FrontendSettingsReport report;
    CHECK(frontend_settings_save("build/storage-settings.ini", &settings, &report));
    FrontendSettings restored;
    CHECK(frontend_settings_load("build/storage-settings.ini", &restored, &report));
    CHECK(!strcmp(restored.game_database_path, settings.game_database_path));
    CHECK(restored.rewind_step_frames == 3);
    CHECK(!strcmp(restored.movie_file_path, settings.movie_file_path) && restored.disable_database_corrections);
    (void)nes_file_remove("build/storage-settings.ini");
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    FrontendSession session;
    frontend_session_init(&session, NULL, NULL);
    FrontendSessionActions actions;
    frontend_session_actions_init(&actions, &session, &settings, NULL);
    FrontendStorage storage;
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    CHECK(frontend_storage_register(&storage, &actions, &execution, NULL));
    char error[256];
    CHECK(frontend_panel_action(0x1C00, 0x1C01, NULL, 4, error, sizeof(error)));
    CHECK(frontend_panel_action(0x1C00, 0x1C02, "build/selected.cst", 0, error, sizeof(error)));
    CHECK(!strcmp(settings.state_file_path, "build/selected.cst"));
    CHECK(frontend_panel_action(0x1C00, 0x1C01, NULL, 11, error, sizeof(error)));
    CHECK(!frontend_panel_action(0x1C00, 0x1C02, "build/no-such-database.txt", 0, error, sizeof(error)));
    CHECK(!strcmp(settings.game_database_path, "build/custom-database.txt"));
    frontend_storage_unregister();
    frontend_execution_shutdown(&execution);
    frontend_panels_reset();
}

static void focus_event(FrontendDesktopUi *root, SDL_Window *window, bool focused) {
    SDL_Event event = {.type = SDL_WINDOWEVENT};
    event.window.windowID = SDL_GetWindowID(window);
    event.window.event = focused ? SDL_WINDOWEVENT_FOCUS_GAINED : SDL_WINDOWEVENT_FOCUS_LOST;
    (void)frontend_desktop_handle_event(root, &event);
}

static bool menu_find(FrontendDesktopUi *ui, int depth, unsigned id, int kind) {
    DesktopMenuItem items[128];
    int count = desktop_menu_level(ui, depth, items);
    CHECK(count <= 10);
    for (int i = 0; i < count; ++i) {
        if (items[i].kind != 7 && items[i].id == id && (kind < 0 || items[i].kind == (unsigned)kind)) {
            ui->menu_depth = depth;
            ui->menu_row = i;
            return true;
        }
        if (items[i].kind == 7 && depth < 4) {
            ui->menu_path[depth] = items[i].id;
            ui->menu_parent_rows[depth] = i;
            if (menu_find(ui, depth + 1, id, kind)) {
                return true;
            }
        }
    }
    return false;
}

static bool menu_noop(void *context, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    return true;
}

static void menu_coverage(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 30; ++i) {
        FrontendCommandSpec command = {0x7f10 + i, "Additional command", "Tools", "", 0, menu_noop, NULL};
        CHECK(frontend_command_register(&command));
    }
    for (int menu = 0; menu < 7; ++menu) {
        ui->open_menu = menu;
        DesktopMenuItem all[128];
        int count = desktop_menu_all(ui, all);
        for (int i = 0; i < count; ++i) {
            bool found = menu_find(ui, 0, all[i].id, all[i].kind);
            CHECK(found);
            if (found) {
                render(ui);
                SDL_FRect bounds;
                CHECK(desktop_clay_bounds(ui->clay, HIT_MENU_ROW, ui->menu_row, ui->menu_depth, &bounds));
                int w, h;
                SDL_GetWindowSize(ui->window, &w, &h);
                CHECK(bounds.x >= 0 && bounds.y >= 32 && bounds.x + bounds.w <= w / ui->ui_scale &&
                      bounds.y + bounds.h <= h / ui->ui_scale - 28);
                CHECK(!desktop_clay_bounds(ui->clay, HIT_SCROLL, 0, 1, &bounds));
            }
        }
    }
    for (unsigned i = 0; i < 30; ++i) {
        CHECK(frontend_command_unregister(0x7f10 + i));
    }
    ui->open_menu = 5;
    if (menu_find(ui, 0, DEBUGGER_FRONTEND_PANEL, 1)) {
        render(ui);
        screenshot(ui, "build/desktop-submenus.png");
        int old_depth = ui->menu_depth;
        CHECK(old_depth > 0);
        key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
        CHECK(ui->menu_depth == old_depth - 1 && ui->open_menu == 5);
    }
    ui->open_menu = -1;
    ui->menu_depth = 0;
}

static void menu_activate(FrontendDesktopUi *ui, int menu, unsigned id) {
    ui->open_menu = menu;
    bool found = menu_find(ui, 0, id, -1);
    CHECK(found);
    if (!found) {
        return;
    }
    render(ui);
    click_control(ui, HIT_MENU_ROW, ui->menu_row, ui->menu_depth);
}

typedef struct {
    bool save, accept, fail;
    unsigned type, calls;
} ChooserProbe;

static bool choose_path(bool save, unsigned type, char *path, size_t size, char *error, size_t error_size,
                        void *context) {
    ChooserProbe *probe = context;
    CHECK(save == probe->save && type == probe->type);
    ++probe->calls;
    if (probe->fail) {
        snprintf(error, error_size, "Chooser test failure");
        return false;
    }
    if (error && error_size) {
        error[0] = 0;
    }
    if (probe->accept) {
        snprintf(path, size, "build/chosen path.bin");
    }
    return probe->accept;
}

static char chosen_panel_path[128];

static bool path_panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    const FrontendPanelControl controls[] = {
        {1, FRONTEND_PANEL_FILE_OPEN, "Script", chosen_panel_path, NULL, 0, FRONTEND_OPEN_SCRIPT, true, false},
        {2, FRONTEND_PANEL_FILE_SAVE, "Recording", chosen_panel_path, NULL, 0, FRONTEND_SAVE_WAV, true, false},
        {3, FRONTEND_PANEL_DIRECTORY, "Folder", chosen_panel_path, NULL, 0, 0, true, false},
        {4, FRONTEND_PANEL_FILE_OPEN, "Read only", chosen_panel_path, NULL, 0, FRONTEND_OPEN_SCRIPT, true, true},
        {5, FRONTEND_PANEL_FILE_SAVE, "Disabled", chosen_panel_path, NULL, 0, FRONTEND_SAVE_WAV, false, false}};
    for (unsigned i = 0; i < 5; ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    return true;
}

static bool path_panel_action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)context;
    (void)selected;
    (void)error;
    (void)size;
    CHECK(id >= 1 && id <= 3 && value);
    if (value) {
        snprintf(chosen_panel_path, sizeof(chosen_panel_path), "%s", value);
    }
    return true;
}

static void path_controls(FrontendDesktopUi *ui) {
    FrontendPanelSpec spec = {0x7F20, "File controls", "Tools", 0, path_panel_snapshot, path_panel_action, NULL};
    CHECK(frontend_panel_register(&spec));
    ui->panel_id = spec.id;
    ui->panel_row = 0;
    ui->panel_open = true;
    ui->panel_scroll = 0;
    ChooserProbe probe = {0};
    frontend_set_file_chooser(choose_path, &probe);
    for (int row = 0; row < 3; ++row) {
        ui->panel_row = row;
        probe.save = row == 1;
        probe.type = row == 0 ? FRONTEND_OPEN_SCRIPT : row == 1 ? FRONTEND_SAVE_WAV : FRONTEND_OPEN_FOLDER;
        probe.accept = false;
        strcpy(chosen_panel_path, "unchanged");
        render(ui);
        click_control(ui, HIT_PANEL, row, 0);
        CHECK(!strcmp(chosen_panel_path, "unchanged") && !ui->edit_text_active);
        probe.accept = true;
        key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(!strcmp(chosen_panel_path, "build/chosen path.bin") && !ui->edit_text_active);
        probe.fail = true;
        key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(strstr(ui->status, "Chooser test failure") && !strcmp(chosen_panel_path, "build/chosen path.bin"));
        probe.fail = false;
    }
    unsigned calls = probe.calls;
    for (int row = 3; row < 5; ++row) {
        ui->panel_row = row;
        key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    }
    CHECK(calls == probe.calls);
    frontend_set_file_chooser(NULL, NULL);
    ui->panel_open = false;
    CHECK(frontend_panel_unregister(spec.id));
}

static void typed_settings(FrontendDesktopUi *ui) {
    CHECK(SDL_InitSubSystem(SDL_INIT_AUDIO) == 0);
    desktop_settings_open(ui, true);
    FrontendSettings original = ui->staged;
    for (int category = 0; category < 8; ++category) {
        ui->settings_category = category;
        int rows = desktop_setting_rows(ui);
        for (int row = 0; row < rows; ++row) {
            ui->staged = original;
            ui->settings_row = row;
            char name[96], value[256];
            desktop_setting_text(ui, row, name, sizeof(name), value, sizeof(value));
            CHECK(name[0]);
            DesktopSettingKind kind = desktop_setting_kind(ui, row);
            if (kind == SETTING_CHOICE) {
                int selected, count = desktop_setting_choices(ui, row, &selected);
                CHECK(count > 0);
                for (int option = 0; option < count; ++option) {
                    desktop_setting_choice_text(ui, row, option, value, sizeof(value));
                    CHECK(value[0]);
                    desktop_setting_choose(ui, row, option);
                    (void)desktop_setting_choices(ui, row, &selected);
                    CHECK(selected == option);
                }
                desktop_activate_setting(ui, row);
                CHECK(ui->choice_open);
                render(ui);
                click_control(ui, HIT_CHOICE, ui->choice_index, 0);
                CHECK(!ui->choice_open);
            } else if (kind == SETTING_NUMBER) {
                desktop_activate_setting(ui, row);
                CHECK(ui->edit_text_active && ui->edit_number);
                CHECK(desktop_setting_commit_number(ui, row, ui->edit_text));
                FrontendSettings before = ui->staged;
                const char *invalid[] = {"garbage",      "NaN",     "inf", "1e999", "-999999999999",
                                         "999999999999", "12 junk", "",    "   "};
                for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
                    CHECK(!desktop_setting_commit_number(ui, row, invalid[i]));
                    CHECK(!memcmp(&before, &ui->staged, sizeof(before)));
                }
                ui->edit_text_active = false;
                SDL_StopTextInput();
            } else if (kind == SETTING_TOGGLE) {
                desktop_activate_setting(ui, row);
                desktop_setting_text(ui, row, name, sizeof(name), value, sizeof(value));
                CHECK(value[0]);
            } else if (kind == SETTING_BINDING) {
                desktop_activate_setting(ui, row);
                CHECK(ui->capture_binding);
                ui->capture_binding = false;
            } else if (kind == SETTING_FILE) {
                ChooserProbe probe = {0};
                if (category == 5) {
                    probe.save = row == 4 || row == 10;
                    probe.type = row == 4    ? FRONTEND_SAVE_DISK_OVERLAY
                                 : row == 10 ? FRONTEND_SAVE_TAPE
                                 : row == 9  ? FRONTEND_OPEN_TAPE
                                             : FRONTEND_OPEN_FIRMWARE;
                } else {
                    probe.save = true;
                    probe.type = row == 1 ? FRONTEND_SAVE_STATE : (unsigned)(row - 2);
                }
                frontend_set_file_chooser(choose_path, &probe);
                FrontendSettings before = ui->staged;
                desktop_activate_setting(ui, row);
                CHECK(probe.calls == 1 && !ui->edit_text_active);
                CHECK(!memcmp(&before, &ui->staged, sizeof(before)));
                probe.accept = true;
                render(ui);
                click_control(ui, HIT_SETTING, row, 0);
                CHECK(probe.calls == 2 && !ui->edit_text_active);
                desktop_setting_text(ui, row, name, sizeof(name), value, sizeof(value));
                CHECK(!strcmp(value, "build/chosen path.bin"));
                render(ui);
                click_control(ui, HIT_CLEAR_SETTING, 0, 0);
                desktop_setting_text(ui, row, name, sizeof(name), value, sizeof(value));
                CHECK(!value[0] && probe.calls == 2 && !ui->edit_text_active);
                frontend_set_file_chooser(NULL, NULL);
            } else if (kind == SETTING_TEXT) {
                desktop_activate_setting(ui, row);
                CHECK(ui->edit_text_active);
                ui->edit_text_active = false;
                SDL_StopTextInput();
            }
            render(ui);
        }
    }
    ui->settings_category = 7;
    ui->settings_row = 1;
    uint32_t seed = ui->staged.power_on_seed;
    key(ui, SDL_SCANCODE_LEFT, KMOD_NONE);
    key(ui, SDL_SCANCODE_RIGHT, KMOD_NONE);
    CHECK(ui->staged.power_on_seed == seed);
    ui->settings_category = 3;
    CHECK(desktop_setting_commit_number(ui, 3, "44100"));
    CHECK(ui->staged.audio_sample_rate == 44100);
    CHECK(desktop_setting_commit_number(ui, 6, "-35"));
    CHECK(ui->staged.audio_mix.pan[0] == -35);
    ui->settings_category = 1;
    CHECK(desktop_setting_commit_number(ui, 1, "1.25"));
    CHECK(ui->staged.speed == 1.25);
    ui->settings_category = 4;
    ui->settings_row = 0;
    render(ui);
    SDL_FRect bar;
    CHECK(desktop_clay_bounds(ui->clay, HIT_SCROLLBAR, 0, 0, &bar));
    SDL_Event drag = {.type = SDL_MOUSEBUTTONDOWN};
    drag.button.button = SDL_BUTTON_LEFT;
    drag.button.x = (int)((bar.x + bar.w / 2) * ui->ui_scale);
    drag.button.y = (int)((bar.y + bar.h - 1) * ui->ui_scale);
    CHECK(frontend_desktop_handle_event(ui, &drag));
    CHECK(ui->settings_row >= desktop_setting_rows(ui) - 2);
    render(ui);
    CHECK(ui->settings_scroll > 0);
    drag.type = SDL_MOUSEBUTTONUP;
    (void)frontend_desktop_handle_event(ui, &drag);
    CHECK(ui->dragging_scroll == -1);
    ui->settings_category = 1;
    ui->settings_row = 0;
    desktop_activate_setting(ui, 0);
    CHECK(ui->choice_open);
    render(ui);
    key(ui, SDL_SCANCODE_DOWN, KMOD_NONE);
    key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(!ui->choice_open);
    desktop_activate_setting(ui, 1);
    strcpy(ui->edit_text, "NaN");
    key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(ui->edit_text_active && ui->status[0]);
    key(ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    CHECK(!ui->edit_text_active);
    ui->staged = original;
    desktop_settings_open(ui, false);
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

static bool choose_cheats(bool save, unsigned type, char *path, size_t size, char *error, size_t error_size,
                          void *context) {
    CHECK(type == (save ? FRONTEND_SAVE_CHEATS : FRONTEND_OPEN_CHEATS));
    if (error && error_size) {
        error[0] = 0;
    }
    if (!*(bool *)context) {
        return false;
    }
    snprintf(path, size, "build/ui-cheats.txt");
    return true;
}

static bool choose_lua(bool save, unsigned type, char *path, size_t size, char *error, size_t error_size,
                       void *context) {
    CHECK(!save && type == FRONTEND_OPEN_SCRIPT);
    if (error && error_size) {
        error[0] = 0;
    }
    if (!*(bool *)context) {
        return false;
    }
    snprintf(path, size, "build/ui-script.lua");
    return true;
}

static void lua_chooser(FrontendDesktopUi *ui) {
    const char script[] = "local value = 1 + 2";
    CHECK(nes_file_write_atomic("build/ui-script.lua", script, sizeof(script) - 1) == NES_FILE_OK);
    bool choose = true;
    frontend_set_file_chooser(choose_lua, &choose);
    menu_activate(ui, 5, DEBUGGER_LUA_LOAD_COMMAND);
    CHECK(debugger_lua_loaded());
    choose = false;
    menu_activate(ui, 5, DEBUGGER_LUA_LOAD_COMMAND);
    CHECK(debugger_lua_loaded());
    frontend_set_file_chooser(NULL, NULL);
    debugger_lua_unload();
    (void)nes_file_remove("build/ui-script.lua");
}

static void cheat_prompt(FrontendDesktopUi *ui) {
    CheatFrontend *cheats = cheat_frontend_create("build/");
    CHECK(cheats && cheat_frontend_register_ui(cheats));
    CHECK(cheats_clear() == CHEAT_OK);
    menu_activate(ui, 5, CHEATS_ADD_COMMAND);
    FrontendDesktopUi *tool = ui->tools;
    CHECK(tool && tool->edit_text_active && tool->prompt_command == CHEATS_ADD_COMMAND);
    if (tool) {
        CHECK(desktop_clay_contains_text(tool->clay, "Synthetic cartridge"));
        CHECK(!desktop_clay_contains_text(tool->clay, "No game loaded"));

        strcpy(tool->edit_text, "invalid");
        key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(tool->edit_text_active && tool->status[0] && cheats_count() == 0);
        render(tool);
        screenshot(tool, "build/desktop-cheat-validation.png");
        strcpy(tool->edit_text, "8000:EA");
        key(tool, SDL_SCANCODE_RETURN, KMOD_NONE);
        CHECK(!tool->edit_text_active && cheats_count() == 1);
        bool choose = true;
        frontend_set_file_chooser(choose_cheats, &choose);
        tool->panel_row = 11;
        render(tool);
        click_control(tool, HIT_PANEL, 11, 0);
        CHECK(cheats_clear() == CHEAT_OK);
        tool->panel_row = 10;
        render(tool);
        click_control(tool, HIT_PANEL, 10, 0);
        CHECK(cheats_count() == 1);
        CHECK(frontend_panel_action(CHEATS_FRONTEND_PANEL, CHEAT_CONTROL_PATH, "build/ui-original-cheats.txt", 0, NULL,
                                    0));
        const char invalid_file[] = "invalid cheat file";
        CHECK(nes_file_write_atomic("build/ui-cheats.txt", invalid_file, sizeof(invalid_file) - 1) == NES_FILE_OK);
        click_control(tool, HIT_PANEL, 10, 0);
        CHECK(cheats_count() == 1 && tool->status[0]);
        FrontendPanelControl controls[64];
        FrontendPanelModel model = {.controls = controls, .capacity = 64};
        CHECK(frontend_panel_snapshot(CHEATS_FRONTEND_PANEL, &model, NULL, 0));
        for (size_t i = 0; i < model.count; ++i) {
            if (controls[i].id == CHEAT_CONTROL_PATH) {
                CHECK(!strcmp(controls[i].value, "build/ui-original-cheats.txt"));
            }
        }
        choose = false;
        click_control(tool, HIT_PANEL, 10, 0);
        CHECK(cheats_count() == 1);
        frontend_set_file_chooser(NULL, NULL);
        desktop_close_windows(ui);
    }
    cheat_frontend_destroy(cheats);
    CHECK(cheats_clear() == CHEAT_OK);
    (void)nes_file_remove("build/ui-cheats.txt");
}

#include "desktop_ppu_accuracy.h"

static void window_regressions(FrontendDesktopUi *ui) {
    uint8_t image[16 + 16384 + 8192] = {0};
    memcpy(image, "NES\x1a", 4);
    image[4] = image[5] = 1;
    image[16] = 0x4c;
    image[17] = 0;
    image[18] = 0x80;
    image[16 + 0x3ffd] = 0x80;
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    debugger_init();
    CHECK(frontend_machine_power_cycle());
    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 44100, NULL, NULL, NULL, NULL);
    FrontendSession session = {.active = true};
    snprintf(session.current_result.title, sizeof(session.current_result.title), "Synthetic cartridge");
    FrontendSessionActions actions = {.session = &session, .execution = &runtime};
    ui->sessions = &actions;
    ui->fps = 59.9;
    ui->settings->show_fps = true;
    ui->execution = &runtime;
    runtime.settings = ui->settings;
    CHECK(frontend_execution_register_commands(&runtime));
    frontend_command_set_session_active(true);
    CHECK(frontend_desktop_register_commands(ui));
    ui->native_windows = true;
    ui->settings->pause_on_ui = ui->settings->pause_on_focus_loss = true;
    FrontendDesktopUi *tool = desktop_open_window(ui, 1, 0x7F00);
    CHECK(tool && tool->window != ui->window && tool->renderer != ui->renderer);
    if (!tool) {
        return;
    }
    /* Check the first native paint, which previously received an empty session. */
    CHECK(desktop_clay_contains_text(tool->clay, "Synthetic cartridge"));
    CHECK(desktop_clay_contains_text(tool->clay, "Running"));
    CHECK(desktop_clay_contains_text(tool->clay, "59.9 FPS"));
    CHECK(!desktop_clay_contains_text(tool->clay, "No game loaded"));
    strcpy(session.current_result.title, "Replacement cartridge");
    desktop_layout(tool, "", "", "");
    CHECK(desktop_clay_contains_text(tool->clay, "Replacement cartridge"));
    CHECK(!desktop_clay_contains_text(tool->clay, "Synthetic cartridge"));
    session.active = false;
    desktop_layout(tool, "", "", "");
    CHECK(desktop_clay_contains_text(tool->clay, "No game loaded"));
    session.active = true;
    strcpy(session.current_result.title, "Synthetic cartridge");
    focus_event(ui, ui->window, false);
    focus_event(ui, tool->window, true);
    frontend_desktop_update_activity(ui);
    CHECK(!frontend_execution_paused(&runtime));
    CHECK(!frontend_desktop_input_captured(ui));
    SDL_SetWindowPosition(tool->window, 100, 120);
    render(tool);
    screenshot(tool, "build/desktop-detached-tool.png");
    SDL_FRect row;
    CHECK(desktop_clay_bounds(tool->clay, HIT_PANEL, 1, 0, &row));
    SDL_Event click = {.type = SDL_MOUSEBUTTONDOWN};
    click.button.windowID = SDL_GetWindowID(tool->window);
    click.button.button = SDL_BUTTON_LEFT;
    click.button.x = (int)((row.x + 10) * tool->ui_scale);
    click.button.y = (int)((row.y + row.h / 2) * tool->ui_scale);
    CHECK(frontend_desktop_handle_event(ui, &click));
    CHECK(tool->panel_row == 1);
    for (int i = 0; i < 8; ++i) {
        focus_event(ui, tool->window, false);
        frontend_desktop_update_activity(ui);
        CHECK(frontend_execution_paused(&runtime) && !runtime.execution.paused);
        focus_event(ui, tool->window, true);
        frontend_desktop_update_activity(ui);
        CHECK(!frontend_execution_paused(&runtime));
    }
    /* SDL sends a left or right modifier, never the combined default binding. */
    key(ui, SDL_SCANCODE_3, KMOD_LCTRL);
    CHECK(frontend_execution_speed(&runtime) == 2);
    key(ui, SDL_SCANCODE_F, (SDL_Keymod)(KMOD_RCTRL | KMOD_LSHIFT));
    CHECK(runtime.execution.fast_forward_toggled);
    key(ui, SDL_SCANCODE_2, KMOD_RCTRL);
    CHECK(frontend_execution_speed(&runtime) == 1 && !runtime.execution.fast_forward_toggled);
    key(ui, SDL_SCANCODE_F, KMOD_LCTRL);
    CHECK(runtime.execution.fast_forward_held);
    SDL_Event release = {.type = SDL_KEYUP};
    release.key.keysym.scancode = SDL_SCANCODE_F;
    (void)frontend_desktop_handle_event(ui, &release);
    CHECK(!runtime.execution.fast_forward_held);
    key(ui, SDL_SCANCODE_P, KMOD_RCTRL);
    CHECK(runtime.execution.paused);
    desktop_layout(tool, "", "", "");
    CHECK(desktop_clay_contains_text(tool->clay, "Paused"));

    focus_event(ui, tool->window, false);
    frontend_desktop_update_activity(ui);
    focus_event(ui, tool->window, true);
    frontend_desktop_update_activity(ui);
    CHECK(runtime.execution.paused);
    key(ui, SDL_SCANCODE_P, KMOD_LCTRL);
    CHECK(!runtime.execution.paused);
    key(ui, SDL_SCANCODE_3, KMOD_LCTRL);
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS, NULL, 0));
    FrontendDesktopUi *settings = ui->tools;
    CHECK(settings != tool && settings->settings_open && settings->staged.speed == 2);
    frontend_desktop_update_activity(ui);
    CHECK(frontend_execution_paused(&runtime) && !runtime.execution.paused);
    render(settings);
    screenshot(settings, "build/desktop-settings-window.png");
    settings->capture_binding = true;
    settings->capture_shortcut = true;
    settings->capture_index = FRONTEND_SHORTCUT_PAUSE;
    key(settings, SDL_SCANCODE_LCTRL, KMOD_LCTRL);
    CHECK(settings->capture_binding);
    key(settings, SDL_SCANCODE_P, KMOD_LCTRL);
    CHECK(!settings->capture_binding);
    const FrontendBindingProfile *captured = frontend_settings_active_profile_const(&settings->staged);
    CHECK(captured->shortcuts[FRONTEND_SHORTCUT_PAUSE].key == SDL_SCANCODE_P);
    SDL_Event close = {.type = SDL_WINDOWEVENT};
    close.window.windowID = SDL_GetWindowID(settings->window);
    close.window.event = SDL_WINDOWEVENT_CLOSE;
    CHECK(frontend_desktop_handle_event(ui, &close));
    CHECK(ui->tools == tool);
    frontend_desktop_update_activity(ui);
    CHECK(!frontend_execution_paused(&runtime));
    CHECK(frontend_execution_speed(&runtime) == 2);
    close.window.windowID = SDL_GetWindowID(tool->window);
    CHECK(frontend_desktop_handle_event(ui, &close));
    CHECK(!ui->tools && !ui->quit_requested);
    CHECK(frontend_desktop_handle_event(ui, &close)); /* Stale tool event. */
    ui->focused = true;
    frontend_desktop_update_activity(ui);
    CHECK(frontend_command_invoke(0x1B00, NULL, 0));
    CHECK(ui->tools && ui->tools->palette_window && !palette_tool_is_visible());
    if (ui->tools) {
        render(ui->tools);
        click_control(ui->tools, HIT_COLOR, 7, 0);
        CHECK(ui->tools->palette_index == 7);
        close.window.windowID = SDL_GetWindowID(ui->tools->window);
        CHECK(frontend_desktop_handle_event(ui, &close));
        CHECK(!ui->tools);
    }
    DebugFrontend *debug = debug_frontend_create(&runtime);
    CHECK(debug && debug_frontend_register_ui(debug));
    frontend_panel_set_session_active(true);
    ppu_windows(ui);
    lua_chooser(ui);
    cheat_prompt(ui);
    menu_coverage(ui);
    menu_activate(ui, 5, DEBUGGER_FRONTEND_PANEL);
    FrontendDesktopUi *debug_window = ui->tools;
    CHECK(debug_window);
    if (debug_window) {
        focus_event(ui, ui->window, false);
        focus_event(ui, debug_window->window, true);
        frontend_desktop_update_activity(ui);
        uint64_t cycles = cpu_total_cycles;
        for (int i = 0; i < 8; ++i) {
            CHECK(frontend_execution_run_frame(&runtime));
        }
        CHECK(cpu_total_cycles > cycles && !frontend_execution_paused(&runtime));
        CHECK(desktop_clay_contains_text(debug_window->clay, "Synthetic cartridge"));
        CHECK(!desktop_clay_contains_text(debug_window->clay, "No game loaded"));
        FrontendDesktopUi *viewer = desktop_open_window(ui, 1, DEBUGGER_VIEWERS_FRONTEND_PANEL);
        CHECK(viewer && desktop_clay_contains_text(viewer->clay, "Synthetic cartridge"));
        render(debug_window);
        screenshot(debug_window, "build/desktop-debugger-window.png");
        CHECK(desktop_open_window(ui, 1, DEBUGGER_FRONTEND_PANEL) == debug_window);
        for (int scale = 1; scale <= 2; ++scale) {
            SDL_SetWindowSize(debug_window->window, 640 * scale, 480 * scale);
            SDL_RenderSetLogicalSize(debug_window->renderer, 640 * scale, 480 * scale);
            debug_window->ui_scale = (float)scale;
            render(debug_window);
            click_control(debug_window, HIT_SCROLL, 0, 1);
            render(debug_window);
        }
        desktop_close_windows(ui);
    }
    FrontendDeviceRuntime devices;
    frontend_devices_init(&devices, &runtime, ui->settings, NULL);
    ui->devices = &devices;
    CHECK(frontend_devices_register(&devices));
    menu_activate(ui, 4, DEVICE_COMMAND_DISK_TOGGLE);
    CHECK(strstr(ui->status, "FDS image") && ui->open_menu == -1);
    NesInputConfiguration old_inputs = {
        joypad_adapter(), {joypad_port_device(0), joypad_port_device(1)}, joypad_expansion_device()};
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    const uint8_t tape[] = {0, 1, 0, 1};
    CHECK(nes_file_write_atomic("build/ui-menu-tape.bin", tape, sizeof(tape)) == NES_FILE_OK);
    CHECK(frontend_devices_set_tape_paths(&devices, "build/ui-menu-tape.bin", "build/ui-menu-record.bin", NULL, 0));
    CHECK(frontend_devices_tape_load(&devices, "build/ui-menu-tape.bin", NULL, 0));
    frontend_devices_refresh(&devices);
    menu_activate(ui, 4, DEVICE_COMMAND_TAPE_PLAY);
    CHECK(family_basic_tape_mode() == FB_TAPE_PLAYING);
    menu_activate(ui, 4, DEVICE_COMMAND_TAPE_STOP);
    CHECK(family_basic_tape_mode() == FB_TAPE_STOPPED);
    menu_activate(ui, 4, DEVICE_COMMAND_TAPE_RECORD);
    CHECK(family_basic_tape_mode() == FB_TAPE_RECORDING && devices.tape_capture_pending);
    menu_activate(ui, 4, DEVICE_COMMAND_TAPE_STOP);
    CHECK(!devices.tape_capture_pending);
    CHECK(joypad_apply_configuration(&old_inputs));
    frontend_devices_unregister();
    ui->devices = NULL;
    (void)nes_file_remove("build/ui-menu-tape.bin");
    (void)nes_file_remove("build/ui-menu-record.bin");
    debug_frontend_destroy(debug);
    CHECK(unload_rom());
    debugger_shutdown();
    frontend_execution_shutdown(&runtime);
    ui->execution = NULL;
    ui->sessions = NULL;
    ui->native_windows = false;
}

static void modifier_bindings(void) {
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    const FrontendBindingProfile *profile = frontend_settings_active_profile_const(&settings);
    SDL_KeyboardEvent event = {.type = SDL_KEYDOWN};
    event.keysym.scancode = SDL_SCANCODE_RSHIFT;
    event.keysym.mod = KMOD_RSHIFT;
    CHECK(frontend_host_input_bound_player_key(profile, &event));
    CHECK(joypad_player(0)->buttons & (1u << BTN_SELECT));
    event.type = SDL_KEYUP;
    event.keysym.mod = KMOD_RCTRL;
    CHECK(frontend_host_input_bound_player_key(profile, &event));
    CHECK(!(joypad_player(0)->buttons & (1u << BTN_SELECT)));
}

int test_desktop_accuracy(void) {
    failures = 0;
    audio_settings();
    storage_settings();
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_Window *window = SDL_CreateWindow("Desktop checks", 0, 0, 768, 720, SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    CHECK(window && renderer);
    if (!renderer) {
        if (window) {
            SDL_DestroyWindow(window);
        }
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return failures;
    }
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.speed = 2;
    settings.audio_mix.master_volume = 40;
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, window, renderer, &settings, NULL, NULL, NULL);
    CHECK(ui.open_menu == -1 && !frontend_desktop_input_captured(&ui));
    frontend_commands_reset();
    frontend_panels_reset();
    CHECK(frontend_desktop_register_commands(&ui));
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS, NULL, 0));
    key(&ui, SDL_SCANCODE_TAB, KMOD_CTRL);
    CHECK(ui.settings_category == 1);
    key(&ui, SDL_SCANCODE_TAB, KMOD_NONE);
    CHECK(ui.settings_focus == 1);
    for (int i = 0; i < 3; ++i) {
        key(&ui, SDL_SCANCODE_RIGHT, KMOD_NONE);
    }
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(ui.staged.speed == 1 && ui.staged.audio_mix.master_volume == 40 && settings.speed == 2);
    key(&ui, SDL_SCANCODE_TAB, KMOD_NONE);
    CHECK(ui.settings_focus == 2);
    key(&ui, SDL_SCANCODE_DOWN, KMOD_NONE);
    CHECK(ui.settings_category == 2);
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(ui.settings_focus == 0);
    SDL_SetRenderDrawColor(renderer, 18, 20, 24, 255);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
    screenshot(&ui, "build/desktop-settings.png");
    key(&ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    CHECK(settings.speed == 2);
    key(&ui, SDL_SCANCODE_F, KMOD_ALT);
    CHECK(frontend_desktop_input_captured(&ui));
    key(&ui, SDL_SCANCODE_LEFT, KMOD_NONE);
    CHECK(ui.open_menu == 6);
    key(&ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    unsigned selected = 0;
    FrontendPanelSpec panel = {0x7F00, "Scrollable panel", "Tools", 0, panel_snapshot, panel_action, &selected};
    CHECK(frontend_panel_register(&panel));
    ui.panel_id = panel.id;
    ui.panel_open = true;
    for (int i = 0; i < 20; ++i) {
        key(&ui, SDL_SCANCODE_DOWN, KMOD_NONE);
    }
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(selected == 20 && ui.panel_scroll > 0);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
    screenshot(&ui, "build/desktop-panel.png");
    ui.panel_row = 24;
    render(&ui);
    click_control(&ui, HIT_PANEL, 24, 0);
    CHECK(ui.choice_open && desktop_choice_count(&ui) == 45);
    render(&ui);
    screenshot(&ui, "build/desktop-choices.png");
    render(&ui);
    click_control(&ui, HIT_CHOICE_PAGE, 0, 1);
    render(&ui);
    click_control(&ui, HIT_CHOICE_PAGE, 0, 1);
    render(&ui);
    click_control(&ui, HIT_CHOICE, 44, 0);
    CHECK(!ui.choice_open && panel_choice_selected == 44);
    ui.panel_open = false;
    SDL_SetWindowSize(window, 1280, 960);
    SDL_RenderSetLogicalSize(renderer, 1280, 960);
    ui.ui_scale = 2;
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SETTINGS, NULL, 0));
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
    screenshot(&ui, "build/desktop-settings-2x.png");
    for (int category = 0; category < 8; ++category) {
        ui.settings_category = category;
        ui.settings_row = ui.settings_scroll = 0;
        SDL_RenderClear(renderer);
        frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
        char path[80];
        snprintf(path, sizeof(path), "build/desktop-category-%d.png", category);
        screenshot(&ui, path);
    }
    ui.ui_scale = 1.5f;
    SDL_SetWindowSize(window, 960, 720);
    SDL_RenderSetLogicalSize(renderer, 960, 720);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "PAL", "Paused");
    screenshot(&ui, "build/desktop-settings-150.png");
    key(&ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 512, 240, "Synthetic dual display", "NTSC", "Running");
    screenshot(&ui, "build/desktop-main.png");
    SDL_Rect scaled_game;
    frontend_desktop_game_rect(&ui, 960, 720, 512, 240, false, &scaled_game);
    CHECK(scaled_game.y >= 96 && scaled_game.y + scaled_game.h <= 684);
    ui.ui_scale = 2;

    SDL_Rect game;
    frontend_desktop_game_rect(&ui, 1280, 960, 256, 240, true, &game);
    CHECK(game.y >= 128 && game.y + game.h <= 912);
    settings.remember_window_size = false;
    unsigned width = settings.window_width;
    frontend_desktop_update_window_settings(&ui);
    CHECK(settings.window_width == width);
    SDL_SetWindowSize(window, 1280, 960);
    SDL_RenderSetLogicalSize(renderer, 1280, 960);
    typed_settings(&ui);
    path_controls(&ui);
    mouse_controls(&ui);
    ui.info_open = ui.log_open = true;
    ui.panel_row = ui.panel_scroll = 0;
    for (int i = 0; i < 12; ++i) {
        desktop_copy_status(&ui, "A diagnostic message with useful details.");
    }
    SDL_SetWindowSize(window, 640, 480);
    SDL_RenderSetLogicalSize(renderer, 640, 480);
    render(&ui);
    SDL_FRect log_bar;
    CHECK(desktop_clay_bounds(ui.clay, HIT_SCROLLBAR, 2, 0, &log_bar));
    for (int i = 0; i < 11; ++i) {
        key(&ui, SDL_SCANCODE_DOWN, KMOD_NONE);
    }
    render(&ui);
    CHECK(ui.panel_scroll > 0 && ui.panel_row == 11);
    click_control(&ui, HIT_LOG, 11, 0);
    click_control(&ui, HIT_CLOSE, 0, 0);
    CHECK(!ui.info_open);
    SDL_SetWindowSize(window, 900, 680);
    SDL_RenderSetLogicalSize(renderer, 900, 680);
    modifier_bindings();
    window_regressions(&ui);
    frontend_desktop_shutdown(&ui);
    frontend_commands_reset();
    frontend_panels_reset();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    NesInputConfiguration previous = {
        joypad_adapter(), {joypad_port_device(0), joypad_port_device(1)}, joypad_expansion_device()};
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    FrontendDeviceRuntime devices;
    frontend_devices_init(&devices, NULL, &settings, NULL);
    char error[256];
    const uint8_t tape[] = {0, 1, 0, 1};
    CHECK(nes_file_write_atomic("build/desktop-tape.bin", tape, sizeof(tape)) == NES_FILE_OK);
    CHECK(frontend_devices_set_tape_paths(&devices, "build/desktop-tape.bin", "build/desktop-tape-out.bin", error,
                                          sizeof(error)));
    CHECK(!frontend_devices_set_tape_paths(&devices, "build/desktop-tape.bin", "build/desktop-tape.bin", error,
                                           sizeof(error)));
    CHECK(frontend_devices_tape_load(&devices, "build/desktop-tape.bin", error, sizeof(error)));
    CHECK(frontend_devices_tape_play(&devices, error, sizeof(error)));
    CHECK(family_basic_tape_mode() == FB_TAPE_PLAYING);
    CHECK(frontend_devices_tape_stop(&devices, error, sizeof(error)));
    CHECK(frontend_devices_tape_record(&devices, error, sizeof(error)));
    CHECK(devices.tape_capture_pending);
    CHECK(frontend_devices_finish(&devices, error, sizeof(error)));
    CHECK(!devices.tape_capture_pending);
    CHECK(!frontend_devices_set_barcode(&devices, "123x", error, sizeof(error)));
    CHECK(frontend_devices_set_barcode(&devices, "12345678", error, sizeof(error)));
    (void)nes_file_remove("build/desktop-tape.bin");
    (void)nes_file_remove("build/desktop-tape-out.bin");
    CHECK(joypad_apply_configuration(&previous));
    family_basic_shutdown();
    printf("Desktop and device controls: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
