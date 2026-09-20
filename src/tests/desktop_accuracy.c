/* Desktop event, layout and device regression checks. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../apu/apu.h"
#include "../capture/capture_writer.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../system/execution_policy.h"
#include "../ui/app_paths.h"
#include "../ui/clay_backend.h"
#include "../ui/desktop_ui.h"
#include "../ui/device_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/palette_tool.h"
#include "../ui/storage_frontend.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x)) {                                                                                          \
            fprintf(stderr, "Desktop check %d failed: %s\n", __LINE__, #x);                                  \
            ++failures;                                                                                      \
        }                                                                                                    \
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
    if (!found)
        return;
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
        click_control(ui, HIT_SETTING, 1, 1);
        CHECK(ui->staged.speed != previous);
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
        if (dpi == 0)
            screenshot(ui, "build/desktop-palette.png");
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
    ui->settings_category = 5;
    ui->settings_row = 0;
    key(ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(ui->edit_text_active && ui->edit_select_all);
    SDL_Event text = {.type = SDL_TEXTINPUT};
    strcpy(text.text.text, "firmware.bin");
    CHECK(frontend_desktop_handle_event(ui, &text));
    CHECK(!strcmp(ui->edit_text, "firmware.bin"));
    render(ui);
    screenshot(ui, "build/desktop-text-entry.png");
    click_control(ui, HIT_EDIT_OK, 0, 0);
    CHECK(!ui->edit_text_active && !strcmp(ui->staged.fds_bios_path, "firmware.bin"));
    SDL_Event quit = {.type = SDL_QUIT};
    CHECK(!frontend_desktop_handle_event(ui, &quit));
    render(ui);
    click_control(ui, HIT_SETTINGS_BUTTON, 2, 0);
}

static bool panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    for (unsigned i = 0; i < 24; ++i) {
        FrontendPanelControl c = {i, FRONTEND_PANEL_ACTION, "Action", NULL, NULL, 0, -1, true, false};
        if (!frontend_panel_add_control(model, &c))
            return false;
    }
    return true;
}
static bool panel_action(void *context, unsigned id, const char *value, int selected, char *error,
                         size_t size) {
    (void)value;
    (void)selected;
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
    if (!pixels)
        return;
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
    if (!device)
        return;
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
    CHECK(!strcmp(restored.movie_file_path, settings.movie_file_path) &&
          restored.disable_database_corrections);
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
int test_desktop_accuracy(void) {
    failures = 0;
    audio_settings();
    storage_settings();
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_Window *window =
        SDL_CreateWindow("Desktop checks", 0, 0, 768, 720, SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    CHECK(window && renderer);
    if (!renderer) {
        if (window)
            SDL_DestroyWindow(window);
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
    for (int i = 0; i < 3; ++i)
        key(&ui, SDL_SCANCODE_RIGHT, KMOD_NONE);
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
    for (int i = 0; i < 4; ++i)
        key(&ui, SDL_SCANCODE_RIGHT, KMOD_NONE);
    CHECK(ui.open_menu == 4);
    key(&ui, SDL_SCANCODE_ESCAPE, KMOD_NONE);
    unsigned selected = 0;
    FrontendPanelSpec panel = {0x7F00,         "Scrollable panel", "Tools",  0,
                               panel_snapshot, panel_action,       &selected};
    CHECK(frontend_panel_register(&panel));
    ui.panel_id = panel.id;
    ui.panel_open = true;
    for (int i = 0; i < 20; ++i)
        key(&ui, SDL_SCANCODE_DOWN, KMOD_NONE);
    key(&ui, SDL_SCANCODE_RETURN, KMOD_NONE);
    CHECK(selected == 20 && ui.panel_scroll > 0);
    SDL_RenderClear(renderer);
    frontend_desktop_render(&ui, 256, 240, "Synthetic cartridge", "NTSC", "Paused");
    screenshot(&ui, "build/desktop-panel.png");
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
    mouse_controls(&ui);
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
    CHECK(frontend_devices_set_tape_paths(&devices, "build/desktop-tape.bin", "build/desktop-tape-out.bin",
                                          error, sizeof(error)));
    CHECK(!frontend_devices_set_tape_paths(&devices, "build/desktop-tape.bin", "build/desktop-tape.bin",
                                           error, sizeof(error)));
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
