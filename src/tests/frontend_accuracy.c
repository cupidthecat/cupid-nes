#include "../util/sha1.h"
/*
 * frontend_accuracy.c - Desktop frontend execution regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../ui/execution_control.h"
#include "../ui/desktop_ui.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_execution.h"
#include "../ui/machine_actions.h"
#include "../ui/frontend_panels.h"
#include "../ui/frontend_session.h"
#include "../ui/session_actions.h"
#include "../ui/settings.h"
#include "../util/file_io.h"
#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static unsigned frontend_checks;

#define CHECK(condition) do { \
    ++frontend_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint8_t image[16 + 0x8000 + 0x2000];

static void build_nrom(uint8_t region) {
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = 2;
    image[5] = 1;
    image[7] = 8;
    image[12] = region;
    memset(image + 16, 0xEA, 0x8000);
    image[16] = 0x4C;
    image[17] = 0x00;
    image[18] = 0x80;
    image[16 + 0x7FFA] = 0x00;
    image[16 + 0x7FFB] = 0x80;
    image[16 + 0x7FFC] = 0x00;
    image[16 + 0x7FFD] = 0x80;
    image[16 + 0x7FFE] = 0x00;
    image[16 + 0x7FFF] = 0x80;
}

static bool run_machine_frame(void *userdata) {
    (void)userdata;
    vs_start_frame();
    while (!ppu.frame_complete) vs_cpu_step();
    return true;
}

static int execution_gate_regions(void) {
    static const NesRegionMode modes[] = {
        NES_REGION_MODE_NTSC, NES_REGION_MODE_PAL, NES_REGION_MODE_DENDY
    };
    static const uint8_t regions[] = {0, 1, 3};
    for (unsigned i = 0; i < 3; ++i) {
        unload_rom();
        CHECK(nes_set_region_mode(modes[i]));
        build_nrom(regions[i]);
        CHECK(load_rom_memory(image, sizeof(image)) == 0);
        CHECK(frontend_machine_power_cycle());

        ExecutionControl control;
        execution_control_init(&control);
        execution_control_set_paused(&control, true);
        uint64_t frame = ppu.frame_count;
        uint64_t cycles = cpu_total_cycles;
        CHECK(!execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame && cpu_total_cycles == cycles);
        CHECK(execution_control_request_frame(&control));
        CHECK(execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame + 1 && cpu_total_cycles > cycles);
        cycles = cpu_total_cycles;
        CHECK(control.paused && !control.frame_advance_pending);
        CHECK(!execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(cpu_total_cycles == cycles);

        const NesTiming before = *nes_timing();
        CHECK(execution_control_set_speed(&control, 0.5));
        CHECK(execution_control_effective_speed(&control) == 0.5);
        execution_control_set_fast_forward_held(&control, true);
        CHECK(execution_control_effective_speed(&control) == 4.0);
        execution_control_set_fast_forward_held(&control, false);
        execution_control_toggle_fast_forward(&control);
        CHECK(execution_control_fast_forward_active(&control));
        CHECK(nes_timing()->region == before.region);
        CHECK(nes_timing()->cpu_hz == before.cpu_hz && nes_timing()->fps == before.fps);
        execution_control_toggle_fast_forward(&control);
        execution_control_set_paused(&control, false);
        frame = ppu.frame_count;
        CHECK(execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame + 1);
    }
    return 0;
}

static int lifecycle_actions(void) {
    unload_rom();
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    build_nrom(0);
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(frontend_machine_power_cycle());
    write_mem(0x0010, 0xA7);
    CHECK(read_mem(0x0010) == 0xA7);
    CHECK(frontend_machine_soft_reset());
    CHECK(read_mem(0x0010) == 0xA7);
    CHECK(frontend_machine_power_cycle());
    CHECK(read_mem(0x0010) == 0x00);
    return 0;
}

typedef struct {
    unsigned calls;
    bool result;
} CommandProbe;

static bool probe_command(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    CommandProbe *probe = (CommandProbe *)userdata;
    ++probe->calls;
    return probe->result;
}

static int command_registry(void) {
    frontend_commands_reset();
    CommandProbe probe = {.result = true};
    FrontendCommandSpec spec = {
        .id = FRONTEND_COMMAND_EXTENSION_BASE + 7,
        .label = "Probe",
        .menu = "Tools",
        .shortcut = "Ctrl+Q",
        .flags = FRONTEND_COMMAND_CHECKABLE,
        .handler = probe_command,
        .userdata = &probe
    };
    CHECK(frontend_command_register(&spec));
    CHECK(!frontend_command_register(&spec));
    CHECK(frontend_command_count() == 1);
    FrontendCommandInfo info;
    CHECK(frontend_command_get(spec.id, &info));
    CHECK(strcmp(info.label, "Probe") == 0 && strcmp(info.menu, "Tools") == 0);
    CHECK(frontend_command_set_checked(spec.id, true));
    CHECK(frontend_command_set_enabled(spec.id, false));
    char error[80];
    CHECK(!frontend_command_invoke(spec.id, error, sizeof(error)));
    CHECK(probe.calls == 0 && strstr(error, "unavailable") != NULL);
    CHECK(frontend_command_set_enabled(spec.id, true));
    CHECK(frontend_command_invoke(spec.id, error, sizeof(error)));
    CHECK(probe.calls == 1);
    CHECK(frontend_command_unregister(spec.id));
    CHECK(frontend_command_count() == 0);

    spec.id = FRONTEND_COMMAND_EXTENSION_BASE + 8;
    spec.flags = FRONTEND_COMMAND_NEEDS_SESSION;
    CHECK(frontend_command_register(&spec));
    FrontendCommandInfo gated;
    CHECK(frontend_command_get(spec.id, &gated) && !gated.enabled);
    CHECK(!frontend_command_invoke(spec.id, error, sizeof(error)));
    frontend_command_set_session_active(true);
    CHECK(frontend_command_get(spec.id, &gated) && gated.enabled);
    CHECK(frontend_command_invoke(spec.id, error, sizeof(error)));
    frontend_command_set_session_active(false);
    CHECK(frontend_command_unregister(spec.id));
    return 0;
}

static bool panel_snapshot_probe(void *userdata, FrontendPanelModel *model,
                                 char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    unsigned *calls = (unsigned *)userdata;
    ++*calls;
    static const char *const choices[] = {"One", "Two"};
    FrontendPanelControl text = {
        .id = 1, .type = FRONTEND_PANEL_TEXT, .label = "Value",
        .value = "ready", .enabled = true, .read_only = true
    };
    FrontendPanelControl choice = {
        .id = 2, .type = FRONTEND_PANEL_CHOICE, .label = "Mode",
        .items = choices, .item_count = 2, .selected = 1, .enabled = true
    };
    return frontend_panel_add_control(model, &text)
        && frontend_panel_add_control(model, &choice);
}

static bool panel_action_probe(void *userdata, unsigned control_id,
                               const char *value, int selected,
                               char *error, size_t error_size) {
    (void)value;
    (void)error;
    (void)error_size;
    unsigned *calls = (unsigned *)userdata;
    ++*calls;
    return control_id == 2 && selected == 0;
}

static int panel_registry(void) {
    frontend_panels_reset();
    unsigned calls = 0;
    FrontendPanelSpec spec = {
        .id = FRONTEND_PANEL_EXTENSION_BASE + 1,
        .title = "Probe Panel",
        .category = "Tools",
        .snapshot = panel_snapshot_probe,
        .action = panel_action_probe,
        .userdata = &calls
    };
    CHECK(frontend_panel_register(&spec));
    FrontendPanelControl controls[4];
    FrontendPanelModel model = {.controls = controls, .capacity = 4};
    char error[80];
    CHECK(frontend_panel_snapshot(spec.id, &model, error, sizeof(error)));
    CHECK(model.count == 2 && calls == 1);
    CHECK(model.controls[0].type == FRONTEND_PANEL_TEXT);
    CHECK(model.controls[1].type == FRONTEND_PANEL_CHOICE);
    CHECK(frontend_panel_action(spec.id, 2, NULL, 0, error, sizeof(error)));
    CHECK(calls == 2);
    CHECK(frontend_panel_unregister(spec.id));
    spec.id = FRONTEND_PANEL_EXTENSION_BASE + 2;
    spec.flags = FRONTEND_PANEL_NEEDS_SESSION;
    CHECK(frontend_panel_register(&spec));
    FrontendPanelInfo gated;
    CHECK(frontend_panel_get(spec.id, &gated) && !gated.enabled);
    frontend_panel_set_session_active(true);
    CHECK(frontend_panel_get(spec.id, &gated) && gated.enabled);
    frontend_panel_set_session_active(false);
    CHECK(frontend_panel_unregister(spec.id));
    return 0;
}

static int settings_round_trip(void) {
    static unsigned serial;
    char path[128];
    unsigned token = (unsigned)time(NULL) ^ ++serial;
    snprintf(path, sizeof(path), ".frontend-settings-%u.ini", token);

    FrontendSettings saved;
    frontend_settings_defaults(&saved);
    saved.region_mode = NES_REGION_MODE_PAL;
    saved.console_model = NES_CONSOLE_HVC101;
    saved.ntsc_composite = true;
    saved.speed = 1.25;
    saved.fast_forward_speed = 5.0;
    saved.reopen_last_image = true;
    saved.pause_on_focus_loss = false;
    saved.pause_on_ui = false;
    saved.show_fps = true;
    saved.fullscreen = true;
    saved.integer_scaling = true;
    saved.muted = true;
    saved.window_width = 1234;
    saved.window_height = 777;
    saved.disk_save_mode = FDS_SAVE_IN_PLACE;
    strcpy(saved.disk_overlay_path, "C:/saves/custom disk.ips");
    saved.fds_write_protected = true;
    saved.fds_auto_insert = true;
    saved.fds_loading_fast_forward = true;
    saved.nsf_player.automatic = false;
    saved.nsf_player.repeat = true;
    saved.nsf_player.shuffle = true;
    saved.nsf_player.detect_silence = false;
    saved.nsf_player.silence_ms = 4321;
    saved.nsf_player.silence_threshold = 0.0025f;
    saved.capture.sample_rate = 96000;
    saved.capture.byte_limit = 123456789u;
    saved.capture.displayed_output = true;
    strcpy(saved.capture_paths[0], "C:/capture/shot.png");
    strcpy(saved.capture_paths[1], "C:/capture/audio.wav");
    strcpy(saved.capture_paths[2], "C:/capture/video.avi");
    saved.state_slot = 7;
    strcpy(saved.state_file_path, "C:/states/game state.cstate");
    saved.input.adapter = NES_ADAPTER_FOUR_SCORE;
    saved.input.ports[0] = NES_PORT_GAMEPAD;
    saved.input.ports[1] = NES_PORT_GAMEPAD;
    saved.zapper_radius = 7;
    saved.saved_input_overrides = NES_INPUT_OVERRIDE_ADAPTER;
    CHECK(frontend_settings_add_profile(&saved, "wasd", "wasd"));
    CHECK(frontend_settings_select_profile(&saved, "wasd"));
    strcpy(saved.device_guid[1], "03000000cafef00d");

    FrontendSettingsReport report;
    CHECK(frontend_settings_save(path, &saved, &report));
    CHECK(report.file_result == NES_FILE_OK);

    FrontendSettings loaded;
    CHECK(frontend_settings_load(path, &loaded, &report));
    CHECK(report.found && !report.migrated && report.unknown_settings == 0);
    CHECK(loaded.region_mode == saved.region_mode);
    CHECK(loaded.console_model == saved.console_model);
    CHECK(loaded.ntsc_composite && loaded.speed == 1.25 && loaded.fast_forward_speed == 5.0);
    CHECK(loaded.reopen_last_image && !loaded.pause_on_focus_loss && !loaded.pause_on_ui
          && loaded.show_fps);
    CHECK(loaded.fullscreen && loaded.integer_scaling && loaded.muted);
    CHECK(loaded.window_width == 1234 && loaded.window_height == 777);
    CHECK(loaded.disk_save_mode == FDS_SAVE_IN_PLACE);
    CHECK(strcmp(loaded.disk_overlay_path, "C:/saves/custom disk.ips") == 0);
    CHECK(loaded.fds_write_protected && loaded.fds_auto_insert && loaded.fds_loading_fast_forward);
    CHECK(!loaded.nsf_player.automatic && loaded.nsf_player.repeat && loaded.nsf_player.shuffle);
    CHECK(!loaded.nsf_player.detect_silence && loaded.nsf_player.silence_ms == 4321);
    CHECK(fabs((double)loaded.nsf_player.silence_threshold - 0.0025) < 0.000001);
    CHECK(loaded.capture.sample_rate == 96000 && loaded.capture.byte_limit == 123456789u);
    CHECK(loaded.capture.displayed_output);
    CHECK(strcmp(loaded.capture_paths[0], "C:/capture/shot.png") == 0);
    CHECK(strcmp(loaded.capture_paths[1], "C:/capture/audio.wav") == 0);
    CHECK(strcmp(loaded.capture_paths[2], "C:/capture/video.avi") == 0);
    CHECK(loaded.state_slot == 7);
    CHECK(strcmp(loaded.state_file_path, "C:/states/game state.cstate") == 0);
    CHECK(loaded.input.adapter == NES_ADAPTER_FOUR_SCORE && loaded.zapper_radius == 7);
    CHECK(strcmp(loaded.active_profile, "wasd") == 0);
    CHECK(strcmp(loaded.device_guid[1], "03000000cafef00d") == 0);

    const FrontendBindingProfile *profile = frontend_settings_active_profile_const(&loaded);
    CHECK(profile != NULL);
    SDL_KeyboardEvent key;
    memset(&key, 0, sizeof(key));
    key.type = SDL_KEYDOWN;
    key.keysym.scancode = SDL_SCANCODE_J;
    unsigned player = 99, button = 99;
    CHECK(frontend_profile_player_key(profile, &key, &player, &button));
    CHECK(player == 0 && button == BTN_A);
    CHECK(frontend_profile_player_gamepad(profile, 2, SDL_CONTROLLER_BUTTON_B, &button));
    CHECK(button == BTN_B);

    key.keysym.scancode = SDL_SCANCODE_P;
    key.keysym.mod = KMOD_CTRL;
    FrontendShortcut shortcut = FRONTEND_SHORTCUT_COUNT;
    CHECK(frontend_profile_shortcut_key(profile, &key, &shortcut));
    CHECK(shortcut == FRONTEND_SHORTCUT_PAUSE);
    key.keysym.mod = (SDL_Keymod)(KMOD_CTRL | KMOD_SHIFT);
    CHECK(!frontend_profile_shortcut_key(profile, &key, &shortcut));

    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int settings_migration_and_errors(void) {
    static unsigned serial;
    char path[128];
    unsigned token = (unsigned)time(NULL) ^ ++serial;
    snprintf(path, sizeof(path), ".frontend-settings-migrate-%u.ini", token);
    const char old_settings[] =
        "version=1\n"
        "speed_percent=150\n"
        "fast_forward_percent=600\n"
        "unknown_future_setting=kept-for-forward-compatibility\n"
        "active_profile=default\n";
    CHECK(nes_file_write_atomic(path, old_settings, sizeof(old_settings) - 1) == NES_FILE_OK);
    FrontendSettings settings;
    FrontendSettingsReport report;
    CHECK(frontend_settings_load(path, &settings, &report));
    CHECK(report.migrated && report.unknown_settings == 1);
    CHECK(settings.version == FRONTEND_SETTINGS_VERSION);
    CHECK(settings.speed == 1.5 && settings.fast_forward_speed == 6.0);

    const char malformed[] = "version=2\nspeed=not-a-number\n";
    CHECK(nes_file_write_atomic(path, malformed, sizeof(malformed) - 1) == NES_FILE_OK);
    CHECK(!frontend_settings_load(path, &settings, &report));
    CHECK(report.line == 2 && strstr(report.message, "speed") != NULL);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int settings_cli_precedence(void) {
    NesRegionMode original_region = nes_region_mode();
    NesConsoleModel original_console = nes_console_model();
    NesInputConfiguration original_input = {
        .adapter = joypad_adapter(),
        .ports = {joypad_port_device(0), joypad_port_device(1)},
        .expansion = joypad_expansion_device()
    };
    unsigned original_radius = joypad_zapper_radius();
    uint8_t original_overrides = joypad_configuration_overrides();

    CHECK(nes_set_region_mode(NES_REGION_MODE_DENDY));
    CHECK(nes_set_console_model(NES_CONSOLE_HVC001));
    CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
    CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(joypad_set_zapper_radius(11));

    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.region_mode = NES_REGION_MODE_PAL;
    settings.console_model = NES_CONSOLE_NES101;
    settings.input.adapter = NES_ADAPTER_FOUR_SCORE;
    settings.input.ports[0] = NES_PORT_NONE;
    settings.input.ports[1] = NES_PORT_NONE;
    settings.zapper_radius = 3;
    settings.cli_overrides = FRONTEND_OVERRIDE_REGION | FRONTEND_OVERRIDE_CONSOLE
                           | FRONTEND_OVERRIDE_ADAPTER | FRONTEND_OVERRIDE_ZAPPER_RADIUS;
    char error[160];
    CHECK(frontend_settings_apply_core(&settings, error, sizeof(error)));
    CHECK(nes_region_mode() == NES_REGION_MODE_DENDY);
    CHECK(nes_console_model() == NES_CONSOLE_HVC001);
    CHECK(joypad_adapter() == NES_ADAPTER_NONE);
    CHECK(joypad_port_device(0) == NES_PORT_NONE && joypad_port_device(1) == NES_PORT_NONE);
    CHECK(joypad_zapper_radius() == 11);
    CHECK((joypad_configuration_overrides() & NES_INPUT_OVERRIDE_ADAPTER) != 0);

    CHECK(nes_set_region_mode(original_region));
    CHECK(nes_set_console_model(original_console));
    CHECK(joypad_apply_configuration(&original_input));
    CHECK(joypad_set_zapper_radius(original_radius));
    joypad_set_configuration_overrides(original_overrides);
    return 0;
}

static int remapped_execution_shortcuts(void) {
    unload_rom();
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    build_nrom(0);
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(frontend_machine_power_cycle());

    SDL_AudioDeviceID device = 0;
    FrontendExecutionRuntime runtime;
    size_t side = 0;
    frontend_execution_init(&runtime, &device, 44100, "binding.nes", NULL, NULL, &side);
    CHECK(frontend_execution_register_commands(&runtime));
    CHECK(frontend_execution_set_speeds(&runtime, 1.25, 5.0));
    CHECK(frontend_execution_speed(&runtime) == 1.25);

    CHECK(frontend_execution_handle_shortcut_action(
        &runtime, FRONTEND_SHORTCUT_PAUSE, true, false));
    CHECK(frontend_execution_paused(&runtime));
    CHECK(frontend_execution_handle_shortcut_action(
        &runtime, FRONTEND_SHORTCUT_FRAME_ADVANCE, true, false));
    uint64_t frame = ppu.frame_count;
    CHECK(frontend_execution_run_frame(&runtime));
    CHECK(ppu.frame_count == frame + 1);
    CHECK(frontend_execution_paused(&runtime));

    CHECK(frontend_execution_handle_shortcut_action(
        &runtime, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, true, false));
    CHECK(runtime.execution.fast_forward_held);
    CHECK(frontend_execution_speed(&runtime) == 5.0);
    frontend_execution_release_host_input(&runtime);
    CHECK(!runtime.execution.fast_forward_held);
    CHECK(frontend_execution_speed(&runtime) == 1.25);
    CHECK(frontend_execution_handle_shortcut_action(
        &runtime, FRONTEND_SHORTCUT_PAUSE, true, false));
    CHECK(!frontend_execution_paused(&runtime));
    frontend_execution_shutdown(&runtime);
    return 0;
}

typedef struct {
    unsigned calls;
    bool fail;
} SessionProbe;

static bool open_session_probe(void *userdata, const FrontendImageRequest *request,
                               FrontendImageResult *result,
                               char *error, size_t error_size) {
    SessionProbe *probe = (SessionProbe *)userdata;
    ++probe->calls;
    if (probe->fail || strstr(request->path, "missing")) {
        snprintf(error, error_size, "image validation failed");
        return false;
    }
    const char *name = request->archive_member[0] ? request->archive_member : request->path;
    const char *suffix = request->patch_path[0] ? ".patched" : ".save";
    if (strlen(name) + 7 >= sizeof(result->title)
        || strlen(request->path) + strlen(suffix) >= sizeof(result->save_identity)) {
        snprintf(error, error_size, "probe image name is too long");
        return false;
    }
    memcpy(result->title, "Loaded ", 7);
    memcpy(result->title + 7, name, strlen(name) + 1);
    memcpy(result->save_identity, request->path, strlen(request->path));
    memcpy(result->save_identity + strlen(request->path), suffix, strlen(suffix) + 1);
    if (!request->archive_member[0])
        snprintf(result->archive_member, sizeof(result->archive_member), "resolved/game.nes");
    return true;
}

static int session_transitions_and_recents(void) {
    char oversized[FRONTEND_IMAGE_PATH_MAX + 20];
    memset(oversized, 'a', sizeof(oversized) - 1);
    oversized[sizeof(oversized) - 1] = '\0';
    FrontendImageRequest request;
    CHECK(!frontend_image_request_init(&request, oversized));
    CHECK(frontend_image_request_init(&request, "C:/games/Crème 日本 🎮.zip"));
    CHECK(frontend_image_request_set_member(&request, "collection/ゲーム.nes"));
    CHECK(frontend_image_request_set_patch(&request, "C:/patches/fix 🎯.bps"));
    CHECK(frontend_image_request_set_fds_bios(&request, "C:/firmware/disk.bin"));
    CHECK(frontend_image_request_set_fds_overlay(&request, "C:/saves/overlay 🎮.ips"));
    request.fds_save_mode = FDS_SAVE_IN_PLACE;
    request.fds_write_protected = true;

    SessionProbe probe = {0};
    FrontendSession session;
    frontend_session_init(&session, open_session_probe, &probe);
    char error[160];
    CHECK(frontend_session_open(&session, &request, error, sizeof(error)));
    CHECK(session.active && probe.calls == 1 && frontend_session_recent_count(&session) == 1);
    CHECK(strcmp(session.current.archive_member, "collection/ゲーム.nes") == 0);
    CHECK(strstr(session.current.save_identity, ".patched") != NULL);

    FrontendImageRequest before = session.current;
    FrontendImageRequest failed;
    CHECK(frontend_image_request_init(&failed, "C:/games/missing.nes"));
    CHECK(!frontend_session_open(&session, &failed, error, sizeof(error)));
    CHECK(strstr(error, "validation") != NULL);
    CHECK(memcmp(&session.current, &before, sizeof(before)) == 0);
    CHECK(frontend_session_recent_count(&session) == 1);

    static unsigned serial;
    char recent_path[128];
    unsigned token = (unsigned)time(NULL) ^ ++serial;
    snprintf(recent_path, sizeof(recent_path), ".frontend-recent-%u.ini", token);
    CHECK(frontend_session_save_recent(&session, recent_path, error, sizeof(error)));

    FrontendSession restored;
    frontend_session_init(&restored, open_session_probe, &probe);
    CHECK(frontend_session_load_recent(&restored, recent_path, error, sizeof(error)));
    CHECK(frontend_session_recent_count(&restored) == 1);
    const FrontendImageRequest *recent = frontend_session_recent(&restored, 0);
    CHECK(recent != NULL);
    CHECK(strcmp(recent->path, "C:/games/Crème 日本 🎮.zip") == 0);
    CHECK(strcmp(recent->archive_member, "collection/ゲーム.nes") == 0);
    CHECK(strcmp(recent->patch_path, "C:/patches/fix 🎯.bps") == 0);
    CHECK(recent->fds_save_mode == FDS_SAVE_IN_PLACE);
    CHECK(strcmp(recent->fds_overlay_path, "C:/saves/overlay 🎮.ips") == 0);
    CHECK(recent->fds_write_protected);
    CHECK(frontend_session_open_recent(&restored, 0, error, sizeof(error)));
    CHECK(restored.active && probe.calls == 3);
    FrontendImageRequest retained = *frontend_session_recent(&restored, 0);
    const char wrong_version[] = "version=999\n";
    const char truncated_entry[] = "version=1\nrecent=missing-fields\n";
    const char embedded_nul[] = "version=1\n\0recent=hidden\n";
    const char *invalid_lists[] = {wrong_version, truncated_entry, embedded_nul};
    const size_t invalid_sizes[] = {sizeof(wrong_version) - 1, sizeof(truncated_entry) - 1,
                                    sizeof(embedded_nul) - 1};
    for (size_t i = 0; i < sizeof(invalid_lists) / sizeof(invalid_lists[0]); i++) {
        CHECK(nes_file_write_atomic(recent_path, invalid_lists[i], invalid_sizes[i]) == NES_FILE_OK);
        CHECK(!frontend_session_load_recent(&restored, recent_path, error, sizeof(error)));
        CHECK(error[0] && frontend_session_recent_count(&restored) == 1);
        CHECK(memcmp(frontend_session_recent(&restored, 0), &retained, sizeof(retained)) == 0);
    }
    const char legacy_list[] =
        "version=1\nrecent=C:/legacy.fds||||C:/firmware/disk.bin||1\n";
    CHECK(nes_file_write_atomic(recent_path, legacy_list, sizeof(legacy_list) - 1) == NES_FILE_OK);
    CHECK(frontend_session_load_recent(&restored, recent_path, error, sizeof(error)));
    recent = frontend_session_recent(&restored, 0);
    CHECK(recent && recent->fds_save_mode == FDS_SAVE_OVERLAY);
    CHECK(recent->fds_overlay_path[0] == '\0' && recent->fds_write_protected);
    CHECK(nes_file_remove(recent_path) == NES_FILE_OK);
    return 0;
}

static int settings_text_editing(void) {
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    memset(settings.fds_bios_path, 'a', 400);
    strcpy(settings.fds_bios_path + 400, "/café-猫-🎮.bin");
    FrontendDesktopUi ui = {0};
    ui.settings = &settings;
    ui.staged = settings;
    ui.settings_open = true;
    ui.settings_category = 5;
    SDL_Event event = {0};
    event.type = SDL_KEYDOWN;
    event.key.keysym.scancode = SDL_SCANCODE_RIGHT;
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(ui.edit_text_active && !strcmp(ui.edit_text, settings.fds_bios_path));
    event.key.keysym.scancode = SDL_SCANCODE_RETURN;
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(!ui.edit_text_active && !strcmp(ui.staged.fds_bios_path, settings.fds_bios_path));

    event.key.keysym.scancode = SDL_SCANCODE_RIGHT;
    CHECK(frontend_desktop_handle_event(&ui, &event));
    strcpy(ui.edit_text, "aé猫🎮");
    event.key.keysym.scancode = SDL_SCANCODE_BACKSPACE;
    const char *expected[] = {"aé猫", "aé", "a", "", ""};
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i) {
        CHECK(frontend_desktop_handle_event(&ui, &event));
        CHECK(!strcmp(ui.edit_text, expected[i]));
    }

    memset(ui.edit_text, 'x', sizeof(ui.edit_text) - 3);
    ui.edit_text[sizeof(ui.edit_text) - 3] = '\0';
    event.type = SDL_TEXTINPUT;
    strcpy(event.text.text, "é");
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(strlen(ui.edit_text) == sizeof(ui.edit_text) - 1);
    strcpy(event.text.text, "🎮");
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(strlen(ui.edit_text) == sizeof(ui.edit_text) - 1);
    CHECK(!strcmp(ui.edit_text + sizeof(ui.edit_text) - 3, "é"));

    ui.settings_category = 4;
    ui.edit_control = 0x80000000u | 7u;
    strcpy(ui.staged.device_guid[0], "retained");
    memset(ui.edit_text, 'x', 80);
    ui.edit_text[80] = '\0';
    memset(&event, 0, sizeof(event));
    event.type = SDL_KEYDOWN;
    event.key.keysym.scancode = SDL_SCANCODE_RETURN;
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(ui.edit_text_active && !strcmp(ui.staged.device_guid[0], "retained"));
    CHECK(ui.status[0]);
    event.key.keysym.scancode = SDL_SCANCODE_ESCAPE;
    CHECK(frontend_desktop_handle_event(&ui, &event));
    CHECK(!ui.edit_text_active && !strcmp(ui.staged.device_guid[0], "retained"));
    return 0;
}

static bool transactional_open_probe(void *userdata, const FrontendImageRequest *request,
                                     FrontendImageResult *result,
                                     char *error, size_t error_size) {
    (void)userdata;
    build_nrom(0);
    if (strstr(request->path, "replacement")) image[16 + 0x123] ^= 0x5Au;
    if (load_rom_memory(image, sizeof(image)) != 0) {
        snprintf(error, error_size, "probe image load failed");
        return false;
    }
    memset(result, 0, sizeof(*result));
    snprintf(result->title, sizeof(result->title), "%.159s", request->path);
    snprintf(result->save_identity, sizeof(result->save_identity), "%s", request->path);
    return true;
}

static int session_replacement_rollback(void) {
    static unsigned serial;
    unsigned token = (unsigned)time(NULL) ^ ++serial;
    char original_path[128];
    char replacement_path[128];
    char replacement_storage[160];
    snprintf(original_path, sizeof(original_path), ".frontend-session-original-%u.nes", token);
    snprintf(replacement_path, sizeof(replacement_path),
             ".frontend-session-replacement-%u.nes", token);
    snprintf(replacement_storage, sizeof(replacement_storage),
             ".frontend-session-replacement-%u.turbofile.sav", token);
    (void)nes_file_remove(replacement_storage);

    unload_rom();
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    FrontendSession session;
    frontend_session_init(&session, transactional_open_probe, NULL);
    FrontendImageRequest original;
    CHECK(frontend_image_request_init(&original, original_path));
    char error[320] = {0};
    CHECK(frontend_session_open(&session, &original, error, sizeof(error)));
    CHECK(frontend_machine_power_cycle());
    uint32_t original_crc = rom_file_crc32();
    write_mem(0x0010, 0xA5);
    CHECK(read_mem(0x0010) == 0xA5);

    NesExpansionDevice previous_expansion = joypad_expansion_device();
    CHECK(joypad_set_expansion_device(NES_EXPANSION_TURBO_FILE));
    CHECK(joypad_persistent_configure(original_path));
    const uint8_t malformed_storage[] = {0x11};
    CHECK(nes_file_write_atomic(replacement_storage, malformed_storage,
                                sizeof(malformed_storage)) == NES_FILE_OK);

    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendSessionActions actions;
    frontend_session_actions_init(&actions, &session, &settings, NULL);
    CHECK(!frontend_session_action_open_path(&actions, replacement_path,
                                             error, sizeof(error)));
    CHECK(strstr(error, "previous session was restored") != NULL);
    CHECK(session.active && strcmp(session.current.path, original_path) == 0);
    CHECK(rom_file_crc32() == original_crc);
    CHECK(read_mem(0x0010) == 0xA5);
    CHECK(joypad_expansion_device() == NES_EXPANSION_TURBO_FILE);
    CHECK(joypad_persistent_flush());
    CHECK(joypad_persistent_shutdown());
    CHECK(joypad_set_expansion_device(previous_expansion));
    CHECK(nes_file_remove(replacement_storage) == NES_FILE_OK);
    return 0;
}

static int image_identity_hash(void) {
    char hash[41];
    nes_sha1(NULL, 0, hash);
    CHECK(!strcmp(hash, "da39a3ee5e6b4b0d3255bfef95601890afd80709"));
    nes_sha1("abc", 3, hash);
    CHECK(!strcmp(hash, "a9993e364706816aba3e25717850c26c9cd0d89d"));
    const char *long_vector = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    nes_sha1(long_vector, strlen(long_vector), hash);
    CHECK(!strcmp(hash, "84983e441c3bd26ebaae4aa1f95129e5e54670f1"));
    return 0;
}

int test_frontend_accuracy(void) {
    const NesRegionMode saved_mode = nes_region_mode();
    const NesRegion saved_region = nes_timing()->region;
    const NesRamPowerOnState saved_ram = nes_ram_power_on_state();
    int failures = 0;
    frontend_checks = 0;
    failures += image_identity_hash();
    failures += execution_gate_regions();
    failures += lifecycle_actions();
    failures += command_registry();
    failures += panel_registry();
    failures += settings_round_trip();
    failures += settings_migration_and_errors();
    failures += settings_cli_precedence();
    failures += remapped_execution_shortcuts();
    failures += session_transitions_and_recents();
    failures += settings_text_editing();
    failures += session_replacement_rollback();
    unload_rom();
    frontend_commands_reset();
    nes_set_region_mode(saved_mode);
    nes_set_region(saved_region);
    nes_set_ram_power_on_state(saved_ram);
    printf("Frontend execution: %u checks, %d failures\n", frontend_checks, failures);
    return failures;
}
