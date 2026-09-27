/*
 * lifecycle_accuracy.c - Production recovery, configuration and unload regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/lifecycle_frontend.h"
#include "../ui/recovery_store.h"
#include "../ui/game_config.h"
#include "../ui/image_open.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../rom/fds.h"
#include "../rom/mapper.h"
#include "../joypad/special_peripherals.h"
#include "../ui/capture_runtime.h"
#include "../ppu/ppu.h"
#include "../system/execution_policy.h"
#include "../util/file_io.h"
#include "../util/sha1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#define RMDIR(p) _rmdir(p)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(p) mkdir(p, 0700)
#define RMDIR(p) rmdir(p)
#endif
#define CHECK(c)                                                                                                       \
    do {                                                                                                               \
        if (!(c)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #c);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool fixture(const char *path, unsigned marker) {
    size_t size = 16 + 32768 + 8192;
    uint8_t *rom = calloc(1, size);
    if (!rom) {
        return false;
    }
    memcpy(rom, "NES\032", 4);
    rom[4] = 2;
    rom[5] = 1;
    rom[6] = 2;
    rom[16] = 0x4c;
    rom[17] = 0;
    rom[18] = 0x80;
    rom[19] = (uint8_t)marker;
    for (unsigned i = 0x7ffa; i <= 0x7ffe; i += 2) {
        rom[16 + i + 1] = 0x80;
    }
    bool ok = nes_file_write_atomic(path, rom, size) == NES_FILE_OK;
    free(rom);
    return ok;
}

static bool finalize_fail(void *context, char *error, size_t size) {
    (void)context;
    if (error && size) {
        snprintf(error, size, "Synthetic capture finalization failure");
    }
    return false;
}

static int exercise(const char *directory) {
    char image[1024], second[1024], path[4096], key[41], error[256] = {0};
    snprintf(image, sizeof(image), "%s/first.nes", directory);
    snprintf(second, sizeof(second), "%s/second.nes", directory);
    CHECK(fixture(image, 1) && fixture(second, 2));
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    CHECK(frontend_settings_apply_core(&settings, error, sizeof(error)));
    FrontendSession session;
    frontend_session_init(&session, frontend_image_open, NULL);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    FrontendSessionActions actions;
    frontend_session_actions_init(&actions, &session, &settings, NULL);
    frontend_session_actions_set_execution(&actions, &execution);
    StateRuntime state;
    state_runtime_init(&state, &settings, directory, &execution);
    LifecycleFrontend lifecycle;
    CHECK(lifecycle_init(&lifecycle, &actions, &state, directory, error, sizeof(error)));
    CHECK(frontend_session_action_open_path(&actions, image, error, sizeof(error)));
    CHECK(session.active && frontend_command_session_active());
    CHECK(recovery_game_key(&session, key));
    write_mem(23, 0x6a);
    CHECK(lifecycle_save_last(&lifecycle, error, sizeof(error)));
    CHECK(lifecycle_register_ui(&lifecycle));
    CHECK(frontend_command_invoke(LIFECYCLE_UNLOAD, error, sizeof(error)));
    CHECK(!session.active && rom_metadata_source() == ROM_METADATA_NONE);
    CHECK(session.recent_count == 1 && !frontend_command_session_active());
    CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    CHECK(lifecycle_resume_startup(&lifecycle, false, error, sizeof(error)) && !session.active);
    CHECK(frontend_command_invoke(LIFECYCLE_LOAD_LAST, error, sizeof(error)));
    CHECK(session.active && read_mem(23) == 0x6a);
    execution.before_machine_change = finalize_fail;
    CHECK(!frontend_session_action_unload(&actions, error, sizeof(error)) && session.active && read_mem(23) == 0x6a);
    execution.before_machine_change = NULL;
    CHECK(frontend_session_action_open_path(&actions, second, error, sizeof(error)));
    write_mem(23, 0x9b);
    CHECK(lifecycle_load_last(&lifecycle, error, sizeof(error)) && read_mem(23) == 0x6a);
    StateRecorderOptions opts = {true, false, 2, 2};
    CHECK(state_recorder_configure(&lifecycle.recorder, &opts));
    lifecycle_image_changed(&lifecycle);
    CHECK(lifecycle.recorder.running && lifecycle.recorder.count == 0);
    CHECK(state_recorder_tick(&lifecycle.recorder, 17, error, sizeof(error)) && lifecycle.recorder.count == 0);
    write_mem(23, 1);
    CHECK(state_recorder_tick(&lifecycle.recorder, 17, error, sizeof(error)) && lifecycle.recorder.count == 1);
    write_mem(23, 2);
    CHECK(state_recorder_capture(&lifecycle.recorder, error, sizeof(error)));
    write_mem(23, 3);
    CHECK(state_recorder_capture(&lifecycle.recorder, error, sizeof(error)));
    CHECK(lifecycle.recorder.count == 2);
    CHECK(state_recorder_restore(&lifecycle.recorder, 0, error, sizeof(error)) && read_mem(23) == 2);
    opts.time_based = true;
    opts.interval = 40;
    CHECK(state_recorder_configure(&lifecycle.recorder, &opts));
    unsigned newest = lifecycle.recorder.slots[1];
    CHECK(state_recorder_tick(&lifecycle.recorder, 20, error, sizeof(error)));
    CHECK(lifecycle.recorder.slots[1] == newest);
    CHECK(state_recorder_tick(&lifecycle.recorder, 20, error, sizeof(error)));
    CHECK(lifecycle.recorder.slots[1] != newest);
    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    CHECK(!state_recorder_restore(&lifecycle.recorder, 0, error, sizeof(error)));
    CHECK(!frontend_session_action_unload(&actions, error, sizeof(error)));
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    char saved_directory[2048];
    strcpy(saved_directory, lifecycle.recorder.directory);
    snprintf(lifecycle.recorder.directory, sizeof(lifecycle.recorder.directory), "%s/missing", directory);
    size_t count = lifecycle.recorder.count;
    CHECK(!state_recorder_capture(&lifecycle.recorder, error, sizeof(error)));
    CHECK(lifecycle.recorder.count == count && !lifecycle.recorder.running);
    strcpy(lifecycle.recorder.directory, saved_directory);
    CHECK(state_recorder_restore(&lifecycle.recorder, 0, error, sizeof(error)));
    CHECK(frontend_session_action_open_path(&actions, second, error, sizeof(error)));
    lifecycle_image_changed(&lifecycle);
    CHECK(lifecycle.recorder.count == 0 && lifecycle.recorder.running);
    CHECK(lifecycle_load_last(&lifecycle, error, sizeof(error)));
    /* Corruption does not replace the running machine. */
    CHECK(recovery_path(directory, key, ".session", path, sizeof(path)));
    uint8_t *bytes = NULL;
    size_t size = 0;
    CHECK(nes_file_read_all(path, 128 * 1024 * 1024, &bytes, &size) == NES_FILE_OK);
    bytes[size - 1] ^= 1;
    CHECK(nes_file_write_atomic(path, bytes, size) == NES_FILE_OK);
    free(bytes);
    write_mem(23, 55);
    CHECK(!lifecycle_load_last(&lifecycle, error, sizeof(error)) && session.active && read_mem(23) == 55);
    CHECK(lifecycle_save_last(&lifecycle, error, sizeof(error)));
    CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    CHECK(nes_file_remove(image) == NES_FILE_OK);
    CHECK(!lifecycle_load_last(&lifecycle, error, sizeof(error)) && !session.active);
    CHECK(fixture(image, 9));
    CHECK(!lifecycle_load_last(&lifecycle, error, sizeof(error)) && !session.active);
    CHECK(fixture(image, 1));
    lifecycle.resume_on_start = true;
    CHECK(lifecycle_save_preferences(&lifecycle, error, sizeof(error)));
    LifecycleFrontend restarted;
    CHECK(lifecycle_init(&restarted, &actions, &state, directory, error, sizeof(error)) && restarted.resume_on_start);
    CHECK(lifecycle_resume_startup(&restarted, false, error, sizeof(error)) && read_mem(23) == 55);
    /* An otherwise valid envelope containing an incompatible core state rolls back. */
    NesStateBlob blob = {0};
    CHECK(nes_state_capture(&blob) == NES_STATE_OK);
    blob.data[8] ^= 0x7f;
    CHECK(recovery_save_session(directory, &session, &blob, error, sizeof(error)));
    nes_state_blob_free(&blob);
    CHECK(!lifecycle_load_last(&lifecycle, error, sizeof(error)) && session.active && read_mem(23) == 55);
    CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    CHECK(!lifecycle_load_last(&lifecycle, error, sizeof(error)) && !session.active);
    CHECK(frontend_session_action_open_path(&actions, second, error, sizeof(error)));
    lifecycle_unregister_ui();
    state_runtime_shutdown(&state);
    CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    frontend_execution_shutdown(&execution);
    return 0;
}

static int configurations(const char *directory) {
    FrontendSettings global, effective;
    frontend_settings_defaults(&global);
    GameConfig *config = calloc(1, sizeof(*config)), *loaded = calloc(1, sizeof(*loaded));
    CHECK(config && loaded);
    int region = game_config_find_field("region"), volume = game_config_find_field("master_volume");
    CHECK(region >= 0 && volume >= 0);
    CHECK(game_config_set(config, (size_t)region, "2") && game_config_set(config, (size_t)volume, "37"));
    CHECK(!game_config_set(config, (size_t)volume, "101") && !game_config_set(config, (size_t)volume, "nan"));
    CHECK(!game_config_set(config, (size_t)game_config_find_field("fds_bios"), "bad\npath"));
    char key[41], error[256];
    nes_sha1("patched-member", 14, key);
    CHECK(game_config_save(directory, key, config, error, sizeof(error)));
    CHECK(game_config_load(directory, key, loaded, error, sizeof(error)) && loaded->present == config->present);
    CHECK(game_config_resolve(&global, loaded, 0, &effective, NULL, 0, error, sizeof(error)));
    CHECK(effective.region_mode == NES_REGION_MODE_PAL && effective.audio_mix.master_volume == 37);
    CHECK(global.region_mode == NES_REGION_MODE_AUTO && global.audio_mix.master_volume == 100);
    global.cli_overrides = FRONTEND_OVERRIDE_REGION;
    CHECK(game_config_resolve(&global, loaded, 0, &effective, NULL, 0, error, sizeof(error)) &&
          effective.region_mode == global.region_mode);
    CHECK(game_config_resolve(&global, loaded, UINT64_C(1) << volume, &effective, NULL, 0, error, sizeof(error)) &&
          effective.audio_mix.master_volume == 100);
    game_config_reset(loaded, (size_t)volume);
    CHECK(game_config_resolve(&global, loaded, 0, &effective, NULL, 0, error, sizeof(error)) &&
          effective.audio_mix.master_volume == 100);
    char path[4096];
    CHECK(recovery_path(directory, key, ".game.ini", path, sizeof(path)));
    CHECK(nes_file_write_atomic(path, "version=1\nregion=2\nregion=3\n", 28) == NES_FILE_OK);
    uint64_t previous = loaded->present;
    CHECK(!game_config_load(directory, key, loaded, error, sizeof(error)) && loaded->present == previous);
    free(config);
    free(loaded);
    return 0;
}

static int configuration_activation(const char *directory) {
    char image[1024], other[1024], error[256], key[41], other_key[41], settings_path[1024];
    snprintf(image, sizeof(image), "%s/configured.nes", directory);
    snprintf(other, sizeof(other), "%s/inherited.nes", directory);
    snprintf(settings_path, sizeof(settings_path), "%s/globals.ini", directory);
    CHECK(fixture(image, 3) && fixture(other, 4));
    FrontendSettings global, effective;
    frontend_settings_defaults(&global);
    effective = global;
    GameConfigFrontend *configuration = calloc(1, sizeof(*configuration));
    CHECK(configuration && game_config_init(configuration, &global, directory));
    FrontendImageEnvironment environment = {.configuration = configuration, .effective = &effective};
    FrontendSession session;
    frontend_session_init(&session, frontend_image_open, &environment);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    FrontendSessionActions actions;
    frontend_session_actions_init(&actions, &session, &effective, NULL);
    actions.game_config = configuration;
    frontend_session_actions_set_execution(&actions, &execution);
    CHECK(frontend_session_action_open_path(&actions, image, error, sizeof(error)));
    CHECK(recovery_game_key(&session, key));
    CHECK(game_config_set(&configuration->config, (size_t)game_config_find_field("region"), "2"));
    CHECK(game_config_set(&configuration->config, (size_t)game_config_find_field("master_volume"), "37"));
    CHECK(game_config_save(directory, key, &configuration->config, error, sizeof(error)));
    CHECK(nes_region_mode() == NES_REGION_MODE_AUTO);
    CHECK(frontend_session_action_reload(&actions, error, sizeof(error)));
    CHECK(effective.region_mode == NES_REGION_MODE_PAL && nes_region_mode() == NES_REGION_MODE_PAL);
    CHECK(effective.audio_mix.master_volume == 37 && global.audio_mix.master_volume == 100);
    FrontendSettingsReport report;
    CHECK(game_config_save_globals(configuration, settings_path, &effective, &report));
    CHECK(global.region_mode == NES_REGION_MODE_AUTO && global.audio_mix.master_volume == 100);
    CHECK(frontend_session_action_open_path(&actions, other, error, sizeof(error)));
    CHECK(recovery_game_key(&session, other_key) && strcmp(key, other_key));
    CHECK(nes_region_mode() == NES_REGION_MODE_AUTO && effective.audio_mix.master_volume == 100);
    CHECK(game_config_set(&configuration->config, (size_t)game_config_find_field("fds_bios"), "missing-firmware.bin"));
    CHECK(game_config_save(directory, other_key, &configuration->config, error, sizeof(error)));
    CHECK(frontend_session_action_open_path(&actions, image, error, sizeof(error)));
    write_mem(23, 73);
    CHECK(!frontend_session_action_open_path(&actions, other, error, sizeof(error)));
    CHECK(session.active && !strcmp(session.current.path, image) && read_mem(23) == 73);
    CHECK(effective.region_mode == NES_REGION_MODE_PAL && !strcmp(configuration->key, key));
    CHECK(game_config_register_ui(configuration, &global, directory));
    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    CHECK(!frontend_panel_action(GAME_CONFIG_PANEL, 4, NULL, 0, error, sizeof(error)));
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    game_config_unregister_ui();
    CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    frontend_execution_shutdown(&execution);
    free(configuration);
    return 0;
}

#define MEDIA_CHECK(c)                                                                                                 \
    do {                                                                                                               \
        if (!(c)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #c);                                                    \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static int media_unload(const char *directory) {
    int failure = 1;
    bool blocked = false, execution_ready = false, capture_ready = false;
    FrontendExecutionRuntime execution = {0};
    NesCaptureRuntime capture = {0};
    uint8_t *saved = NULL;
    size_t saved_size = 0;
    char disk_path[1024], bios_path[1024], music_path[1024], cart_path[1024], wav_path[1024], save_path[2048],
        error[256];
    snprintf(disk_path, sizeof(disk_path), "%s/media.fds", directory);
    snprintf(bios_path, sizeof(bios_path), "%s/bios.bin", directory);
    snprintf(music_path, sizeof(music_path), "%s/music.nsf", directory);
    snprintf(cart_path, sizeof(cart_path), "%s/peripheral.nes", directory);
    snprintf(wav_path, sizeof(wav_path), "%s/unload.wav", directory);
    uint8_t *disk = calloc(1, 65516), bios[8192];
    MEDIA_CHECK(disk);
    memcpy(disk, "FDS\032", 4);
    disk[4] = 1;
    disk[16] = 1;
    disk[17] = 0x2a;
    disk[16 + 56] = 2;
    memset(bios, 0xea, sizeof(bios));
    for (unsigned i = 0x1ffa; i <= 0x1ffe; i += 2) {
        bios[i] = 0;
        bios[i + 1] = 0xe0;
    }
    MEDIA_CHECK(nes_file_write_atomic(disk_path, disk, 65516) == NES_FILE_OK);
    MEDIA_CHECK(nes_file_write_atomic(bios_path, bios, sizeof(bios)) == NES_FILE_OK);
    free(disk);
    disk = NULL;
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    strcpy(settings.fds_bios_path, bios_path);
    MEDIA_CHECK(frontend_settings_apply_core(&settings, error, sizeof(error)));
    FrontendSession session;
    frontend_session_init(&session, frontend_image_open, NULL);
    FrontendSessionActions actions;
    frontend_session_actions_init(&actions, &session, &settings, NULL);
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    execution_ready = true;
    frontend_session_actions_set_execution(&actions, &execution);
    MEDIA_CHECK(frontend_session_action_open_path(&actions, disk_path, error, sizeof(error)) && rom_is_fds());
    MEDIA_CHECK(fds_insert_disk(0));
    cart_cpu_write(0x4025, 0xc5);
    for (unsigned cycle = 0; cycle < 700000 && !fds_irq_pending(); ++cycle) {
        fds_clock_cpu(1);
    }
    MEDIA_CHECK(fds_irq_pending() && cart_cpu_read_bus(0x4031, 0) == 1);
    cart_cpu_write(0x4024, 0x66);
    cart_cpu_write(0x4025, 0x41);
    fds_clock_cpu(150);
    MEDIA_CHECK(fds_disk_dirty());
    snprintf(save_path, sizeof(save_path), "%s", fds_save_path());
    MEDIA_CHECK(MKDIR(save_path) == 0);
    blocked = true;
    MEDIA_CHECK(!frontend_session_action_unload(&actions, error, sizeof(error)) && session.active && fds_disk_dirty());
    MEDIA_CHECK(RMDIR(save_path) == 0);
    blocked = false;
    MEDIA_CHECK(frontend_session_action_unload(&actions, error, sizeof(error)) && !rom_is_fds());
    MEDIA_CHECK(nes_file_read_all(save_path, 128 * 1024, &saved, &saved_size) == NES_FILE_OK && saved_size > 0);
    free(saved);
    saved = NULL;
    uint8_t nsf[160] = {0};
    memcpy(nsf, "NESM\032", 5);
    nsf[5] = 1;
    nsf[6] = 1;
    nsf[7] = 1;
    nsf[9] = nsf[11] = nsf[13] = 0x80;
    nsf[12] = 0x10;
    nsf[0x80] = nsf[0x90] = 0x60;
    MEDIA_CHECK(nes_file_write_atomic(music_path, nsf, sizeof(nsf)) == NES_FILE_OK);
    settings.fds_bios_path[0] = 0;
    MEDIA_CHECK(frontend_session_action_open_path(&actions, music_path, error, sizeof(error)) && rom_is_nsf());
    MEDIA_CHECK(nes_capture_runtime_init(&capture, &execution, NULL, NULL, NULL, 0));
    capture_ready = true;
    MEDIA_CHECK(nes_capture_session_start(&capture.frontend.session, wav_path, false, NULL,
                                          &capture.frontend.options) == NES_FILE_OK);
    MEDIA_CHECK(capture.frontend.session.info.recording);
    MEDIA_CHECK(frontend_session_action_unload(&actions, error, sizeof(error)) && !rom_is_nsf());
    MEDIA_CHECK(!capture.frontend.session.info.recording);
    MEDIA_CHECK(nes_file_read_all(wav_path, 1024, &saved, &saved_size) == NES_FILE_OK && saved_size >= 44 &&
                !memcmp(saved, "RIFF", 4));
    free(saved);
    saved = NULL;
    MEDIA_CHECK(nes_capture_runtime_shutdown(&capture) == NES_FILE_OK);
    capture_ready = false;
    MEDIA_CHECK(fixture(cart_path, 5));
    settings.console_model = NES_CONSOLE_HVC001;
    settings.input.expansion = NES_EXPANSION_TURBO_FILE;
    settings.saved_input_overrides = NES_INPUT_OVERRIDE_EXPANSION;
    MEDIA_CHECK(frontend_settings_apply_core(&settings, error, sizeof(error)));
    MEDIA_CHECK(frontend_session_action_open_path(&actions, cart_path, error, sizeof(error)));
    joypad_write_ports(0);
    joypad_write_ports(7);
    joypad_write_ports(3);
    snprintf(save_path, sizeof(save_path), "%s/peripheral.turbofile.sav", directory);
    MEDIA_CHECK(MKDIR(save_path) == 0);
    blocked = true;
    MEDIA_CHECK(!frontend_session_action_unload(&actions, error, sizeof(error)) && session.active);
    MEDIA_CHECK(RMDIR(save_path) == 0);
    blocked = false;
    MEDIA_CHECK(frontend_session_action_unload(&actions, error, sizeof(error)));
    MEDIA_CHECK(nes_file_read_all(save_path, 8192, &saved, &saved_size) == NES_FILE_OK && saved_size == 8192 &&
                saved[0] == 1);
    free(saved);
    saved = NULL;
    MEDIA_CHECK(session.recent_count == 3);
    MEDIA_CHECK(joypad_persistent_shutdown());
    frontend_execution_shutdown(&execution);
    execution_ready = false;
    frontend_settings_defaults(&settings);
    MEDIA_CHECK(frontend_settings_apply_core(&settings, error, sizeof(error)));
    failure = 0;
cleanup:
    if (blocked) {
        (void)RMDIR(save_path);
    }
    if (capture_ready) {
        (void)nes_capture_runtime_shutdown(&capture);
    }
    free(disk);
    free(saved);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)joypad_persistent_shutdown();
    (void)unload_rom();
    if (execution_ready) {
        frontend_execution_shutdown(&execution);
    }
    return failure;
}

#undef MEDIA_CHECK

int test_lifecycle_accuracy(void) {
    char directory[128];
    snprintf(directory, sizeof(directory), "build/lifecycle-test-%llu",
             (unsigned long long)SDL_GetPerformanceCounter());
    CHECK(MKDIR(directory) == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    frontend_commands_reset();
    frontend_panels_reset();
    int failures = configurations(directory);
    if (!failures) {
        failures = exercise(directory);
    }
    if (!failures) {
        failures = configuration_activation(directory);
    }
    if (!failures) {
        failures = media_unload(directory);
    }
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)joypad_persistent_shutdown();
    (void)unload_rom();
    frontend_commands_reset();
    frontend_panels_reset();
    frontend_command_set_session_active(false);
    frontend_panel_set_session_active(false);
    FrontendSettings defaults;
    frontend_settings_defaults(&defaults);
    (void)frontend_settings_apply_core(&defaults, NULL, 0);
    printf("Lifecycle recovery/configuration: %s\n", failures ? "FAIL" : "PASS");
    return failures;
}
