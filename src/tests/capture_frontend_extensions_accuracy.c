/* capture_frontend_extensions_accuracy.c - Capture panels, settings and authoritative frames
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/capture_runtime.h"
#include "../ui/capture_tools.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../capture/capture_riff.h"
#include "../replay/tas_session.h"
#include "../replay/tas_project.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../debugger/debugger.h"
#include "../../include/globals.h"
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#define mkdir_test(path) _mkdir(path)
#define rmdir_test(path) _rmdir(path)
#define process_id _getpid
#else
#include <sys/stat.h>
#include <unistd.h>
#define mkdir_test(path) mkdir(path, 0700)
#define rmdir_test(path) rmdir(path)
#define process_id getpid
#endif

#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            failed = 1;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

static int preferences_persistence(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/settings.ini", directory);
    FrontendSettings saved, loaded;
    FrontendSettingsReport report;
    frontend_settings_defaults(&saved);
    NesMoviePreferences *p = &saved.movie_preferences;
    p->end_behavior = NES_MOVIE_END_STOP;
    p->read_only = false;
    p->record_insert = true;
    p->record_power_on = true;
    p->subtitles = false;
    p->subtitle_duration = 321;
    p->display_overlays = 13;
    p->capture_overlays = 15;
    p->position = NES_OVERLAY_BOTTOM_RIGHT;
    p->backup.enabled = false;
    p->backup.retained = 7;
    strcpy(p->backup.directory, "movie recovery/versions");
    saved.capture.codec = NES_CAPTURE_CODEC_ZMBV;
    saved.capture.compression_level = 9;
    saved.capture.format = NES_CAPTURE_FORMAT_GIF;
    saved.capture.gif_scale = 3;
    CHECK(frontend_settings_save(path, &saved, &report));
    CHECK(frontend_settings_load(path, &loaded, &report) && report.found && !report.unknown_settings);
    for (unsigned i = 0; i < NES_MOVIE_PREFERENCE_COUNT; ++i) {
        char left[1024], right[1024];
        const char *key = nes_movie_preferences_key(i);
        CHECK(nes_movie_preferences_get(p, key, left, sizeof(left)));
        CHECK(nes_movie_preferences_get(&loaded.movie_preferences, key, right, sizeof(right)) && !strcmp(left, right));
    }
    CHECK(loaded.capture.codec == NES_CAPTURE_CODEC_ZMBV && loaded.capture.compression_level == 9 &&
          loaded.capture.format == NES_CAPTURE_FORMAT_GIF && loaded.capture.gif_scale == 3);
    loaded.movie_preferences.backup.retained = 0;
    CHECK(!frontend_settings_save(path, &loaded, &report));
    CHECK(frontend_settings_load(path, &loaded, &report) && loaded.movie_preferences.backup.retained == 7);
    const char invalid[] = "version=4\nmovie.backup_count=0\n";
    CHECK(nes_file_write_atomic(path, invalid, sizeof(invalid) - 1) == NES_FILE_OK);
    CHECK(!frontend_settings_load(path, &loaded, &report));
    CHECK(loaded.movie_preferences.backup.retained == 7);
cleanup:
    (void)nes_file_remove(path);
    return failed;
}

static bool fixture(NesRegion region) {
    size_t size = 16 + 32768 + 8192;
    uint8_t *image = calloc(1, size);
    if (!image) {
        return false;
    }
    memcpy(image, "NES\x1A", 4);
    image[4] = 2;
    image[5] = 1;
    image[7] = 8;
    image[12] = region == NES_REGION_PAL ? 1 : region == NES_REGION_DENDY ? 3 : 0;
    /* Strobe and consume P1 on each iteration, then return to the strobe. */
    const uint8_t program[] = {0xA9, 1,    0x8D, 0x16, 0x40, 0xA9, 0,    0x8D, 0x16,
                               0x40, 0xAD, 0x16, 0x40, 0x85, 0,    0x4C, 0,    0x80};
    memcpy(image + 16, program, sizeof(program));
    for (unsigned i = 0x7FFA; i <= 0x7FFE; i += 2) {
        image[16 + i + 1] = 0x80;
    }
    bool ok = nes_set_region_mode(NES_REGION_MODE_AUTO) && load_rom_memory(image, size) == 0;
    free(image);
    if (!ok) {
        return false;
    }
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    apu_audio_init(44100);
    debugger_init();
    return cpu_power_on(&cpu);
}

static bool advance(NesCaptureRuntime *capture, FrontendExecutionRuntime *execution) {
    nes_capture_frontend_begin_frame(&capture->frontend);
    bool completed = frontend_execution_run_frame(execution);
    nes_capture_frontend_end_frame(&capture->frontend, completed);
    return completed;
}

static int recording_paths(const char *directory, NesRegion region) {
    int failed = 0;
    FrontendExecutionRuntime execution = {0};
    NesCaptureRuntime capture = {0};
    bool composite = false;
    char avi[256], gif[256], error[256] = {0};
    snprintf(avi, sizeof(avi), "%s/region-%u.avi", directory, (unsigned)region);
    snprintf(gif, sizeof(gif), "%s/region-%u.gif", directory, (unsigned)region);
    NesRiffReport report = {0};
    uint8_t *data = NULL;
    size_t size = 0;
    frontend_commands_reset();
    frontend_panels_reset();
    CHECK(fixture(region));
    frontend_execution_init(&execution, NULL, 44100, "capture-fixture.nes", NULL, NULL, NULL);
    CHECK(frontend_execution_register_commands(&execution));
    frontend_command_set_session_active(true);
    frontend_panel_set_session_active(true);
    CHECK(nes_capture_runtime_init(&capture, &execution, &composite, NULL, NULL, 0));
    FrontendPanelControl controls[32];
    FrontendPanelModel model = {.controls = controls, .capacity = 32};
    CHECK(frontend_panel_snapshot(CAPTURE_MOVIE_PANEL, &model, error, sizeof(error)) && model.count == 17);
    CHECK(frontend_panel_action(CAPTURE_MOVIE_PANEL, 20, NULL, 0, error, sizeof(error)));
    CHECK(capture.frontend.preferences.display_overlays == NES_OVERLAY_INPUT &&
          !capture.frontend.preferences.capture_overlays);
    CHECK(frontend_panel_action(CAPTURE_ENCODING_PANEL, 2, NULL, 1, error, sizeof(error)));
    CHECK(!frontend_panel_action(CAPTURE_ENCODING_PANEL, 3, "10", 0, error, sizeof(error)));
    CHECK(nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_AVI, avi, error, sizeof(error)));
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_VIDEO, error, sizeof(error)));
    CHECK(joypad_set_player(0, BTN_A, true));
    CHECK(advance(&capture, &execution));
    CHECK(capture.frontend.overlay_state.pads[0] == 1);
    CHECK(frontend_execution_set_speeds(&execution, 0.5, 4));
    CHECK(advance(&capture, &execution));
    CHECK(frontend_execution_set_speeds(&execution, 2, 4));
    CHECK(advance(&capture, &execution));
    execution_control_set_paused(&execution.execution, true);
    CHECK(joypad_set_player(0, BTN_A, false));
    CHECK(!advance(&capture, &execution));
    CHECK(capture.frontend.overlay_state.pads[0] == 1 && capture.frontend.session.info.completed_frames == 3);
    CHECK(execution_control_request_frame(&execution.execution));
    CHECK(advance(&capture, &execution));
    CHECK(!capture.frontend.overlay_state.pads[0] && capture.frontend.session.info.completed_frames == 4);
    NesCaptureFrameRate rate = capture.frontend.session.info.frame_rate;
    CHECK(capture.frontend.session.info.audio_frames == (uint64_t)4 * 44100 * rate.denominator / rate.numerator);
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_STOP, error, sizeof(error)));
    CHECK(nes_capture_riff_inspect(avi, &report) == NES_FILE_OK && !report.warnings && report.video_frames == 4);
    CHECK(frontend_panel_action(CAPTURE_RIFF_PANEL, 1, avi, 0, error, sizeof(error)));
    CHECK(frontend_panel_action(CAPTURE_RIFF_PANEL, 2, NULL, 0, error, sizeof(error)));
    CHECK(frontend_panel_snapshot(CAPTURE_RIFF_PANEL, &model, error, sizeof(error)));
    CHECK(frontend_panel_action(CAPTURE_ENCODING_PANEL, 5, gif, 0, error, sizeof(error)));
    CHECK(frontend_panel_action(CAPTURE_ENCODING_PANEL, 4, NULL, 1, error, sizeof(error)));
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_GIF, error, sizeof(error)));
    CHECK(execution_control_request_frame(&execution.execution));
    CHECK(advance(&capture, &execution));
    CHECK(!advance(&capture, &execution) && capture.frontend.session.info.completed_frames == 1);
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_STOP, error, sizeof(error)));
    CHECK(nes_file_read_all(gif, 1000000, &data, &size) == NES_FILE_OK && size > 800);
    CHECK(!memcmp(data, "GIF89a\0\2\xE0\1", 10) && data[size - 1] == ';');
cleanup:
    if (failed && error[0]) {
        fprintf(stderr, "Capture frontend: %s\n", error);
    }
    free(data);
    nes_capture_riff_free(&report);
    (void)nes_capture_runtime_shutdown(&capture);
    frontend_execution_shutdown(&execution);
    frontend_commands_reset();
    frontend_panels_reset();
    (void)nes_file_remove(avi);
    (void)nes_file_remove(gif);
    (void)unload_rom();
    return failed;
}

static int movie_overlay_paths(const char *directory) {
    int failed = 0;
    FrontendExecutionRuntime execution = {0};
    NesCaptureRuntime capture = {0};
    bool composite = false;
    char path[256], native[256], error[256] = {0};
    snprintf(path, sizeof(path), "%s/input.fm2", directory);
    snprintf(native, sizeof(native), "%s/input.cmv", directory);
    NesFm2Movie movie;
    nes_fm2_movie_init(&movie);
    uint8_t *data = NULL;
    size_t size = 0;
    NesFm2Diagnostic diagnostic;
    CHECK(fixture(NES_REGION_NTSC));
    frontend_execution_init(&execution, NULL, 44100, "capture-fixture.nes", NULL, NULL, NULL);
    CHECK(nes_capture_runtime_init(&capture, &execution, &composite, NULL, NULL, 0));
    NesTasSession *tas = nes_movie_tas(execution.movie);
    CHECK(nes_tas_session_new(tas, path, true) == NES_MOVIE_OK);
    (void)joypad_set_player(0, BTN_A, true);
    CHECK(advance(&capture, &execution) && capture.frontend.overlay_state.pads[0] == 1);
    (void)joypad_set_player(0, BTN_A, false);
    (void)joypad_set_player(0, BTN_B, true);
    CHECK(advance(&capture, &execution) && capture.frontend.overlay_state.pads[0] == 2);
    CHECK(nes_tas_session_stop(tas) == NES_MOVIE_OK);
    CHECK(nes_file_read_all(path, 1000000, &data, &size) == NES_FILE_OK);
    CHECK(nes_fm2_parse(data, size, NULL, &movie, &diagnostic) == NES_FM2_OK);
    free(data);
    data = NULL;
    movie.subtitles.items = calloc(2, sizeof(char *));
    CHECK(movie.subtitles.items);
    movie.subtitles.count = 2;
    movie.subtitles.items[0] = malloc(16);
    movie.subtitles.items[1] = malloc(16);
    CHECK(movie.subtitles.items[0] && movie.subtitles.items[1]);
    strcpy(movie.subtitles.items[0], "0 First");
    strcpy(movie.subtitles.items[1], "1 Second");
    CHECK(nes_fm2_measure(&movie, NES_FM2_TEXT, &size, &diagnostic) == NES_FM2_OK);
    data = malloc(size);
    CHECK(data);
    CHECK(nes_fm2_export(&movie, NES_FM2_TEXT, data, size, &size, &diagnostic) == NES_FM2_OK);
    CHECK(nes_movie_save_atomic(path, data, size) == NES_FILE_OK);
    CHECK(nes_tas_session_open(tas, path) == NES_MOVIE_OK);
    /* Host B events are rejected by movie ownership; recorded A reaches P1. */
    CHECK(!joypad_set_player(0, BTN_B, true));
    CHECK(advance(&capture, &execution));
    CHECK(capture.frontend.overlay_state.pads[0] == 1 && !strcmp(capture.frontend.overlay_state.subtitle, "First"));
    CHECK(advance(&capture, &execution));
    CHECK(capture.frontend.overlay_state.pads[0] == 2 && !strcmp(capture.frontend.overlay_state.subtitle, "Second"));
    CHECK(nes_tas_session_seek(tas, 1) == NES_MOVIE_OK);
    while (nes_tas_session_seeking(tas)) {
        CHECK(advance(&capture, &execution));
    }
    nes_capture_frontend_end_frame(&capture.frontend, false);
    CHECK(!strcmp(capture.frontend.overlay_state.subtitle, "First"));
    NesMoviePreferences next = capture.frontend.preferences;
    next.read_only = false;
    next.record_insert = true;
    next.subtitles = false;
    next.capture_overlays = 15;
    CHECK(nes_capture_runtime_preferences(&capture, &next, error, sizeof(error)));
    NesTasProgress progress;
    nes_tas_session_progress(tas, &progress);
    CHECK(!progress.read_only && progress.record_mode == NES_TAS_RECORD_INSERT);
    next.end_behavior = NES_MOVIE_END_PAUSE;
    CHECK(nes_capture_movie_end(&execution, &next, error, sizeof(error)) && execution.execution.paused);
    CHECK(nes_tas_session_active(tas));
    next.end_behavior = NES_MOVIE_END_STOP;
    CHECK(nes_capture_movie_end(&execution, &next, error, sizeof(error)) && !nes_tas_session_active(tas));
    free(data);
    data = NULL;
    CHECK(nes_file_read_all(path, 1000000, &data, &size) == NES_FILE_OK);
    CHECK(nes_fm2_parse(data, size, NULL, &movie, &diagnostic) == NES_FM2_OK);
    CHECK(movie.subtitles.count == 2 && !strcmp(movie.subtitles.items[1], "1 Second"));
    /* Native movies obey the same Pause default at the production boundary. */
    execution_control_set_paused(&execution.execution, false);
    CHECK(nes_movie_record_start(execution.movie, native) == NES_MOVIE_OK);
    CHECK(advance(&capture, &execution));
    CHECK(nes_movie_stop(execution.movie) == NES_MOVIE_OK);
    CHECK(nes_movie_play_start(execution.movie, native) == NES_MOVIE_OK);
    CHECK(advance(&capture, &execution));
    uint64_t final_cycles = cpu_total_cycles;
    CHECK(!advance(&capture, &execution));
    CHECK(nes_movie_mode(execution.movie) == NES_MOVIE_PLAYBACK && execution.execution.paused);
    CHECK(cpu_total_cycles == final_cycles);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.movie_preferences.end_behavior = NES_MOVIE_END_STOP;
    execution.settings = &settings;
    CHECK(execution_control_request_frame(&execution.execution));
    CHECK(!advance(&capture, &execution));
    CHECK(nes_movie_mode(execution.movie) == NES_MOVIE_IDLE);
    execution.settings = NULL;

cleanup:
    free(data);
    nes_fm2_movie_free(&movie);
    (void)nes_capture_runtime_shutdown(&capture);
    frontend_execution_shutdown(&execution);
    frontend_commands_reset();
    frontend_panels_reset();
    (void)nes_file_remove(path);
    (void)nes_file_remove(native);
    NesMovieBackupOptions options;
    nes_movie_backup_defaults(&options);
    nes_movie_backup_set_policy(&options);
    for (unsigned i = 1; i <= 5; ++i) {
        char name[320];
        if (nes_movie_backup_path(path, &options, i, name, sizeof(name)) == NES_FILE_OK) {
            (void)nes_file_remove(name);
        }
    }
    (void)unload_rom();
    return failed;
}

int test_capture_frontend_extensions_accuracy(void) {
    char directory[128];
    snprintf(directory, sizeof(directory), "capture-frontend-%ld", (long)process_id());
    if (mkdir_test(directory)) {
        return 1;
    }
    NesRegionMode region = nes_region_mode();
    NesRamPowerOnState ram = nes_ram_power_on_state();
    uint32_t policy = nes_execution_policy();
    bool restriction = ppu_startup_write_restriction_enabled();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)nes_set_ram_power_on_state(NES_RAM_POWER_ZERO);
    ppu_set_startup_write_restriction(false);
    int failed = preferences_persistence(directory) + recording_paths(directory, NES_REGION_NTSC) +
                 recording_paths(directory, NES_REGION_PAL) + recording_paths(directory, NES_REGION_DENDY) +
                 movie_overlay_paths(directory);
    (void)nes_set_region_mode(region);
    (void)nes_set_ram_power_on_state(ram);
    (void)nes_execution_set_policy(policy);
    ppu_set_startup_write_restriction(restriction);
    if (rmdir_test(directory)) {
        ++failed;
    }
    printf("Capture frontend extensions: %s\n", failed ? "FAIL" : "PASS");
    return failed;
}
