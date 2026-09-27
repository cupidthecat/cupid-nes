/*
 * movie_frontend_accuracy.c - Movie controls, audio restoration, and output ownership
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../joypad/joypad.h"
#include "../state/state.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_execution.h"
#include "../ui/frontend_panels.h"
#include "../ui/replay_frontend.h"
#include "../util/file_io.h"
#include "../../include/globals.h"
#include <SDL2/SDL.h>
#include <stdatomic.h>
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#include <windows.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
    ++failures; goto cleanup; \
} } while (0)

typedef struct {
    BoardImage image;
    FrontendExecutionRuntime execution;
    FrontendSettings settings;
    SDL_AudioDeviceID audio;
    bool audio_initialized;
    char rom[256], movie[256], retry[256], saved[256], bad[256];
    unsigned changes, restores;
    bool reject_change;
} MovieFixture;

static void silence_callback(void *context, Uint8 *stream, int length) {
    (void)context;
    memset(stream, 0, (size_t)length);
}

static bool before_change(void *context, char *error, size_t error_size) {
    MovieFixture *fixture = context;
    ++fixture->changes;
    if (error && error_size)
        snprintf(error, error_size, "%s", fixture->reject_change ? "Capture write failed" : "");
    return !fixture->reject_change;
}

static void after_restore(void *context) {
    MovieFixture *fixture = context;
    ++fixture->restores;
}

static bool advance_frame(void) {
    vs_start_frame();
    while (!ppu.frame_complete) if (vs_cpu_step() <= 0 || cpu.halted) return false;
    return true;
}

static bool same_live_state(const NesStateBlob *expected) {
    NesStateBlob actual = {0};
    bool same = nes_state_capture(&actual) == NES_STATE_OK
        && actual.size == expected->size && !memcmp(actual.data, expected->data, actual.size);
    nes_state_blob_free(&actual);
    return same;
}

static bool fixture_create(MovieFixture *fixture, const char *directory, unsigned index) {
    memset(fixture, 0, sizeof(*fixture));
    snprintf(fixture->rom, sizeof(fixture->rom), "%s/game-%u.nes", directory, index);
    snprintf(fixture->movie, sizeof(fixture->movie), "%s/movie-%u.cmv", directory, index);
    snprintf(fixture->retry, sizeof(fixture->retry), "%s/retry-%u.cmv", directory, index);
    snprintf(fixture->saved, sizeof(fixture->saved), "%s/game-%u.sav", directory, index);
    snprintf(fixture->bad, sizeof(fixture->bad), "%s/missing/movie-%u.cmv", directory, index);
    if (!nes_execution_set_policy(NES_EXECUTION_LIVE)
        || !nes_set_ram_power_on_state(NES_RAM_POWER_ZERO)
        || !board_image_create(&fixture->image, 0, 0x8000, 0x2000, true)) return false;
    static const uint8_t program[] = {
        0xA9,0x01,0x8D,0x15,0x40, 0xA9,0xBF,0x8D,0x00,0x40,
        0xA9,0x40,0x8D,0x02,0x40, 0xA9,0x08,0x8D,0x03,0x40,
        0xE6,0x10,0x4C,0x14,0x80
    };
    memcpy(fixture->image.data + 16, program, sizeof(program));
    for (unsigned offset = 0x7FFA; offset <= 0x7FFE; offset += 2) {
        fixture->image.data[16 + offset] = 0;
        fixture->image.data[16 + offset + 1] = 0x80;
    }
    if (nes_file_write_atomic(fixture->rom, fixture->image.data, fixture->image.size) != NES_FILE_OK
        || load_rom(fixture->rom)) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    ppu_begin_frame_render(framebuffer);
    vs_audio_init(44100);
    if (!advance_frame() || !advance_frame() || SDL_AudioInit("dummy")) return false;
    fixture->audio_initialized = true;
    SDL_AudioSpec want = {0}, have = {0};
    want.freq = 44100;
    want.format = AUDIO_F32;
    want.channels = 2;
    want.samples = 256;
    want.callback = silence_callback;
    fixture->audio = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!fixture->audio) return false;
    frontend_execution_init(&fixture->execution, &fixture->audio, have.freq,
                              fixture->rom, NULL, NULL, NULL);
    frontend_settings_defaults(&fixture->settings);
    fixture->settings.movie_preferences.end_behavior = NES_MOVIE_END_STOP;
    fixture->execution.settings = &fixture->settings;
    execution_control_set_paused(&fixture->execution.execution, true);
    fixture->execution.before_machine_change = before_change;
    fixture->execution.machine_change_context = fixture;
    frontend_execution_set_restore_handler(&fixture->execution, after_restore, fixture);
    return frontend_execution_register_commands(&fixture->execution);
}

static void fixture_destroy(MovieFixture *fixture) {
    if (fixture->audio) SDL_PauseAudioDevice(fixture->audio, 1);
    frontend_execution_shutdown(&fixture->execution);
    frontend_commands_reset();
    frontend_panels_reset();
    if (fixture->audio) SDL_CloseAudioDevice(fixture->audio);
    if (fixture->audio_initialized) SDL_AudioQuit();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)unload_rom();
    board_image_free(&fixture->image);
    (void)nes_file_remove(fixture->rom);
    (void)nes_file_remove(fixture->movie);
    (void)nes_file_remove(fixture->retry);
    (void)nes_file_remove(fixture->saved);
}

static bool step_movie(MovieFixture *fixture) {
    return execution_control_request_frame(&fixture->execution.execution)
        && frontend_execution_run_frame(&fixture->execution);
}

static int movie_audio_round_trip(const char *directory, NesMovieStartKind kind) {
    int failures = 0;
    MovieFixture fixture = {0};
    NesStateBlob before = {0}, start = {0}, recorded = {0};
    char error[256] = {0};
    FrontendCommandInfo info;
    CHECK(fixture_create(&fixture, directory, (unsigned)kind));
    CHECK(atomic_load_explicit(&apu.ring_w, memory_order_relaxed) > 0);
    CHECK(apu.audio_transition_count > 0);
    CHECK(frontend_execution_movie_set_path(&fixture.execution, fixture.movie, error, sizeof(error)));
    CHECK(frontend_execution_movie_set_start_kind(&fixture.execution, kind));
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_RECORD, error, sizeof(error)));
    CHECK(nes_state_capture(&start) == NES_STATE_OK);
    CHECK(frontend_command_get(REPLAY_COMMAND_MOVIE_RECORD, &info) && info.checked && !info.enabled);
    CHECK(frontend_command_get(REPLAY_COMMAND_MOVIE_STOP, &info) && info.enabled);
    CHECK(!frontend_command_invoke(REPLAY_COMMAND_MOVIE_PLAY, error, sizeof(error)));
    CHECK(!frontend_command_invoke(FRONTEND_COMMAND_FAST_FORWARD_TOGGLE, error, sizeof(error)));
    CHECK(frontend_execution_set_speeds(&fixture.execution, 1.0, 4.0));
    CHECK(!frontend_execution_set_speeds(&fixture.execution, 2.0, 4.0));
    frontend_execution_set_muted(&fixture.execution, true);
    frontend_execution_set_muted(&fixture.execution, false);
    CHECK(same_live_state(&start));
    CHECK(step_movie(&fixture) && step_movie(&fixture));
    CHECK(nes_state_capture(&recorded) == NES_STATE_OK);
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&before));
    CHECK(SDL_GetAudioDeviceStatus(fixture.audio) == SDL_AUDIO_PAUSED);
    CHECK(frontend_command_get(REPLAY_COMMAND_MOVIE_STOP, &info) && !info.enabled);

    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_PLAY, error, sizeof(error)));
    CHECK(same_live_state(&start));
    CHECK(!frontend_execution_movie_set_path(&fixture.execution, fixture.retry, error, sizeof(error)));
    CHECK(step_movie(&fixture) && step_movie(&fixture));
    CHECK(same_live_state(&recorded));
    CHECK(!step_movie(&fixture));
    CHECK(nes_movie_mode(fixture.execution.movie) == NES_MOVIE_IDLE);
    CHECK(same_live_state(&before));
    CHECK(!fixture.execution.execution.frame_advance_pending);
    CHECK(strstr(fixture.execution.movie_status, "Playback completed") != NULL);
    CHECK(frontend_command_get(REPLAY_COMMAND_MOVIE_PLAY, &info) && !info.checked && info.enabled);
    CHECK(fixture.restores == (kind == NES_MOVIE_START_POWER_ON ? 4u : 3u));
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    CHECK(frontend_panel_snapshot(REPLAY_PANEL, &model, error, sizeof(error)));
    bool open_control = false, save_control = false;
    for (size_t i = 0; i < model.count; ++i) {
        open_control |= controls[i].id == REPLAY_CONTROL_MOVIE_OPEN_FILE && controls[i].enabled;
        save_control |= controls[i].id == REPLAY_CONTROL_MOVIE_SAVE_FILE && controls[i].enabled;
    }
    CHECK(open_control && save_control);
    CHECK(step_movie(&fixture));
    CHECK(frontend_execution_rewind_available(&fixture.execution) == 1);
    CHECK(frontend_execution_rewind_step(&fixture.execution, error, sizeof(error)));
    CHECK(same_live_state(&before));
    CHECK(SDL_GetAudioDeviceStatus(fixture.audio) == SDL_AUDIO_PAUSED);

cleanup:
    nes_state_blob_free(&before);
    nes_state_blob_free(&start);
    nes_state_blob_free(&recorded);
    fixture_destroy(&fixture);
    return failures;
}

static int movie_paths_and_failed_load(const char *directory) {
    int failures = 0;
    MovieFixture fixture = {0};
    NesStateBlob before = {0}, active = {0};
    char error[256] = {0};
    CHECK(fixture_create(&fixture, directory, 2));
    CHECK(step_movie(&fixture));
    size_t history = frontend_execution_rewind_available(&fixture.execution);
    CHECK(history > 0);
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    const uint8_t original[] = {0x21, 0x43, 0x65, 0x87};
    CHECK(nes_file_write_atomic(fixture.saved, original, sizeof(original)) == NES_FILE_OK);
    CHECK(!frontend_execution_movie_set_path(&fixture.execution, fixture.saved, error, sizeof(error)));
    const char *protected_paths[] = {fixture.retry};
    frontend_execution_set_protected_paths(&fixture.execution, protected_paths, 1);
    CHECK(!frontend_execution_movie_set_path(&fixture.execution, fixture.retry, error, sizeof(error)));
    frontend_execution_set_protected_paths(&fixture.execution, NULL, 0);
    strcpy(fixture.execution.movie_path, fixture.rom);
    CHECK(!frontend_command_invoke(REPLAY_COMMAND_MOVIE_RECORD, error, sizeof(error)));
    CHECK(same_live_state(&before) && fixture.changes == 0);
    CHECK(nes_file_write_atomic(fixture.movie, original, sizeof(original)) == NES_FILE_OK);
    CHECK(frontend_execution_movie_set_path(&fixture.execution, fixture.movie, error, sizeof(error)));
    CHECK(!frontend_command_invoke(REPLAY_COMMAND_MOVIE_PLAY, error, sizeof(error)));
    CHECK(same_live_state(&before));
    CHECK(frontend_execution_rewind_available(&fixture.execution) == history);
    CHECK(nes_file_remove(fixture.movie) == NES_FILE_OK);
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_RECORD, error, sizeof(error)));
    CHECK(step_movie(&fixture));
    CHECK(nes_state_capture(&active) == NES_STATE_OK);
    /* A destination that becomes an alias of a save file is rechecked at Stop. */
#ifdef _WIN32
    CHECK(CreateHardLinkA(fixture.movie, fixture.saved, NULL) != 0);
#else
    CHECK(link(fixture.saved, fixture.movie) == 0);
#endif
    CHECK(!frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(nes_movie_mode(fixture.execution.movie) == NES_MOVIE_RECORDING);
    CHECK(same_live_state(&active));
    uint8_t *data = NULL;
    size_t size = 0;
    CHECK(nes_file_read_all(fixture.saved, sizeof(original), &data, &size) == NES_FILE_OK);
    bool preserved = size == sizeof(original) && !memcmp(data, original, size);
    free(data);
    CHECK(preserved);
    CHECK(frontend_execution_movie_set_path(&fixture.execution, fixture.retry, error, sizeof(error)));
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&before));

cleanup:
    nes_state_blob_free(&before);
    nes_state_blob_free(&active);
    fixture_destroy(&fixture);
    return failures;
}

static int movie_partial_frame_and_retry(const char *directory) {
    int failures = 0;
    MovieFixture fixture = {0};
    NesStateBlob before = {0}, completed = {0}, partial = {0};
    char error[256] = {0};
    CHECK(fixture_create(&fixture, directory, 3));
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    CHECK(frontend_execution_movie_set_path(&fixture.execution, fixture.bad, error, sizeof(error)));
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_RECORD, error, sizeof(error)));
    CHECK(step_movie(&fixture));
    CHECK(nes_state_capture(&completed) == NES_STATE_OK);
    CHECK(joypad_set_player(0, BTN_A, true));
    CHECK(nes_movie_frame_boundary(fixture.execution.movie) == NES_MOVIE_OK);
    vs_start_frame();
    for (unsigned i = 0; i < 10; ++i) CHECK(vs_cpu_step() > 0);
    CHECK(nes_movie_frame_complete(fixture.execution.movie, false) == NES_MOVIE_OK);
    CHECK(nes_state_capture(&partial) == NES_STATE_OK);
    NesMovieProgress progress;
    frontend_execution_movie_progress(&fixture.execution, &progress);
    size_t events = progress.event_count;
    CHECK(progress.frame == 1 && events > 0);
    CHECK(!frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&partial));
    frontend_execution_movie_progress(&fixture.execution, &progress);
    CHECK(progress.mode == NES_MOVIE_RECORDING && progress.event_count == events);
    CHECK(frontend_execution_movie_set_path(&fixture.execution, fixture.retry, error, sizeof(error)));
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&before));

    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_PLAY, error, sizeof(error)));
    frontend_execution_movie_progress(&fixture.execution, &progress);
    CHECK(progress.total_frames == 1 && progress.event_count == 0);
    CHECK(step_movie(&fixture) && same_live_state(&completed));
    fixture.reject_change = true;
    CHECK(!step_movie(&fixture));
    CHECK(nes_movie_mode(fixture.execution.movie) == NES_MOVIE_PLAYBACK);
    CHECK(same_live_state(&completed));
    CHECK(frontend_execution_paused(&fixture.execution)
        && !fixture.execution.execution.frame_advance_pending);
    CHECK(strstr(fixture.execution.movie_status, "Capture write failed") != NULL);
    fixture.reject_change = false;
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&before));

    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_PLAY, error, sizeof(error)));
    CHECK(nes_movie_frame_boundary(fixture.execution.movie) == NES_MOVIE_OK);
    vs_start_frame();
    for (unsigned i = 0; i < 10; ++i) CHECK(vs_cpu_step() > 0);
    CHECK(frontend_command_invoke(REPLAY_COMMAND_MOVIE_STOP, error, sizeof(error)));
    CHECK(same_live_state(&before));

cleanup:
    nes_state_blob_free(&before);
    nes_state_blob_free(&completed);
    nes_state_blob_free(&partial);
    fixture_destroy(&fixture);
    return failures;
}

int test_movie_frontend_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    snprintf(directory, sizeof(directory), "build/movie-ui-%u", (unsigned)_getpid());
    if (_mkdir(directory)) return 1;
#else
    snprintf(directory, sizeof(directory), "build/movie-ui-%u", (unsigned)getpid());
    if (mkdir(directory, 0700)) return 1;
#endif
    uint32_t previous_policy = nes_execution_policy();
    NesRamPowerOnState previous_ram = nes_ram_power_on_state();
    int failures = movie_audio_round_trip(directory, NES_MOVIE_START_STATE)
        + movie_audio_round_trip(directory, NES_MOVIE_START_POWER_ON)
        + movie_paths_and_failed_load(directory)
        + movie_partial_frame_and_retry(directory);
    (void)nes_set_ram_power_on_state(previous_ram);
    (void)nes_execution_set_policy(previous_policy);
#ifdef _WIN32
    if (_rmdir(directory)) ++failures;
#else
    if (rmdir(directory)) ++failures;
#endif
    printf("Desktop input movies: 4 groups, %d failures\n", failures);
    return failures;
}
