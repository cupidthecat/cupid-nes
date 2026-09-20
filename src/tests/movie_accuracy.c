/*
 * movie_accuracy.c - Deterministic input movie acceptance regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or later.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../debugger/debugger.h"
#include "../debugger/lua_runtime.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../replay/movie.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../ui/frontend_execution.h"
#include "../ui/machine_actions.h"
#include "../util/file_io.h"
#include "../../include/globals.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t ram[0x800];
extern uint64_t cpu_total_cycles;

#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); return 1; } } while (0)

static bool blobs_equal(const NesStateBlob *left, const NesStateBlob *right) {
    return left && right && left->size == right->size
        && (!left->size || memcmp(left->data, right->data, left->size) == 0);
}

static bool make_movie_fixture(BoardImage *image) {
    if (!board_image_create(image, 2, 0x8000, 0x2000, false)) return false;
    uint8_t *prg = image->data + sizeof(iNESHeader);
    uint8_t *fixed = prg + 0x4000;
    static const uint8_t program[] = {
        0x78,                   /* SEI */
        0xA9,0x80,             /* LDA #$80 */
        0x8D,0x00,0x20,        /* STA $2000 */
        0xA9,0x01,
        0x8D,0x16,0x40,        /* strobe high */
        0xA9,0x00,
        0x8D,0x16,0x40,        /* strobe low */
        0xAD,0x16,0x40,
        0x85,0x00,             /* sampled controller */
        0xE6,0x04,             /* visible changing palette */
        0xA9,0x3F,
        0x8D,0x06,0x20,
        0xA9,0x00,
        0x8D,0x06,0x20,
        0xA5,0x04,
        0x8D,0x07,0x20,
        0x4C,0x06,0xC0
    };
    memcpy(fixed, program, sizeof(program));
    fixed[0x100] = 0xE6; fixed[0x101] = 0x02; fixed[0x102] = 0x40;
    prg[0x7FFA] = 0x00; prg[0x7FFB] = 0xC1;
    prg[0x7FFC] = 0x00; prg[0x7FFD] = 0xC0;
    prg[0x7FFE] = 0x00; prg[0x7FFF] = 0xC1;
    return true;
}

static bool power_machine(void) {
    memset(ram, 0, sizeof(ram));
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_power_on_secondary();
    ppu_begin_frame_render(framebuffer);
    return true;
}

static bool run_frame(void) {
    vs_start_frame();
    while (!ppu.frame_complete) {
        if (vs_cpu_step() <= 0 || cpu.halted) return false;
    }
    return true;
}

static void movie_actions(unsigned frame) {
    (void)joypad_set_player(0, BTN_A, (frame & 1u) != 0);
    (void)joypad_set_player(1, BTN_RIGHT, (frame & 2u) != 0);
    if (frame == 1) (void)frontend_machine_soft_reset();
}

static bool run_movie_record_frame(NesMovieSession *movie, unsigned frame) {
    movie_actions(frame);
    return nes_movie_frame_boundary(movie) == NES_MOVIE_OK
        && run_frame()
        && nes_movie_frame_complete(movie, true) == NES_MOVIE_OK;
}

static bool run_movie_play_frame(NesMovieSession *movie) {
    return nes_movie_frame_boundary(movie) == NES_MOVIE_OK
        && run_frame()
        && nes_movie_frame_complete(movie, true) == NES_MOVIE_OK;
}

static int test_state_start_round_trip(void) {
    const char *path = "build/movie-state-start.cmv";
    (void)remove(path);
    BoardImage image = {0};
    CHECK(make_movie_fixture(&image) && board_image_load(&image) == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    cpu_use_default_startup_alignment();
    CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    CHECK(power_machine());
    vs_audio_init(48000);

    NesStateBlob live_before = {0}, expected = {0}, restored = {0}, replayed = {0};
    CHECK(nes_state_capture(&live_before) == NES_STATE_OK);
    NesMovieSession *movie = nes_movie_create();
    CHECK(movie && nes_movie_record_start(movie, path) == NES_MOVIE_OK);
    CHECK(!nes_execution_allows_persistence());
    for (unsigned frame = 0; frame < 4; ++frame) CHECK(run_movie_record_frame(movie, frame));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    uint32_t expected_video[SCREEN_WIDTH * SCREEN_HEIGHT];
    memcpy(expected_video, framebuffer, sizeof(expected_video));
    float expected_audio[128];
    apu_audio_pull(&apu, expected_audio, 128);
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
    CHECK(nes_state_capture(&restored) == NES_STATE_OK && blobs_equal(&live_before, &restored));

    CHECK(nes_movie_play_start(movie, path) == NES_MOVIE_OK);
    for (unsigned frame = 0; frame < 4; ++frame) CHECK(run_movie_play_frame(movie));
    CHECK(nes_movie_frame_boundary(movie) == NES_MOVIE_COMPLETE);
    CHECK(nes_state_capture(&replayed) == NES_STATE_OK && blobs_equal(&expected, &replayed));
    CHECK(memcmp(expected_video, framebuffer, sizeof(expected_video)) == 0);
    float replayed_audio[128];
    apu_audio_pull(&apu, replayed_audio, 128);
    CHECK(memcmp(expected_audio, replayed_audio, sizeof(expected_audio)) == 0);
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);

    nes_movie_destroy(movie);
    nes_state_blob_free(&replayed);
    nes_state_blob_free(&restored);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&live_before);
    CHECK(unload_rom());
    board_image_free(&image);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_power_on_start_and_guards(void) {
    const char *path = "build/movie-power-start.cmv";
    (void)remove(path);
    BoardImage image = {0};
    CHECK(make_movie_fixture(&image) && board_image_load(&image) == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_RANDOM));
    nes_seed_power_on_random(0x12345678u);
    cpu_seed_startup_alignment(0xABCDEF01u);
    CHECK(power_machine());
    vs_audio_init(48000);
    ram[7] = 0xA5;

    NesStateBlob live_before = {0}, expected = {0}, restored = {0}, replayed = {0};
    CHECK(nes_state_capture(&live_before) == NES_STATE_OK);
    NesMovieSession *movie = nes_movie_create();
    CHECK(movie && nes_movie_record_start_power_on(movie, path) == NES_MOVIE_OK);
    CHECK(ram[7] != 0xA5);
    CHECK(!nes_set_console_model(NES_CONSOLE_HVC001));
    CHECK(!nes_set_region_mode(NES_REGION_MODE_PAL));
    CHECK(!ppu_set_revision(PPU_REVISION_2C02_PRE_E));
    CHECK(!apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03));
    CHECK(!cpu_set_startup_alignment(1, 1));
    CHECK(!debugger_lua_load("blocked", "return true"));
    CHECK(load_rom_memory(image.data, image.size) != 0);
    for (unsigned frame = 0; frame < 3; ++frame) CHECK(run_movie_record_frame(movie, frame));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);
    CHECK(nes_state_capture(&restored) == NES_STATE_OK && blobs_equal(&live_before, &restored));

    CHECK(nes_movie_play_start(movie, path) == NES_MOVIE_OK);
    for (unsigned frame = 0; frame < 3; ++frame) CHECK(run_movie_play_frame(movie));
    CHECK(nes_state_capture(&replayed) == NES_STATE_OK && blobs_equal(&expected, &replayed));
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);

    nes_movie_destroy(movie);
    nes_state_blob_free(&replayed);
    nes_state_blob_free(&restored);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&live_before);
    CHECK(unload_rom());
    board_image_free(&image);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    return 0;
}

static int test_transactional_rejection_and_retry(void) {
    const char *path = "build/movie-transactional.cmv";
    const char *retry = "build/movie-transactional-retry.cmv";
    (void)remove(path);
    (void)remove(retry);
    BoardImage image = {0};
    CHECK(make_movie_fixture(&image) && board_image_load(&image) == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    cpu_use_default_startup_alignment();
    CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    CHECK(power_machine());

    NesMovieSession *movie = nes_movie_create();
    CHECK(movie && nes_movie_record_start(movie, path) == NES_MOVIE_OK);
    CHECK(run_movie_record_frame(movie, 0));
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);
    NesStateBlob before = {0}, after = {0};
    CHECK(nes_state_capture(&before) == NES_STATE_OK);

    uint8_t *file = NULL;
    size_t file_size = 0;
    CHECK(nes_file_read_all(path, 8u * 1024u * 1024u, &file, &file_size) == NES_FILE_OK);
    CHECK(file_size > 32);
    file[file_size / 2] ^= 0x80;
    CHECK(nes_file_write_atomic(retry, file, file_size) == NES_FILE_OK);
    free(file);
    CHECK(nes_movie_play_start(movie, retry) == NES_MOVIE_CORRUPT);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
    CHECK(nes_state_capture(&after) == NES_STATE_OK && blobs_equal(&before, &after));
    nes_state_blob_free(&after);

    PpuRevision revision = ppu_revision();
    CHECK(ppu_set_revision(revision == PPU_REVISION_2C02_PRE_E
                           ? PPU_REVISION_2C02_E_PLUS : PPU_REVISION_2C02_PRE_E));
    CHECK(nes_movie_play_start(movie, path) == NES_MOVIE_INCOMPATIBLE);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
    CHECK(ppu_set_revision(revision));
    CHECK(nes_state_capture(&after) == NES_STATE_OK && blobs_equal(&before, &after));
    nes_state_blob_free(&after);

    CHECK(nes_movie_record_start(movie, ".") == NES_MOVIE_OK);
    CHECK(run_movie_record_frame(movie, 0));
    CHECK(nes_movie_stop(movie) == NES_MOVIE_IO_ERROR);
    CHECK(nes_movie_mode(movie) == NES_MOVIE_RECORDING);
    CHECK(nes_movie_set_path(movie, retry) == NES_MOVIE_OK);
    CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);

    nes_movie_destroy(movie);
    nes_state_blob_free(&before);
    CHECK(unload_rom());
    board_image_free(&image);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    CHECK(nes_file_remove(retry) == NES_FILE_OK);
    return 0;
}

static int test_frontend_frame_ownership_and_path_guard(void) {
    const char *rom_path = "build/movie-active-image.nes";
    const char *movie_path = "build/movie-frontend.cmv";
    (void)remove(rom_path);
    (void)remove(movie_path);
    BoardImage image = {0};
    CHECK(make_movie_fixture(&image));
    CHECK(nes_file_write_atomic(rom_path, image.data, image.size) == NES_FILE_OK);
    CHECK(load_rom(rom_path) == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE) && power_machine());

    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 0, rom_path, NULL, NULL, NULL);
    char error[160] = {0};
    CHECK(!frontend_execution_movie_set_path(&runtime, rom_path, error, sizeof(error)));
    CHECK(frontend_execution_movie_set_path(&runtime, movie_path, error, sizeof(error)));
    CHECK(frontend_execution_movie_record(&runtime, error, sizeof(error)));
    CHECK(joypad_set_player(0, BTN_A, true));
    CHECK(nes_movie_frame_boundary(runtime.movie) == NES_MOVIE_OK);
    NesMovieProgress progress;
    frontend_execution_movie_progress(&runtime, &progress);
    CHECK(progress.frame == 0);
    CHECK(nes_movie_frame_complete(runtime.movie, false) == NES_MOVIE_OK);
    CHECK(nes_movie_frame_boundary(runtime.movie) == NES_MOVIE_OK);
    frontend_execution_movie_progress(&runtime, &progress);
    CHECK(progress.frame == 0);
    CHECK(nes_movie_frame_complete(runtime.movie, true) == NES_MOVIE_OK);
    frontend_execution_movie_progress(&runtime, &progress);
    CHECK(progress.frame == 1);
    CHECK(frontend_execution_movie_stop(&runtime, error, sizeof(error)));

    frontend_execution_shutdown(&runtime);
    CHECK(unload_rom());
    board_image_free(&image);
    CHECK(nes_file_remove(movie_path) == NES_FILE_OK);
    CHECK(nes_file_remove(rom_path) == NES_FILE_OK);
    return 0;
}

int test_movie_accuracy(void) {
    uint32_t previous_policy = nes_execution_policy();
    NesRamPowerOnState previous_ram = nes_ram_power_on_state();
    NesRegionMode previous_region = nes_region_mode();
    PpuRevision previous_ppu = ppu_revision();
    ApuCpuRevision previous_apu = apu_get_cpu_revision();
    int failures = test_state_start_round_trip()
                 + test_power_on_start_and_guards()
                 + test_transactional_rejection_and_retry()
                 + test_frontend_frame_ownership_and_path_guard();
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    (void)nes_set_ram_power_on_state(previous_ram);
    (void)nes_set_region_mode(previous_region);
    (void)ppu_set_revision(previous_ppu);
    (void)apu_set_cpu_revision(previous_apu);
    if (!nes_execution_set_policy(previous_policy)) ++failures;
    printf("Input movies: 4 groups, %d failures\n", failures);
    return failures;
}
