/*
 * history_media_accuracy.c - History input replay, media bounds and navigation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "board_tests.h"
#include "../replay/rewind.h"
#include "../replay/movie.h"
#include "../ui/history_frontend.h"
#include "../ui/frontend_panels.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../apu/apu.h"
#include "../debugger/debugger.h"
#include "../../include/globals.h"
#include <string.h>

extern uint8_t ram[0x800];
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "History media %d: %s\n", __LINE__, #x); return 1; } } while (0)

static bool run_frame(void) {
    vs_start_frame();
    while (!ppu.frame_complete) if (vs_cpu_step() <= 0 || cpu.halted) return false;
    return true;
}

static bool reject(const NesInputEvent *event, void *context) { (void)event; (void)context; return false; }

int run_history_media_accuracy_tests(void) {
    BoardImage image = {0};
    CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    const uint8_t code[] = {0xA9,1,0x8D,0x16,0x40,0xA9,0,0x8D,0x16,0x40,
        0xAD,0x16,0x40,0x85,7,0xE6,8,0x4C,0,0x80};
    memcpy(image.data + 16, code, sizeof(code));
    image.data[16 + 0x7ffc] = 0;
    image.data[16 + 0x7ffd] = 0x80;
    apu_power_on(&apu);
    CHECK(board_image_load(&image) == 0);
    board_image_free(&image);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    debugger_init();
    cpu_set_test_mode(false);
    cpu_use_default_startup_alignment();
    cpu_total_cycles = 0;
    memset(ram, 0, 0x800);
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    vs_power_on_secondary();
    ppu_begin_frame_render(framebuffer);
    vs_audio_init(48000);
    NesRewindHistory ring;
    nes_rewind_init(&ring);
    CHECK(nes_rewind_configure(&ring, 5, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    nes_rewind_track_inputs(&ring);
    NesStateResult state;
    for (unsigned i = 0; i < 7; ++i) {
        CHECK(joypad_set_player(0, BTN_A, i % 2));
        CHECK(nes_rewind_capture(&ring, &state) == NES_REPLAY_OK);
        CHECK(run_frame());
        nes_rewind_complete(&ring, true);
        float pcm[] = {.25f, -.5f, .125f, -.25f};
        nes_rewind_append_audio(&ring, pcm, 2, 48000);
    }
    CHECK(ring.count == 5 && ring.head != 0);
    size_t bytes = 0;
    for (size_t i = 0; i < ring.count; ++i) {
        const NesRewindMedia *media = nes_rewind_media(&ring, i);
        CHECK(media && media->complete && media->inputs_valid && media->event_count == 1);
        CHECK(media->audio_frames == 2 && media->audio_rate == 48000 && media->audio[1] == -.5f);
        bytes += ring.entries[(ring.head + i) % ring.capacity].size + sizeof(NesInputEvent) + 4 * sizeof(float);
    }
    CHECK(bytes == ring.total_bytes && bytes <= ring.max_bytes);
    NesStateBlob expected = {0}, after_export = {0}, replayed = {0};
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    const char *path = "build/history-range.movie";
    for (size_t first = 0; first <= 2; first += 2) {
        CHECK(nes_movie_export_history(&ring, first, 5, path) == NES_MOVIE_OK);
        CHECK(nes_state_capture(&after_export) == NES_STATE_OK);
        CHECK(expected.size == after_export.size && !memcmp(expected.data, after_export.data, expected.size));
        NesMovieSession *movie = nes_movie_create();
        CHECK(movie && nes_movie_play_start(movie, path) == NES_MOVIE_OK);
        for (size_t i = first; i < 5; ++i) {
            CHECK(nes_movie_frame_boundary(movie) == NES_MOVIE_OK && run_frame());
            CHECK(nes_movie_frame_complete(movie, true) == NES_MOVIE_OK);
        }
        CHECK(nes_movie_frame_boundary(movie) == NES_MOVIE_COMPLETE);
        CHECK(nes_state_capture(&replayed) == NES_STATE_OK);
        CHECK(expected.size == replayed.size && !memcmp(expected.data, replayed.data, expected.size));
        CHECK(nes_movie_stop(movie) == NES_MOVIE_OK);
        nes_movie_destroy(movie);
        nes_state_blob_free(&after_export);
        nes_state_blob_free(&replayed);
    }
    nes_state_blob_free(&expected);
    nes_state_blob_free(&after_export);
    nes_state_blob_free(&replayed);
    CHECK(remove(path) == 0);
    /* A bounded observer never blocks gameplay; incomplete logs cannot export. */
    for (unsigned i = 0; i < 257; ++i) CHECK(joypad_set_player(0, BTN_A, i % 2));
    CHECK(nes_rewind_capture(&ring, &state) == NES_REPLAY_OK);
    nes_rewind_complete(&ring, true);
    CHECK(!nes_rewind_media(&ring, 4)->inputs_valid);
    CHECK(nes_movie_export_history(&ring, 0, 5, path) == NES_MOVIE_UNSUPPORTED_HOST_STATE);
    nes_rewind_clear(&ring);
    nes_input_event_set_observer(reject, NULL);
    CHECK(!joypad_set_player(0, BTN_A, true));
    nes_input_event_clear_observer();
    CHECK(nes_rewind_capture(&ring, &state) == NES_REPLAY_OK);
    CHECK(nes_rewind_media(&ring, 0)->event_count == 0);
    /* PCM cannot exceed the shared budget or change snapshot ownership. */
    ring.max_bytes = ring.total_bytes;
    nes_rewind_complete(&ring, true);
    float pcm[4] = {0};
    nes_rewind_append_audio(&ring, pcm, 2, 48000);
    CHECK(!nes_rewind_media(&ring, 0)->audio_frames && ring.total_bytes == ring.max_bytes);
    nes_rewind_destroy(&ring);
    nes_rewind_track_inputs(NULL);

    FrontendExecutionRuntime execution = {0};
    CHECK(nes_rewind_configure(&execution.rewind, 3, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    for (unsigned i = 0; i < 3; ++i) CHECK(nes_rewind_capture(&execution.rewind, &state) == NES_REPLAY_OK);
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    FrontendHistory *viewer = frontend_history_create(&execution);
    CHECK(viewer);
    frontend_history_use_native(viewer);
    CHECK(frontend_history_action(viewer, HISTORY_OPEN, NULL, 0, NULL, 0));
    FrontendHistoryState view;
    CHECK(frontend_history_state(viewer, &view) && view.open && !view.playing && view.index == 0);
    CHECK(frontend_history_action(viewer, HISTORY_LAST, NULL, 0, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_BOOKMARK_SET, NULL, 3, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_FIRST, NULL, 0, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_BOOKMARK_JUMP, NULL, 3, NULL, 0));
    CHECK(frontend_history_state(viewer, &view) && view.index == 2 && view.bookmarks[3] == 2);
    CHECK(!frontend_history_action(viewer, HISTORY_BOOKMARK_JUMP, NULL, 10, NULL, 0));
    CHECK(!frontend_history_action(viewer, HISTORY_POSITION, "4", 0, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_PLAY, NULL, 0, NULL, 0));
    CHECK(frontend_history_state(viewer, &view) && view.playing && view.index == 0);
    CHECK(frontend_history_action(viewer, HISTORY_VOLUME, NULL, 0, NULL, 0));
    CHECK(!frontend_history_action(viewer, HISTORY_VOLUME, NULL, 101, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_LAST, NULL, 0, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_RANGE_FIRST, NULL, 0, NULL, 0));
    CHECK(frontend_history_action(viewer, HISTORY_FIRST, NULL, 0, NULL, 0));
    CHECK(!frontend_history_action(viewer, HISTORY_RANGE_LAST, NULL, 0, NULL, 0));
    nes_rewind_clear(&execution.rewind);
    frontend_history_tick(viewer);
    CHECK(frontend_history_state(viewer, &view) && !view.open && !frontend_execution_paused(&execution));
    frontend_history_destroy(viewer);
    nes_rewind_destroy(&execution.rewind);
    frontend_panels_reset();
    debugger_shutdown();
    CHECK(unload_rom());
    puts("History media replay/export/budget/bookmark/navigation regressions passed");
    return 0;
}
