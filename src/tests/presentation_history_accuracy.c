/*
 * presentation_history_accuracy.c - Authoritative rewind preview and branching
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../video/history_view.h"
#include "../video/frame_snapshot.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../util/file_io.h"
#include "../ui/history_frontend.h"
#include "../ui/frontend_panels.h"
#include "../../include/globals.h"
#include <string.h>
extern uint8_t ram[0x800];
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "History %d: %s\n", __LINE__, #x);                                                         \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

int run_presentation_history_accuracy_tests(void) {
    BoardImage image = {0};
    apu_power_on(&apu);
    CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    CHECK(board_image_load(&image) == 0);
    board_image_free(&image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    NesRewindHistory history;
    nes_rewind_init(&history);
    CHECK(nes_rewind_configure(&history, 3, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    NesStateResult state;
    for (unsigned i = 0; i < 5; ++i) {
        ram[7] = (uint8_t)i;
        framebuffer[7] = 0xff000000u | i;
        CHECK(nes_rewind_capture(&history, &state) == NES_REPLAY_OK);
    }
    CHECK(history.count == 3 && history.head != 0);
    ram[7] = 99;
    framebuffer[7] = 0xff000063u;
    static uint16_t signal[256 * 240];
    signal[7] = 0x123;
    nes_video_snapshot_complete(0, framebuffer, signal, 2, 12345);
    NesStateBlob before = {0}, after = {0};
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    uint32_t *pixels = malloc(512u * 240u * sizeof(*pixels));
    CHECK(pixels != NULL);
    unsigned width = 0;
    CHECK(nes_history_preview(&history, 0, pixels, 512u * 240u, &width, &state) == NES_REPLAY_OK);
    CHECK(width == 256 && ram[7] == 99 && history.count == 3);
    CHECK(pixels[7] == 0xff000002u && framebuffer[7] == 0xff000063u);
    const NesCompletedVideoFrame *completed = nes_video_snapshot_frame(0);
    CHECK(completed && completed->frame_number == 12345 && completed->phase == 2 &&
          completed->pixels[7] == 0xff000063u && completed->signal[7] == 0x123);
    CHECK(nes_state_capture(&after) == NES_STATE_OK);
    CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);
    CHECK(nes_history_preview(&history, 3, pixels, 512u * 240u, &width, &state) == NES_REPLAY_EMPTY);
    CHECK(nes_history_preview(&history, 0, pixels, 8, &width, &state) == NES_REPLAY_STATE_ERROR);
    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_RECORDING));
    CHECK(nes_history_resume(&history, 0, &state) == NES_REPLAY_CONFLICT && ram[7] == 99);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    const char *path = "build/presentation-history.state";
    CHECK(nes_history_save(&history, 1, path) == NES_STATE_OK && ram[7] == 99);
    CHECK(nes_state_load_file(path) == NES_STATE_OK && ram[7] == 3);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    ram[7] = 99;
    NesStateBlob *entry = &history.entries[(history.head + 1) % history.capacity];
    uint8_t magic = entry->data[0];
    entry->data[0] ^= 0xff;
    CHECK(nes_history_resume(&history, 1, &state) == NES_REPLAY_STATE_ERROR && ram[7] == 99 && history.count == 3);
    entry->data[0] = magic;
    CHECK(nes_history_resume(&history, 1, &state) == NES_REPLAY_OK && ram[7] == 3 && history.count == 1);
    CHECK(history.total_bytes == history.entries[history.head].size);
    CHECK(nes_rewind_step(&history, &state) == NES_REPLAY_OK && ram[7] == 2 && !history.count);
    free(pixels);
    nes_rewind_destroy(&history);
    CHECK(SDL_setenv("SDL_VIDEODRIVER", "dummy", 1) == 0 && SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    FrontendExecutionRuntime execution = {0};
    execution.muted = true;
    CHECK(nes_rewind_configure(&execution.rewind, 3, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    for (unsigned i = 10; i < 13; ++i) {
        ram[7] = (uint8_t)i;
        CHECK(nes_rewind_capture(&execution.rewind, &state) == NES_REPLAY_OK);
    }
    ram[7] = 99;
    frontend_panels_reset();
    frontend_panel_set_session_active(true);
    FrontendHistory *viewer = frontend_history_create(&execution);
    CHECK(viewer);
    char error[512];
    CHECK(frontend_panel_action(0x2700, 0x2701, NULL, 0, error, sizeof(error)));
    CHECK(frontend_execution_paused(&execution) && execution.rewind.count == 3 && ram[7] == 99);
    CHECK(frontend_panel_action(0x2700, 0x2702, "2", 0, error, sizeof(error)));
    frontend_history_tick(viewer);
    CHECK(ram[7] == 99 && execution.rewind.count == 3);
    CHECK(frontend_panel_action(0x2700, 0x2707, NULL, 0, error, sizeof(error)));
    CHECK(ram[7] == 11 && execution.rewind.count == 1 && !frontend_execution_paused(&execution));
    frontend_history_destroy(viewer);
    nes_rewind_destroy(&execution.rewind);
    frontend_panels_reset();
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    CHECK(unload_rom());
    puts("History preview/branch/save/corruption/ownership regressions passed");
    return 0;
}
