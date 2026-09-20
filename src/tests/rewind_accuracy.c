/*
 * rewind_accuracy.c - Rewind and speculative run-ahead regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or later.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../replay/rewind.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../state/state_alloc.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_execution.h"
#include "../ui/frontend_panels.h"
#include "../ui/replay_frontend.h"
#include "../video/video_trace.h"
#include "../../include/globals.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t ram[0x800];
extern uint64_t cpu_total_cycles;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static bool blobs_equal(const NesStateBlob *left, const NesStateBlob *right) {
    return left && right && left->size == right->size
        && (!left->size || memcmp(left->data, right->data, left->size) == 0);
}

static bool run_frame(void *userdata) {
    (void)userdata;
    vs_start_frame();
    while (!ppu.frame_complete) {
        if (vs_cpu_step() <= 0 || cpu.halted) return false;
    }
    return true;
}

static bool power_machine(void) {
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
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

static bool make_vrc7_fixture(BoardImage *image) {
    if (!board_image_create(image, 85, 0x10000, 0x2000, false)) return false;
    uint8_t *prg = image->data + sizeof(iNESHeader);
    uint8_t *fixed = prg + 0xE000;
    static const uint8_t program[] = {
        0x78,                   /* SEI */
        0xA9,0x80,             /* LDA #$80 */
        0x8D,0x00,0x20,        /* STA $2000: vblank NMI */
        0xA9,0x02,             /* loop: LDA #$02 */
        0x8D,0x14,0x40,        /* STA $4014: OAM DMA */
        0xA9,0x01,
        0x8D,0x16,0x40,        /* controller strobe high */
        0xA9,0x00,
        0x8D,0x16,0x40,        /* controller strobe low */
        0xAD,0x16,0x40,
        0x85,0x00,             /* preserve sampled controller bit */
        0xE6,0x04,             /* changing background palette value */
        0xA9,0x3F,
        0x8D,0x06,0x20,
        0xA9,0x00,
        0x8D,0x06,0x20,
        0xA5,0x04,
        0x8D,0x07,0x20,
        0x4C,0x06,0xE0         /* JMP loop */
    };
    memcpy(fixed, program, sizeof(program));
    fixed[0x100] = 0xE6; fixed[0x101] = 0x02; fixed[0x102] = 0x40; /* NMI */
    fixed[0x110] = 0xE6; fixed[0x111] = 0x03; fixed[0x112] = 0x40; /* IRQ */
    prg[0xFFFA] = 0x00; prg[0xFFFB] = 0xE1;
    prg[0xFFFC] = 0x00; prg[0xFFFD] = 0xE0;
    prg[0xFFFE] = 0x10; prg[0xFFFF] = 0xE1;
    return true;
}

static void apply_frame_action(unsigned frame) {
    (void)joypad_set_player(0, BTN_A, (frame & 1u) != 0);
    (void)joypad_set_player(0, BTN_RIGHT, (frame & 2u) != 0);
    cart_cpu_write(0x8000, (uint8_t)(frame + 1u));

    /* VRC7 channel 0: frequency, key-on/block, instrument/volume. */
    cart_cpu_write(0x9010, 0x10);
    cart_cpu_write(0x9030, (uint8_t)(0x40u + frame * 7u));
    cart_cpu_write(0x9010, 0x20);
    cart_cpu_write(0x9030, (uint8_t)(0x11u | ((frame & 3u) << 1)));
    cart_cpu_write(0x9010, 0x30);
    cart_cpu_write(0x9030, (uint8_t)(0x10u | (frame & 0x0Fu)));

    /* Fast cycle-mode mapper IRQs exercise pending interrupt state. */
    cart_cpu_write(0xE008, (uint8_t)(0xF8u + frame));
    cart_cpu_write(0xF000, 0x06);
}

static int test_rewind_forward_determinism(void) {
    BoardImage image = {0};
    CHECK(make_vrc7_fixture(&image));
    CHECK(board_image_load(&image) == 0);
    CHECK(power_machine());
    vs_audio_init(48000);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));

    NesRewindHistory history;
    nes_rewind_init(&history);
    CHECK(nes_rewind_configure(&history, 4, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    NesStateResult state_result = NES_STATE_OK;

    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    apply_frame_action(0);
    CHECK(run_frame(NULL));
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    apply_frame_action(1);
    CHECK(run_frame(NULL));
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    apply_frame_action(2);
    CHECK(run_frame(NULL));
    CHECK(nes_rewind_count(&history) == 3);

    NesStateBlob expected = {0}, replayed = {0};
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    static uint32_t expected_video[SCREEN_WIDTH * SCREEN_HEIGHT];
    memcpy(expected_video, framebuffer, sizeof(expected_video));
    float expected_audio[256], replayed_audio[256];
    apu_audio_pull(&apu, expected_audio, 256);
    uint8_t expected_irq_count = ram[3];
    float expected_expansion = cart_expansion_audio();

    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_OK);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_OK);
    CHECK(nes_rewind_count(&history) == 1);

    apply_frame_action(1);
    CHECK(run_frame(NULL));
    apply_frame_action(2);
    CHECK(run_frame(NULL));
    CHECK(nes_state_capture(&replayed) == NES_STATE_OK);
    apu_audio_pull(&apu, replayed_audio, 256);

    CHECK(blobs_equal(&expected, &replayed));
    CHECK(memcmp(expected_video, framebuffer, sizeof(expected_video)) == 0);
    CHECK(memcmp(expected_audio, replayed_audio, sizeof(expected_audio)) == 0);
    CHECK(ram[3] == expected_irq_count);
    CHECK(cart_expansion_audio() == expected_expansion);

    nes_state_blob_free(&replayed);
    nes_state_blob_free(&expected);
    nes_rewind_destroy(&history);
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

typedef struct {
    unsigned calls;
    bool policies_ok;
    uint8_t expected_buttons;
} RunAheadProbe;

static bool runahead_frame(void *userdata) {
    RunAheadProbe *probe = (RunAheadProbe *)userdata;
    uint32_t policy = nes_execution_policy();
    if (probe->calls == 0) {
        if (policy != NES_EXECUTION_LIVE) probe->policies_ok = false;
    } else {
        if (!(policy & NES_EXECUTION_SPECULATIVE)
            || nes_execution_allows_persistence()
            || nes_execution_allows_automatic_media()) probe->policies_ok = false;
    }
    if (joypad_player(0)->buttons != probe->expected_buttons) probe->policies_ok = false;
    probe->calls++;
    return run_frame(NULL);
}

static int test_runahead_authoritative_machine_video_audio(void) {
    BoardImage image = {0};
    CHECK(make_vrc7_fixture(&image));
    CHECK(board_image_load(&image) == 0);
    CHECK(power_machine());
    vs_audio_init(48000);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    apply_frame_action(3);

    NesStateBlob start = {0}, expected = {0}, actual = {0};
    CHECK(nes_state_capture(&start) == NES_STATE_OK);

    CHECK(run_frame(NULL));
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    static uint32_t expected_authoritative[SCREEN_WIDTH * SCREEN_HEIGHT];
    memcpy(expected_authoritative, framebuffer, sizeof(expected_authoritative));
    float expected_audio[256], actual_audio[256];
    apu_audio_pull(&apu, expected_audio, 256);

    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_OK);
    CHECK(run_frame(NULL) && run_frame(NULL) && run_frame(NULL));
    static uint32_t expected_presented[SCREEN_WIDTH * SCREEN_HEIGHT];
    CHECK(vs_video_copy_completed_frame(expected_presented, SCREEN_WIDTH * SCREEN_HEIGHT));
    static uint16_t expected_signals[SCREEN_WIDTH * SCREEN_HEIGHT];
    unsigned expected_phase = 99;
    memcpy(expected_signals, vs_video_completed_signal(0, &expected_phase), sizeof(expected_signals));
    const NesVideoTraceFrame *completed = nes_video_trace_frame(0);
    CHECK(completed && completed->complete);
    uint64_t expected_frame_number = completed->frame_number;
    size_t tile_bytes = sizeof(NesVideoPixel) * SCREEN_WIDTH * SCREEN_HEIGHT;
    NesVideoPixel *expected_tiles = malloc(tile_bytes);
    CHECK(expected_tiles != NULL);
    memcpy(expected_tiles, completed->pixels, tile_bytes);

    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_OK);
    RunAheadProbe probe = {0, true, joypad_player(0)->buttons};
    NesStateResult state_result = NES_STATE_OK;
    CHECK(nes_runahead_execute(2, runahead_frame, &probe, &state_result) == NES_REPLAY_OK);
    CHECK(probe.calls == 3 && probe.policies_ok);
    CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
    CHECK(nes_state_capture(&actual) == NES_STATE_OK);
    apu_audio_pull(&apu, actual_audio, 256);

    CHECK(blobs_equal(&expected, &actual));
    CHECK(memcmp(expected_audio, actual_audio, sizeof(expected_audio)) == 0);
    CHECK(memcmp(expected_authoritative, framebuffer, sizeof(expected_authoritative)) == 0);
    unsigned future_width = 0, future_height = 0;
    const uint32_t *future = nes_runahead_presented_frame(&future_width, &future_height);
    CHECK(future != NULL && future_width == SCREEN_WIDTH && future_height == SCREEN_HEIGHT);
    CHECK(memcmp(expected_presented, future, sizeof(expected_presented)) == 0);
    unsigned future_phase = 99;
    const uint16_t *future_signal = nes_runahead_presented_signal(0, &future_phase);
    CHECK(future_signal && future_phase == expected_phase);
    CHECK(!memcmp(expected_signals, future_signal, sizeof(expected_signals)));
    const NesVideoTraceFrame *future_trace = nes_runahead_presented_trace(0);
    CHECK(future_trace && future_trace->complete && future_trace->frame_number == expected_frame_number);
    CHECK(!memcmp(expected_tiles, future_trace->pixels, tile_bytes));
    CHECK(future_trace->pixels != nes_video_trace_frame(0)->pixels);
    CHECK(!nes_runahead_presented_signal(1, &future_phase) && future_phase == 0);
    CHECK(!nes_runahead_presented_trace(1) && !nes_runahead_presented_trace(2));

    /* The next real frames reuse both live trace buffers. Presentation must keep
     * the complete speculative frame, including its original composite phase. */
    CHECK(run_frame(NULL) && run_frame(NULL));
    CHECK(!memcmp(expected_tiles, future_trace->pixels, tile_bytes));
    CHECK(!memcmp(expected_signals, nes_runahead_presented_signal(0, &future_phase), sizeof(expected_signals)));
    CHECK(future_phase == expected_phase);
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false));
    CHECK(!memcmp(expected_tiles, future_trace->pixels, tile_bytes));
    free(expected_tiles);
    nes_runahead_shutdown();
    CHECK(!nes_runahead_presented_frame(NULL, NULL));
    CHECK(!nes_runahead_presented_signal(0, NULL) && !nes_runahead_presented_trace(0));

    nes_state_blob_free(&actual);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&start);
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_runahead_dual_video_metadata(void) {
    BoardImage image = {0};
    CHECK(board_image_create(&image, 99, 0x10000, 0x8000, false));
    image.data[7] |= 1u;
    const uint8_t program[] = {
        0xE6,0x00, 0xA9,0x3F, 0x8D,0x06,0x20,
        0xA9,0x00, 0x8D,0x06,0x20, 0xA5,0x00,
        0x8D,0x07,0x20, 0x4C,0x00,0x80
    };
    for (unsigned side = 0; side < 2; ++side) {
        uint8_t *prg = image.data + 16 + side * 0x8000u;
        memcpy(prg, program, sizeof(program));
        for (unsigned vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
            prg[vector] = 0;
            prg[vector + 1] = 0x80;
        }
    }
    CHECK(board_image_load(&image) == 0 && vs_dual_system() && power_machine());
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    NesStateResult state_result;
    CHECK(nes_runahead_execute(2, run_frame, NULL, &state_result) == NES_REPLAY_OK);
    unsigned width = 0, height = 0;
    const uint32_t *pixels = nes_runahead_presented_frame(&width, &height);
    CHECK(pixels && width == 512 && height == 240);
    for (unsigned side = 0; side < 2; ++side) {
        unsigned phase = 99;
        const uint16_t *signal = nes_runahead_presented_signal(side, &phase);
        const NesVideoTraceFrame *trace = nes_runahead_presented_trace(side);
        CHECK(signal && phase < 3 && trace && trace->complete);
        CHECK(trace->frame_number > vs_side_frame_count(side));
        for (unsigned y = 0; y < 240; ++y) {
            for (unsigned x = 0; x < 256; ++x) {
                unsigned index = y * 256 + x;
                if (trace->pixels[index].original_rgb != pixels[y * width + side * 256 + x]) {
                    const PPU *side_ppu = vs_side_ppu(side);
                    fprintf(stderr, "Run-ahead side %u pixel %u,%u: trace=%08X frame=%08X signal=%04X/%04X trace-frame=%llu current-frame=%llu beam=%d:%d\n",
                            side, x, y, (unsigned)trace->pixels[index].original_rgb,
                            (unsigned)pixels[y * width + side * 256 + x],
                            (unsigned)trace->pixels[index].original_signal, (unsigned)signal[index],
                            (unsigned long long)trace->frame_number,
                            (unsigned long long)vs_side_frame_count(side),
                            side_ppu->scanline, side_ppu->dot);
                }
                CHECK(trace->pixels[index].original_rgb == pixels[y * width + side * 256 + x]);
                CHECK(trace->pixels[index].original_signal == signal[index]);
            }
        }
    }
    nes_runahead_clear_presented_frame();
    CHECK(!nes_runahead_presented_trace(0) && !nes_runahead_presented_trace(1));
    CHECK(!nes_runahead_presented_signal(0, NULL) && !nes_runahead_presented_signal(1, NULL));
    nes_runahead_shutdown();
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false));
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_rewind_bounds_and_execution_conflicts(void) {
    BoardImage image = {0};
    CHECK(make_vrc7_fixture(&image));
    CHECK(board_image_load(&image) == 0);
    CHECK(power_machine());
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));

    NesRewindHistory history;
    nes_rewind_init(&history);
    CHECK(nes_rewind_configure(&history, 2, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    NesStateResult state_result = NES_STATE_OK;
    ram[7] = 0x11;
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    ram[7] = 0x22;
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    ram[7] = 0x33;
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    CHECK(nes_rewind_count(&history) == 2);
    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_OK && ram[7] == 0x33);
    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_OK && ram[7] == 0x22);
    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_EMPTY);

    CHECK(nes_rewind_configure(&history, 2, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    ram[7] = 0x44;
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_OK);
    size_t before_count = nes_rewind_count(&history);
    size_t before_capacity = nes_rewind_capacity(&history);
    CHECK(!nes_rewind_configure(&history, NES_REWIND_MAX_FRAMES + 1u,
                                NES_REWIND_DEFAULT_MEMORY_LIMIT));
    CHECK(nes_rewind_count(&history) == before_count
          && nes_rewind_capacity(&history) == before_capacity);

    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    CHECK(nes_rewind_capture(&history, &state_result) == NES_REPLAY_CONFLICT);
    CHECK(nes_rewind_count(&history) == before_count);
    RunAheadProbe blocked = {0, true, joypad_player(0)->buttons};
    CHECK(nes_runahead_execute(2, runahead_frame, &blocked, &state_result)
          == NES_REPLAY_CONFLICT);
    CHECK(blocked.calls == 0);
    CHECK(nes_execution_set_policy(NES_EXECUTION_NETPLAY));
    CHECK(nes_rewind_step(&history, &state_result) == NES_REPLAY_CONFLICT);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));

    nes_rewind_destroy(&history);
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

typedef struct {
    unsigned calls;
    unsigned fail_after_frame;
} AllocationProbe;

static bool allocation_probe_frame(void *userdata) {
    AllocationProbe *probe = userdata;
    bool completed = run_frame(NULL);
    if (++probe->calls == probe->fail_after_frame) nes_state_test_fail_allocation_after(0);
    return completed;
}

static int test_runahead_restore_allocation_failure(void) {
    BoardImage image = {0};
    CHECK(make_vrc7_fixture(&image) && board_image_load(&image) == 0 && power_machine());
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    apply_frame_action(2);
    NesStateBlob initial = {0}, expected = {0}, actual = {0};
    CHECK(nes_state_capture(&initial) == NES_STATE_OK);
    CHECK(run_frame(NULL) && nes_state_capture(&expected) == NES_STATE_OK);

    for (unsigned fail_after = 1; fail_after <= 2; ++fail_after) {
        CHECK(nes_state_restore(initial.data, initial.size) == NES_STATE_OK);
        AllocationProbe probe = {0, fail_after};
        NesStateResult state_result;
        NesReplayResult result = nes_runahead_execute(2, allocation_probe_frame, &probe, &state_result);
        nes_state_test_fail_allocation_after(-1);
        if (fail_after == 1) {
            CHECK(result == NES_REPLAY_STATE_ERROR && state_result == NES_STATE_ERROR_OUT_OF_MEMORY);
            CHECK(probe.calls == 1 && nes_runahead_presented_frame(NULL, NULL) == NULL);
        } else {
            CHECK(result == NES_REPLAY_OK && state_result == NES_STATE_OK && probe.calls == 3);
        }
        CHECK(nes_execution_policy() == NES_EXECUTION_LIVE);
        CHECK(nes_state_capture(&actual) == NES_STATE_OK && blobs_equal(&expected, &actual));
        nes_state_blob_free(&actual);
    }

    nes_state_blob_free(&initial);
    nes_state_blob_free(&expected);
    nes_runahead_clear_presented_frame();
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static bool reject_timeline_change(void *context, char *error, size_t error_size) {
    unsigned *calls = context;
    ++*calls;
    if (error && error_size) snprintf(error, error_size, "Recording output could not be finalized");
    return false;
}

static int test_replay_frontend_contract(void) {
    BoardImage image = {0};
    CHECK(make_vrc7_fixture(&image));
    CHECK(board_image_load(&image) == 0);
    CHECK(power_machine());
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));

    frontend_commands_reset();
    frontend_panels_reset();
    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 0, NULL, NULL, NULL, NULL);
    CHECK(frontend_execution_register_commands(&runtime));

    FrontendCommandInfo command;
    CHECK(frontend_command_get(REPLAY_COMMAND_REWIND_FRAME, &command));
    CHECK(frontend_command_get(REPLAY_COMMAND_RUNAHEAD_CYCLE, &command));
    FrontendPanelInfo panel;
    CHECK(frontend_panel_get(REPLAY_PANEL, &panel));

    FrontendPanelControl controls[8];
    FrontendPanelModel model = {.controls = controls, .capacity = 8};
    char error[160] = {0};
    CHECK(frontend_panel_snapshot(REPLAY_PANEL, &model, error, sizeof(error)));
    CHECK(model.count == 5);
    CHECK(frontend_panel_action(REPLAY_PANEL, REPLAY_CONTROL_RUNAHEAD,
                                NULL, 2, error, sizeof(error)));
    CHECK(frontend_execution_run_ahead(&runtime) == 2);
    CHECK(frontend_panel_action(REPLAY_PANEL, REPLAY_CONTROL_HISTORY,
                                NULL, 2, error, sizeof(error)));
    CHECK(frontend_execution_rewind_seconds(&runtime) == 5);

    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    CHECK(frontend_execution_run_frame(&runtime));
    CHECK(frontend_execution_rewind_available(&runtime) == 1);
    CHECK(nes_runahead_presented_frame(NULL, NULL) != NULL);
    CHECK(nes_video_trace_frame(0)->complete);
    unsigned guard_calls = 0;
    runtime.before_machine_change = reject_timeline_change;
    runtime.machine_change_context = &guard_calls;
    uint64_t before_cycles = cpu_total_cycles;
    CHECK(!frontend_execution_rewind_step(&runtime, error, sizeof(error)));
    CHECK(guard_calls == 1 && cpu_total_cycles == before_cycles);
    CHECK(frontend_execution_rewind_available(&runtime) == 1);
    runtime.before_machine_change = NULL;
    runtime.machine_change_context = NULL;
    CHECK(frontend_execution_rewind_step(&runtime, error, sizeof(error)));
    CHECK(nes_runahead_presented_frame(NULL, NULL) == NULL);
    CHECK(!nes_video_trace_frame(0)->complete);
    CHECK(frontend_execution_run_frame(&runtime));
    CHECK(nes_video_trace_frame(0)->complete);
    CHECK(frontend_panel_action(REPLAY_PANEL, REPLAY_CONTROL_CLEAR,
                                NULL, 0, error, sizeof(error)));
    CHECK(frontend_execution_rewind_available(&runtime) == 0);
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false));

    frontend_execution_shutdown(&runtime);
    frontend_commands_reset();
    frontend_panels_reset();
    CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

int test_rewind_accuracy(void) {
    NesRamPowerOnState old_power = nes_ram_power_on_state();
    uint32_t old_policy = nes_execution_policy();
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_ZERO)
        || !nes_execution_set_policy(NES_EXECUTION_LIVE)) return 1;

    int failures = test_rewind_forward_determinism()
                 + test_runahead_authoritative_machine_video_audio()
                 + test_runahead_dual_video_metadata()
                 + test_rewind_bounds_and_execution_conflicts()
                 + test_runahead_restore_allocation_failure()
                 + test_replay_frontend_contract();

    if (!nes_execution_set_policy(old_policy)) ++failures;
    if (!nes_set_ram_power_on_state(old_power)) ++failures;
    printf("Rewind/run-ahead: 6 groups, %d failures\n", failures);
    return failures;
}
