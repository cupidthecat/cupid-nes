/*
 * capture_session_accuracy.c - Production frames, stereo and capture command routing
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../system/execution_policy.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../third_party/miniz/miniz.h"
#include "../ui/capture_runtime.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../video/ntsc_composite.h"
#include "../../include/globals.h"
#ifdef _WIN32
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

int capture_test_png_matches(const char *path, const NesCaptureFrame *frame);

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        failed = 1; \
        goto cleanup; \
    } \
} while (0)

static bool fixture(NesRegion region, bool dual, bool stereo) {
    BoardImage image = {0};
    size_t prg_size = dual ? 0x10000 : 0x8000;
    if (!board_image_create(&image, dual ? 99 : 0, prg_size, dual ? 0x8000 : 0x2000, !dual)) return false;
    if (dual) {
        image.data[6] = 0x30;
        image.data[7] = 0x61;
    } else {
        image.data[12] = region == NES_REGION_PAL ? 1 : region == NES_REGION_DENDY ? 3 : 0;
        if (stereo) {
            image.data[7] = 0x0B;
            image.data[13] = 4;
        }
    }
    uint8_t *prg = image.data + 16;
    memset(prg, 0xEA, prg_size);
    const uint8_t paint[] = {
        0xA9,0x3F, 0x8D,0x06,0x20, 0xA9,0, 0x8D,0x06,0x20,
        0xA9,1, 0x8D,0x07,0x20, 0xA9,0, 0x8D,0x06,0x20, 0x8D,0x06,0x20
    };
    const uint8_t pulse[] = {
        0xA9,1, 0x8D,0x15,0x40, 0xA9,0xBF, 0x8D,0,0x40,
        0xA9,0x40, 0x8D,2,0x40, 0xA9,0, 0x8D,3,0x40
    };
    for (unsigned machine = 0; machine < (dual ? 2u : 1u); ++machine) {
        uint8_t *program = prg + machine * 0x8000u;
        memcpy(program, paint, sizeof(paint));
        program[11] = (uint8_t)(machine + 1);
        size_t offset = sizeof(paint);
        if ((!dual || machine == 1) && !stereo) {
            memcpy(program + offset, pulse, sizeof(pulse));
            offset += sizeof(pulse);
        }
        program[offset] = 0x4C;
        program[offset + 1] = (uint8_t)offset;
        program[offset + 2] = 0x80;
        for (size_t vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
            program[vector] = 0;
            program[vector + 1] = 0x80;
        }
    }
    bool ok = nes_set_region_mode(NES_REGION_MODE_AUTO) && load_rom_memory(image.data, image.size) == 0;
    board_image_free(&image);
    if (!ok) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_power_on_secondary();
    vs_audio_init(44100);
    return vs_dual_system() == dual && epsm_enabled() == stereo;
}

static bool run_frame(FrontendExecutionRuntime *execution, NesCaptureSession *session) {
    if (session && !nes_capture_session_begin_frame(session)) return false;
    if (!frontend_execution_run_frame(execution)) return false;
    NesCaptureFrame frame = {vs_video_framebuffer(), vs_video_width(), SCREEN_HEIGHT, vs_video_width()};
    return !session || nes_capture_session_end_frame(session, &frame) == NES_FILE_OK;
}

typedef struct {
    uint64_t cycles;
    uint64_t ppu_frames;
    uint16_t pc;
    uint8_t a, x, y, sp, flags;
    uint32_t memory_crc, video_crc, audio_crc;
    uint32_t audio_phase;
    unsigned audio_write;
} MachineDigest;

static MachineDigest digest(void) {
    MachineDigest result = {0};
    result.cycles = cpu_total_cycles;
    result.ppu_frames = ppu.frame_count;
    result.pc = cpu.pc;
    result.a = cpu.a; result.x = cpu.x; result.y = cpu.y;
    result.sp = cpu.sp; result.flags = cpu.status;
    uint8_t memory[0x800];
    for (unsigned i = 0; i < sizeof(memory); ++i) memory[i] = cpu_peek_internal_ram((uint16_t)i);
    result.memory_crc = (uint32_t)mz_crc32(0, memory, sizeof(memory));
    result.video_crc = (uint32_t)mz_crc32(0, (const uint8_t *)vs_video_framebuffer(),
                                          vs_video_width() * SCREEN_HEIGHT * sizeof(uint32_t));
    result.audio_crc = (uint32_t)mz_crc32(0, (const uint8_t *)apu.ring, sizeof(apu.ring));
    result.audio_crc = (uint32_t)mz_crc32(result.audio_crc, (const uint8_t *)apu.ring_side, sizeof(apu.ring_side));
    result.audio_phase = apu.cycle_in_seq;
    result.audio_write = atomic_load(&apu.ring_w);
    return result;
}

static bool digest_equal(const MachineDigest *a, const MachineDigest *b) {
    return a->cycles == b->cycles && a->ppu_frames == b->ppu_frames
        && a->pc == b->pc && a->a == b->a && a->x == b->x && a->y == b->y
        && a->sp == b->sp && a->flags == b->flags && a->memory_crc == b->memory_crc
        && a->video_crc == b->video_crc && a->audio_crc == b->audio_crc
        && a->audio_phase == b->audio_phase && a->audio_write == b->audio_write;
}

static uint32_t read32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int test_clocks_and_fidelity(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/fidelity.wav", directory);
    NesCaptureSession session = {0};
    uint8_t *data = NULL;
    size_t size = 0;
    FrontendExecutionRuntime execution;
    for (unsigned region = NES_REGION_NTSC; region <= NES_REGION_DENDY; ++region) {
        CHECK(fixture((NesRegion)region, false, false));
        frontend_execution_init(&execution, NULL, 44100, "fixture.nes", NULL, NULL, NULL);
        NesCaptureOptions options;
        nes_capture_options_defaults(&options);
        options.sample_rate = region == NES_REGION_PAL ? 48000 : 44100;
        CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
        for (unsigned frame = 0; frame < 6; ++frame) {
            CHECK(run_frame(&execution, &session));
            CHECK(execution_control_set_speed(&execution.execution, frame & 1u ? 0.5 : 2.0));
            if (frame == 2) {
                execution_control_set_paused(&execution.execution, true);
                uint64_t paused_cycles = cpu_total_cycles;
                for (unsigned i = 0; i < 20; ++i) {
                    CHECK(nes_capture_session_begin_frame(&session));
                    CHECK(!frontend_execution_run_frame(&execution));
                }
                CHECK(cpu_total_cycles == paused_cycles && session.info.completed_frames == 3);
                execution_control_set_paused(&execution.execution, false);
            }
        }
        CHECK(session.info.completed_frames == 6 && session.info.recording);
        uint64_t expected_samples = 6u * (uint64_t)options.sample_rate * session.info.frame_rate.denominator
                                 / session.info.frame_rate.numerator;
        CHECK(session.info.audio_frames == expected_samples);
        MachineDigest captured = digest();
        CHECK(nes_capture_session_stop(&session) == NES_FILE_OK && !session.info.recording);
        CHECK(nes_file_read_all(path, 100000, &data, &size) == NES_FILE_OK);
        CHECK(size == 44 + expected_samples * 4 && read32(data + 40) == expected_samples * 4);
        bool audible = false;
        for (size_t i = 44; i < size; ++i) audible |= data[i] != 0;
        CHECK(audible);
        free(data); data = NULL;
        CHECK(fixture((NesRegion)region, false, false));
        frontend_execution_init(&execution, NULL, 44100, "fixture.nes", NULL, NULL, NULL);
        for (unsigned frame = 0; frame < 6; ++frame) CHECK(run_frame(&execution, NULL));
        MachineDigest ordinary = digest();
        CHECK(digest_equal(&captured, &ordinary));
    }
cleanup:
    nes_capture_session_discard(&session);
    free(data);
    (void)nes_file_remove(path);
    (void)unload_rom();
    return failed;
}

static void fm_write(uint8_t reg, uint8_t value) {
    write_mem(0x401C, reg);
    write_mem(0x401D, value);
}

static void fm_left_tone(void) {
    fm_write(0x29, 0x83);
    for (unsigned op = 0; op < 4; ++op) {
        uint8_t offset = (uint8_t)(op * 4u);
        fm_write((uint8_t)(0x30 + offset), 1);
        fm_write((uint8_t)(0x40 + offset), 0x18);
        fm_write((uint8_t)(0x50 + offset), 0x1F);
        fm_write((uint8_t)(0x60 + offset), 0);
        fm_write((uint8_t)(0x70 + offset), 0);
        fm_write((uint8_t)(0x80 + offset), 0x0F);
    }
    fm_write(0xB0, 7);
    fm_write(0xB4, 0x80);
    fm_write(0xA4, 0x22);
    fm_write(0xA0, 0x69);
    fm_write(0x28, 0xF0);
}

static int test_stereo_and_dual(const char *directory) {
    int failed = 0;
    char path[256], screenshot[256];
    snprintf(path, sizeof(path), "%s/stereo.wav", directory);
    snprintf(screenshot, sizeof(screenshot), "%s/dual.png", directory);
    NesCaptureSession session = {0};
    uint8_t *data = NULL;
    size_t size = 0;
    NesCaptureOptions options;
    nes_capture_options_defaults(&options);
    FrontendExecutionRuntime execution;
    CHECK(fixture(NES_REGION_NTSC, false, true));
    fm_left_tone();
    frontend_execution_init(&execution, NULL, 44100, "fixture.nes", NULL, NULL, NULL);
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
    for (unsigned i = 0; i < 3; ++i) CHECK(run_frame(&execution, &session));
    CHECK(nes_capture_session_stop(&session) == NES_FILE_OK);
    CHECK(nes_file_read_all(path, 100000, &data, &size) == NES_FILE_OK && size > 1000);
    bool left = false;
    for (size_t i = 44; i + 3 < size; i += 4) {
        left |= data[i] != 0 || data[i + 1] != 0;
        CHECK(data[i + 2] == 0 && data[i + 3] == 0);
    }
    CHECK(left);
    free(data); data = NULL;
    CHECK(fixture(NES_REGION_NTSC, true, false));
    frontend_execution_init(&execution, NULL, 44100, "dual.nes", NULL, NULL, NULL);
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
    for (unsigned i = 0; i < 3; ++i) CHECK(run_frame(&execution, &session));
    CHECK(!apu.pulse1.enabled && vs_side_apu(1)->pulse1.enabled);
    CHECK(nes_capture_session_stop(&session) == NES_FILE_OK);
    CHECK(nes_file_read_all(path, 100000, &data, &size) == NES_FILE_OK && size > 1000);
    bool secondary_audio = false;
    for (size_t i = 44; i + 3 < size; i += 4) {
        secondary_audio |= data[i] != 0 || data[i + 1] != 0;
        CHECK(data[i] == data[i + 2] && data[i + 1] == data[i + 3]);
    }
    CHECK(secondary_audio);
    NesCaptureFrame frame = {vs_video_framebuffer(), vs_video_width(), SCREEN_HEIGHT, vs_video_width()};
    CHECK(frame.width == 512 && frame.pixels[100 * frame.stride + 100] != frame.pixels[100 * frame.stride + 356]);
    CHECK(nes_capture_png(screenshot, &frame) == NES_FILE_OK);
    CHECK(capture_test_png_matches(screenshot, &frame) == 0);
cleanup:
    nes_capture_session_discard(&session);
    free(data);
    (void)nes_file_remove(path);
    (void)nes_file_remove(screenshot);
    (void)unload_rom();
    return failed;
}

static int test_frontend_and_interruptions(const char *directory) {
    int failed = 0;
    char png[256], wav[256], avi[256], protected_path[256], alias[260], error[256];
    snprintf(png, sizeof(png), "%s/screen.png", directory);
    snprintf(wav, sizeof(wav), "%s/panel.wav", directory);
    snprintf(avi, sizeof(avi), "%s/panel.avi", directory);
    snprintf(protected_path, sizeof(protected_path), "%s/game.nes", directory);
    snprintf(alias, sizeof(alias), "./%s", protected_path);
    NesCaptureRuntime capture = {0};
    FrontendExecutionRuntime execution;
    bool composite = false;
    uint32_t *filtered = calloc(NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT, sizeof(uint32_t));
    uint8_t *data = NULL;
    size_t size = 0;
    CHECK(filtered && fixture(NES_REGION_NTSC, false, false));
    CHECK(nes_file_write_atomic(protected_path, "keep", 4) == NES_FILE_OK);
    frontend_execution_init(&execution, NULL, 44100, protected_path, NULL, NULL, NULL);
    CHECK(frontend_execution_register_commands(&execution));
    CHECK(nes_capture_runtime_init(&capture, &execution, &composite, filtered, NULL, 0));
    CHECK(run_frame(&execution, NULL));
    CHECK(!nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_PNG, alias, error, sizeof(error)) && error[0]);
    CHECK(nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_PNG, png, error, sizeof(error)));
    CHECK(nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_WAV, wav, error, sizeof(error)));
    CHECK(nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_AVI, avi, error, sizeof(error)));
    FrontendPanelControl controls[16];
    FrontendPanelModel panel = {.controls = controls, .capacity = 16};
    CHECK(frontend_panel_snapshot(CAPTURE_PANEL, &panel, error, sizeof(error)) && panel.count == 13);
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_SCREENSHOT, error, sizeof(error)));
    NesCaptureFrame frame = {vs_video_framebuffer(), vs_video_width(), SCREEN_HEIGHT, vs_video_width()};
    CHECK(capture_test_png_matches(png, &frame) == 0);
    composite = true;
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_SCREENSHOT, error, sizeof(error)));
    frame = (NesCaptureFrame){filtered, NTSC_COMPOSITE_WIDTH, NTSC_COMPOSITE_HEIGHT, NTSC_COMPOSITE_WIDTH};
    CHECK(capture_test_png_matches(png, &frame) == 0);
    CHECK(frontend_command_invoke(CAPTURE_COMMAND_AUDIO, error, sizeof(error)));
    CHECK(!nes_capture_frontend_set_path(&capture.frontend, FRONTEND_SAVE_WAV, avi, error, sizeof(error)));
    CHECK(!frontend_panel_action(CAPTURE_PANEL, 4, NULL, 0, error, sizeof(error)));
    CHECK(run_frame(&execution, &capture.frontend.session));
    CHECK(frontend_command_invoke(FRONTEND_COMMAND_SOFT_RESET, error, sizeof(error)));
    CHECK(!capture.frontend.session.info.recording && capture.frontend.session.info.completed_frames == 1);
    CHECK(nes_file_read_all(wav, 100000, &data, &size) == NES_FILE_OK && size > 44);
    free(data); data = NULL;

    CHECK(frontend_command_invoke(CAPTURE_COMMAND_VIDEO, error, sizeof(error)));
    nes_capture_frontend_begin_frame(&capture.frontend);
    CHECK(frontend_execution_run_frame(&execution));
    nes_capture_frontend_end_frame(&capture.frontend, true);
    CHECK(capture.frontend.session.info.completed_frames == 1);
    /* Dimension changes finalize the complete preceding frame. */
    composite = false;
    nes_capture_frontend_begin_frame(&capture.frontend);
    CHECK(frontend_execution_run_frame(&execution));
    nes_capture_frontend_end_frame(&capture.frontend, true);
    CHECK(!capture.frontend.session.info.recording && capture.frontend.session.error[0]);
    CHECK(nes_file_read_all(avi, 2u * 1024u * 1024u, &data, &size) == NES_FILE_OK && size > 324);
    CHECK(read32(data + 48) == 1);
    free(data); data = NULL;
    CHECK(nes_file_read_all(protected_path, 4, &data, &size) == NES_FILE_OK && size == 4 && !memcmp(data, "keep", 4));
cleanup:
    (void)nes_capture_runtime_shutdown(&capture);
    frontend_commands_reset();
    free(filtered);
    free(data);
    (void)nes_file_remove(png);
    (void)nes_file_remove(wav);
    (void)nes_file_remove(avi);
    (void)nes_file_remove(protected_path);
    (void)unload_rom();
    return failed;
}

static bool music_fixture(void) {
    uint8_t image[256] = {0};
    memcpy(image, "NESM\x1A", 5);
    image[5] = 1; image[6] = 2; image[7] = 1;
    image[9] = image[11] = image[13] = 0x80;
    image[12] = 0x40;
    image[110] = 0xFF; image[111] = 0x40;
    image[120] = 0x1D; image[121] = 0x4E;
    memset(image + 128, 0xEA, 128);
    image[128] = image[192] = 0x60;
    if (load_rom_memory(image, sizeof(image)) != 0) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    apu_audio_init(44100);
    return cpu_power_on(&cpu);
}

static int test_track_resets_and_policy(const char *directory) {
    int failed = 0;
    char path[256];
    snprintf(path, sizeof(path), "%s/music.wav", directory);
    NesCaptureSession session = {0};
    NesCaptureOptions options;
    nes_capture_options_defaults(&options);
    FrontendExecutionRuntime execution;
    uint8_t *data = NULL;
    size_t size = 0;
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO) && music_fixture());
    frontend_execution_init(&execution, NULL, 44100, "fixture.nsf", NULL, NULL, NULL);
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
    for (unsigned i = 0; i < 3; ++i) CHECK(run_frame(&execution, &session));
    uint64_t before_reset = cpu_total_cycles;
    CHECK(rom_nsf_select_track(1) && cpu_total_cycles == before_reset + 7);
    CHECK(rom_nsf_current_track() == 1 && rom_nsf_elapsed_seconds() == 0.0);
    for (unsigned i = 0; i < 2; ++i) CHECK(run_frame(&execution, &session));
    CHECK(session.info.completed_frames == 5 && session.info.recording);
    CHECK(nes_execution_set_policy(NES_EXECUTION_REWIND));
    CHECK(!nes_capture_session_begin_frame(&session) && !session.info.recording && session.error[0]);
    CHECK(nes_file_read_all(path, 100000, &data, &size) == NES_FILE_OK);
    CHECK(size == 44 + session.info.audio_frames * 4);
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_INVALID_ARGUMENT);
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    options.byte_limit = 44;
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
    CHECK(nes_capture_session_begin_frame(&session));
    CHECK(frontend_execution_run_frame(&execution));
    CHECK(nes_capture_session_end_frame(&session, NULL) == NES_FILE_TOO_LARGE);
    CHECK(!session.info.recording && !session.info.completed_frames);
    options.byte_limit = 100000;
    CHECK(nes_capture_session_start(&session, path, false, NULL, &options) == NES_FILE_OK);
    CHECK(run_frame(&execution, &session));
    /* A genuine machine-clock reset is distinct from a track's elapsed timer.
     * The capture boundary also protects callers outside the desktop commands. */
    CHECK(cpu_power_on(&cpu));
    CHECK(!nes_capture_session_begin_frame(&session) && !session.info.recording);
    CHECK(session.info.completed_frames == 1 && strstr(session.error, "clock was restored"));
cleanup:
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    nes_capture_session_discard(&session);
    free(data);
    (void)nes_file_remove(path);
    (void)unload_rom();
    return failed;
}

int test_capture_session_accuracy(void) {
    char directory[128];
#ifdef _WIN32
    snprintf(directory, sizeof(directory), "build/capture-sessions-%u", (unsigned)_getpid());
    if (_mkdir(directory) != 0) return 1;
#else
    snprintf(directory, sizeof(directory), "build/capture-sessions-%u", (unsigned)getpid());
    if (mkdir(directory, 0700) != 0) return 1;
#endif
    NesRegionMode previous_mode = nes_region_mode();
    NesRamPowerOnState previous_ram = nes_ram_power_on_state();
    uint32_t previous_policy = nes_execution_policy();
    bool previous_startup = ppu_startup_write_restriction_enabled();
    (void)nes_set_ram_power_on_state(NES_RAM_POWER_ZERO);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    ppu_set_startup_write_restriction(false);
    int failures = test_clocks_and_fidelity(directory) + test_stereo_and_dual(directory)
        + test_frontend_and_interruptions(directory) + test_track_resets_and_policy(directory);
    (void)nes_set_region_mode(previous_mode);
    (void)nes_set_ram_power_on_state(previous_ram);
    (void)nes_execution_set_policy(previous_policy);
    ppu_set_startup_write_restriction(previous_startup);
#ifdef _WIN32
    if (_rmdir(directory) != 0) ++failures;
#else
    if (rmdir(directory) != 0) ++failures;
#endif
    printf("Capture sessions: 4 groups, %d failures\n", failures);
    return failures;
}
