/*
 * nsf_player_accuracy.c - Music transport and emulated-time progression checks
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../ui/nsf_player.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../audio/audio_observer.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VERIFY(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        failed = 1; \
        goto cleanup; \
    } \
} while (0)

static void put32(uint8_t *data, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) data[i] = (uint8_t)(value >> (i * 8u));
}

static size_t chunk(uint8_t *image, size_t offset, const char id[4],
                      const void *data, size_t size) {
    put32(image + offset, (uint32_t)size);
    memcpy(image + offset + 4u, id, 4);
    if (size) memcpy(image + offset + 8u, data, size);
    return offset + 8u + size;
}

static bool music_fixture(unsigned tracks, unsigned length_ms, unsigned fade_ms,
                           bool native_tone, bool expansion_tone) {
    uint8_t image[1024] = {0};
    uint8_t program[128];
    memset(program, 0xEA, sizeof(program));
    const uint8_t silent_init[] = {0x8D,0x00,0x60,0x60};
    const uint8_t pulse_init[] = {
        0x8D,0x00,0x60,0xA9,0x9F,0x8D,0x00,0x40,
        0xA9,0x80,0x8D,0x02,0x40,0xA9,0x08,0x8D,0x03,0x40,0x60
    };
    const uint8_t expansion_init[] = {
        0x8D,0x00,0x60,0xA9,0x4F,0x8D,0x00,0x90,
        0xA9,0x40,0x8D,0x01,0x90,0xA9,0x80,0x8D,0x02,0x90,0x60
    };
    const uint8_t play[] = {0xEE,0x01,0x60,0x60};
    memcpy(program, expansion_tone ? expansion_init : native_tone ? pulse_init : silent_init,
           expansion_tone ? sizeof(expansion_init) : native_tone ? sizeof(pulse_init) : sizeof(silent_init));
    memcpy(program + 64, play, sizeof(play));
    size_t size;
    if (length_ms) {
        memcpy(image, "NSFE", 4);
        uint8_t info[10] = {0,0x80,0,0x80,0x40,0x80,0,0,(uint8_t)tracks,0};
        info[7] = expansion_tone ? NSF_SOUND_VRC6 : 0;
        size = chunk(image, 4, "INFO", info, sizeof(info));
        size = chunk(image, size, "DATA", program, sizeof(program));
        uint8_t times[16], fades[16];
        for (unsigned i = 0; i < tracks; ++i) {
            put32(times + i * 4u, length_ms);
            put32(fades + i * 4u, fade_ms);
        }
        size = chunk(image, size, "time", times, tracks * 4u);
        size = chunk(image, size, "fade", fades, tracks * 4u);
        const char authors[] = "Test music\0Test artist\0Test copyright\0Test ripper";
        const char names[] = "First\0Second\0Third\0Fourth";
        size = chunk(image, size, "auth", authors, sizeof(authors));
        size = chunk(image, size, "tlbl", names, sizeof(names));
        size = chunk(image, size, "NEND", NULL, 0);
    } else {
        memcpy(image, "NESM\x1A", 5);
        image[5] = 1;
        image[6] = (uint8_t)tracks;
        image[7] = 1;
        image[9] = image[11] = image[13] = 0x80;
        image[12] = 0x40;
        memcpy(image + 14, "Untimed test music", 18);
        image[110] = 0xFF;
        image[111] = 0x40;
        image[120] = 0x1D;
        image[121] = 0x4E;
        image[123] = expansion_tone ? NSF_SOUND_VRC6 : 0;
        memcpy(image + 128, program, sizeof(program));
        size = 128 + sizeof(program);
    }

    cpu_use_default_startup_alignment();
    if (!nes_set_region_mode(NES_REGION_MODE_AUTO) || load_rom_memory(image, size) != 0) return false;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    apu_audio_init(44100);
    if (!cpu_power_on(&cpu)) return false;
    for (unsigned i = 0; i < 100 && cpu.pc != 0x4108; ++i) (void)cpu_step(&cpu);
    return cpu.pc == 0x4108;
}

static bool run_to(double elapsed) {
    uint64_t limit = cpu_total_cycles + (uint64_t)(nes_timing()->cpu_hz * (elapsed + 0.1));
    while (rom_nsf_elapsed_seconds() < elapsed && cpu_total_cycles < limit)
        (void)cpu_step(&cpu);
    return rom_nsf_elapsed_seconds() >= elapsed;
}

typedef struct {
    ExecutionControl *execution;
    unsigned locks;
    unsigned unlocks;
    unsigned pauses;
    bool locked;
    bool active_audio;
    bool reset_under_lock;
} Guard;

static void guard_lock(void *context) {
    Guard *guard = context;
    ++guard->locks;
    guard->locked = true;
}

static void guard_unlock(void *context) {
    Guard *guard = context;
    ++guard->unlocks;
    if (guard->locked && cpu.pc == 0x4100 && cart_cpu_read(0x6001) == 0)
        guard->reset_under_lock = true;
    guard->locked = false;
}

static bool guard_active(void *context) {
    return ((Guard *)context)->active_audio;
}

static void guard_pause(void *context, bool paused) {
    Guard *guard = context;
    ++guard->pauses;
    execution_control_set_paused(guard->execution, paused);
}

static bool init_player(NsfPlayer *player, ExecutionControl *execution, Guard *guard,
                          uint32_t seed) {
    execution_control_init(execution);
    *guard = (Guard){.execution = execution};
    NsfPlayerHooks hooks = {guard_lock, guard_unlock, guard_pause, guard_active, guard};
    return nsf_player_init(player, execution, &hooks, seed);
}

static int test_transport_and_panel(void) {
    int failed = 0;
    NsfPlayer player = {0};
    ExecutionControl execution;
    Guard guard;
    char error[160];
    VERIFY(music_fixture(4, 5000, 1000, false, false));
    VERIFY(init_player(&player, &execution, &guard, 123) && nsf_player_register_ui(&player));
    FrontendPanelControl controls[24];
    FrontendPanelModel panel = {.controls = controls, .capacity = 24};
    VERIFY(frontend_panel_snapshot(NSF_PLAYER_PANEL, &panel, error, sizeof(error)));
    VERIFY(panel.count == 17 && strstr(panel.status, "track 1 of 4"));
    VERIFY(!strcmp(controls[0].value, "Test music") && !strcmp(controls[1].value, "Test artist"));
    VERIFY(controls[5].item_count == 4 && !strcmp(controls[5].items[2], "Third"));
    VERIFY(frontend_command_invoke(NSF_COMMAND_NEXT, error, sizeof(error)) && rom_nsf_current_track() == 1);
    VERIFY(nsf_player_select(&player, 3, error, sizeof(error)));
    VERIFY(nsf_player_next(&player, error, sizeof(error)) && rom_nsf_current_track() == 0);
    VERIFY(nsf_player_previous(&player, error, sizeof(error)) && rom_nsf_current_track() == 3);
    VERIFY(guard.locks == guard.unlocks && guard.reset_under_lock && !guard.locked);
    uint64_t cycles = cpu_total_cycles;
    VERIFY(nsf_player_play_pause(&player, error, sizeof(error)) && execution.paused);
    VERIFY(cpu_total_cycles == cycles && !nsf_player_poll(&player, error, sizeof(error)));
    VERIFY(frontend_panel_action(NSF_PLAYER_PANEL, controls[5].id, NULL, 2, error, sizeof(error)));
    VERIFY(rom_nsf_current_track() == 2 && execution.paused);
    VERIFY(!nsf_player_select(&player, 99, error, sizeof(error)) && error[0]);
    VERIFY(rom_nsf_current_track() == 2 && execution.paused);
    VERIFY(nsf_player_stop(&player, error, sizeof(error)) && player.stopped && execution.paused);
    VERIFY(rom_nsf_elapsed_seconds() < 0.001);
    VERIFY(nsf_player_play_pause(&player, error, sizeof(error)) && !player.stopped && !execution.paused);
    VERIFY(guard.pauses == 3);
    VERIFY(frontend_panel_action(NSF_PLAYER_PANEL, 7, "100", -1, error, sizeof(error)));
    VERIFY(player.options.silence_ms == 100);
    VERIFY(!frontend_panel_action(NSF_PLAYER_PANEL, 7, "-1", -1, error, sizeof(error)));
    VERIFY(player.options.silence_ms == 100);
    VERIFY(unload_rom());
    nsf_player_image_changed(&player);
    FrontendCommandInfo command;
    VERIFY(frontend_command_get(NSF_COMMAND_NEXT, &command) && !command.enabled);
    VERIFY(!frontend_command_invoke(NSF_COMMAND_NEXT, error, sizeof(error)));
cleanup:
    nsf_player_shutdown(&player);
    (void)unload_rom();
    return failed;
}

typedef struct {
    uint64_t samples;
    double seconds;
    double energy;
} SampleTrace;

static void trace_sample(void *context, unsigned machine, double rate, float left, float right) {
    SampleTrace *trace = context;
    if (machine) return;
    ++trace->samples;
    trace->seconds += 1.0 / rate;
    trace->energy += fabsf(left) + fabsf(right);
}

static int test_duration_fade_and_audio_queue(void) {
    int failed = 0;
    NsfPlayer player = {0};
    ExecutionControl execution;
    Guard guard;
    SampleTrace trace = {0};
    uint32_t token = 0;
    float *expected = NULL;
    float *consumed = NULL;
    char error[160];
    VERIFY(music_fixture(2, 40, 30, true, false));
    VERIFY(init_player(&player, &execution, &guard, 44));
    player.options.detect_silence = false;
    guard.active_audio = true;
    token = nes_audio_observer_add(trace_sample, &trace);
    VERIFY(token != 0);
    VERIFY(run_to(0.055));
    VERIFY(fabsf(cart_audio_gain() - 0.5f) < 0.002f);
    VERIFY(!nsf_player_poll(&player, error, sizeof(error)) && rom_nsf_current_track() == 0);
    VERIFY(run_to(0.0701) && cart_audio_gain() == 0.0f);
    uint32_t read = atomic_load_explicit(&apu.ring_r, memory_order_relaxed);
    uint32_t write = atomic_load_explicit(&apu.ring_w, memory_order_acquire);
    unsigned queued = (write - read) & (APU_RING_CAP - 1u);
    VERIFY(queued > 3000 && queued < 3200 && trace.energy > 10.0);
    expected = malloc(queued * 2u * sizeof(float));
    consumed = malloc(queued * 2u * sizeof(float));
    VERIFY(expected && consumed);
    for (unsigned i = 0; i < queued; ++i) {
        unsigned index = (read + i) & (APU_RING_CAP - 1u);
        expected[i * 2u] = apu.ring[index] + apu.ring_side[index];
        expected[i * 2u + 1u] = apu.ring[index] - apu.ring_side[index];
    }
    uint64_t produced = trace.samples;
    VERIFY(nsf_player_poll(&player, error, sizeof(error)) && rom_nsf_current_track() == 1);
    VERIFY(trace.samples >= produced && trace.samples <= produced + 1u);
    apu_audio_pull_stereo(&apu, consumed, (int)queued);
    VERIFY(!memcmp(expected, consumed, queued * 2u * sizeof(float)));
    VERIFY(guard.locks == 1 && guard.unlocks == 1 && guard.reset_under_lock);
    NsfPlayerInfo info;
    VERIFY(nsf_player_info(&player, &info));
    VERIFY(info.position_seconds < 0.001 && fabs(info.length_seconds - 0.070) < 0.000001);
    VERIFY(!nsf_player_poll(&player, error, sizeof(error)));
cleanup:
    (void)nes_audio_observer_remove(token);
    free(expected);
    free(consumed);
    nsf_player_shutdown(&player);
    (void)unload_rom();
    return failed;
}

static int test_repeat_shuffle_and_wrap(void) {
    int failed = 0;
    NsfPlayer player = {0};
    ExecutionControl execution;
    Guard guard;
    char error[160];
    unsigned sequence[32];
    VERIFY(music_fixture(4, 2, 0, false, false));
    VERIFY(init_player(&player, &execution, &guard, 0x13579u));
    player.options.detect_silence = false;
    for (unsigned i = 1; i <= 4; ++i) {
        VERIFY(run_to(0.0021) && nsf_player_poll(&player, error, sizeof(error)));
        VERIFY(rom_nsf_current_track() == i % 4u);
    }
    player.options.repeat = true;
    VERIFY(run_to(0.0021) && nsf_player_poll(&player, error, sizeof(error)));
    VERIFY(rom_nsf_current_track() == 0 && rom_nsf_elapsed_seconds() < 0.001);
    player.options.repeat = false;
    player.options.shuffle = true;
    unsigned previous = rom_nsf_current_track();
    unsigned seen = 0;
    for (unsigned i = 0; i < 32; ++i) {
        VERIFY(run_to(0.0021) && nsf_player_poll(&player, error, sizeof(error)));
        sequence[i] = rom_nsf_current_track();
        VERIFY(sequence[i] < 4 && sequence[i] != previous);
        seen |= 1u << sequence[i];
        previous = sequence[i];
    }
    VERIFY(seen == 15);
    nsf_player_shutdown(&player);
    VERIFY(music_fixture(4, 2, 0, false, false));
    VERIFY(init_player(&player, &execution, &guard, 0x13579u));
    player.options.detect_silence = false;
    player.options.shuffle = true;
    for (unsigned i = 0; i < 32; ++i) {
        VERIFY(run_to(0.0021) && nsf_player_poll(&player, error, sizeof(error)));
        VERIFY(rom_nsf_current_track() == sequence[i]);
    }
cleanup:
    nsf_player_shutdown(&player);
    (void)unload_rom();
    return failed;
}

static int test_silence_pause_and_expansion(void) {
    int failed = 0;
    NsfPlayer player = {0};
    ExecutionControl execution;
    Guard guard;
    char error[160];
    VERIFY(music_fixture(2, 0, 0, false, true));
    VERIFY(init_player(&player, &execution, &guard, 77));
    NsfPlayerOptions options = player.options;
    options.silence_ms = 40;
    VERIFY(nsf_player_set_options(&player, &options));
    VERIFY(run_to(0.15));
    VERIFY(!nsf_player_poll(&player, error, sizeof(error)) && rom_nsf_current_track() == 0);
    VERIFY(player.silent_seconds < 0.01);
    execution_control_set_paused(&execution, true);
    double silence = player.silent_seconds;
    for (unsigned i = 0; i < 5000; ++i) nes_audio_observe(0, 44100.0, 0.0f, 0.0f);
    VERIFY(player.silent_seconds == silence && !nsf_player_poll(&player, error, sizeof(error)));
    execution_control_set_paused(&execution, false);
    cart_cpu_write(0x9002, 0);
    VERIFY(run_to(0.4));
    VERIFY(nsf_player_poll(&player, error, sizeof(error)) && rom_nsf_current_track() == 1);
    NsfPlayerInfo info;
    VERIFY(nsf_player_info(&player, &info) && info.length_seconds == 0.0);
    options.detect_silence = false;
    VERIFY(nsf_player_set_options(&player, &options));
    cart_cpu_write(0x9002, 0);
    VERIFY(run_to(0.1) && !nsf_player_poll(&player, error, sizeof(error)));
    VERIFY(rom_nsf_current_track() == 1);
cleanup:
    nsf_player_shutdown(&player);
    (void)unload_rom();
    return failed;
}

static int test_observer_and_session_isolation(void) {
    int failed = 0;
    NsfPlayer player = {0};
    ExecutionControl execution;
    Guard guard;
    SampleTrace trace = {0};
    uint32_t token = 0;
    char error[160];
    VERIFY(music_fixture(2, 5, 0, false, false));
    VERIFY(init_player(&player, &execution, &guard, 3));
    token = nes_audio_observer_add(trace_sample, &trace);
    VERIFY(token != 0);
    uint64_t cycles = cpu_total_cycles;
    nes_audio_observe(0, 44100.0, 0.5f, -0.5f);
    VERIFY(trace.samples == 1 && cpu_total_cycles == cycles);
    const uint32_t policies[] = {NES_EXECUTION_SPECULATIVE, NES_EXECUTION_REWIND};
    for (unsigned i = 0; i < 2; ++i) {
        VERIFY(nes_execution_set_policy(policies[i]));
        nes_audio_observe(0, 44100.0, 0.0f, 0.0f);
        VERIFY(trace.samples == 1);
    }
    VERIFY(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    VERIFY(run_to(0.02));
    unsigned track = rom_nsf_current_track();
    VERIFY(!nsf_player_poll(&player, error, sizeof(error)) && rom_nsf_current_track() == track);
    VERIFY(!nsf_player_select(&player, 1, error, sizeof(error)) && error[0]);
    VERIFY(nes_execution_set_policy(NES_EXECUTION_LIVE));
    NsfPlayerOptions bad = player.options;
    bad.silence_threshold = NAN;
    VERIFY(!nsf_player_set_options(&player, &bad) && isfinite(player.options.silence_threshold));
    bad = player.options;
    bad.silence_ms = 0;
    VERIFY(!nsf_player_set_options(&player, &bad) && player.options.silence_ms == 3000);
    VERIFY(nes_audio_observer_remove(token));
    VERIFY(!nes_audio_observer_remove(token));
    token = 0;
    uint64_t samples = trace.samples;
    nes_audio_observe(0, 44100.0, 0.5f, 0.5f);
    VERIFY(trace.samples == samples);
    VERIFY(music_fixture(1, 0, 0, false, false));
    nsf_player_image_changed(&player);
    VERIFY(player.silent_seconds == 0.0 && !player.stopped);
    VERIFY(!nsf_player_poll(&player, error, sizeof(error)));
cleanup:
    (void)nes_audio_observer_remove(token);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    nsf_player_shutdown(&player);
    (void)unload_rom();
    return failed;
}

int test_nsf_player_accuracy(void) {
    NesRegionMode previous_mode = nes_region_mode();
    int failures = test_transport_and_panel() + test_duration_fade_and_audio_queue()
        + test_repeat_shuffle_and_wrap() + test_silence_pause_and_expansion()
        + test_observer_and_session_isolation();
    (void)nes_set_region_mode(previous_mode);
    printf("Music player: 5 groups, %d failures\n", failures);
    return failures;
}
