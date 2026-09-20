/*
 * audio_mix_accuracy.c - Listening controls preserve emulated sound hardware
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../audio/audio_mix.h"
#include "../audio/audio_observer.h"
#include "../rom/mapper_state.h"
#include "../state/state.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include <math.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

enum { MIX_FRAMES = 1024 };
static float samples[MIX_FRAMES * 2], expected_samples[MIX_FRAMES * 2];
static uint32_t probe_token;
static double observed_energy[2];

static void observe_sample(void *context, unsigned machine, double rate, float left, float right) {
    (void)context;
    if (machine != 0 || rate != 44100.0) return;
    observed_energy[0] += (double)left * left;
    observed_energy[1] += (double)right * right;
}

static bool load_mix_machine(bool epsm, bool dual) {
    BoardImage image = {0};
    size_t prg_bytes = dual ? 0x10000 : 0x8000;
    if (!board_image_create(&image, dual ? 99 : 0, prg_bytes, dual ? 0x8000 : 0x2000, !dual)) return false;
    if (dual) {
        image.data[6] = 0x30;
        image.data[7] = 0x61;
    } else if (epsm) {
        image.data[7] = 0x0B;
        image.data[13] = 4;
    }
    uint8_t *prg = image.data + 16;
    memset(prg, 0xEA, prg_bytes);
    for (unsigned side = 0; side < (dual ? 2u : 1u); ++side) {
        size_t base = side * 0x8000u;
        prg[base] = 0x4C;
        prg[base + 1] = 0;
        prg[base + 2] = 0x80;
        for (unsigned vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
            prg[base + vector] = 0;
            prg[base + vector + 1] = 0x80;
        }
    }
    bool loaded = load_rom_memory(image.data, image.size) == 0;
    board_image_free(&image);
    if (!loaded) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_power_on_secondary();
    vs_audio_init(44100);
    return true;
}

static void pulse_tone(void) {
    write_mem(0x4015, 1);
    write_mem(0x4000, 0xBF);
    write_mem(0x4002, 0x20);
    write_mem(0x4003, 8);
}

static int collect_stereo(float *output, unsigned frames) {
    uint64_t start = cpu_total_cycles;
    while (((atomic_load(&apu.ring_w) - atomic_load(&apu.ring_r)) & (APU_RING_CAP - 1u)) < frames) {
        CHECK(vs_cpu_step() > 0 && !cpu.halted);
        CHECK(cpu_total_cycles - start < 1000000);
    }
    vs_audio_stereo_callback(NULL, (uint8_t *)output, (int)(frames * 2 * sizeof(float)));
    for (unsigned i = 0; i < frames * 2; ++i) CHECK(isfinite(output[i]));
    return 0;
}

static double energy(const float *output, unsigned side, unsigned frames) {
    double result = 0;
    for (unsigned i = 0; i < frames; ++i) {
        double sample = output[i * 2 + side];
        result += sample * sample;
    }
    return result;
}

static bool capture_hardware(NesStateWriter *writer) {
    nes_state_writer_init(writer, NES_STATE_MAX_SIZE);
    return cpu_state_capture(writer) && ppu_state_capture(writer)
        && apu_hardware_state_capture(writer) && mapper_state_capture(writer);
}

static int test_settings_and_hardware_invariance(void) {
    NesAudioMixSettings settings, actual;
    nes_audio_mix_defaults(&settings);
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    CHECK(load_mix_machine(false, false));
    pulse_tone();
    NesStateBlob start = {0};
    CHECK(nes_state_capture(&start) == NES_STATE_OK);
    for (unsigned i = 0; i < 7000; ++i) CHECK(cpu_step(&cpu) > 0);
    NesStateWriter normal, muted;
    CHECK(capture_hardware(&normal));
    CHECK(nes_state_restore(start.data, start.size) == NES_STATE_OK);
    for (unsigned channel = 0; channel < NES_AUDIO_CHANNEL_COUNT; ++channel) {
        settings.volume[channel] = 0;
        settings.pan[channel] = channel & 1 ? -100 : 100;
    }
    settings.master_volume = 17;
    settings.muted = true;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    for (unsigned i = 0; i < 7000; ++i) CHECK(cpu_step(&cpu) > 0);
    CHECK(capture_hardware(&muted));
    CHECK(normal.size == muted.size && !memcmp(normal.data, muted.data, normal.size));
    nes_state_writer_destroy(&normal);
    nes_state_writer_destroy(&muted);
    nes_state_blob_free(&start);

    NesAudioMixSettings invalid = settings;
    invalid.pan[NES_AUDIO_DMC] = 101;
    char error[160];
    CHECK(!nes_audio_mix_set(&invalid, error, sizeof(error)) && error[0]);
    nes_audio_mix_get(&actual);
    CHECK(!memcmp(&actual, &settings, sizeof(settings)));
    invalid = settings;
    invalid.volume[NES_AUDIO_VRC7] = 201;
    CHECK(!nes_audio_mix_set(&invalid, error, sizeof(error)));
    invalid = settings;
    invalid.master_volume = 101;
    CHECK(!nes_audio_mix_set(&invalid, error, sizeof(error)));
    nes_audio_mix_defaults(&settings);
    CHECK(nes_audio_mix_set(&settings, NULL, 0) && nes_audio_mix_channels_default());
    CHECK(!nes_audio_mix_has_panning());
    CHECK(unload_rom());
    return 0;
}

static int test_base_panning_master_and_capture(void) {
    NesAudioMixSettings settings;
    nes_audio_mix_defaults(&settings);
    for (int pan = -100; pan <= 100; pan += 100) {
        CHECK(load_mix_machine(false, false));
        settings.pan[NES_AUDIO_PULSE1] = pan;
        CHECK(nes_audio_mix_set(&settings, NULL, 0));
        pulse_tone();
        CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
        double left = energy(samples, 0, MIX_FRAMES), right = energy(samples, 1, MIX_FRAMES);
        if (pan < 0) CHECK(left > 0.01 && right == 0.0);
        else if (pan > 0) CHECK(right > 0.01 && left == 0.0);
        else {
            CHECK(left > 0.01 && right == left);
            for (unsigned i = 0; i < MIX_FRAMES; ++i) CHECK(samples[i * 2] == samples[i * 2 + 1]);
        }
    }

    CHECK(load_mix_machine(false, false));
    settings.pan[NES_AUDIO_PULSE1] = -100;
    settings.muted = true;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    observed_energy[0] = observed_energy[1] = 0;
    probe_token = nes_audio_observer_add(observe_sample, NULL);
    CHECK(probe_token != 0);
    pulse_tone();
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(energy(samples, 0, MIX_FRAMES) == 0 && energy(samples, 1, MIX_FRAMES) == 0);
    CHECK(observed_energy[0] > 0.01 && observed_energy[1] == 0);
    CHECK(nes_audio_observer_remove(probe_token));
    probe_token = 0;

    CHECK(load_mix_machine(false, false));
    settings.muted = false;
    settings.volume[NES_AUDIO_PULSE1] = 0;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    pulse_tone();
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(energy(samples, 0, MIX_FRAMES) == 0 && energy(samples, 1, MIX_FRAMES) == 0);
    CHECK(apu.pulse1.enabled && apu.pulse1.lc.length != 0);
    CHECK(unload_rom());
    return 0;
}

static int test_stereo_reconstruction_round_trip(void) {
    NesAudioMixSettings settings, restored;
    nes_audio_mix_defaults(&settings);
    settings.pan[NES_AUDIO_PULSE1] = 35;
    settings.volume[NES_AUDIO_PULSE1] = 75;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    CHECK(load_mix_machine(false, false));
    pulse_tone();
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(energy(samples, 0, MIX_FRAMES) > 0.01 && energy(samples, 1, MIX_FRAMES) > 0.01);
    CHECK(energy(samples, 0, MIX_FRAMES) != energy(samples, 1, MIX_FRAMES));
    NesStateBlob saved = {0}, expected = {0}, actual = {0};
    CHECK(nes_state_capture(&saved) == NES_STATE_OK);
    CHECK(collect_stereo(expected_samples, MIX_FRAMES) == 0);
    CHECK(nes_state_capture(&expected) == NES_STATE_OK);
    CHECK(nes_state_restore(saved.data, saved.size) == NES_STATE_OK);
    nes_audio_mix_get(&restored);
    CHECK(!memcmp(&restored, &settings, sizeof(settings)));
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(!memcmp(samples, expected_samples, sizeof(samples)));
    CHECK(nes_state_capture(&actual) == NES_STATE_OK);
    CHECK(actual.size == expected.size && !memcmp(actual.data, expected.data, actual.size));
    nes_state_blob_free(&saved);
    nes_state_blob_free(&expected);
    nes_state_blob_free(&actual);
    CHECK(unload_rom());
    return 0;
}

static int load_expansion(unsigned chip) {
    uint8_t image[0x180] = {0};
    memcpy(image, "NESM\x1A", 5);
    image[5] = 1;
    image[6] = image[7] = 1;
    image[9] = image[11] = image[13] = 0x80;
    image[12] = 0x10;
    image[110] = image[120] = 0xE8;
    image[111] = image[121] = 3;
    image[123] = (uint8_t)chip;
    image[0x80] = image[0x90] = 0x60;
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    if (chip == NSF_SOUND_MMC5) cart_cpu_write(0x5011, 0x20);
    else if (chip == NSF_SOUND_VRC6) {
        cart_cpu_write(0x9000, 0x8F);
        cart_cpu_write(0x9001, 0);
        cart_cpu_write(0x9002, 0x80);
    } else if (chip == NSF_SOUND_VRC7) {
        cart_cpu_write(0x9010, 0x30); cart_cpu_write(0x9030, 0x10);
        cart_cpu_write(0x9010, 0x10); cart_cpu_write(0x9030, 0x80);
        cart_cpu_write(0x9010, 0x20); cart_cpu_write(0x9030, 0x15);
    } else if (chip == NSF_SOUND_SUNSOFT5B) {
        cart_cpu_write(0xC000, 8); cart_cpu_write(0xE000, 0x0F);
        cart_cpu_write(0xC000, 7); cart_cpu_write(0xE000, 0x3F);
    } else if (chip == NSF_SOUND_FDS) {
        cart_cpu_write(0x4089, 0x80);
        for (uint16_t address = 0x4040; address <= 0x407F; ++address) cart_cpu_write(address, 0x3F);
        cart_cpu_write(0x4089, 0); cart_cpu_write(0x4080, 0xBF);
        cart_cpu_write(0x4082, 0xFF); cart_cpu_write(0x4083, 0x0F);
    } else if (chip == NSF_SOUND_NAMCO163) {
        cart_cpu_write(0xF800, 0x80); cart_cpu_write(0x4800, 0xFF);
        const uint8_t registers[][2] = {{0x78,1},{0x7A,0},{0x7C,0},{0x7E,0},{0x7F,15}};
        for (unsigned i = 0; i < sizeof(registers) / sizeof(registers[0]); ++i) {
            cart_cpu_write(0xF800, registers[i][0]);
            cart_cpu_write(0x4800, registers[i][1]);
        }
    }
    for (unsigned i = 0; i < 10000 && fabsf(cart_expansion_audio()) <= 0.0001f; ++i)
        cart_clock_cpu_cycle(false);
    CHECK(fabsf(cart_expansion_audio()) > 0.0001f);
    return 0;
}

static int test_expansion_group_volume(void) {
    const unsigned chips[] = {NSF_SOUND_MMC5, NSF_SOUND_VRC6, NSF_SOUND_VRC7,
                               NSF_SOUND_SUNSOFT5B, NSF_SOUND_FDS, NSF_SOUND_NAMCO163};
    const unsigned channels[] = {NES_AUDIO_MMC5, NES_AUDIO_VRC6, NES_AUDIO_VRC7,
                                  NES_AUDIO_SUNSOFT5B, NES_AUDIO_FDS, NES_AUDIO_NAMCO163};
    for (unsigned i = 0; i < sizeof(chips) / sizeof(chips[0]); ++i) {
        NesAudioMixSettings settings;
        nes_audio_mix_defaults(&settings);
        CHECK(nes_audio_mix_set(&settings, NULL, 0));
        CHECK(load_expansion(chips[i]) == 0);
        float original = cart_expansion_audio(), separated[NES_AUDIO_CHANNEL_COUNT];
        cart_expansion_audio_channels(separated);
        CHECK(separated[channels[i]] == original);
        for (unsigned channel = 0; channel < NES_AUDIO_CHANNEL_COUNT; ++channel) {
            if (channel != channels[i]) CHECK(separated[channel] == 0);
            settings.volume[channel] = 0;
        }
        CHECK(nes_audio_mix_set(&settings, NULL, 0));
        apu_audio_refresh(&apu);
        CHECK(apu.reconstructed_level == 0 && apu.right_output.level == 0);
        CHECK(cart_expansion_audio() == original);
        settings.volume[channels[i]] = 100;
        settings.pan[channels[i]] = -100;
        CHECK(nes_audio_mix_set(&settings, NULL, 0));
        apu_audio_refresh(&apu);
        CHECK(apu.reconstructed_level != 0 && apu.right_output.level == 0);
        CHECK(cart_expansion_audio() == original);
    }
    CHECK(unload_rom());
    return 0;
}

static void epsm_register(uint8_t reg, uint8_t value) {
    write_mem(0x401C, reg);
    write_mem(0x401D, value);
}

static void epsm_left_tone(void) {
    epsm_register(0x29, 0x83);
    for (unsigned op = 0; op < 4; ++op) {
        uint8_t offset = (uint8_t)(op * 4);
        epsm_register((uint8_t)(0x30 + offset), 1);
        epsm_register((uint8_t)(0x40 + offset), 0x18);
        epsm_register((uint8_t)(0x50 + offset), 0x1F);
        epsm_register((uint8_t)(0x60 + offset), 0);
        epsm_register((uint8_t)(0x70 + offset), 0);
        epsm_register((uint8_t)(0x80 + offset), 0x0F);
    }
    epsm_register(0xB0, 7);
    epsm_register(0xB4, 0x80);
    epsm_register(0xA4, 0x22);
    epsm_register(0xA0, 0x69);
    epsm_register(0x28, 0xF0);
}

static int test_epsm_and_dual_stereo(void) {
    NesAudioMixSettings settings;
    nes_audio_mix_defaults(&settings);
    settings.volume[NES_AUDIO_EPSM] = 50;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    CHECK(load_mix_machine(true, false) && epsm_enabled());
    epsm_left_tone();
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(energy(samples, 0, MIX_FRAMES) > 0.001 && energy(samples, 1, MIX_FRAMES) == 0);
    settings.volume[NES_AUDIO_EPSM] = 0;
    CHECK(nes_audio_mix_set(&settings, NULL, 0));
    apu_audio_init_state(&apu, 44100);
    CHECK(collect_stereo(samples, MIX_FRAMES) == 0);
    CHECK(energy(samples, 0, MIX_FRAMES) == 0 && energy(samples, 1, MIX_FRAMES) == 0);

    CHECK(load_mix_machine(false, true) && vs_dual_system());
    APU *first = vs_side_apu(0), *second = vs_side_apu(1);
    first->ring[0] = 0.25f; first->ring_side[0] = 0.125f;
    second->ring[0] = -0.5f; second->ring_side[0] = -0.125f;
    atomic_store(&first->ring_r, 0); atomic_store(&first->ring_w, 1);
    atomic_store(&second->ring_r, 0); atomic_store(&second->ring_w, 1);
    float output[2] = {1, 1};
    uint64_t before = cpu_total_cycles;
    vs_audio_stereo_callback(NULL, (uint8_t *)output, sizeof(output));
    CHECK(output[0] == -0.125f && output[1] == -0.125f);
    CHECK(atomic_load(&first->ring_r) == 1 && atomic_load(&second->ring_r) == 1);
    CHECK(cpu_total_cycles == before);
    CHECK(unload_rom());
    return 0;
}

int test_audio_mix_accuracy(void) {
    NesAudioMixSettings previous;
    nes_audio_mix_get(&previous);
    NesRegionMode region = nes_region_mode();
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    int failures = test_settings_and_hardware_invariance()
                 + test_base_panning_master_and_capture()
                 + test_stereo_reconstruction_round_trip()
                 + test_expansion_group_volume()
                 + test_epsm_and_dual_stereo();
    if (probe_token) (void)nes_audio_observer_remove(probe_token);
    probe_token = 0;
    if (!nes_audio_mix_set(&previous, NULL, 0)) ++failures;
    if (!nes_set_region_mode(region)) ++failures;
    printf("Audio presentation: 5 groups, %d failures\n", failures);
    return failures;
}
