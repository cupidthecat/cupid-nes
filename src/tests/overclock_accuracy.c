/*
 * overclock_accuracy.c - Extra scanline hardware and state regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../system/timing.h"
#include "../system/hardware.h"
#include "../system/execution_policy.h"
#include "../rom/rom.h"
#include "../rom/game_db.h"
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../state/state.h"
#include "../replay/rewind.h"
#include "../ui/overclock_frontend.h"
#include "../ui/frontend_panels.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static unsigned checks;
static uint8_t image[16 + 0x8000 + 0x2000];
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        ++checks;                                                                                                      \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool machine(NesRegion region, NesOverclockConfig config, unsigned mapper) {
    unload_rom();
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1a", 4);
    image[4] = 2;
    image[5] = 1;
    image[6] = (uint8_t)(mapper << 4);
    image[7] = (uint8_t)(mapper & 0xf0);
    memset(image + 16, 0xea, 0x8000);
    image[16] = 0x4c;
    image[17] = 0;
    image[18] = 0x80;
    image[16 + 0x100] = 0xe6;
    image[16 + 0x101] = 0;
    image[16 + 0x102] = 0x40;
    image[16 + 0x110] = 0xe6;
    image[16 + 0x111] = 1;
    image[16 + 0x112] = 0x40;
    image[16 + 0x7ffa] = 0;
    image[16 + 0x7ffb] = 0x81;
    image[16 + 0x7ffc] = 0;
    image[16 + 0x7ffd] = 0x80;
    image[16 + 0x7ffe] = 0x10;
    image[16 + 0x7fff] = 0x81;
    if (!nes_set_overclock_config(&config) || load_rom_memory(image, sizeof(image))) {
        return false;
    }
    nes_set_region(region);
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) {
        return false;
    }
    ppu_reg_write(PPUCTRL, 0x80);
    while (ppu.scanline != 0) {
        if (cpu_step(&cpu) <= 0) {
            return false;
        }
    }
    return true;
}

static bool frame(void) {
    uint64_t target = ppu.frame_count + 1;
    for (unsigned i = 0; i < 300000; ++i) {
        if (cpu_step(&cpu) <= 0) {
            return false;
        }
        if (ppu.frame_count == target) {
            return true;
        }
    }
    return false;
}

static bool core_bytes(NesStateWriter *writer) {
    nes_state_writer_init(writer, NES_STATE_MAX_SIZE);
    return cpu_state_capture(writer) && ppu_state_capture(writer) && apu_state_capture(writer);
}

static int defaults(void) {
    NesStateWriter baseline, current, timing;
    for (unsigned region = 0; region <= NES_REGION_DENDY; ++region) {
        CHECK(machine((NesRegion)region, (NesOverclockConfig){false, 0, 0, true}, 0));
        CHECK(frame());
        CHECK(core_bytes(&baseline));
        nes_state_writer_init(&timing, 100);
        CHECK(timing_state_capture(&timing) && timing.size == 2);
        nes_state_writer_destroy(&timing);
        CHECK(machine((NesRegion)region, (NesOverclockConfig){false, 500, 800, true}, 0));
        CHECK(frame());
        CHECK(core_bytes(&current));
        CHECK(current.size == baseline.size && !memcmp(current.data, baseline.data, baseline.size));
        nes_state_writer_destroy(&current);
        CHECK(machine((NesRegion)region, (NesOverclockConfig){true, 0, 0, false}, 0));
        CHECK(frame());
        CHECK(core_bytes(&current));
        CHECK(current.size == baseline.size && !memcmp(current.data, baseline.data, baseline.size));
        nes_state_writer_destroy(&baseline);
        nes_state_writer_destroy(&current);
    }
    return 0;
}

static int intervals(void) {
    for (unsigned region = 0; region <= NES_REGION_DENDY; ++region) {
        for (unsigned after = 0; after < 2; ++after) {
            for (unsigned render = 0; render < 2; ++render) {
                NesOverclockConfig config = {true, after ? 0 : 7, after ? 11 : 0, false};
                CHECK(machine((NesRegion)region, config, 0));
                /* Dot stepping measures exact NMI and frame boundaries independently
                 * of the instruction that contains either boundary. */
                ppu.scanline = 0;
                ppu.dot = 0;
                ppu.status = 0;
                ppu.nmi_out = false;
                ppu.odd_frame = true;
                ppu.mask = render ? 0x18 : 0;
                ppu.rendering_enabled = ppu.fetches_enabled = render != 0;
                uint64_t origin = ppu.total_cycles, nmi = 0;
                uint64_t target = ppu.frame_count + 1;
                unsigned edges = 0;
                bool previous = false;
                while (ppu.frame_count != target && ppu.total_cycles - origin < 200000) {
                    ppu_step_dots(1);
                    if (ppu.nmi_out && !previous) {
                        ++edges;
                        nmi = ppu.total_cycles - origin;
                    }
                    previous = ppu.nmi_out;
                }
                CHECK(ppu.frame_count == target);
                CHECK(edges == 1);
                CHECK(nmi == (nes_timing()->vblank_scanline + config.postrender_scanlines) * 341u + 2u);
                CHECK(ppu.total_cycles - origin ==
                      (nes_timing()->scanlines + config.postrender_scanlines + config.vblank_scanlines) * 341u -
                          (render && region == NES_REGION_NTSC ? 1u : 0u));
                CHECK(nes_timing()->cpu_divider == nes_timing_for_region((NesRegion)region)->cpu_divider);
            }
        }
    }
    return 0;
}

static int audio_and_cpu(void) {
    uint64_t ordinary_cycles = 0;
    unsigned ordinary_samples = 0;
    for (unsigned enabled = 0; enabled < 2; ++enabled) {
        CHECK(machine(NES_REGION_NTSC, (NesOverclockConfig){enabled != 0, 90, 90, false}, 0));
        apu_audio_init(44100);
        write_mem(0x4015, 1);
        write_mem(0x4000, 0xbf);
        write_mem(0x4002, 0x80);
        write_mem(0x4003, 8);
        uint64_t origin = cpu_total_cycles;
        CHECK(frame());
        unsigned samples = atomic_load(&apu.ring_w);
        if (!enabled) {
            ordinary_cycles = cpu_total_cycles - origin;
            ordinary_samples = samples;
        } else {
            CHECK(cpu_total_cycles - origin >= ordinary_cycles + 20457 &&
                  cpu_total_cycles - origin <= ordinary_cycles + 20463);
            CHECK(abs((int)samples - (int)ordinary_samples) <= 1);
            CHECK(cpu_peek_internal_ram(0) == 1);
            CHECK(apu.cpu_cycle_odd == ((cpu_total_cycles & 1u) != 0));
        }
        float output[2048];
        apu_audio_pull(&apu, output, 2048);
        double energy = 0;
        for (unsigned i = 0; i < 2048; ++i) {
            CHECK(isfinite(output[i]) && fabsf(output[i]) < 4.0f);
            energy += output[i] * output[i];
        }
        CHECK(energy > 0.001);
    }
    return 0;
}

static bool enter_extra(void) {
    for (unsigned i = 0; i < 100000; ++i) {
        if (nes_overclock_extra_active()) {
            return true;
        }
        if (cpu_step(&cpu) <= 0) {
            return false;
        }
    }
    return false;
}

static int dma_and_irq(void) {
    for (unsigned region = 0; region <= NES_REGION_DENDY; ++region) {
        for (unsigned after = 0; after < 2; ++after) {
            CHECK(machine((NesRegion)region, (NesOverclockConfig){true, after ? 0 : 100, after ? 100 : 0, false}, 0));
            CHECK(enter_extra());
            uint32_t sequence = apu.cycle_in_seq;
            uint16_t timer = apu.dmc.timer;
            for (unsigned i = 0; i < 256; ++i) {
                write_mem((uint16_t)(0x200 + i), (uint8_t)i);
            }
            write_mem(OAMADDR, 0);
            write_mem(0x4014, 2);
            uint64_t before = cpu_total_cycles;
            CHECK(cpu_step(&cpu) > 0);
            CHECK(cpu_total_cycles - before >= 516 && cpu_total_cycles - before <= 517);
            uint8_t oam[256];
            ppu_debug_copy_oam(oam);
            for (unsigned i = 0; i < 256; ++i) {
                CHECK(oam[i] == (uint8_t)(i % 4 == 2 ? i & 0xe3u : i));
            }
            CHECK(apu.cycle_in_seq == sequence);
            CHECK(abs((int)apu.dmc.timer - (int)timer) <= 1);
            write_mem(0x4010, 0x8f);
            write_mem(0x4012, 0);
            write_mem(0x4013, 0);
            write_mem(0x4015, 0x10);
            for (unsigned i = 0; i < 8; ++i) {
                CHECK(cpu_step(&cpu) > 0);
            }
            CHECK(apu.dmc.bytes_remaining == 0 && apu.dmc.irq_flag && !apu.dmc.dma_pending);
            CHECK(apu.cycle_in_seq == sequence);
        }
    }
    CHECK(machine(NES_REGION_NTSC, (NesOverclockConfig){true, 100, 100, false}, 24));
    CHECK(enter_extra());
    cart_cpu_write(0xf000, 0xff);
    cart_cpu_write(0xf001, 6);
    CHECK(!cart_irq_pending());
    CHECK(cpu_step(&cpu) > 0);
    CHECK(cart_irq_pending());
    /* Expansion timers still receive every master clock during blank time. */
    EpsmDevice *device = epsm_create();
    CHECK(device != NULL);
    epsm_activate(device);
    uint64_t clocks = epsm_clock_count();
    uint64_t cpu_origin = cpu_total_cycles;
    CHECK(cpu_step(&cpu) > 0);
    double expected = (double)(cpu_total_cycles - cpu_origin) * EPSM_CLOCK_RATE / nes_timing()->cpu_hz;
    CHECK(fabs((double)(epsm_clock_count() - clocks) - expected) < 1.0);
    epsm_activate(NULL);
    return 0;
}

static int switching_and_states(void) {
    CHECK(machine(NES_REGION_NTSC, (NesOverclockConfig){true, 50, 60, false}, 0));
    CHECK(enter_extra());
    unsigned dots = nes_overclock_extra_dots();
    NesOverclockConfig off = {false, 0, 0, true};
    CHECK(nes_set_overclock_config(&off));
    CHECK(nes_overclock_active_config().enabled && nes_overclock_extra_dots() == dots);
    NesStateBlob initial = {0}, first = {0}, second = {0};
    CHECK(nes_state_capture(&initial) == NES_STATE_OK);
    for (unsigned i = 0; i < 100; ++i) {
        CHECK(cpu_step(&cpu) > 0);
    }
    CHECK(nes_state_capture(&first) == NES_STATE_OK);
    CHECK(nes_state_restore(initial.data, initial.size) == NES_STATE_OK);
    CHECK(nes_overclock_extra_dots() == dots && !nes_overclock_config().enabled);
    for (unsigned i = 0; i < 100; ++i) {
        CHECK(cpu_step(&cpu) > 0);
    }
    CHECK(nes_state_capture(&second) == NES_STATE_OK);
    CHECK(first.size == second.size && !memcmp(first.data, second.data, first.size));
    nes_state_blob_free(&initial);
    nes_state_blob_free(&first);
    nes_state_blob_free(&second);
    NesRewindHistory history;
    nes_rewind_init(&history);
    CHECK(nes_rewind_configure(&history, 2, 8 * 1024 * 1024));
    CHECK(nes_rewind_capture(&history, NULL) == NES_REPLAY_OK);
    unsigned saved = nes_overclock_extra_dots();
    CHECK(frame());
    CHECK(!nes_overclock_active_config().enabled);
    CHECK(nes_rewind_step(&history, NULL) == NES_REPLAY_OK);
    CHECK(nes_overclock_active_config().enabled && nes_overclock_extra_dots() == saved);
    nes_rewind_destroy(&history);
    uint8_t legacy[] = {NES_REGION_NTSC, NES_REGION_MODE_AUTO};
    NesStateReader reader;
    nes_state_reader_init(&reader, legacy, sizeof(legacy));
    CHECK(timing_state_apply(&reader));
    CHECK(!nes_overclock_config().enabled && !nes_overclock_extra_active());
    return 0;
}

static int state_validation(void) {
    NesOverclockConfig invalid = {true, 1001, 0, false};
    CHECK(!nes_set_overclock_config(&invalid));
    CHECK(!nes_set_overclock_config(NULL));
    CHECK(machine(NES_REGION_NTSC, (NesOverclockConfig){true, 50, 60, false}, 0));
    CHECK(enter_extra());
    NesStateWriter writer;
    nes_state_writer_init(&writer, 100);
    CHECK(timing_state_capture(&writer) && writer.size == 20);
    NesStateReader reader;
    for (size_t length = 0; length < writer.size; ++length) {
        nes_state_reader_init(&reader, writer.data, length);
        CHECK(timing_state_validate(&reader) == (length == 2));
    }
    uint8_t original = writer.data[2];
    writer.data[2] = 2;
    nes_state_reader_init(&reader, writer.data, writer.size);
    CHECK(!timing_state_apply(&reader));
    CHECK(nes_overclock_extra_active() && nes_overclock_config().postrender_scanlines == 50);
    writer.data[2] = original;
    writer.data[4] = 0xe9;
    writer.data[5] = 3;
    nes_state_reader_init(&reader, writer.data, writer.size);
    CHECK(!timing_state_validate(&reader));
    nes_state_writer_destroy(&writer);
    return 0;
}

static int compatibility_and_panel(void) {
    CHECK(machine(NES_REGION_NTSC, (NesOverclockConfig){true, 20, 30, true}, 0));
    write_mem(0x4011, 32);
    CHECK(frame());
    CHECK(!nes_overclock_active_config().enabled);
    CHECK(frame());
    CHECK(nes_overclock_active_config().enabled);
    write_mem(0x4010, 0x4f);
    write_mem(0x4013, 0xff);
    write_mem(0x4015, 0x10);
    CHECK(frame());
    CHECK(!nes_overclock_active_config().enabled);
    FrontendOverclock frontend = {0};
    CHECK(frontend_overclock_register(&frontend));
    FrontendPanelControl controls[5];
    FrontendPanelModel model = {.controls = controls, .capacity = 5};
    char error[160];
    CHECK(frontend_panel_snapshot(0x2c00, &model, error, sizeof(error)) && model.count == 5);
    CHECK(!frontend_panel_action(0x2c00, 0x2c02, "1001", 0, error, sizeof(error)));
    CHECK(!frontend_panel_action(0x2c00, 0x2c02, "-1", 0, error, sizeof(error)));
    CHECK(frontend_panel_action(0x2c00, 0x2c02, "1000", 0, error, sizeof(error)));
    CHECK(nes_overclock_config().postrender_scanlines == 1000);
    CHECK(nes_execution_set_policy(NES_EXECUTION_NETPLAY));
    CHECK(!frontend_panel_action(0x2c00, 0x2c03, "1", 0, error, sizeof(error)));
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(frontend_panel_action(0x2c00, 0x2c05, NULL, 0, error, sizeof(error)));
    CHECK(!nes_overclock_config().enabled && !nes_overclock_config().postrender_scanlines);
    frontend_overclock_unregister();
    return 0;
}

int test_overclock_accuracy(void) {
    NesOverclockConfig saved = nes_overclock_config();
    NesRegion region = nes_timing()->region;
    NesRegionMode mode = nes_region_mode();
    bool restriction = ppu_startup_write_restriction_enabled();
    bool database = rom_database_overrides_enabled();
    NesRamPowerOnState ram_state = nes_ram_power_on_state();
    nes_set_ram_power_on_state(NES_RAM_POWER_ZERO);
    nes_set_region_mode(NES_REGION_MODE_AUTO);
    rom_database_set_overrides(false);
    ppu_set_startup_write_restriction(false);
    cpu_use_default_startup_alignment();
    checks = 0;
    int failures = defaults() + intervals() + audio_and_cpu() + dma_and_irq() + switching_and_states() +
                   state_validation() + compatibility_and_panel();
    nes_execution_set_policy(NES_EXECUTION_LIVE);
    frontend_overclock_unregister();
    unload_rom();
    nes_set_overclock_config(&saved);
    nes_overclock_begin_frame(true, false);
    nes_set_region(region);
    nes_set_region_mode(mode);
    nes_set_ram_power_on_state(ram_state);
    ppu_set_startup_write_restriction(restriction);
    rom_database_set_overrides(database);
    printf("Overclock: %u checks, %d failures\n", checks, failures);
    return failures;
}
