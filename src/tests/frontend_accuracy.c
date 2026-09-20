/*
 * frontend_accuracy.c - Desktop frontend execution regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../ui/execution_control.h"
#include "../ui/frontend_commands.h"
#include "../ui/machine_actions.h"
#include "../ui/frontend_panels.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned frontend_checks;

#define CHECK(condition) do { \
    ++frontend_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint8_t image[16 + 0x8000 + 0x2000];

static void build_nrom(uint8_t region) {
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = 2;
    image[5] = 1;
    image[7] = 8;
    image[12] = region;
    memset(image + 16, 0xEA, 0x8000);
    image[16] = 0x4C;
    image[17] = 0x00;
    image[18] = 0x80;
    image[16 + 0x7FFA] = 0x00;
    image[16 + 0x7FFB] = 0x80;
    image[16 + 0x7FFC] = 0x00;
    image[16 + 0x7FFD] = 0x80;
    image[16 + 0x7FFE] = 0x00;
    image[16 + 0x7FFF] = 0x80;
}

static bool run_machine_frame(void *userdata) {
    (void)userdata;
    vs_start_frame();
    while (!ppu.frame_complete) vs_cpu_step();
    return true;
}

static int execution_gate_regions(void) {
    static const NesRegionMode modes[] = {
        NES_REGION_MODE_NTSC, NES_REGION_MODE_PAL, NES_REGION_MODE_DENDY
    };
    static const uint8_t regions[] = {0, 1, 3};
    for (unsigned i = 0; i < 3; ++i) {
        unload_rom();
        CHECK(nes_set_region_mode(modes[i]));
        build_nrom(regions[i]);
        CHECK(load_rom_memory(image, sizeof(image)) == 0);
        CHECK(frontend_machine_power_cycle());

        ExecutionControl control;
        execution_control_init(&control);
        execution_control_set_paused(&control, true);
        uint64_t frame = ppu.frame_count;
        uint64_t cycles = cpu_total_cycles;
        CHECK(!execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame && cpu_total_cycles == cycles);
        CHECK(execution_control_request_frame(&control));
        CHECK(execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame + 1 && cpu_total_cycles > cycles);
        cycles = cpu_total_cycles;
        CHECK(control.paused && !control.frame_advance_pending);
        CHECK(!execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(cpu_total_cycles == cycles);

        const NesTiming before = *nes_timing();
        CHECK(execution_control_set_speed(&control, 0.5));
        CHECK(execution_control_effective_speed(&control) == 0.5);
        execution_control_set_fast_forward_held(&control, true);
        CHECK(execution_control_effective_speed(&control) == 4.0);
        execution_control_set_fast_forward_held(&control, false);
        execution_control_toggle_fast_forward(&control);
        CHECK(execution_control_fast_forward_active(&control));
        CHECK(nes_timing()->region == before.region);
        CHECK(nes_timing()->cpu_hz == before.cpu_hz && nes_timing()->fps == before.fps);
        execution_control_toggle_fast_forward(&control);
        execution_control_set_paused(&control, false);
        frame = ppu.frame_count;
        CHECK(execution_control_run_frame(&control, run_machine_frame, NULL));
        CHECK(ppu.frame_count == frame + 1);
    }
    return 0;
}

static int lifecycle_actions(void) {
    unload_rom();
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    build_nrom(0);
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(frontend_machine_power_cycle());
    write_mem(0x0010, 0xA7);
    CHECK(read_mem(0x0010) == 0xA7);
    CHECK(frontend_machine_soft_reset());
    CHECK(read_mem(0x0010) == 0xA7);
    CHECK(frontend_machine_power_cycle());
    CHECK(read_mem(0x0010) == 0x00);
    return 0;
}

typedef struct {
    unsigned calls;
    bool result;
} CommandProbe;

static bool probe_command(void *userdata, char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    CommandProbe *probe = (CommandProbe *)userdata;
    ++probe->calls;
    return probe->result;
}

static int command_registry(void) {
    frontend_commands_reset();
    CommandProbe probe = {.result = true};
    FrontendCommandSpec spec = {
        .id = FRONTEND_COMMAND_EXTENSION_BASE + 7,
        .label = "Probe",
        .menu = "Tools",
        .shortcut = "Ctrl+Q",
        .flags = FRONTEND_COMMAND_CHECKABLE,
        .handler = probe_command,
        .userdata = &probe
    };
    CHECK(frontend_command_register(&spec));
    CHECK(!frontend_command_register(&spec));
    CHECK(frontend_command_count() == 1);
    FrontendCommandInfo info;
    CHECK(frontend_command_get(spec.id, &info));
    CHECK(strcmp(info.label, "Probe") == 0 && strcmp(info.menu, "Tools") == 0);
    CHECK(frontend_command_set_checked(spec.id, true));
    CHECK(frontend_command_set_enabled(spec.id, false));
    char error[80];
    CHECK(!frontend_command_invoke(spec.id, error, sizeof(error)));
    CHECK(probe.calls == 0 && strstr(error, "unavailable") != NULL);
    CHECK(frontend_command_set_enabled(spec.id, true));
    CHECK(frontend_command_invoke(spec.id, error, sizeof(error)));
    CHECK(probe.calls == 1);
    CHECK(frontend_command_unregister(spec.id));
    CHECK(frontend_command_count() == 0);
    return 0;
}

static bool panel_snapshot_probe(void *userdata, FrontendPanelModel *model,
                                 char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    unsigned *calls = (unsigned *)userdata;
    ++*calls;
    static const char *const choices[] = {"One", "Two"};
    FrontendPanelControl text = {
        .id = 1, .type = FRONTEND_PANEL_TEXT, .label = "Value",
        .value = "ready", .enabled = true, .read_only = true
    };
    FrontendPanelControl choice = {
        .id = 2, .type = FRONTEND_PANEL_CHOICE, .label = "Mode",
        .items = choices, .item_count = 2, .selected = 1, .enabled = true
    };
    return frontend_panel_add_control(model, &text)
        && frontend_panel_add_control(model, &choice);
}

static bool panel_action_probe(void *userdata, unsigned control_id,
                               const char *value, int selected,
                               char *error, size_t error_size) {
    (void)value;
    (void)error;
    (void)error_size;
    unsigned *calls = (unsigned *)userdata;
    ++*calls;
    return control_id == 2 && selected == 0;
}

static int panel_registry(void) {
    frontend_panels_reset();
    unsigned calls = 0;
    FrontendPanelSpec spec = {
        .id = FRONTEND_PANEL_EXTENSION_BASE + 1,
        .title = "Probe Panel",
        .category = "Tools",
        .snapshot = panel_snapshot_probe,
        .action = panel_action_probe,
        .userdata = &calls
    };
    CHECK(frontend_panel_register(&spec));
    FrontendPanelControl controls[4];
    FrontendPanelModel model = {.controls = controls, .capacity = 4};
    char error[80];
    CHECK(frontend_panel_snapshot(spec.id, &model, error, sizeof(error)));
    CHECK(model.count == 2 && calls == 1);
    CHECK(model.controls[0].type == FRONTEND_PANEL_TEXT);
    CHECK(model.controls[1].type == FRONTEND_PANEL_CHOICE);
    CHECK(frontend_panel_action(spec.id, 2, NULL, 0, error, sizeof(error)));
    CHECK(calls == 2);
    CHECK(frontend_panel_unregister(spec.id));
    return 0;
}

int test_frontend_accuracy(void) {
    const NesRegionMode saved_mode = nes_region_mode();
    const NesRegion saved_region = nes_timing()->region;
    const NesRamPowerOnState saved_ram = nes_ram_power_on_state();
    int failures = 0;
    frontend_checks = 0;
    failures += execution_gate_regions();
    failures += lifecycle_actions();
    failures += command_registry();
    failures += panel_registry();
    unload_rom();
    frontend_commands_reset();
    nes_set_region_mode(saved_mode);
    nes_set_region(saved_region);
    nes_set_ram_power_on_state(saved_ram);
    printf("Frontend execution: %u checks, %d failures\n", frontend_checks, failures);
    return failures;
}
