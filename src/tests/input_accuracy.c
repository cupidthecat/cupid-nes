/*
 * input_accuracy.c - Console input hardware regression tests
 *
 * Author: @frankischilling
 *
 * These tests exercise controller wiring and read clocks through the CPU
 * bus, including DMA, microphone input, and machine selection across reset.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif
#include "../joypad/family_basic.h"
#include "../joypad/special_peripherals.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../system/hardware.h"
#include "../system/timing.h"

extern uint8_t ram[0x0800];

static uint8_t input_memory[0x10000];
static uint64_t dmc_request_cycle;
static unsigned input_checks;

#define CHECK(condition) do { \
    ++input_checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint8_t input_cpu_read(uint16_t address) {
    return input_memory[address];
}

static void input_cpu_write(uint16_t address, uint8_t value) {
    input_memory[address] = value;
}

static uint8_t input_ppu_read(uint16_t address) {
    return input_memory[address & 0x1FFFu];
}

static void input_ppu_write(uint16_t address, uint8_t value) {
    input_memory[address & 0x1FFFu] = value;
}

static void input_clock(int cycles) {
    (void)cycles;
    if (dmc_request_cycle && cpu_total_cycles == dmc_request_cycle) {
        apu.dmc.sample_buffer_empty = true;
        apu.dmc.dma_pending = true;
        apu.dmc.start_delay = 0;
    }
}

static Mapper input_mapper = {
    .cpu_read = input_cpu_read,
    .cpu_write = input_cpu_write,
    .ppu_read = input_ppu_read,
    .ppu_write = input_ppu_write,
    .clock = input_clock
};

static void input_fixture(NesConsoleModel model, NesRegion region) {
    unload_rom();
    memset(input_memory, 0, sizeof(input_memory));
    memset(ram, 0, 0x0800);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        memset(joypad_player(player), 0, sizeof(Joypad));
    joypad_set_adapter(NES_ADAPTER_NONE);
    joypad_set_port_device(0, NES_PORT_GAMEPAD);
    joypad_set_port_device(1, NES_PORT_GAMEPAD);
    joypad_set_expansion_device(NES_EXPANSION_NONE);
    joypad_set_configuration_overrides(0);
    joypad_set_zapper_radius(0);
    for (unsigned slot = 0; slot < 3; ++slot) joypad_set_zapper(slot, -1, -1, false);
    for (unsigned slot = 0; slot < 3; ++slot)
        for (unsigned pad = 0; pad < 12; ++pad) joypad_set_mat_pad(slot, pad, false);
    nes_set_console_model(model);
    nes_set_region(region);
    joypad_set_microphone(false);
    dmc_request_cycle = 0;
    cart = &input_mapper;
    input_memory[0xFFFD] = 0x80;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    apu.frame_reset_pending = false;
    apu.frame_reset_delay = 0;
    apu.cycle_in_seq = 0;
}

static void input_program(uint8_t opcode, uint16_t address) {
    input_memory[0x8000] = opcode;
    input_memory[0x8001] = (uint8_t)address;
    input_memory[0x8002] = (uint8_t)(address >> 8);
    input_memory[0x8003] = 0xEA;
}

static void latch_controllers(void) {
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
}

#include "input_accuracy_ports.h"
#include "input_accuracy_devices.h"

int test_input_accuracy(void) {
    static int (*const tests[])(void) = {
        console_open_bus, console_read_clocks, console_strobe_timing,
        famicom_microphone, console_dma_reads, console_selection_lifetime, default_input_metadata,
        adapter_reports, adapter_strobes_and_disconnect, adapter_cpu_and_dma_clocks,
        adapter_selection_lifetime, extended_port_protocols, fcns_controller_protocol,
        arkanoid_reports, arkanoid_latching,
        arkanoid_cpu_clocks, device_selection, power_pad_button_order,
        power_pad_latching, family_trainer_rows, family_trainer_cpu_writes,
        mat_selection_and_disconnect, zapper_port_signals, zapper_beam_and_persistence,
        zapper_brightness_and_area, zapper_cpu_and_dma, zapper_selection_and_disconnect,
        family_basic_matrix_scan, family_basic_cpu_tape_and_controller,
        family_basic_recording_and_media, turbo_file_protocol,
        turbo_file_persistence, turbo_file_failed_save, battle_box_protocol,
        battle_box_persistence, battle_box_failed_save, subor_keyboard_matrix,
        subor_mouse_packets, hori_track_reports, konami_hyper_shot_signals,
        bandai_hyper_shot_signals, party_tap_reports, pachinko_reports,
        exciting_boxing_signals, jissen_mahjong_rows, barcode_battler_stream,
        barcode_battler_lifetime, oeka_kids_tablet_reports, oeka_kids_cartridge_and_tablet
    };
    NesConsoleModel saved_model = nes_console_model();
    NesRegion saved_region = nes_timing()->region;
    NesInputAdapter saved_adapter = joypad_adapter();
    NesPortDevice saved_ports[] = {joypad_port_device(0), joypad_port_device(1)};
    NesExpansionDevice saved_expansion = joypad_expansion_device();
    uint8_t saved_overrides = joypad_configuration_overrides();
    unsigned saved_zapper_radius = joypad_zapper_radius();
    int failures = 0;
    input_checks = 0;
    for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    family_basic_shutdown();
    nes_set_console_model(saved_model);
    nes_set_region(saved_region);
    joypad_set_microphone(false);
    joypad_set_adapter(saved_adapter);
    joypad_set_port_device(0, saved_ports[0]);
    joypad_set_port_device(1, saved_ports[1]);
    joypad_set_expansion_device(saved_expansion);
    joypad_set_configuration_overrides(saved_overrides);
    joypad_set_zapper_radius(saved_zapper_radius);
    printf("Input: %u checks, %d failures\n", input_checks, failures);
    return failures;
}
