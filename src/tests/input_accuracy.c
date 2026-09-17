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
#include <string.h>
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
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
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
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

static int console_open_bus(void) {
    static const uint8_t masks[4][2] = {
        {0xE0, 0xE0}, {0xE4, 0xE0}, {0xF8, 0xE0}, {0xF8, 0xE0}
    };
    static const uint8_t previous[] = {0x00, 0xFF, 0xA5, 0x5A, 0xE4, 0x18};
    for (unsigned model = 0; model < 4; ++model) {
        for (unsigned port = 0; port < 2; ++port) {
            for (unsigned bus = 0; bus < sizeof(previous); ++bus) {
                for (unsigned serial = 0; serial < 2; ++serial) {
                    input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
                    pad1.buttons = pad2.buttons = (uint8_t)serial;
                    latch_controllers();
                    write_mem(0x4018, previous[bus]);
                    CHECK(read_mem((uint16_t)(0x4016 + port)) ==
                          (uint8_t)((previous[bus] & masks[model][port]) | serial));
                }
            }
        }
    }
    return 0;
}

static int console_read_clocks(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    for (unsigned model = 0; model < 4; ++model) {
        for (unsigned region = 0; region < 3; ++region) {
            input_fixture((NesConsoleModel)model, regions[region]);
            pad2.buttons = 0x02;
            latch_controllers();
            // Indexed RMW supplies two adjacent reads. Its writes target the
            // frame counter at $4017, so they cannot re-latch the controllers.
            input_program(0x1E, 0x4017);
            CHECK(cpu_step(&cpu) == 7);
            CHECK(joypad_read(&pad2) ==
                  (model == NES_CONSOLE_HVC001 && regions[region] == NES_REGION_NTSC ? 0 : 1));

            for (unsigned port = 0; port < 2; ++port) {
                input_fixture((NesConsoleModel)model, regions[region]);
                pad1.buttons = pad2.buttons = 0x02;
                latch_controllers();
                input_program(0xAD, (uint16_t)(0x4016 + port));
                memcpy(input_memory + 0x8003, input_memory + 0x8000, 3);
                CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x40);
                CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x41);
            }

            input_fixture((NesConsoleModel)model, regions[region]);
            pad1.buttons = 0x02;
            pad2.buttons = 0x01;
            latch_controllers();
            input_program(0x6C, 0x4016); // JMP ($4016) reads the two ports consecutively.
            CHECK(cpu_step(&cpu) == 5 && cpu.pc == 0x4140);
            CHECK(joypad_read(&pad1) == 1 && joypad_read(&pad2) == 0);
        }
    }
    return 0;
}

static int console_strobe_timing(void) {
    for (unsigned model = 0; model < 4; ++model) {
        for (unsigned phase = 0; phase < 2; ++phase) {
            input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
            cpu_total_cycles = phase;
            pad1.shift = 0xFF;
            input_program(0xCE, 0x4016); // DEC emits high then low OUT writes.
            CHECK(cpu_step(&cpu) == 6);
            CHECK(cpu_step(&cpu) == 2);
            CHECK(pad1.strobe == 0);
            CHECK(pad1.shift == (phase ? 0xFF : 0x00));
        }
        input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
        write_mem(0x4016, 1);
        input_program(0xAD, 0x4016);
        memcpy(input_memory + 0x8003, input_memory + 0x8000, 3);
        CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x40);
        joypad_set(&pad1, BTN_A, 1);
        CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x41);
    }
    return 0;
}

static int famicom_microphone(void) {
    for (unsigned model = 0; model < 4; ++model) {
        input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
        pad1.buttons = 0x01;
        pad2.buttons = 0x02;
        latch_controllers();
        joypad_set_microphone(true);
        write_mem(0x4018, 0xA0);
        CHECK(read_mem(0x4016) == (model == NES_CONSOLE_HVC001 ? 0xA5 : 0xA1));
        CHECK(read_mem(0x4017) == 0xA0);
        CHECK(joypad_read(&pad2) == 1);
        joypad_set_microphone(false);
        write_mem(0x4018, 0xA0);
        CHECK(read_mem(0x4016) == 0xA0);
    }
    return 0;
}

static int console_dma_reads(void) {
    static const uint8_t masks[] = {0xE0, 0xE4, 0xF8, 0xF8};
    for (unsigned model = 0; model < 4; ++model) {
        for (unsigned phase = 0; phase < 2; ++phase) {
            input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
            cpu_total_cycles = phase;
            pad1.buttons = 0x12; // B and Up expose the deleted report positions.
            latch_controllers();
            input_program(0xAD, 0x4016);
            input_memory[0xC016] = 0xFF;
            apu.dmc.current_addr = 0xC016;
            apu.dmc.bytes_remaining = 1;
            apu.dmc.enabled = true;
            apu.dmc.sample_buffer_empty = false;
            apu.dmc.timer = 1000;
            dmc_request_cycle = phase + 3;
            CHECK(cpu_step(&cpu) == (phase ? 7 : 8));
            CHECK(!apu_dmc_dma_pending(&apu));
            CHECK(apu.dmc.sample_buffer == masks[model]);
            uint8_t serial = model == NES_CONSOLE_HVC001 && !phase ? 1 : 0;
            CHECK(cpu.a == (uint8_t)(masks[model] | serial));
            CHECK(joypad_read(&pad1) == (model == NES_CONSOLE_HVC001 ? phase : 1));
        }

        input_fixture((NesConsoleModel)model, NES_REGION_NTSC);
        pad1.buttons = 1;
        latch_controllers();
        for (unsigned i = 0; i < 256; ++i) ram[0x0200 + i] = (uint8_t)i;
        input_program(0xAD, 0x4016);
        write_mem(0x4014, 2);
        CHECK(cpu_step(&cpu) == 518 && cpu.a == 0x41);
        CHECK(joypad_read(&pad1) == 0);
        CHECK(ppu.oam[0] == 0 && ppu.oam[0x81] == 0x81 && ppu.oam[0xFF] == 0xFF);
    }
    return 0;
}

static int console_selection_lifetime(void) {
    static const char *const names[] = {"nes-001", "nes-101", "famicom", "av-famicom"};
    static uint8_t image[16 + 0x4000];
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = 1;
    image[7] = 0x08;
    image[11] = 7; // NES 2.0: 8KB CHR-RAM.
    image[12] = 1; // PAL timing must not replace the selected console wiring.
    image[16 + 0x3FFD] = 0x80;

    for (unsigned model = 0; model < 4; ++model) {
        input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
        CHECK(nes_set_console_model_name(names[model]));
        CHECK(nes_console_model() == (NesConsoleModel)model);
        CHECK(strcmp(nes_console_model_name(), names[model]) == 0);
        CHECK(!nes_set_console_model((NesConsoleModel)-1));
        CHECK(!nes_set_console_model((NesConsoleModel)4));
        CHECK(!nes_set_console_model_name(NULL));
        CHECK(!nes_set_console_model_name("unknown"));
        CHECK(nes_console_model() == (NesConsoleModel)model);

        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(nes_console_model() == (NesConsoleModel)model);
        CHECK(load_rom_memory(image, sizeof(image)) == 0);
        CHECK(nes_timing()->region == NES_REGION_PAL);
        CHECK(nes_console_model() == (NesConsoleModel)model);
        Mapper *loaded_cart = cart;
        CHECK(load_rom_memory(image, 8) == -1 && cart == loaded_cart);
        CHECK(nes_console_model() == (NesConsoleModel)model);
        unload_rom();
        CHECK(nes_console_model() == (NesConsoleModel)model);
    }
    return 0;
}

int test_input_accuracy(void) {
    static int (*const tests[])(void) = {
        console_open_bus, console_read_clocks, console_strobe_timing,
        famicom_microphone, console_dma_reads, console_selection_lifetime
    };
    NesConsoleModel saved_model = nes_console_model();
    NesRegion saved_region = nes_timing()->region;
    int failures = 0;
    input_checks = 0;
    for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    nes_set_console_model(saved_model);
    nes_set_region(saved_region);
    joypad_set_microphone(false);
    printf("Input: %u checks, %d failures\n", input_checks, failures);
    return failures;
}
