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
#include <time.h>
#include "../joypad/family_basic.h"
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
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        memset(joypad_player(player), 0, sizeof(Joypad));
    joypad_set_adapter(NES_ADAPTER_NONE);
    joypad_set_port_device(0, NES_PORT_GAMEPAD);
    joypad_set_port_device(1, NES_PORT_GAMEPAD);
    joypad_set_expansion_device(NES_EXPANSION_NONE);
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

static int adapter_reports(void) {
    static const uint8_t buttons[] = {0xA5, 0x5A, 0x3C, 0xC3, 0x96, 0x69};
    static const uint32_t nes_reports[] = {0x083CA5, 0x04C35A};
    static const uint32_t famicom_reports[] = {0x04963C, 0x0869C3};
    for (unsigned adapter = NES_ADAPTER_FOUR_SCORE; adapter <= NES_ADAPTER_FAMICOM_FOUR; ++adapter) {
        input_fixture(adapter == NES_ADAPTER_FOUR_SCORE ? NES_CONSOLE_NES001 : NES_CONSOLE_HVC001,
                      NES_REGION_NTSC);
        CHECK(joypad_set_adapter((NesInputAdapter)adapter));
        for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
            joypad_player(player)->buttons = buttons[player];
        latch_controllers();
        // Button changes after the falling edge must not replace the latched packet.
        for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
            joypad_player(player)->buttons = (uint8_t)~buttons[player];
        for (unsigned bit = 0; bit < 32; ++bit) {
            for (unsigned port = 0; port < 2; ++port) {
                uint8_t expected;
                if (adapter == NES_ADAPTER_FOUR_SCORE) {
                    expected = bit < 24 ? (uint8_t)((nes_reports[port] >> bit) & 1u) : 1;
                } else {
                    uint8_t built_in = bit < 8 ? (uint8_t)((buttons[port] >> bit) & 1u) : 1;
                    uint8_t expansion;
                    if (adapter == NES_ADAPTER_FAMICOM_TWO)
                        expansion = bit < 8 ? (uint8_t)((buttons[port + 2] >> bit) & 1u) : 1;
                    else
                        expansion = bit < 24 ? (uint8_t)((famicom_reports[port] >> bit) & 1u) : 1;
                    expected = built_in | (expansion << 1);
                }
                write_mem(0x4018, 0xA0);
                CHECK(read_mem((uint16_t)(0x4016 + port)) == (uint8_t)(0xA0 | expected));
            }
        }
    }
    return 0;
}

static int adapter_strobes_and_disconnect(void) {
    for (unsigned adapter = NES_ADAPTER_FOUR_SCORE; adapter <= NES_ADAPTER_FAMICOM_FOUR; ++adapter) {
        input_fixture(adapter == NES_ADAPTER_FOUR_SCORE ? NES_CONSOLE_NES001 : NES_CONSOLE_HVC001,
                      NES_REGION_NTSC);
        CHECK(joypad_set_adapter((NesInputAdapter)adapter));
        write_mem(0x4016, 1);
        for (unsigned pressed = 0; pressed < 2; ++pressed) {
            for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
                CHECK(joypad_set_player(player, BTN_A, pressed != 0));
            for (unsigned read = 0; read < 28; ++read) {
                write_mem(0x4018, 0);
                CHECK(read_mem(0x4016) == (pressed ? (adapter == NES_ADAPTER_FOUR_SCORE ? 1 : 3) : 0));
            }
        }
        CHECK(!joypad_set_adapter((NesInputAdapter)4));
        CHECK(joypad_adapter() == (NesInputAdapter)adapter);
        CHECK(!joypad_set_player(NES_INPUT_PLAYERS, BTN_A, true));
        CHECK(!joypad_set_player(0, -1, true));
        CHECK(!joypad_set_player(0, 8, true));
        CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
        pad1.buttons = 0x81;
        pad2.buttons = 0x18;
        latch_controllers();
        for (unsigned bit = 0; bit < 8; ++bit) {
            write_mem(0x4018, 0);
            CHECK(read_mem(0x4016) == ((0x81u >> bit) & 1u));
            CHECK(read_mem(0x4017) == ((0x18u >> bit) & 1u));
        }
    }
    return 0;
}

static int adapter_cpu_and_dma_clocks(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    joypad_set_adapter(NES_ADAPTER_FOUR_SCORE);
    pad2.buttons = 0x02;
    latch_controllers();
    input_program(0x1E, 0x4017);
    CHECK(cpu_step(&cpu) == 7);
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 1);

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    joypad_set_adapter(NES_ADAPTER_FAMICOM_FOUR);
    pad2.buttons = 0x02;
    joypad_player(3)->buttons = 0x04;
    latch_controllers();
    input_program(0x1E, 0x4017);
    CHECK(cpu_step(&cpu) == 7);
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 2);

    for (unsigned famicom = 0; famicom < 2; ++famicom) {
        input_fixture(famicom ? NES_CONSOLE_HVC001 : NES_CONSOLE_NES001, NES_REGION_NTSC);
        joypad_set_adapter(famicom ? NES_ADAPTER_FAMICOM_FOUR : NES_ADAPTER_FOUR_SCORE);
        pad1.buttons = 0x12;
        joypad_player(2)->buttons = 0x22;
        latch_controllers();
        input_program(0xAD, 0x4016);
        input_memory[0xC016] = 0xFF;
        apu.dmc.current_addr = 0xC016;
        apu.dmc.bytes_remaining = 1;
        apu.dmc.enabled = true;
        apu.dmc.sample_buffer_empty = false;
        apu.dmc.timer = 1000;
        dmc_request_cycle = 3;
        CHECK(cpu_step(&cpu) == 8);
        CHECK(apu.dmc.sample_buffer == (famicom ? 0xF8 : 0xE0));
        CHECK(cpu.a == (famicom ? 0xF9 : 0xE0));
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4016) == (famicom ? 2 : 1));
    }
    return 0;
}

static int adapter_selection_lifetime(void) {
    static const char *const names[] = {"none", "four-score", "famicom-2", "famicom-4"};
    for (unsigned adapter = 0; adapter < 4; ++adapter) {
        input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
        CHECK(joypad_set_adapter_name(names[adapter]));
        CHECK(joypad_adapter() == (NesInputAdapter)adapter);
        CHECK(strcmp(joypad_adapter_name(), names[adapter]) == 0);
        CHECK(!joypad_set_adapter_name(NULL) && !joypad_set_adapter_name("unknown"));
        cpu_soft_reset(&cpu);
        CHECK(joypad_adapter() == (NesInputAdapter)adapter);
        unload_rom();
        CHECK(joypad_adapter() == (NesInputAdapter)adapter);
    }
    return 0;
}

static int arkanoid_reports(void) {
    static const int positions[] = {0x54, 0xA4, 0xF4};
    static const uint8_t reports[] = {0xAB, 0x5B, 0x0B};
    for (unsigned console = 0; console < 2; ++console) {
        for (unsigned port = 0; port < 2; ++port) {
            for (unsigned position = 0; position < 3; ++position) {
                for (unsigned fire = 0; fire < 2; ++fire) {
                    input_fixture(console ? NES_CONSOLE_NES101 : NES_CONSOLE_NES001, NES_REGION_NTSC);
                    CHECK(joypad_set_port_device(port, NES_PORT_ARKANOID));
                    CHECK(joypad_set_paddle(port, positions[position], fire != 0));
                    latch_controllers();
                    for (unsigned bit = 0; bit < 12; ++bit) {
                        uint8_t serial = bit < 8 ? (reports[position] >> (7 - bit)) & 1u : 1;
                        uint8_t bus = console && !port ? 0xE4 : 0xE0;
                        write_mem(0x4018, 0xE4);
                        CHECK(read_mem((uint16_t)(0x4016 + port)) ==
                              (uint8_t)(bus | (serial << 4) | (fire << 3)));
                    }
                }
            }
        }
    }

    for (unsigned position = 0; position < 3; ++position) {
        for (unsigned fire = 0; fire < 2; ++fire) {
            input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
            joypad_set_expansion_device(NES_EXPANSION_ARKANOID);
            joypad_set_paddle(2, positions[position], fire != 0);
            pad1.buttons = 0x69;
            pad2.buttons = 0x96;
            latch_controllers();
            for (unsigned bit = 0; bit < 12; ++bit) {
                uint8_t serial = bit < 8 ? (reports[position] >> (7 - bit)) & 1u : 1;
                uint8_t first = bit < 8 ? (0x69u >> bit) & 1u : 1;
                uint8_t second = bit < 8 ? (0x96u >> bit) & 1u : 1;
                write_mem(0x4018, 0xA0);
                CHECK(read_mem(0x4016) == (uint8_t)(0xA0 | first | (fire << 1)));
                CHECK(read_mem(0x4017) == (uint8_t)(0xA0 | second | (serial << 1)));
            }
        }
    }
    return 0;
}

static int arkanoid_latching(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    joypad_set_port_device(1, NES_PORT_ARKANOID);
    joypad_set_paddle(1, -1, false); // Clamp to the left stop, report $AB.
    latch_controllers();
    for (unsigned bit = 0; bit < 4; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (((0xABu >> (7 - bit)) & 1u) << 4));
    }
    joypad_set_paddle(1, 999, true);
    for (unsigned bit = 4; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (0x08 | (((0xABu >> (7 - bit)) & 1u) << 4)));
    }
    // Reads with strobe high do not reload the Arkanoid position register.
    write_mem(0x4016, 1);
    for (unsigned bit = 0; bit < 3; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == 0x18);
    }
    write_mem(0x4016, 0);
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (0x08 | (((0x0Bu >> (7 - bit)) & 1u) << 4)));
    }
    joypad_set_paddle(1, 0xA4, false);
    latch_controllers();
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 0);
    write_mem(0x4016, 0); // Repeated low writes must not re-latch the position.
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 0x10);
    write_mem(0x4016, 0);
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 0);

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    joypad_set_expansion_device(NES_EXPANSION_ARKANOID);
    joypad_set_paddle(2, 0x54, false);
    latch_controllers();
    for (unsigned read = 0; read < 20; ++read) {
        write_mem(0x4018, 0);
        CHECK((read_mem(0x4016) & 2u) == 0);
    }
    joypad_set_paddle(2, 0xF4, true);
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4016) & 2u) == 2);
    CHECK((read_mem(0x4017) & 2u) == 2); // Button-port reads did not consume the old $54 report.
    return 0;
}

static int arkanoid_cpu_clocks(void) {
    for (unsigned famicom = 0; famicom < 2; ++famicom) {
        input_fixture(famicom ? NES_CONSOLE_HVC001 : NES_CONSOLE_NES001, NES_REGION_NTSC);
        if (famicom) joypad_set_expansion_device(NES_EXPANSION_ARKANOID);
        else joypad_set_port_device(1, NES_PORT_ARKANOID);
        joypad_set_paddle(famicom ? 2 : 1, 0xA4, false);
        latch_controllers();
        input_program(0x1E, 0x4017);
        CHECK(cpu_step(&cpu) == 7);
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (famicom ? 0 : 0x10));
    }
    for (unsigned phase = 0; phase < 2; ++phase) {
        input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
        joypad_set_port_device(1, NES_PORT_ARKANOID);
        joypad_set_paddle(1, 0x54, false);
        latch_controllers();
        joypad_set_paddle(1, 0xF4, false);
        pad1.shift = 0xFF;
        cpu_total_cycles = phase;
        input_program(0xCE, 0x4016);
        CHECK(cpu_step(&cpu) == 6);
        CHECK(cpu_step(&cpu) == 2);
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (phase ? 0x10 : 0));
    }
    return 0;
}

static int device_selection(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    CHECK(joypad_set_port_device_name(0, "arkanoid"));
    CHECK(joypad_port_device(0) == NES_PORT_ARKANOID);
    CHECK(strcmp(joypad_port_device_name(0), "arkanoid") == 0);
    CHECK(!joypad_set_port_device_name(0, NULL));
    CHECK(!joypad_set_port_device_name(0, "unknown"));
    CHECK(!joypad_set_port_device(2, NES_PORT_GAMEPAD));
    CHECK(!joypad_set_port_device(0, (NesPortDevice)999));
    CHECK(!joypad_set_paddle(3, 0xA4, false));
    CHECK(joypad_set_expansion_device_name("arkanoid"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_ARKANOID);
    CHECK(strcmp(joypad_expansion_device_name(), "arkanoid") == 0);
    CHECK(!joypad_set_expansion_device_name(NULL));
    CHECK(!joypad_set_expansion_device_name("unknown"));
    CHECK(!joypad_set_expansion_device((NesExpansionDevice)999));
    CHECK(joypad_configuration_valid());
    joypad_set_adapter(NES_ADAPTER_FAMICOM_TWO);
    CHECK(!joypad_configuration_valid());
    joypad_set_expansion_device(NES_EXPANSION_NONE);
    CHECK(joypad_configuration_valid());
    joypad_set_adapter(NES_ADAPTER_FOUR_SCORE);
    CHECK(!joypad_configuration_valid());
    CHECK(joypad_set_port_device_name(0, "pad"));
    CHECK(joypad_configuration_valid());
    joypad_set_adapter(NES_ADAPTER_NONE);
    pad1.buttons = 0x81;
    latch_controllers();
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4016) == ((0x81u >> bit) & 1u));
    }
    CHECK(joypad_set_port_device_name(0, "none"));
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4016) == 0);
    return 0;
}

static int power_pad_button_order(void) {
    static const uint8_t low_masks[2][12] = {
        {2, 1, 0, 0, 4, 0x10, 0x80, 0, 8, 0x20, 0x40, 0},
        {0, 0, 1, 2, 0, 0x80, 0x10, 4, 0, 0x40, 0x20, 8}
    };
    static const uint8_t high_masks[2][12] = {
        {0, 0, 2, 1, 0, 0, 0, 8, 0, 0, 0, 4},
        {1, 2, 0, 0, 8, 0, 0, 0, 4, 0, 0, 0}
    };
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned port = 0; port < 2; ++port) {
            for (unsigned pad = 0; pad < 12; ++pad) {
                input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
                CHECK(joypad_set_port_device(port, side ? NES_PORT_POWER_PAD_B : NES_PORT_POWER_PAD_A));
                CHECK(joypad_set_mat_pad(port, pad, true));
                latch_controllers();
                joypad_set_mat_pad(port, pad, false);
                for (unsigned bit = 0; bit < 12; ++bit) {
                    uint8_t low = bit < 8 ? (low_masks[side][pad] >> bit) & 1u : 1;
                    uint8_t high = bit < 8 ? ((0xF0 | high_masks[side][pad]) >> bit) & 1u : 1;
                    write_mem(0x4018, 0xA0);
                    CHECK(read_mem((uint16_t)(0x4016 + port)) == (uint8_t)(0xA0 | (low << 3) | (high << 4)));
                }
            }
        }
    }
    return 0;
}

static int power_pad_latching(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    joypad_set_port_device(1, NES_PORT_POWER_PAD_A);
    write_mem(0x4016, 1);
    for (unsigned buttons = 0; buttons < 4; ++buttons) {
        joypad_set_mat_pad(1, 1, (buttons & 1u) != 0);
        joypad_set_mat_pad(1, 3, (buttons & 2u) != 0);
        for (unsigned read = 0; read < 12; ++read) {
            write_mem(0x4018, 0);
            CHECK(read_mem(0x4017) == ((buttons & 1u) << 3 | (buttons & 2u) << 3));
        }
    }
    for (unsigned pad = 0; pad < 12; ++pad) joypad_set_mat_pad(1, pad, true);
    write_mem(0x4016, 0);
    for (unsigned pad = 0; pad < 12; ++pad) joypad_set_mat_pad(1, pad, false);
    for (unsigned bit = 0; bit < 12; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == 0x18);
    }
    latch_controllers();
    for (unsigned bit = 0; bit < 12; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == (bit < 4 ? 0 : bit < 8 ? 0x10 : 0x18));
    }

    joypad_set_mat_pad(1, 1, true);
    latch_controllers();
    for (unsigned read = 0; read < 24; ++read) (void)read_mem(0x4016);
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 0x08);
    CHECK(read_mem(0x4017) == 0);

    latch_controllers();
    input_program(0x1E, 0x4017); // One report bit is clocked across the held read line.
    CHECK(cpu_step(&cpu) == 7);
    write_mem(0x4018, 0);
    CHECK(read_mem(0x4017) == 0);
    return 0;
}

static int family_trainer_rows(void) {
    static const uint8_t columns[2][4] = {{0x10, 0x08, 0x04, 0x02}, {0x02, 0x04, 0x08, 0x10}};
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned pad = 0; pad < 12; ++pad) {
            input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
            CHECK(joypad_set_expansion_device(side ? NES_EXPANSION_FAMILY_TRAINER_B
                                                   : NES_EXPANSION_FAMILY_TRAINER_A));
            CHECK(joypad_set_mat_pad(2, pad, true));
            for (unsigned ignore = 0; ignore < 8; ++ignore) {
                write_mem(0x4016, (uint8_t)(0xF8 | ignore));
                write_mem(0x4018, 0xA0);
                CHECK((read_mem(0x4016) & 0xFEu) == 0xA0);
                uint8_t expected = 0x1E;
                if (!(ignore & (1u << (2 - pad / 4)))) expected ^= columns[side][pad % 4];
                CHECK((read_mem(0x4017) & 0xFEu) == (uint8_t)(0xA0 | expected));
            }
        }
    }

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    joypad_set_expansion_device(NES_EXPANSION_FAMILY_TRAINER_A);
    write_mem(0x4016, 0);
    joypad_set_mat_pad(2, 0, true);
    joypad_set_mat_pad(2, 4, true); // Two rows share a column; the result stays asserted.
    joypad_set_mat_pad(2, 10, true);
    write_mem(0x4018, 0);
    CHECK((read_mem(0x4017) & 0x1E) == 0x0A);
    joypad_set_mat_pad(2, 0, false);
    CHECK((read_mem(0x4017) & 0x1E) == 0x0A);
    joypad_set_mat_pad(2, 4, false);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1A);
    joypad_set_mat_pad(2, 10, false);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E);

    pad1.buttons = 0xA5;
    pad2.buttons = 0x5A;
    latch_controllers();
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4016) == ((0xA5u >> bit) & 1u));
        CHECK(read_mem(0x4017) == (0x1E | ((0x5Au >> bit) & 1u)));
    }
    return 0;
}

static int family_trainer_cpu_writes(void) {
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned phase = 0; phase < 2; ++phase) {
            input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
            joypad_set_expansion_device(side ? NES_EXPANSION_FAMILY_TRAINER_B
                                             : NES_EXPANSION_FAMILY_TRAINER_A);
            joypad_set_mat_pad(2, 3, true);
            joypad_set_mat_pad(2, 4, true);
            cpu_total_cycles = phase;
            input_program(0x8D, 0x4016);
            input_memory[0x8003] = 0xAD;
            input_memory[0x8004] = 0x17;
            input_memory[0x8005] = 0x40;
            cpu.a = 5; // Only the middle row is selected.
            CHECK(cpu_step(&cpu) == 4);
            CHECK(cpu_step(&cpu) == 4 && cpu.a == (side ? 0x5C : 0x4E));
        }
    }
    return 0;
}

static int mat_selection_and_disconnect(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    CHECK(joypad_set_port_device_name(1, "power-pad-a"));
    CHECK(joypad_port_device(1) == NES_PORT_POWER_PAD_A);
    CHECK(joypad_set_port_device_name(1, "power-pad-b"));
    CHECK(joypad_port_device(1) == NES_PORT_POWER_PAD_B);
    CHECK(joypad_set_expansion_device_name("family-trainer-a"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_A);
    CHECK(joypad_set_expansion_device_name("family-trainer-b"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_B);
    CHECK(!joypad_set_mat_pad(3, 0, true));
    CHECK(!joypad_set_mat_pad(0, 12, true));
    for (unsigned slot = 0; slot < 3; ++slot)
        for (unsigned pad = 0; pad < 12; ++pad) joypad_set_mat_pad(slot, pad, true);
    joypad_set_port_device(1, NES_PORT_GAMEPAD);
    joypad_set_expansion_device(NES_EXPANSION_NONE);
    pad2.buttons = 0xA5;
    latch_controllers();
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == ((0xA5u >> bit) & 1u));
    }
    return 0;
}

static void sensor_pixel(unsigned x, unsigned y, uint8_t color) {
    ppu.scanline = (int)y;
    ppu.dot = (int)x + 1;
    ppu.mask = 0;
    ppu.rendering_enabled = false;
    ppu.fetches_enabled = false;
    ppu.v = 0;
    ppu_write(0x3F00, color);
    ppu_step_dots(1);
}

static int zapper_port_signals(void) {
    for (unsigned console = 0; console < 2; ++console) {
        for (unsigned port = 0; port < 2; ++port) {
            input_fixture(console ? NES_CONSOLE_NES101 : NES_CONSOLE_NES001, NES_REGION_NTSC);
            CHECK(joypad_set_port_device(port, NES_PORT_ZAPPER));
            CHECK(joypad_set_zapper(port, 32, 20, false));
            uint8_t floating = console && !port ? 0xE4 : 0xE0;
            ppu.scanline = 20;
            ppu.dot = 33;
            write_mem(0x4018, 0xE4);
            CHECK(read_mem((uint16_t)(0x4016 + port)) == (uint8_t)(floating | 0x08));
            sensor_pixel(32, 20, 0x20);
            write_mem(0x4018, 0xE4);
            CHECK(read_mem((uint16_t)(0x4016 + port)) == floating);
            joypad_set_zapper(port, 32, 20, true);
            for (unsigned read = 0; read < 16; ++read) {
                write_mem(0x4016, (uint8_t)(read & 1u));
                write_mem(0x4018, 0xE4);
                CHECK(read_mem((uint16_t)(0x4016 + port)) == (uint8_t)(floating | 0x10));
            }
            joypad_set_zapper(port, -1, -1, true);
            write_mem(0x4018, 0xE4);
            CHECK(read_mem((uint16_t)(0x4016 + port)) == (uint8_t)(floating | 0x18));
        }
    }

    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    joypad_set_expansion_device(NES_EXPANSION_ZAPPER);
    joypad_set_zapper(2, 32, 20, true);
    pad1.buttons = 0xA5;
    pad2.buttons = 0x5A;
    latch_controllers();
    sensor_pixel(32, 20, 0x20);
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0xA0);
        CHECK(read_mem(0x4016) == (uint8_t)(0xA0 | ((0xA5u >> bit) & 1u)));
        CHECK(read_mem(0x4017) == (uint8_t)(0xB0 | ((0x5Au >> bit) & 1u)));
    }
    return 0;
}

static int zapper_beam_and_persistence(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    for (unsigned region = 0; region < 3; ++region) {
        input_fixture(NES_CONSOLE_NES001, regions[region]);
        joypad_set_port_device(1, NES_PORT_ZAPPER);
        joypad_set_zapper(1, 32, 20, false);
        sensor_pixel(32, 20, 0x20);
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == 0);
        ppu_step_dots(20 * 341);
        CHECK(ppu.scanline == 40 && read_mem(0x4017) == 0);
        ppu_step_dots(341);
        CHECK(ppu.scanline == 41 && read_mem(0x4017) == 8);

        // The old bright pixel must not be detected ahead of the current beam.
        ppu.scanline = 19;
        ppu.dot = 340;
        CHECK(read_mem(0x4017) == 8);
        ppu_step_dots(2);
        CHECK(ppu.scanline == 20 && ppu.dot == 1 && read_mem(0x4017) == 8);
        ppu_step_dots(32);
        CHECK(ppu.dot == 33 && read_mem(0x4017) == 8);
        ppu_step_dots(1);
        CHECK(ppu.dot == 34 && read_mem(0x4017) == 0);
        ppu.scanline = (int)nes_timing()->scanlines - 1;
        CHECK(read_mem(0x4017) == 8);
    }
    return 0;
}

static int zapper_brightness_and_area(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    joypad_set_port_device(1, NES_PORT_ZAPPER);
    joypad_set_zapper(1, 32, 20, false);
    sensor_pixel(32, 20, 0x09);
    write_mem(0x4018, 0);
    CHECK(ppu_pixel_brightness(32, 20) == 83 && read_mem(0x4017) == 8);
    sensor_pixel(32, 20, 0x0B);
    CHECK(ppu_pixel_brightness(32, 20) == 87 && read_mem(0x4017) == 0);
    sensor_pixel(32, 20, 0x0F);
    CHECK(ppu_pixel_brightness(32, 20) == 0 && read_mem(0x4017) == 8);
    ppu.mask = 1;
    CHECK(ppu_pixel_brightness(32, 20) == 306 && read_mem(0x4017) == 0);
    ppu.mask = 0;
    sensor_pixel(32, 20, 0x20);
    joypad_set_zapper(1, 33, 20, false);
    CHECK(read_mem(0x4017) == 8);
    CHECK(joypad_set_zapper_radius(1));
    CHECK(read_mem(0x4017) == 0);
    joypad_set_zapper(1, 34, 20, false);
    CHECK(read_mem(0x4017) == 8);
    CHECK(joypad_set_zapper_radius(2));
    CHECK(read_mem(0x4017) == 0);
    joypad_set_zapper(1, -1, 20, false);
    CHECK(read_mem(0x4017) == 8);
    joypad_set_zapper(1, 0x7FFFFFFF, 0x7FFFFFFF, true);
    CHECK(read_mem(0x4017) == 0x18);
    CHECK(!joypad_set_zapper_radius(NES_ZAPPER_MAX_RADIUS + 1));
    CHECK(joypad_zapper_radius() == 2);
    CHECK(!joypad_set_zapper(3, 0, 0, false));
    CHECK(ppu_pixel_brightness(256, 0) == 0 && ppu_pixel_brightness(0, 240) == 0);

    sensor_pixel(0, 0, 0x20);
    joypad_set_zapper(1, 0, 0, false);
    CHECK(read_mem(0x4017) == 0);
    ppu_power_on(&ppu);
    CHECK(read_mem(0x4017) == 8);
    return 0;
}

static int zapper_cpu_and_dma(void) {
    for (unsigned expansion = 0; expansion < 2; ++expansion) {
        for (unsigned phase = 0; phase < 2; ++phase) {
            input_fixture(expansion ? NES_CONSOLE_HVC001 : NES_CONSOLE_NES001, NES_REGION_NTSC);
            if (expansion) joypad_set_expansion_device(NES_EXPANSION_ZAPPER);
            else joypad_set_port_device(1, NES_PORT_ZAPPER);
            joypad_set_zapper(expansion ? 2 : 1, 32, 20, true);
            sensor_pixel(32, 20, 0x20);
            input_program(0xAD, 0x4017);
            CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x50);

            cpu_total_cycles = phase;
            cpu.pc = 0x8000;
            input_memory[0xC017] = 0xFF;
            apu.dmc.current_addr = 0xC017;
            apu.dmc.bytes_remaining = 1;
            apu.dmc.enabled = true;
            apu.dmc.sample_buffer_empty = false;
            apu.dmc.timer = 1000;
            dmc_request_cycle = phase + 3;
            CHECK(cpu_step(&cpu) == (phase ? 7 : 8));
            CHECK(apu.dmc.sample_buffer == 0xF0 && cpu.a == 0xF0);
            CHECK(!apu_dmc_dma_pending(&apu));
        }
    }
    return 0;
}

static int zapper_selection_and_disconnect(void) {
    input_fixture(NES_CONSOLE_NES001, NES_REGION_NTSC);
    CHECK(joypad_set_port_device_name(1, "zapper"));
    CHECK(joypad_port_device(1) == NES_PORT_ZAPPER);
    CHECK(joypad_set_expansion_device_name("zapper"));
    CHECK(joypad_expansion_device() == NES_EXPANSION_ZAPPER);
    joypad_set_zapper(1, -1, -1, true);
    joypad_set_zapper(2, -1, -1, true);
    CHECK(joypad_set_port_device_name(1, "pad"));
    CHECK(joypad_set_expansion_device_name("none"));
    pad2.buttons = 0xA5;
    latch_controllers();
    for (unsigned bit = 0; bit < 8; ++bit) {
        write_mem(0x4018, 0);
        CHECK(read_mem(0x4017) == ((0xA5u >> bit) & 1u));
    }
    return 0;
}

static int family_basic_matrix_scan(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device_name("family-basic"));
    CHECK(joypad_configuration_valid());
    static const FamilyBasicKey held[] = {
        FB_KEY_RETURN, FB_KEY_KANA, FB_KEY_F7, FB_KEY_CARET,
        FB_KEY_O, FB_KEY_COMMA, FB_KEY_U, FB_KEY_M,
        FB_KEY_Y, FB_KEY_G, FB_KEY_B, FB_KEY_R, FB_KEY_F, FB_KEY_4,
        FB_KEY_A, FB_KEY_S, FB_KEY_X, FB_KEY_E, FB_KEY_CONTROL,
        FB_KEY_LEFT_SHIFT, FB_KEY_2, FB_KEY_CLEAR_HOME, FB_KEY_DOWN, FB_KEY_INSERT
    };
    static const uint8_t rows[10][2] = {
        {0x1A, 0x1C}, {0x1C, 0x0E}, {0x1A, 0x1A}, {0x16, 0x1C},
        {0x12, 0x1C}, {0x16, 0x0C}, {0x06, 0x14}, {0x0E, 0x0C},
        {0x1C, 0x0C}, {0x1E, 0x1E}
    };
    for (unsigned i = 0; i < sizeof(held) / sizeof(held[0]); ++i)
        CHECK(family_basic_set_key(held[i], true));
    CHECK(!family_basic_set_key(FB_KEY_COUNT, true));
    CHECK(!family_basic_set_key((FamilyBasicKey)-1, true));
    write_mem(0x4016, 5);
    for (unsigned row = 0; row < 10; ++row) {
        CHECK((read_mem(0x4017) & 0x1E) == rows[row][0]);
        CHECK((read_mem(0x4017) & 0x1E) == rows[row][0]);
        write_mem(0x4016, 6);
        CHECK((read_mem(0x4017) & 0x1E) == rows[row][1]);
        write_mem(0x4016, 6); // Holding the high half does not advance the row.
        CHECK((read_mem(0x4017) & 0x1E) == rows[row][1]);
        write_mem(0x4016, 4);
    }
    CHECK((read_mem(0x4017) & 0x1E) == 0x1A); // Row ten wraps to zero.
    write_mem(0x4016, 6);
    write_mem(0x4016, 0); // Disable while advancing the physical scan row.
    CHECK((read_mem(0x4017) & 0x1E) == 0);
    write_mem(0x4016, 4);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1C); // F7 is in row one.
    write_mem(0x4016, 7); // Reset has priority and may select the upper half.
    CHECK((read_mem(0x4017) & 0x1E) == 0x1C); // Kana, row zero upper half.
    family_basic_set_key(FB_KEY_KANA, false);
    CHECK((read_mem(0x4017) & 0x1E) == 0x1E); // Keys are live, not latched by reads.
    joypad_set_expansion_device(NES_EXPANSION_NONE);
    CHECK((read_mem(0x4017) & 0x1E) == 0);
    return 0;
}

static int family_basic_cpu_tape_and_controller(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    static const uint8_t samples[] = {0xA5, 0x3C};
    CHECK(family_basic_tape_load(samples, sizeof(samples)));
    CHECK(family_basic_tape_play(0));
    pad1.buttons = 1;
    joypad_set_microphone(true);
    write_mem(0x4016, 5);
    input_program(0xAD, 0x4016); // LDA $4016: input and tape share the actual CPU bus.
    for (unsigned sample = 0; sample < 16; ++sample) {
        cpu_total_cycles = (uint64_t)sample * 88;
        cpu.pc = 0x8000;
        CHECK(cpu_step(&cpu) == 4);
        uint8_t expected = (uint8_t)(((samples[sample / 8] >> (sample & 7u)) & 1u) << 1);
        CHECK((cpu.a & 7u) == (uint8_t)(expected | 5u));
        CHECK((read_mem(0x4016) & 7u) == (uint8_t)(expected | 5u));
    }
    cpu_total_cycles = sizeof(samples) * 8 * 88;
    CHECK((read_mem(0x4016) & 2u) == 0);
    CHECK(family_basic_tape_mode() == FB_TAPE_STOPPED);

    uint64_t playback_start = cpu_total_cycles;
    CHECK(family_basic_tape_play(playback_start));
    for (unsigned instruction = 1; instruction < 176; ++instruction) {
        cpu.pc = 0x8000;
        CHECK(cpu_step(&cpu) == 4);
        CHECK(cpu_total_cycles == playback_start + instruction * 4);
        unsigned sample = instruction * 4 / 88;
        CHECK((cpu.a & 2u) == (((0xA5u >> sample) & 1u) << 1));
    }

    CHECK(family_basic_tape_play(1000));
    cpu_total_cycles = 1000;
    write_mem(0x4016, 0);
    CHECK((read_mem(0x4016) & 2u) == 0);
    cpu_total_cycles = 1000 + 2 * 88;
    write_mem(0x4016, 4);
    CHECK((read_mem(0x4016) & 2u) == 2); // Time advances while the recorder input is disabled.
    CHECK(!family_basic_tape_load(NULL, 1));
    CHECK(family_basic_tape_mode() == FB_TAPE_PLAYING);
    CHECK((read_mem(0x4016) & 2u) == 2);

    family_basic_tape_stop();
    pad2.buttons = 0xA5;
    write_mem(0x4016, 5);
    write_mem(0x4016, 4);
    for (unsigned bit = 0; bit < 8; ++bit) {
        uint8_t value = read_mem(0x4017);
        CHECK((value & 0x1E) == 0x1E);
        CHECK((value & 1u) == ((0xA5u >> bit) & 1u));
    }
    CHECK(family_basic_tape_load(NULL, 0));
    CHECK(!family_basic_tape_play(cpu_total_cycles));
    return 0;
}

static int family_basic_recording_and_media(void) {
    input_fixture(NES_CONSOLE_HVC001, NES_REGION_NTSC);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    family_basic_tape_record(0);
    cpu_total_cycles = 88;
    write_mem(0x4016, 0); // Exactly 88 cycles does not finish a recording sample yet.
    cpu_total_cycles = 89;
    write_mem(0x4016, 1);
    for (unsigned sample = 1; sample < 8; ++sample) {
        cpu_total_cycles = (uint64_t)(sample + 1) * 88 + 1;
        write_mem(0x4016, (uint8_t)((0xA5u >> sample) & 1u));
    }
    CHECK(!family_basic_tape_failed());
    family_basic_tape_stop();
    CHECK(family_basic_tape_play(2000));
    for (unsigned sample = 0; sample < 8; ++sample) {
        cpu_total_cycles = 2000 + (uint64_t)sample * 88;
        write_mem(0x4016, 4);
        CHECK((read_mem(0x4016) & 2u) == (((0xA5u >> sample) & 1u) << 1));
    }
    family_basic_tape_stop();
    char path[128];
    snprintf(path, sizeof(path), ".family-basic-tape-%llu-%llu.bin",
             (unsigned long long)time(NULL), (unsigned long long)clock());
    FILE *reservation = fopen(path, "wbx");
    CHECK(reservation != NULL);
    CHECK(fclose(reservation) == 0);
    CHECK(!family_basic_tape_save_file(""));
    CHECK(family_basic_tape_save_file(path));
    FILE *saved = fopen(path, "rb");
    CHECK(saved != NULL);
    int first = fgetc(saved);
    int next = fgetc(saved);
    CHECK(fclose(saved) == 0);
    CHECK(first == 0xA5 && next == EOF);
    CHECK(family_basic_tape_load_file(path));
    CHECK(remove(path) == 0);
    CHECK(!family_basic_tape_load_file(path)); // Failed load preserves the current tape.
    CHECK(family_basic_tape_play(4000));
    cpu_total_cycles = 4000;
    CHECK((read_mem(0x4016) & 2u) == 2);
    joypad_set_expansion_device(NES_EXPANSION_NONE);
    CHECK(family_basic_tape_mode() == FB_TAPE_STOPPED);
    CHECK((read_mem(0x4016) & 2u) == 0);
    family_basic_shutdown();
    return 0;
}

int test_input_accuracy(void) {
    static int (*const tests[])(void) = {
        console_open_bus, console_read_clocks, console_strobe_timing,
        famicom_microphone, console_dma_reads, console_selection_lifetime,
        adapter_reports, adapter_strobes_and_disconnect, adapter_cpu_and_dma_clocks,
        adapter_selection_lifetime, arkanoid_reports, arkanoid_latching,
        arkanoid_cpu_clocks, device_selection, power_pad_button_order,
        power_pad_latching, family_trainer_rows, family_trainer_cpu_writes,
        mat_selection_and_disconnect, zapper_port_signals, zapper_beam_and_persistence,
        zapper_brightness_and_area, zapper_cpu_and_dma, zapper_selection_and_disconnect,
        family_basic_matrix_scan, family_basic_cpu_tape_and_controller,
        family_basic_recording_and_media
    };
    NesConsoleModel saved_model = nes_console_model();
    NesRegion saved_region = nes_timing()->region;
    NesInputAdapter saved_adapter = joypad_adapter();
    NesPortDevice saved_ports[] = {joypad_port_device(0), joypad_port_device(1)};
    NesExpansionDevice saved_expansion = joypad_expansion_device();
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
    joypad_set_zapper_radius(saved_zapper_radius);
    printf("Input: %u checks, %d failures\n", input_checks, failures);
    return failures;
}
