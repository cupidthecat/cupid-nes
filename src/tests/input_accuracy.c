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
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        memset(joypad_player(player), 0, sizeof(Joypad));
    joypad_set_adapter(NES_ADAPTER_NONE);
    joypad_set_port_device(0, NES_PORT_GAMEPAD);
    joypad_set_port_device(1, NES_PORT_GAMEPAD);
    joypad_set_expansion_device(NES_EXPANSION_NONE);
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

int test_input_accuracy(void) {
    static int (*const tests[])(void) = {
        console_open_bus, console_read_clocks, console_strobe_timing,
        famicom_microphone, console_dma_reads, console_selection_lifetime,
        adapter_reports, adapter_strobes_and_disconnect, adapter_cpu_and_dma_clocks,
        adapter_selection_lifetime, arkanoid_reports, arkanoid_latching,
        arkanoid_cpu_clocks, device_selection
    };
    NesConsoleModel saved_model = nes_console_model();
    NesRegion saved_region = nes_timing()->region;
    NesInputAdapter saved_adapter = joypad_adapter();
    NesPortDevice saved_ports[] = {joypad_port_device(0), joypad_port_device(1)};
    NesExpansionDevice saved_expansion = joypad_expansion_device();
    int failures = 0;
    input_checks = 0;
    for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    nes_set_console_model(saved_model);
    nes_set_region(saved_region);
    joypad_set_microphone(false);
    joypad_set_adapter(saved_adapter);
    joypad_set_port_device(0, saved_ports[0]);
    joypad_set_port_device(1, saved_ports[1]);
    joypad_set_expansion_device(saved_expansion);
    printf("Input: %u checks, %d failures\n", input_checks, failures);
    return failures;
}
