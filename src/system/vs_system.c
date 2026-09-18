/*
 * vs_system.c - Nintendo VS System hardware
 *
 * Author: @frankischilling
 *
 * This file models VS System board metadata, controls, protection reads and
 * dual-console execution. Dual systems keep independent CPU, PPU and APU
 * instances while sharing the cartridge RAM selected by the board control line.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "vs_system.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "timing.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    VsRomConfig config;
    unsigned active_side;
    uint16_t dips;
    bool coins[4];
    bool service[2];
    uint8_t shift[2][2];
    bool strobe[2];
    uint8_t select_bit[2];
    unsigned ram_owner;
    bool external_irq[2];
    uint8_t protection_counter[2];
    CpuMachineContext sub_cpu;
    PpuMachineContext sub_ppu;
    APU sub_apu;
    uint32_t sub_framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
    uint32_t dual_framebuffer[2 * SCREEN_WIDTH * SCREEN_HEIGHT];
} VsState;

static VsState vs;

static void reset_control_state(void) {
    memset(vs.protection_counter, 0, sizeof(vs.protection_counter));
    if (vs.config.dual) {
        vs.ram_owner = 1;
        vs.external_irq[0] = false;
        vs.external_irq[1] = true;
    } else {
        vs.ram_owner = 0;
        vs.external_irq[0] = false;
        vs.external_irq[1] = false;
    }
}

static void set_reason(char *reason, size_t size, const char *text) {
    if (!reason || !size) return;
    snprintf(reason, size, "%s", text);
}

static bool mapper_supported(int mapper) {
    return mapper == 0 || mapper == 1 || mapper == 2 || mapper == 99;
}

static bool mapper99_layout_supported(const iNESHeader *header, const VsRomConfig *config,
                                      size_t prg_bytes, size_t chr_bytes) {
    if (!header->chr_rom_chunks) return false;
    if (config->dual)
        return (prg_bytes == 0xC000 || prg_bytes == 0x10000) && chr_bytes == 0x8000;
    if (config->type == VS_TYPE_RAID_ON_BUNGELING_BAY)
        return prg_bytes == 0xC000 && (chr_bytes == 0x2000 || chr_bytes == 0x4000);
    return (prg_bytes == 0x8000 || prg_bytes == 0xA000 || prg_bytes == 0xC000)
        && (chr_bytes == 0x2000 || chr_bytes == 0x4000);
}

bool vs_decode_header(const iNESHeader *header, int mapper, size_t prg_bytes,
                      size_t chr_bytes, VsRomConfig *config,
                      char *reason, size_t reason_size) {
    if (!header || !config) return false;
    memset(config, 0, sizeof(*config));
    config->type = VS_TYPE_DEFAULT;
    config->ppu_model = VS_PPU_2C03;
    config->input_type = VS_INPUT_STANDARD;

    bool nes2 = (header->flags7 & 0x0C) == 0x08;
    unsigned console = header->flags7 & 3u;
    if (nes2) {
        if (console == 0 || (console == 3 && (header->zero[2] & 0x0Fu) == 0)) return true;
        bool extended_vs = console == 3 && (header->zero[2] & 0x0Fu) == 1;
        if (console != 1 && !extended_vs) {
            set_reason(reason, reason_size, "unsupported NES 2.0 console type");
            return false;
        }
        config->enabled = true;
        uint8_t descriptor = header->zero[2];
        uint8_t type = descriptor >> 4;
        // The extended subtype occupies the PPU field. Use the 2C03 fallback.
        uint8_t ppu = extended_vs ? 0 : descriptor & 0x0F;
        if (type > VS_TYPE_RAID_ON_BUNGELING_BAY) {
            set_reason(reason, reason_size, "unsupported VS hardware type");
            return false;
        }
        config->type = (VsSystemType)type;
        config->dual = type == VS_TYPE_DUAL;
        switch (ppu) {
            case 0: case 6: case 7: config->ppu_model = VS_PPU_2C03; break;
            case 2: config->ppu_model = VS_PPU_2C04_0001; break;
            case 3: config->ppu_model = VS_PPU_2C04_0002; break;
            case 4: config->ppu_model = VS_PPU_2C04_0003; break;
            case 5: config->ppu_model = VS_PPU_2C04_0004; break;
            case 8: config->ppu_model = VS_PPU_2C05_01; break;
            case 9: config->ppu_model = VS_PPU_2C05_02; break;
            case 10: config->ppu_model = VS_PPU_2C05_03; break;
            case 11: config->ppu_model = VS_PPU_2C05_04; break;
            case 12: config->ppu_model = VS_PPU_2C05_05; break;
            default:
                set_reason(reason, reason_size, "unsupported VS PPU model");
                return false;
        }
        uint8_t input = header->zero[4] & 0x3F;
        if (input == 0) input = VS_INPUT_STANDARD;
        if (input < VS_INPUT_STANDARD || input > VS_INPUT_SWAP_AB) {
            set_reason(reason, reason_size, "unsupported VS controller wiring");
            return false;
        }
        config->input_type = (VsInputType)input;
    } else {
        bool clean_header = (header->flags7 & 0x0Cu) == 0;
        if (clean_header && console && console != 1) {
            set_reason(reason, reason_size, "unsupported iNES console type");
            return false;
        }
        if ((clean_header && console == 1) || mapper == 99) {
            config->enabled = true;
            config->dual = mapper == 99 && prg_bytes >= 0x10000;
            config->type = config->dual ? VS_TYPE_DUAL : VS_TYPE_DEFAULT;
        } else {
            return true;
        }
    }

    if (!mapper_supported(mapper)) {
        set_reason(reason, reason_size, "unsupported mapper for VS System");
        return false;
    }
    bool non_ntsc = nes2 ? (header->zero[1] & 3u) == 1u || (header->zero[1] & 3u) == 3u
                         : (header->flags9 & 1u) != 0;
    if (non_ntsc) {
        set_reason(reason, reason_size, "VS System requires NTSC timing");
        return false;
    }
    if (config->dual && mapper != 99) {
        set_reason(reason, reason_size, "dual VS System requires mapper 99");
        return false;
    }
    if (mapper == 99 && !mapper99_layout_supported(header, config, prg_bytes, chr_bytes)) {
        set_reason(reason, reason_size, "unsupported mapper 99 PRG/CHR layout");
        return false;
    }
    return true;
}

static void select_side(unsigned side) {
    if (!vs.config.dual || side == 0) {
        vs.active_side = 0;
        cpu_select_machine(NULL);
        ppu_select_machine(NULL, framebuffer);
        apu_select_machine(NULL);
        return;
    }
    vs.active_side = 1;
    cpu_select_machine(&vs.sub_cpu);
    ppu_select_machine(&vs.sub_ppu, vs.sub_framebuffer);
    apu_select_machine(&vs.sub_apu);
}

void vs_commit_config(const VsRomConfig *config) {
    select_side(0);
    memset(&vs, 0, sizeof(vs));
    if (config) vs.config = *config;
    reset_control_state();
}

void vs_clear_config(void) {
    select_side(0);
    memset(&vs, 0, sizeof(vs));
}

bool vs_enabled(void) { return vs.config.enabled; }
bool vs_dual_system(void) { return vs.config.enabled && vs.config.dual; }
VsSystemType vs_system_type(void) { return vs.config.type; }
VsPpuModel vs_ppu_model(void) { return vs.config.ppu_model; }
unsigned vs_active_side(void) { return vs.active_side; }

void vs_power_on_secondary(void) {
    if (!vs_dual_system()) return;
    memset(&vs.sub_cpu, 0, sizeof(vs.sub_cpu));
    memset(&vs.sub_ppu, 0, sizeof(vs.sub_ppu));
    memset(&vs.sub_apu, 0, sizeof(vs.sub_apu));
    memset(vs.sub_framebuffer, 0, sizeof(vs.sub_framebuffer));
    vs.sub_cpu.runtime.ppu_master_phase = (uint8_t)(nes_timing()->ppu_divider - 1);
    select_side(1);
    ppu_power_on(&vs.sub_ppu.state);
    apu_power_on(&vs.sub_apu);
    cpu_power_on(&vs.sub_cpu.cpu);
    CpuStartupAlignment alignment = cpu_get_startup_alignment();
    printf("VS secondary startup alignment: CPU %u, PPU %u\n",
           (unsigned)alignment.cpu_offset, (unsigned)alignment.ppu_phase);
    select_side(0);
}

void vs_soft_reset(void) {
    if (!vs_enabled()) return;
    reset_control_state();
    if (!vs_dual_system()) return;
    select_side(1);
    ppu_soft_reset(&vs.sub_ppu.state);
    apu_soft_reset(&vs.sub_apu);
    cpu_soft_reset(&vs.sub_cpu.cpu);
    select_side(0);
}

int vs_cpu_step(void) {
    if (!vs_dual_system()) return cpu_step(&cpu);
    select_side(0);
    int main_cycles = cpu_step(&cpu);
    uint64_t main_count = cpu_total_cycles;
    uint64_t main_frame = ppu.frame_count;
    while ((main_count > vs.sub_cpu.total_cycles && main_count - vs.sub_cpu.total_cycles > 5)
           || main_frame > vs.sub_ppu.state.frame_count) {
        select_side(1);
        cpu_step(&vs.sub_cpu.cpu);
    }
    select_side(0);
    return main_cycles;
}

void vs_start_frame(void) {
    start_frame();
    if (vs_dual_system()) vs.sub_ppu.state.frame_complete = false;
}

unsigned vs_video_width(void) {
    return vs_dual_system() ? 2 * SCREEN_WIDTH : SCREEN_WIDTH;
}

const uint32_t *vs_video_framebuffer(void) {
    if (!vs_dual_system()) return framebuffer;
    for (size_t row = 0; row < SCREEN_HEIGHT; ++row) {
        uint32_t *output = vs.dual_framebuffer + row * 2 * SCREEN_WIDTH;
        memcpy(output, framebuffer + row * SCREEN_WIDTH, SCREEN_WIDTH * sizeof(*output));
        memcpy(output + SCREEN_WIDTH, vs.sub_framebuffer + row * SCREEN_WIDTH,
               SCREEN_WIDTH * sizeof(*output));
    }
    return vs.dual_framebuffer;
}

APU *vs_side_apu(unsigned side) {
    if (side == 0) return &apu;
    return side == 1 && vs_dual_system() ? &vs.sub_apu : NULL;
}

void vs_audio_init(int sample_rate) {
    apu_audio_init_state(vs_side_apu(0), sample_rate);
    apu_audio_init_state(vs_side_apu(1), sample_rate);
}

void vs_audio_callback(void *userdata, uint8_t *stream, int len) {
    if (!vs_dual_system()) {
        apu_sdl_audio_callback(userdata, stream, len);
        return;
    }
    if (!stream || len <= 0) return;
    float *output = (float *)stream;
    int count = len / (int)sizeof(*output);
    // The callback uses stable APU storage, never the CPU's active-machine selector.
    apu_audio_pull(&apu, output, count);
    float secondary[256];
    for (int offset = 0; offset < count;) {
        int chunk = count - offset;
        if (chunk > (int)(sizeof(secondary) / sizeof(secondary[0]))) chunk = 256;
        apu_audio_pull(&vs.sub_apu, secondary, chunk);
        for (int i = 0; i < chunk; ++i)
            output[offset + i] = 0.5f * (output[offset + i] + secondary[i]);
        offset += chunk;
    }
}

uint64_t vs_side_cpu_cycles(unsigned side) {
    if (side == 0) return cpu_total_cycles;
    return vs_dual_system() && side == 1 ? vs.sub_cpu.total_cycles : 0;
}

uint64_t vs_side_frame_count(unsigned side) {
    if (side == 0) return ppu.frame_count;
    return vs_dual_system() && side == 1 ? vs.sub_ppu.state.frame_count : 0;
}

static void remapped_buttons(unsigned side, uint8_t out[2]) {
    uint8_t first = joypad_player(side * 2)->buttons;
    uint8_t second = joypad_player(side * 2 + 1)->buttons;
    if (vs.config.input_type == VS_INPUT_SWAPPED) {
        uint8_t first_system = first & ((1u << BTN_SELECT) | (1u << BTN_START));
        uint8_t second_system = second & ((1u << BTN_SELECT) | (1u << BTN_START));
        uint8_t system_mask = (1u << BTN_SELECT) | (1u << BTN_START);
        uint8_t swapped_first = (second & (uint8_t)~system_mask) | first_system;
        uint8_t swapped_second = (first & (uint8_t)~system_mask) | second_system;
        first = swapped_first;
        second = swapped_second;
    } else if (vs.config.input_type == VS_INPUT_SWAP_AB) {
        bool first_b = (first & (1u << BTN_B)) != 0;
        bool second_a = (second & (1u << BTN_A)) != 0;
        first = (uint8_t)((first & ~(1u << BTN_B)) | (second_a ? (1u << BTN_B) : 0));
        second = (uint8_t)((second & ~(1u << BTN_A)) | (first_b ? (1u << BTN_A) : 0));
    }
    for (unsigned port = 0; port < 2; ++port) {
        uint8_t value = port ? second : first;
        bool select = (value & (1u << BTN_SELECT)) != 0;
        bool start = (value & (1u << BTN_START)) != 0;
        value &= (uint8_t)~((1u << BTN_SELECT) | (1u << BTN_START));
        if (select) value |= 1u << BTN_START;
        if (start) value |= 1u << BTN_SELECT;
        if (vs.config.type == VS_TYPE_ICE_CLIMBER || vs.config.type == VS_TYPE_RAID_ON_BUNGELING_BAY)
            value |= 1u << BTN_START;
        out[port] = value;
    }
}

static void latch_controllers(unsigned side) {
    uint8_t buttons[2];
    remapped_buttons(side, buttons);
    vs.shift[side][0] = buttons[0];
    vs.shift[side][1] = buttons[1];
}

void vs_write_4016(uint8_t value) {
    if (!vs_enabled()) return;
    unsigned side = vs.active_side;
    bool strobe = (value & 1) != 0;
    if (vs.strobe[side] && !strobe) latch_controllers(side);
    vs.strobe[side] = strobe;
    vs.select_bit[side] = (value >> 2) & 1u;
    if (vs_dual_system()) {
        bool main_has_ram = (value & 0x02) != 0;
        if (side == 0) vs.ram_owner = main_has_ram ? 0u : 1u;
        unsigned peer = side ^ 1u;
        vs.external_irq[peer] = !main_has_ram;
    }
}

uint8_t vs_read_controller_port(unsigned port) {
    if (!vs_enabled() || port > 1) return 0;
    unsigned side = vs.active_side;
    uint8_t bit;
    if (vs.strobe[side]) {
        uint8_t buttons[2];
        remapped_buttons(side, buttons);
        bit = buttons[port] & 1u;
    } else {
        bit = vs.shift[side][port] & 1u;
        vs.shift[side][port] = (vs.shift[side][port] >> 1) | 0x80;
    }
    if (port == 0) {
        unsigned coin = side * 2;
        uint8_t value = bit;
        if (vs.service[side]) value |= 0x04;
        uint8_t local_dips = (uint8_t)(vs.dips >> (side * 8));
        value |= (uint8_t)((local_dips & 0x03) << 3);
        if (vs.coins[coin]) value |= 0x20;
        if (vs.coins[coin + 1]) value |= 0x40;
        if (side) value |= 0x80;
        return value;
    }
    uint8_t local_dips = (uint8_t)(vs.dips >> (side * 8));
    return (uint8_t)(bit | (local_dips & 0xFC));
}

bool vs_set_dip_switches(uint16_t value) {
    if (!vs_enabled()) return false;
    vs.dips = value;
    return true;
}

uint16_t vs_dip_switches(void) { return vs.dips; }

bool vs_set_coin(unsigned slot, bool pressed) {
    if (!vs_enabled() || slot >= (vs_dual_system() ? 4u : 2u)) return false;
    vs.coins[slot] = pressed;
    return true;
}

bool vs_set_service(unsigned side, bool pressed) {
    if (!vs_enabled() || side >= (vs_dual_system() ? 2u : 1u)) return false;
    vs.service[side] = pressed;
    return true;
}

uint8_t vs_prg_chr_select_bit(void) {
    return vs.config.enabled ? vs.select_bit[vs.active_side] : 0;
}

bool vs_shared_ram_access_allowed(void) {
    return !vs_dual_system() || vs.active_side == vs.ram_owner;
}

bool vs_external_irq_pending(void) {
    return vs_dual_system() && vs.external_irq[vs.active_side];
}

static const uint8_t tko_table[32] = {
    0xFF,0xBF,0xB7,0x97,0x97,0x17,0x57,0x4F,0x6F,0x6B,0xEB,0xA9,0xB1,0x90,0x94,0x14,
    0x56,0x4E,0x6F,0x6B,0xEB,0xA9,0xB1,0x90,0xD4,0x5C,0x3E,0x26,0x87,0x83,0x13,0x00
};
static const uint8_t rbi_table[32] = {
    0x00,0x00,0x00,0x00,0xB4,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0x00,0x94,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
static const uint8_t xevious_table[32] = {
    0x05,0x01,0x89,0x37,0x05,0x00,0xD1,0x3E,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

bool vs_protection_read(uint16_t address, uint8_t *value) {
    if (!vs_enabled() || !value || address < 0x4020 || address > 0x5FFF) return false;
    unsigned side = vs.active_side;
    *value = 0;
    if (address == 0x5E00) {
        vs.protection_counter[side] = 0;
        return true;
    }
    if (address == 0x5E01) {
        if (vs.config.type == VS_TYPE_TKO_BOXING || vs.config.type == VS_TYPE_RBI_BASEBALL) {
            const uint8_t *table = vs.config.type == VS_TYPE_TKO_BOXING ? tko_table : rbi_table;
            *value = table[vs.protection_counter[side]++ & 31u];
        }
        return true;
    }
    if (vs.config.type == VS_TYPE_SUPER_XEVIOUS) {
        *value = xevious_table[vs.protection_counter[side]++ & 31u];
    }
    return true;
}

bool vs_ppu_is_2c05(void) {
    return vs_enabled() && vs.config.ppu_model >= VS_PPU_2C05_01;
}

bool vs_ppu_status_signature(uint8_t *signature) {
    if (!signature || !vs_ppu_is_2c05()) return false;
    static const uint8_t ids[] = {0x1B, 0x3D, 0x1C, 0x1B, 0x00};
    *signature = ids[vs.config.ppu_model - VS_PPU_2C05_01];
    return true;
}

static const uint32_t rgb_ppu_palette[64] = {
    0xFF6D6D6D,0xFF002491,0xFF0000DA,0xFF6D48DA,0xFF91006D,0xFFB6006D,0xFFB62400,0xFF914800,
    0xFF6D4800,0xFF244800,0xFF006D24,0xFF009100,0xFF004848,0xFF000000,0xFF000000,0xFF000000,
    0xFFB6B6B6,0xFF006DDA,0xFF0048FF,0xFF9100FF,0xFFB600FF,0xFFFF0091,0xFFFF0000,0xFFDA6D00,
    0xFF916D00,0xFF249100,0xFF009100,0xFF00B66D,0xFF009191,0xFF000000,0xFF000000,0xFF000000,
    0xFFFFFFFF,0xFF6DB6FF,0xFF9191FF,0xFFDA6DFF,0xFFFF00FF,0xFFFF6DFF,0xFFFF9100,0xFFFFB600,
    0xFFDADA00,0xFF6DDA00,0xFF00FF00,0xFF48FFDA,0xFF00FFFF,0xFF000000,0xFF000000,0xFF000000,
    0xFFFFFFFF,0xFFB6DAFF,0xFFDAB6FF,0xFFFFB6FF,0xFFFF91FF,0xFFFFB6B6,0xFFFFDA91,0xFFFFFF48,
    0xFFFFFF6D,0xFFB6FF48,0xFF91FF6D,0xFF48FFDA,0xFF91DAFF,0xFF000000,0xFF000000,0xFF000000
};

static const uint32_t rgb_2c04_palette[4][64] = {
    {0xFFFFB6B6u,0xFFDA6DFFu,0xFFFF0000u,0xFF9191FFu,0xFF009191u,0xFF244800u,0xFF484848u,0xFFFF0091u,0xFFFFFFFFu,0xFF6D6D6Du,0xFFFFB600u,0xFFB6006Du,0xFF91006Du,0xFFDADA00u,0xFF6D4800u,0xFFFFFFFFu,0xFF6DB6FFu,0xFFDAB66Du,0xFF6D2400u,0xFF6DDA00u,0xFF91DAFFu,0xFFDAB6FFu,0xFFFFDA91u,0xFF0048FFu,0xFFFFDA00u,0xFF48FFDAu,0xFF000000u,0xFF480000u,0xFFDADADAu,0xFF919191u,0xFFFF00FFu,0xFF002491u,0xFF00006Du,0xFFB6DAFFu,0xFFFFB6FFu,0xFF00FF00u,0xFF00FFFFu,0xFF004848u,0xFF00B66Du,0xFFB600FFu,0xFF000000u,0xFF914800u,0xFFFF91FFu,0xFFB62400u,0xFF9100FFu,0xFF0000DAu,0xFFFF9100u,0xFF000000u,0xFF000000u,0xFF249100u,0xFFB6B6B6u,0xFF006D24u,0xFFB6FF48u,0xFF6D48DAu,0xFFFFFF00u,0xFFDA6D00u,0xFF004800u,0xFF006DDAu,0xFF009100u,0xFF242424u,0xFFFFFF6Du,0xFFFF6DFFu,0xFF916D00u,0xFF91FF6Du},
    {0xFF000000u,0xFFFFB600u,0xFF916D00u,0xFFB6FF48u,0xFF91FF6Du,0xFFFF6DFFu,0xFF009191u,0xFFB6DAFFu,0xFFFF0000u,0xFF9100FFu,0xFFFFFF6Du,0xFFFF91FFu,0xFFFFFFFFu,0xFFDA6DFFu,0xFF91DAFFu,0xFF009100u,0xFF004800u,0xFF6DB6FFu,0xFFB62400u,0xFFDADADAu,0xFF00B66Du,0xFF6DDA00u,0xFF480000u,0xFF9191FFu,0xFF484848u,0xFFFF00FFu,0xFF00006Du,0xFF48FFDAu,0xFFDAB6FFu,0xFF6D4800u,0xFF000000u,0xFF6D48DAu,0xFF91006Du,0xFFFFDA91u,0xFFFF9100u,0xFFFFB6FFu,0xFF006DDAu,0xFF6D2400u,0xFFB6B6B6u,0xFF0000DAu,0xFFB600FFu,0xFFFFDA00u,0xFF6D6D6Du,0xFF244800u,0xFF0048FFu,0xFF000000u,0xFFDADA00u,0xFFFFFFFFu,0xFFDAB66Du,0xFF242424u,0xFF00FF00u,0xFFDA6D00u,0xFF004848u,0xFF002491u,0xFFFF0091u,0xFF249100u,0xFF000000u,0xFF00FFFFu,0xFF914800u,0xFFFFFF00u,0xFFFFB6B6u,0xFFB6006Du,0xFF006D24u,0xFF919191u},
    {0xFFB600FFu,0xFFFF6DFFu,0xFF91FF6Du,0xFFB6B6B6u,0xFF009100u,0xFFFFFFFFu,0xFFB6DAFFu,0xFF244800u,0xFF002491u,0xFF000000u,0xFFFFDA91u,0xFF6D4800u,0xFFFF0091u,0xFFDADADAu,0xFFDAB66Du,0xFF91DAFFu,0xFF9191FFu,0xFF009191u,0xFFB6006Du,0xFF0048FFu,0xFF249100u,0xFF916D00u,0xFFDA6D00u,0xFF00B66Du,0xFF6D6D6Du,0xFF6D48DAu,0xFF000000u,0xFF0000DAu,0xFFFF0000u,0xFFB62400u,0xFFFF91FFu,0xFFFFB6B6u,0xFFDA6DFFu,0xFF004800u,0xFF00006Du,0xFFFFFF00u,0xFF242424u,0xFFFFB600u,0xFFFF9100u,0xFFFFFFFFu,0xFF6DDA00u,0xFF91006Du,0xFF6DB6FFu,0xFFFF00FFu,0xFF006DDAu,0xFF919191u,0xFF000000u,0xFF6D2400u,0xFF00FFFFu,0xFF480000u,0xFFB6FF48u,0xFFFFB6FFu,0xFF914800u,0xFF00FF00u,0xFFDADA00u,0xFF484848u,0xFF006D24u,0xFF000000u,0xFFDAB6FFu,0xFFFFFF6Du,0xFF9100FFu,0xFF48FFDAu,0xFFFFDA00u,0xFF004848u},
    {0xFF916D00u,0xFF6D48DAu,0xFF009191u,0xFFDADA00u,0xFF000000u,0xFFFFB6B6u,0xFF002491u,0xFFDA6D00u,0xFFB6B6B6u,0xFF6D2400u,0xFF00FF00u,0xFF00006Du,0xFFFFDA91u,0xFFFFFF00u,0xFF009100u,0xFFB6FF48u,0xFFFF6DFFu,0xFF480000u,0xFF0048FFu,0xFFFF91FFu,0xFF000000u,0xFF484848u,0xFFB62400u,0xFFFF9100u,0xFFDAB66Du,0xFF00B66Du,0xFF9191FFu,0xFF249100u,0xFF91006Du,0xFF000000u,0xFF91FF6Du,0xFF6DB6FFu,0xFFB6006Du,0xFF006D24u,0xFF914800u,0xFF0000DAu,0xFF9100FFu,0xFFB600FFu,0xFF6D6D6Du,0xFFFF0091u,0xFF004848u,0xFFDADADAu,0xFF006DDAu,0xFF004800u,0xFF242424u,0xFFFFFF6Du,0xFF919191u,0xFFFF00FFu,0xFFFFB6FFu,0xFFFFFFFFu,0xFF6D4800u,0xFFFF0000u,0xFFFFDA00u,0xFF48FFDAu,0xFFFFFFFFu,0xFF91DAFFu,0xFF000000u,0xFFFFB600u,0xFFDA6DFFu,0xFFB6DAFFu,0xFF6DDA00u,0xFFDAB6FFu,0xFF00FFFFu,0xFF244800u}
};

static const uint8_t rgb_lut[5][64] = {
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,15,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,15,62,63},
    {53,35,22,34,28,9,29,21,32,0,39,5,4,40,8,32,33,62,31,41,60,50,54,18,63,43,46,30,61,45,36,1,14,49,51,42,44,12,27,20,46,7,52,6,19,2,38,46,46,25,16,10,57,3,55,23,15,17,11,13,56,37,24,58},
    {46,39,24,57,58,37,28,49,22,19,56,52,32,35,60,11,15,33,6,61,27,41,30,34,29,36,14,43,50,8,46,3,4,54,38,51,17,31,16,2,20,63,0,9,18,46,40,32,62,13,42,23,12,1,21,25,46,44,7,55,53,5,10,45},
    {20,37,58,16,11,32,49,9,1,46,54,8,21,61,62,60,34,28,5,18,25,24,23,27,0,3,46,2,22,6,52,53,35,15,14,55,13,39,38,32,41,4,33,36,17,45,46,31,44,30,57,51,7,42,40,29,10,46,50,56,19,43,63,12},
    {24,3,28,40,46,53,1,23,16,31,42,14,54,55,11,57,37,30,18,52,46,29,6,38,62,27,34,25,4,46,58,33,5,10,7,2,19,20,0,21,12,61,17,15,13,56,45,36,51,32,8,22,63,43,32,60,46,39,35,49,41,50,44,9}
};

bool vs_ppu_rgb_color(uint8_t color, uint8_t mask, uint32_t *argb) {
    if (!vs_enabled() || !argb) return false;
    color &= 0x3F;
    if (mask & 1) color &= 0x30;
    uint32_t result;
    if (vs.config.ppu_model >= VS_PPU_2C04_0001 && vs.config.ppu_model <= VS_PPU_2C04_0004) {
        unsigned model = (unsigned)(vs.config.ppu_model - VS_PPU_2C04_0001);
        result = rgb_2c04_palette[model][color];
    } else {
        result = rgb_ppu_palette[rgb_lut[0][color]];
    }
    if (mask & 0x20) result |= 0x00FF0000;
    if (mask & 0x40) result |= 0x0000FF00;
    if (mask & 0x80) result |= 0x000000FF;
    *argb = result;
    return true;
}
