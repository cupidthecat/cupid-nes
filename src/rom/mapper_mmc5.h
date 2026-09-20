/*
 * mapper_mmc5.h - MMC5 cartridge mapper implementation
 *
 * Author: @frankischilling
 *
 * MMC5 banking, mirroring, IRQs, multiplication, audio, and ExRAM are handled here.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MAPPER_MMC5_H
#define MAPPER_MMC5_H

// Mapper 5: MMC5/ExROM.
typedef struct {
    uint8_t duty;
    uint8_t duty_pos;
    uint8_t volume;
    uint8_t envelope_decay;
    uint8_t envelope_divider;
    uint8_t length_counter;
    uint8_t length_reload;
    uint8_t length_previous;
    uint8_t output;
    uint16_t period;
    uint16_t timer;
    bool constant_volume;
    bool envelope_loop;
    bool length_halt;
    bool new_length_halt;
    bool envelope_start;
    bool enabled;
} Mmc5Pulse;

static struct {
    uint8_t prg_mode;
    uint8_t chr_mode;
    uint8_t exram_mode;      // $5104
    uint8_t prg_ram_protect1;// $5102
    uint8_t prg_ram_protect2;// $5103
    uint8_t prg_regs[4];     // $5114-$5117
    uint8_t prg_ram_bank;    // $5113
    uint16_t chr_regs_a[8];  // $5120-$5127 (sprite set in 8x16)
    uint16_t chr_regs_b[4];  // $5128-$512B (background set in 8x16)
    bool chr_last_set_b;     // last set written, used for PPUDATA CHR access
    uint8_t chr_upper;       // $5130 (high bits)
    uint8_t nt_control;      // $5105
    uint8_t fill_tile;       // $5106
    uint8_t fill_attr;       // $5107 (2-bit palette index)
    uint8_t mul_a;           // $5205
    uint8_t mul_b;           // $5206
    uint8_t irq_scanline;    // $5203
    uint8_t scanline_counter;
    bool in_frame;           // $5204 bit6
    bool need_in_frame;
    bool irq_enabled;        // $5204 bit7
    bool irq_pending;
    uint8_t ppu_idle_counter;
    uint16_t last_ppu_read_addr;
    uint8_t nt_read_counter;
    uint8_t split_tile_number;
    bool split_in_region;
    uint16_t split_tile;
    bool split_enabled;
    bool split_right;
    uint8_t split_delimiter;
    uint8_t split_scroll;
    uint8_t split_bank;
    uint16_t exattr_last_nt_fetch;
    uint8_t exattr_fetch_counter;
    uint8_t exattr_chr_bank;
    bool ppu_large_sprites;
    Mmc5Pulse pulse[2];
    unsigned audio_frame_counter;
    bool pcm_read_mode;
    bool pcm_irq_enabled;
    bool pcm_irq_pending;
    uint8_t pcm_output;
    Mirroring mirr;
} mmc5;

static const uint8_t mmc5_duty[4][8] = {
    {0, 0, 0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 0, 1, 1},
    {0, 0, 0, 0, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 0, 0}
};

static const uint8_t mmc5_length_table[32] = {
    10, 254, 20, 2, 40, 4, 80, 6,
    160, 8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22,
    192, 24, 72, 26, 16, 28, 32, 30
};

typedef struct {
    uint8_t volume;
    uint8_t duty;
    bool ignore_duty;
    uint16_t frequency;
    bool enabled;
    int32_t timer;
    uint8_t step;
} Vrc6Pulse;

typedef struct {
    uint8_t accumulator_rate;
    uint8_t accumulator;
    uint16_t frequency;
    bool enabled;
    int32_t timer;
    uint8_t step;
} Vrc6Saw;

static struct {
    uint8_t prg16_bank;
    uint8_t prg8_bank;
    uint8_t banking_mode;
    uint8_t chr_regs[8];
    bool nt_chr[16];
    size_t nt_offsets[16];
    bool prg16_selected;
    bool prg8_selected;
    bool ppu_initialized;
    bool variant_b;
    VrcIrq irq;
    Vrc6Pulse pulse[2];
    Vrc6Saw saw;
    bool halt_audio;
    uint8_t frequency_shift;
} vrc6;

static uint8_t vrc6_pulse_volume(const Vrc6Pulse *pulse) {
    if (!pulse->enabled) return 0;
    if (pulse->ignore_duty) return pulse->volume;
    return pulse->step <= pulse->duty ? pulse->volume : 0;
}

static uint8_t vrc6_saw_volume(void) {
    return vrc6.saw.enabled ? (uint8_t)(vrc6.saw.accumulator >> 3) : 0;
}

static void mmc5_update_irq_line(void) {
    mapper_irq_line = (mmc5.irq_enabled && mmc5.irq_pending)
                   || (mmc5.pcm_irq_enabled && mmc5.pcm_irq_pending);
}

static uint8_t mmc5_pulse_volume(const Mmc5Pulse *pulse) {
    if (!pulse->enabled || !pulse->length_counter) return 0;
    if (!mmc5_duty[pulse->duty][pulse->duty_pos]) return 0;
    return pulse->constant_volume ? pulse->volume : pulse->envelope_decay;
}

float cart_expansion_audio(void) {
    if (active_board) return board_audio(active_board);
    if (cart == &mapper_nsf) {
        float output = 0.0f;
        uint8_t chips = nsf_player.metadata.sound_chips;
        if (chips & NSF_SOUND_MMC5) {
            unsigned pulse = (unsigned)mmc5.pulse[0].output + (unsigned)mmc5.pulse[1].output;
            output -= (float)(pulse * 3u + mmc5.pcm_output) * (14.0f / 5000.0f);
        }
        if (chips & NSF_SOUND_VRC6) {
            unsigned raw = (unsigned)vrc6_pulse_volume(&vrc6.pulse[0])
                         + (unsigned)vrc6_pulse_volume(&vrc6.pulse[1])
                         + (unsigned)vrc6_saw_volume();
            output -= (float)raw * (75.0f / 5000.0f);
        }
        if (chips & NSF_SOUND_VRC7) output += vrc7_fm_output(&nsf_vrc7);
        if (chips & NSF_SOUND_NAMCO163) output += namco163_audio_output(&namco163_audio);
        if (chips & NSF_SOUND_SUNSOFT5B) output += sunsoft5b_output(&sunsoft5b_audio);
        if (chips & NSF_SOUND_FDS) output += fds_nsf_audio_output();
        return output;
    }
    if (cart == &mapper_fds) return fds_expansion_audio();
    if (cart == &mapper_vrc7) return vrc7_expansion_output();
    if (cart == &mapper_vrc6) {
        unsigned raw = (unsigned)vrc6_pulse_volume(&vrc6.pulse[0])
                     + (unsigned)vrc6_pulse_volume(&vrc6.pulse[1])
                     + (unsigned)vrc6_saw_volume();
        return -(float)raw * (75.0f / 5000.0f);
    }
    if (cart == &mapper_mmc5) {
        unsigned pulse = (unsigned)mmc5.pulse[0].output + (unsigned)mmc5.pulse[1].output;
        unsigned raw = pulse * 3u + mmc5.pcm_output;
        // Expansion audio is mixed linearly; the common mixer uses a gain of 14
        // against the native nonlinear mixer's 5000-unit reference scale.
        return -(float)raw * (14.0f / 5000.0f);
    }
    if (cart == &mapper_namco && namco.variant == NAMCO_VARIANT_163)
        return namco163_audio_output(&namco163_audio);
    return 0.0f;
}

float cart_audio_gain(void) {
    if (cart != &mapper_nsf) return 1.0f;
    unsigned track = nsf_player.song;
    if (track >= nsf_player.metadata.total_songs) return 1.0f;
    int32_t length = nsf_player.metadata.track_length[track];
    if (length <= 0) return 1.0f;
    double elapsed_ms = (double)(cpu_total_cycles - nsf_player.track_start_cycle)
                      * 1000.0 / nes_timing()->cpu_hz;
    if (elapsed_ms <= length) return 1.0f;
    int32_t fade = nsf_player.metadata.track_fade[track];
    if (fade <= 0 || elapsed_ms >= (double)length + fade) return 0.0f;
    return (float)(1.0 - (elapsed_ms - length) / fade);
}

static void mmc5_clock_envelope(Mmc5Pulse *pulse) {
    if (pulse->envelope_start) {
        pulse->envelope_start = false;
        pulse->envelope_decay = 15;
        pulse->envelope_divider = pulse->volume;
    } else if (pulse->envelope_divider) {
        pulse->envelope_divider--;
    } else {
        pulse->envelope_divider = pulse->volume;
        if (pulse->envelope_decay) pulse->envelope_decay--;
        else if (pulse->envelope_loop) pulse->envelope_decay = 15;
    }
}

static void mmc5_clock_pulse(Mmc5Pulse *pulse) {
    if (pulse->timer) {
        pulse->timer--;
    } else {
        pulse->duty_pos = (uint8_t)((pulse->duty_pos - 1u) & 7u);
        pulse->output = mmc5_pulse_volume(pulse);
        pulse->timer = (uint16_t)(pulse->period * 2u + 1u);
    }
}

static void mmc5_audio_frame_clock(void) {
    for (unsigned i = 0; i < 2; ++i) {
        Mmc5Pulse *pulse = &mmc5.pulse[i];
        if (pulse->length_counter && !pulse->length_halt) pulse->length_counter--;
        mmc5_clock_envelope(pulse);
    }
}

static void mmc5_reload_length(Mmc5Pulse *pulse) {
    if (pulse->length_reload) {
        if (pulse->length_counter == pulse->length_previous)
            pulse->length_counter = pulse->length_reload;
        pulse->length_reload = 0;
    }
    pulse->length_halt = pulse->new_length_halt;
}

static void mmc5_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        mmc5_clock_pulse(&mmc5.pulse[0]);
        mmc5_clock_pulse(&mmc5.pulse[1]);
        if (mmc5.audio_frame_counter) mmc5.audio_frame_counter--;
        if (!mmc5.audio_frame_counter) {
            mmc5.audio_frame_counter = (unsigned)(nes_timing()->cpu_hz / 240.0);
            if (!mmc5.audio_frame_counter) mmc5.audio_frame_counter = 1;
            mmc5_audio_frame_clock();
        }
        mmc5_reload_length(&mmc5.pulse[0]);
        mmc5_reload_length(&mmc5.pulse[1]);
        if (mmc5.ppu_idle_counter && --mmc5.ppu_idle_counter == 0)
            mmc5.in_frame = false;
    }
}

static void mmc5_pulse_write(unsigned channel, uint16_t addr, uint8_t value) {
    Mmc5Pulse *pulse = &mmc5.pulse[channel];
    switch (addr & 3u) {
        case 0:
            pulse->duty = value >> 6;
            pulse->envelope_loop = (value & 0x20u) != 0;
            pulse->new_length_halt = (value & 0x20u) != 0;
            pulse->constant_volume = (value & 0x10u) != 0;
            pulse->volume = value & 0x0Fu;
            break;
        case 1:
            break; // These channels have no sweep unit.
        case 2:
            pulse->period = (uint16_t)((pulse->period & 0x0700u) | value);
            break;
        case 3:
            pulse->period = (uint16_t)((pulse->period & 0x00FFu) | ((uint16_t)(value & 7u) << 8));
            pulse->duty_pos = 0;
            pulse->envelope_start = true;
            if (pulse->enabled) {
                pulse->length_reload = mmc5_length_table[value >> 3];
                pulse->length_previous = pulse->length_counter;
            }
            break;
    }
}

static void mmc5_pcm_write(uint8_t value) {
    if (value == 0) {
        mmc5.pcm_irq_pending = true;
    } else {
        mmc5.pcm_output = value;
        mmc5.pcm_irq_pending = false;
    }
    mmc5_update_irq_line();
}

static inline uint8_t mmc5_nt_source(uint16_t addr) {
    uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
    uint8_t nt = (uint8_t)((off >> 10) & 0x03u);
    return (uint8_t)((mmc5.nt_control >> (nt * 2)) & 0x03u);
}

static inline uint8_t mmc5_fill_attr_byte(void) {
    uint8_t p = (uint8_t)(mmc5.fill_attr & 0x03u);
    return (uint8_t)(p | (p << 2) | (p << 4) | (p << 6));
}

static void mmc5_update_mirroring(uint8_t nt_control) {
    uint8_t n0 = (uint8_t)(nt_control & 0x03);
    uint8_t n1 = (uint8_t)((nt_control >> 2) & 0x03);
    uint8_t n2 = (uint8_t)((nt_control >> 4) & 0x03);
    uint8_t n3 = (uint8_t)((nt_control >> 6) & 0x03);

    // Nametable reads/writes honor all four MMC5 sources directly. Return a
    // representative Mirroring value here for generic cartridge callers.
    if (n0 == 0 && n1 == 0 && n2 == 0 && n3 == 0) {
        mmc5.mirr = MIRROR_SINGLE0;
    } else if (n0 == 1 && n1 == 1 && n2 == 1 && n3 == 1) {
        mmc5.mirr = MIRROR_SINGLE1;
    } else if (n0 == 0 && n1 == 0 && n2 == 1 && n3 == 1) {
        mmc5.mirr = MIRROR_HORIZONTAL;
    } else if (n0 == 0 && n1 == 1 && n2 == 0 && n3 == 1) {
        mmc5.mirr = MIRROR_VERTICAL;
    } else {
        mmc5.mirr = MIRROR_FOUR;
    }
}

static inline size_t mmc5_prg_bank_count_8k(void) {
    return (C.prg_sz / PRG_BANK_8K);
}

static inline size_t mmc5_map_prg_slot_to_bank(size_t slot) {
    size_t banks = mmc5_prg_bank_count_8k();
    if (banks == 0) return 0;

    uint8_t r0 = mmc5.prg_regs[0] & 0x7F;
    uint8_t r1 = mmc5.prg_regs[1] & 0x7F;
    uint8_t r2 = mmc5.prg_regs[2] & 0x7F;
    uint8_t r3 = mmc5.prg_regs[3] & 0x7F;

    size_t bank = 0;
    switch (mmc5.prg_mode & 0x03) {
        case 0: {
            // One 32KB bank selected by $5117.
            size_t base = (size_t)(r3 & 0x7C);
            bank = base + (slot & 0x03);
        } break;
        case 1: {
            // $8000-$BFFF uses $5115 (16KB), $C000-$FFFF uses $5117 (16KB).
            size_t base_lo = (size_t)(r1 & 0x7E);
            size_t base_hi = (size_t)(r3 & 0x7E);
            bank = (slot < 2) ? (base_lo + slot) : (base_hi + (slot - 2));
        } break;
        case 2: {
            // $8000-$BFFF uses $5115 (16KB), then two 8KB banks from $5116/$5117.
            size_t base = (size_t)(r1 & 0x7E);
            if (slot < 2) bank = base + slot;
            else if (slot == 2) bank = r2;
            else bank = r3;
        } break;
        case 3:
        default:
            // Four independent 8KB banks.
            if (slot == 0) bank = r0;
            else if (slot == 1) bank = r1;
            else if (slot == 2) bank = r2;
            else bank = r3;
            break;
    }

    return bank % banks;
}

static inline size_t mmc5_map_chr_bank_1k(uint16_t a) {
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return 0;

    uint8_t slot = (uint8_t)((a >> 10) & 0x07);
    bool use_bg_set = false;
    if (mmc5.ppu_large_sprites) {
        if (cart_ppu_fetch_source == CART_PPU_FETCH_BG) {
            use_bg_set = true;
        } else if (cart_ppu_fetch_source == CART_PPU_FETCH_CPU) {
            // During rendering the mapper follows the current A/B fetch phase.
            // Outside rendering PPUDATA uses the most recently written CHR set.
            use_bg_set = mmc5.in_frame
                ? !(mmc5.split_tile_number >= 32 && mmc5.split_tile_number < 48)
                : mmc5.chr_last_set_b;
        }
    }

    uint16_t reg = 0;
    uint8_t mode = (uint8_t)(mmc5.chr_mode & 0x03);
    uint8_t sub = 0;

    if (!use_bg_set) {
        switch (mode) {
            case 0: reg = mmc5.chr_regs_a[7]; sub = slot; break;                         // 8KB
            // 4 KiB mode uses one register for each half of pattern space.
            case 1: reg = (slot < 4) ? mmc5.chr_regs_a[3] : mmc5.chr_regs_a[7]; sub = (uint8_t)(slot & 0x03u); break;
            // 2 KiB mode uses one register for each pair of slots.
            case 2: reg = mmc5.chr_regs_a[1 + ((slot >> 1) * 2)]; sub = (uint8_t)(slot & 0x01u); break;
            case 3:
            default: reg = mmc5.chr_regs_a[slot]; sub = 0; break;                        // 1KB
        }
    } else {
        switch (mode) {
            case 0:
                reg = mmc5.chr_regs_b[3];
                sub = slot;
                break;
            case 1:
                reg = mmc5.chr_regs_b[3];
                sub = (uint8_t)(slot & 0x03u);
                break;
            case 2:
                reg = ((slot & 0x02u) == 0) ? mmc5.chr_regs_b[1] : mmc5.chr_regs_b[3];
                sub = (uint8_t)(slot & 0x01u);
                break;
            case 3:
            default:
                reg = mmc5.chr_regs_b[slot & 0x03u];
                sub = 0;
                break;
        }
    }

    uint8_t shift = (uint8_t)(3u - mode);  // mode0=8KB, mode1=4KB, mode2=2KB, mode3=1KB
    size_t bank = ((size_t)reg << shift) + sub;
    return bank % chr_1k_banks;
}

void cart_notify_ppu_ctrl_write(uint8_t value) {
    if (cart != &mapper_mmc5) return;
    mmc5.ppu_large_sprites = (value & 0x20u) != 0;
    if (!mmc5.ppu_large_sprites) mmc5.chr_last_set_b = false;
}

static bool mmc5_is_nt_tile_fetch(uint16_t addr) {
    return addr >= 0x2000 && addr <= 0x2FFF && (addr & 0x03FFu) < 0x03C0u;
}

static void mmc5_detect_scanline_start(uint16_t addr) {
    if (mmc5.nt_read_counter >= 2) {
        if (!mmc5.in_frame && !mmc5.need_in_frame) {
            mmc5.need_in_frame = true;
            mmc5.scanline_counter = 0;
        } else {
            mmc5.scanline_counter++;
            if (mmc5.scanline_counter == mmc5.irq_scanline) {
                mmc5.irq_pending = true;
                mmc5_update_irq_line();
            }
        }
    } else if (addr >= 0x2000 && addr <= 0x2FFF && mmc5.last_ppu_read_addr == addr) {
        mmc5.nt_read_counter++;
        if (mmc5.nt_read_counter >= 2) mmc5.split_tile_number = 0;
    }

    if (mmc5.last_ppu_read_addr != addr) mmc5.nt_read_counter = 0;
}

static void mmc5_begin_ppu_read(uint16_t addr) {
    if (mmc5_is_nt_tile_fetch(addr)) {
        mmc5.split_tile_number++;
        if (!mmc5.in_frame && mmc5.need_in_frame) {
            mmc5.need_in_frame = false;
            mmc5.in_frame = true;
        }
    }
    mmc5_detect_scanline_start(addr);
    mmc5.ppu_idle_counter = 3;
    mmc5.last_ppu_read_addr = addr;
}

static uint8_t mmc5_read_chr_raw(size_t offset) {
    return C.chr_sz ? C.chr[offset & (C.chr_sz - 1)] : 0;
}

static unsigned mmc5_split_vertical_scroll(void) {
    unsigned scanline = mmc5.scanline_counter;
    if (mmc5.split_tile_number >= 49) scanline++;
    return (scanline + mmc5.split_scroll) % 240u;
}

static bool mmc5_split_nt_read(uint16_t addr, uint8_t *value) {
    if (!mmc5.split_enabled || !mmc5.in_frame || mmc5.exram_mode > 1) return false;
    unsigned vertical_scroll = mmc5_split_vertical_scroll();
    unsigned column = (mmc5.split_tile_number + 2u) % 50u;
    if (mmc5_is_nt_tile_fetch(addr)) {
        if (column == 0) mmc5.split_in_region = !mmc5.split_right;
        if (column == mmc5.split_delimiter && mmc5.split_tile_number < 50) {
            mmc5.split_in_region = !mmc5.split_in_region;
        } else if (column > 32) {
            mmc5.split_in_region = false;
        }
        if (mmc5.split_in_region) {
            mmc5.split_tile = (uint16_t)(((vertical_scroll & 0xF8u) << 2) | column);
            *value = mmc5_exram[mmc5.split_tile & 0x03FFu];
            return true;
        }
    } else if (addr >= 0x2000 && addr <= 0x2FFF && mmc5.split_in_region) {
        unsigned shift = ((mmc5.split_tile >> 4) & 4u) | (mmc5.split_tile & 2u);
        unsigned attr = 0x3C0u | ((mmc5.split_tile & 0x0380u) >> 4)
                      | ((mmc5.split_tile & 0x001Fu) >> 2);
        uint8_t palette = (uint8_t)((mmc5_exram[attr] >> shift) & 3u);
        *value = (uint8_t)(palette * 0x55u);
        return true;
    }
    return false;
}

static bool mmc5_split_chr_read(uint16_t addr, uint8_t *value) {
    if (!mmc5.split_enabled || !mmc5.in_frame || mmc5.exram_mode > 1
        || !mmc5.split_in_region) return false;
    unsigned vertical_scroll = mmc5_split_vertical_scroll();
    size_t chr_addr = ((size_t)mmc5.split_bank << 12)
                    + ((((size_t)addr & ~(size_t)7u) | (vertical_scroll & 7u)) & 0x0FFFu);
    *value = mmc5_read_chr_raw(chr_addr);
    return true;
}

static bool mmc5_extended_attr_read(uint16_t addr, bool nametable_bus, uint8_t *value) {
    if (mmc5.exram_mode != 1 || !mmc5.in_frame
        || (mmc5.split_tile_number >= 32 && mmc5.split_tile_number < 48)) return false;
    if (nametable_bus && mmc5_is_nt_tile_fetch(addr)) {
        mmc5.exattr_last_nt_fetch = addr & 0x03FFu;
        mmc5.exattr_fetch_counter = 3;
        return false;
    }
    if (!mmc5.exattr_fetch_counter) return false;

    mmc5.exattr_fetch_counter--;
    if (mmc5.exattr_fetch_counter == 2) {
        uint8_t ext = mmc5_exram[mmc5.exattr_last_nt_fetch];
        mmc5.exattr_chr_bank = (uint8_t)((ext & 0x3Fu) | (mmc5.chr_upper << 6));
        *value = (uint8_t)(((ext >> 6) & 3u) * 0x55u);
        return true;
    }
    if (mmc5.exattr_fetch_counter <= 1) {
        size_t chr_addr = ((size_t)mmc5.exattr_chr_bank << 12) | (addr & 0x0FFFu);
        *value = mmc5_read_chr_raw(chr_addr);
        return true;
    }
    return false;
}

static void mmc5_clear_frame_irq_on_nmi_vector(void) {
    mmc5.in_frame = false;
    mmc5.need_in_frame = false;
    mmc5.ppu_idle_counter = 0;
    mmc5.last_ppu_read_addr = 0;
    mmc5.nt_read_counter = 0;
    mmc5.scanline_counter = 0;
    mmc5.irq_pending = false;
    mmc5_update_irq_line();
}

static bool mmc5_ram_mapped(uint16_t a) {
    if (a < 0x6000) return false;
    if (a < 0x8000) return true;
    unsigned slot = (a - 0x8000) >> 13;
    switch (mmc5.prg_mode) {
        case 0: return false; // $5117 always selects ROM.
        case 1: return slot < 2 && !(mmc5.prg_regs[1] & 0x80);
        case 2:
            return slot < 2 ? !(mmc5.prg_regs[1] & 0x80)
                           : slot == 2 && !(mmc5.prg_regs[2] & 0x80);
        default: return slot < 3 && !(mmc5.prg_regs[slot] & 0x80);
    }
}

static RamBlock *mmc5_ram_location(uint16_t a, size_t *offset) {
    unsigned bank;
    if (a < 0x8000) {
        bank = mmc5.prg_ram_bank;
    } else if ((mmc5.prg_mode == 1 || mmc5.prg_mode == 2) && a < 0xC000) {
        bank = (mmc5.prg_regs[1] & 0xFE) + ((a - 0x8000) >> 13);
    } else {
        bank = mmc5.prg_regs[(a - 0x8000) >> 13];
    }
    RamBlock *ram = default_prg_ram();
    if (C.nes2 && (C.ram.prg_ram >= 0x10000 || C.ram.prg_nvram >= 0x10000)) {
        // Large NES 2.0 RAM chips use the four-bit selector. A battery-backed
        // chip remains the selected memory when both work and save RAM exist.
        if (prg_save_ram.size) ram = &prg_save_ram;
        else ram = &prg_work_ram;
        bank &= 0x0F;
    } else {
        bank &= 7;
        if (C.nes2) {
            if (C.ram.prg_ram == 0x2000 && C.ram.prg_nvram == 0x2000) {
                ram = (bank & 4) ? &prg_work_ram : &prg_save_ram;
            } else if (C.ram.prg_ram + C.ram.prg_nvram != 0x4000 && bank >= 4) {
                return NULL; // The second RAM socket is empty.
            }
        }
    }
    *offset = bank * PRG_BANK_8K + (a & 0x1FFF);
    return ram;
}

static uint8_t mmc5_cpu_read(uint16_t a) {
    if (a == 0xFFFA || a == 0xFFFB) mmc5_clear_frame_irq_on_nmi_vector();
    if (a == 0x5010) {
        uint8_t status = (uint8_t)((mmc5.pcm_irq_enabled && mmc5.pcm_irq_pending) ? 0x80u : 0u);
        status |= 0x01u;
        mmc5.pcm_irq_pending = false;
        mmc5_update_irq_line();
        return status;
    }
    if (a == 0x5015) {
        return (uint8_t)((mmc5.pulse[0].length_counter ? 0x01u : 0u)
                       | (mmc5.pulse[1].length_counter ? 0x02u : 0u));
    }
    if (a >= 0x5C00 && a <= 0x5FFF) {
        uint8_t mode = (uint8_t)(mmc5.exram_mode & 0x03);
        if (mode == 0 || mode == 1) return cart_cpu_bus_input;
        return mmc5_exram[a - 0x5C00u];
    }
    if (a == 0x5204) {
        uint8_t status = 0;
        if (mmc5.irq_pending) status |= 0x80;
        if (mmc5.in_frame) status |= 0x40;
        mmc5.irq_pending = false;
        mmc5_update_irq_line();
        return status;
    }
    if (a == 0x5205) {
        uint16_t product = (uint16_t)mmc5.mul_a * (uint16_t)mmc5.mul_b;
        return (uint8_t)(product & 0xFF);
    }
    if (a == 0x5206) {
        uint16_t product = (uint16_t)mmc5.mul_a * (uint16_t)mmc5.mul_b;
        return (uint8_t)(product >> 8);
    }

    uint8_t value;
    if (mmc5_ram_mapped(a)) {
        size_t offset = 0;
        RamBlock *ram = mmc5_ram_location(a, &offset);
        value = ram_read(ram, offset);
    } else if (a >= 0x8000) {
        if (C.prg_sz < PRG_BANK_8K) {
            value = repeated_prg_window_read(a, 0x8000, PRG_BANK_32K);
            if (mmc5.pcm_read_mode && a <= 0xBFFF) mmc5_pcm_write(value);
            return value;
        }
        size_t banks = mmc5_prg_bank_count_8k();
        if (banks == 0) return cart_cpu_bus_input;

        size_t slot = (size_t)((a - 0x8000u) >> 13); // 0..3
        size_t bank = mmc5_map_prg_slot_to_bank(slot);
        value = C.prg[bank * PRG_BANK_8K + (a & 0x1FFF)];
    } else {
        return cart_cpu_bus_input;
    }
    if (mmc5.pcm_read_mode && a >= 0x8000 && a <= 0xBFFF) mmc5_pcm_write(value);
    return value;
}

static void mmc5_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x5000 && a <= 0x5003) {
        mmc5_pulse_write(0, a, v);
        return;
    }
    if (a >= 0x5004 && a <= 0x5007) {
        mmc5_pulse_write(1, a, v);
        return;
    }
    if (a == 0x5010) {
        mmc5.pcm_read_mode = (v & 0x01u) != 0;
        mmc5.pcm_irq_enabled = (v & 0x80u) != 0;
        mmc5_update_irq_line();
        return;
    }
    if (a == 0x5011) {
        if (!mmc5.pcm_read_mode) mmc5_pcm_write(v);
        return;
    }
    if (a == 0x5015) {
        for (unsigned i = 0; i < 2; ++i) {
            bool enabled = (v & (1u << i)) != 0;
            mmc5.pulse[i].enabled = enabled;
            if (!enabled) mmc5.pulse[i].length_counter = 0;
        }
        return;
    }
    if (a >= 0x5C00 && a <= 0x5FFF) {
        uint8_t mode = (uint8_t)(mmc5.exram_mode & 0x03);
        if (mode != 3) {
            size_t offset = a - 0x5C00u;
            uint8_t value = (mode <= 1 && !mmc5.in_frame) ? 0 : v;
            if (mmc5_exram[offset] != value) {
                mmc5_exram[offset] = value;
                if (battery_enabled) mmc5_exram_dirty = true;
            }
        }
        return;
    }
    if (a >= 0x6000) {
        bool prg_ram_write_enable = ((mmc5.prg_ram_protect1 & 0x03) == 0x02)
                                 && ((mmc5.prg_ram_protect2 & 0x03) == 0x01);
        if (prg_ram_write_enable && mmc5_ram_mapped(a)) {
            size_t offset = 0;
            RamBlock *ram = mmc5_ram_location(a, &offset);
            ram_write(ram, offset, v);
        }
        return;
    }

    if (a == 0x5100) {
        mmc5.prg_mode = v & 0x03;
    } else if (a == 0x5101) {
        mmc5.chr_mode = v & 0x03;
    } else if (a == 0x5102) {
        mmc5.prg_ram_protect1 = v;
    } else if (a == 0x5103) {
        mmc5.prg_ram_protect2 = v;
    } else if (a == 0x5104) {
        mmc5.exram_mode = (uint8_t)(v & 0x03);
    } else if (a == 0x5106) {
        mmc5.fill_tile = v;
    } else if (a == 0x5107) {
        mmc5.fill_attr = (uint8_t)(v & 0x03);
    } else if (a == 0x5105) {
        mmc5.nt_control = v;
        mmc5_update_mirroring(v);
    } else if (a == 0x5113) {
        mmc5.prg_ram_bank = v;
    } else if (a >= 0x5114 && a <= 0x5117) {
        mmc5.prg_regs[a - 0x5114] = v;
    } else if (a >= 0x5120 && a <= 0x5127) {
        mmc5.chr_regs_a[a - 0x5120] = (uint16_t)(v | ((uint16_t)(mmc5.chr_upper & 0x03) << 8));
        mmc5.chr_last_set_b = false;
    } else if (a >= 0x5128 && a <= 0x512B) {
        mmc5.chr_regs_b[a - 0x5128] = (uint16_t)(v | ((uint16_t)(mmc5.chr_upper & 0x03) << 8));
        mmc5.chr_last_set_b = mmc5.ppu_large_sprites;
    } else if (a == 0x5130) {
        mmc5.chr_upper = v & 0x03;
    } else if (a == 0x5200) {
        mmc5.split_enabled = (v & 0x80u) != 0;
        mmc5.split_right = (v & 0x40u) != 0;
        mmc5.split_delimiter = v & 0x1Fu;
    } else if (a == 0x5201) {
        mmc5.split_scroll = v;
    } else if (a == 0x5202) {
        mmc5.split_bank = v;
    } else if (a == 0x5203) {
        mmc5.irq_scanline = v;
    } else if (a == 0x5204) {
        mmc5.irq_enabled = (v & 0x80) != 0;
        // Writing only toggles enable; pending state is retained.
        mmc5_update_irq_line();
    } else if (a == 0x5205) {
        mmc5.mul_a = v;
    } else if (a == 0x5206) {
        mmc5.mul_b = v;
    }
}

static uint8_t mmc5_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    mmc5_begin_ppu_read(a);
    uint8_t value;
    if (mmc5_split_chr_read(a, &value)) return value;
    if (mmc5_extended_attr_read(a, false, &value)) return value;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return nrom_ppu_read(a);

    size_t bank = mmc5_map_chr_bank_1k(a);
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc5_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) { nrom_ppu_write(a, v); return; }

    size_t bank = mmc5_map_chr_bank_1k(a);
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FF), v);
}

static Mirroring mmc5_mirr(void) { return mmc5.mirr; }
static void mmc5_reset(void) {
    memset(&mmc5, 0, sizeof(mmc5));
    memset(mmc5_exram, 0, sizeof(mmc5_exram));
    mmc5_exram_dirty = false;
    mmc5.mirr = MIRROR_SINGLE0;
    mmc5.prg_mode = 3;
    mmc5.prg_regs[3] = 0xFF; // Reset vectors are in the last ROM bank.
    mmc5_update_irq_line();
}

#endif // MAPPER_MMC5_H

