/*
 * mapper_expansion.h - Expansion cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * This private header contains cartridge mapper state and behavior for boards with
 * expansion hardware.
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
#ifndef MAPPER_EXPANSION_H
#define MAPPER_EXPANSION_H

// Mappers 24/26: Konami VRC6.
static uint16_t vrc6_decode_register(uint16_t addr) {
    if (!vrc6.variant_b) return addr;
    return (uint16_t)((addr & 0xFFFCu) | ((addr & 0x0001u) << 1) | ((addr & 0x0002u) >> 1));
}

static void vrc6_pulse_write(Vrc6Pulse *pulse, uint16_t addr, uint8_t value) {
    switch (addr & 3u) {
        case 0:
            pulse->volume = value & 0x0Fu;
            pulse->duty = (value >> 4) & 7u;
            pulse->ignore_duty = (value & 0x80u) != 0;
            break;
        case 1:
            pulse->frequency = (uint16_t)((pulse->frequency & 0x0F00u) | value);
            break;
        case 2:
            pulse->frequency = (uint16_t)((pulse->frequency & 0x00FFu) | ((uint16_t)(value & 0x0Fu) << 8));
            pulse->enabled = (value & 0x80u) != 0;
            if (!pulse->enabled) pulse->step = 0;
            break;
    }
}

static void vrc6_saw_write(uint16_t addr, uint8_t value) {
    switch (addr & 3u) {
        case 0:
            vrc6.saw.accumulator_rate = value & 0x3Fu;
            break;
        case 1:
            vrc6.saw.frequency = (uint16_t)((vrc6.saw.frequency & 0x0F00u) | value);
            break;
        case 2:
            vrc6.saw.frequency = (uint16_t)((vrc6.saw.frequency & 0x00FFu) | ((uint16_t)(value & 0x0Fu) << 8));
            vrc6.saw.enabled = (value & 0x80u) != 0;
            if (!vrc6.saw.enabled) {
                vrc6.saw.accumulator = 0;
                vrc6.saw.step = 0;
            }
            break;
    }
}

static void vrc6_audio_write(uint16_t addr, uint8_t value) {
    switch (addr & 0xF003u) {
        case 0x9000: case 0x9001: case 0x9002:
            vrc6_pulse_write(&vrc6.pulse[0], addr, value);
            break;
        case 0x9003:
            vrc6.halt_audio = (value & 0x01u) != 0;
            vrc6.frequency_shift = (value & 0x04u) ? 8u : ((value & 0x02u) ? 4u : 0u);
            break;
        case 0xA000: case 0xA001: case 0xA002:
            vrc6_pulse_write(&vrc6.pulse[1], addr, value);
            break;
        case 0xB000: case 0xB001: case 0xB002:
            vrc6_saw_write(addr, value);
            break;
    }
}

static void vrc6_clock_pulse(Vrc6Pulse *pulse) {
    if (!pulse->enabled) return;
    pulse->timer--;
    if (pulse->timer == 0) {
        pulse->step = (uint8_t)((pulse->step + 1u) & 0x0Fu);
        pulse->timer = (int32_t)(pulse->frequency >> vrc6.frequency_shift) + 1;
    }
}

static void vrc6_clock_saw(void) {
    if (!vrc6.saw.enabled) return;
    vrc6.saw.timer--;
    if (vrc6.saw.timer == 0) {
        vrc6.saw.step = (uint8_t)((vrc6.saw.step + 1u) % 14u);
        vrc6.saw.timer = (int32_t)(vrc6.saw.frequency >> vrc6.frequency_shift) + 1;
        if (vrc6.saw.step == 0) {
            vrc6.saw.accumulator = 0;
        } else if ((vrc6.saw.step & 1u) == 0) {
            vrc6.saw.accumulator = (uint8_t)(vrc6.saw.accumulator + vrc6.saw.accumulator_rate);
        }
    }
}

static void vrc6_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        vrc_irq_clock(&vrc6.irq);
        if (!vrc6.halt_audio) {
            vrc6_clock_pulse(&vrc6.pulse[0]);
            vrc6_clock_pulse(&vrc6.pulse[1]);
            vrc6_clock_saw();
        }
    }
}

static size_t vrc6_chr_bank(unsigned slot) {
    unsigned mode = vrc6.banking_mode & 3u;
    uint8_t mask = (vrc6.banking_mode & 0x20u) ? 0xFEu : 0xFFu;
    uint8_t or_mask = (vrc6.banking_mode & 0x20u) ? 1u : 0u;

    if (mode == 0) return vrc6.chr_regs[slot];
    if (mode == 1) {
        uint8_t reg = vrc6.chr_regs[slot >> 1];
        return (size_t)((reg & mask) | ((slot & 1u) ? or_mask : 0u));
    }
    if (slot < 4) return vrc6.chr_regs[slot];
    uint8_t reg = vrc6.chr_regs[4u + ((slot - 4u) >> 1)];
    return (size_t)((reg & mask) | ((slot & 1u) ? or_mask : 0u));
}

static uint8_t vrc6_ciram_page(unsigned nt) {
    switch (vrc6.banking_mode & 0x2Fu) {
        case 0x20: case 0x27: return (uint8_t)(nt & 1u);
        case 0x23: case 0x24: return (uint8_t)(nt >> 1);
        case 0x28: case 0x2F: return 0;
        case 0x2B: case 0x2C: return 1;
        default:
            switch (vrc6.banking_mode & 7u) {
                case 0: case 6: case 7:
                    return (nt < 2 ? vrc6.chr_regs[6] : vrc6.chr_regs[7]) & 1u;
                case 1: case 5:
                    return vrc6.chr_regs[4u + nt] & 1u;
                default:
                    return vrc6.chr_regs[6u + (nt & 1u)] & 1u;
            }
    }
}

static size_t vrc6_nt_chr_bank(unsigned nt) {
    switch (vrc6.banking_mode & 0x2Fu) {
        case 0x20: case 0x27:
            return (vrc6.chr_regs[6u + (nt >> 1)] & 0xFEu) | (nt & 1u);
        case 0x23: case 0x24:
            return (vrc6.chr_regs[6u + (nt & 1u)] & 0xFEu) | (nt >> 1);
        case 0x28: case 0x2F:
            return vrc6.chr_regs[6u + (nt >> 1)] & 0xFEu;
        case 0x2B: case 0x2C:
            return (vrc6.chr_regs[6u + (nt & 1u)] & 0xFEu) | 1u;
        default:
            switch (vrc6.banking_mode & 7u) {
                case 0: case 6: case 7:
                    return nt < 2 ? vrc6.chr_regs[6] : vrc6.chr_regs[7];
                case 1: case 5:
                    return vrc6.chr_regs[4u + nt];
                default:
                    return vrc6.chr_regs[6u + (nt & 1u)];
            }
    }
}

static uint8_t vrc6_cpu_read(uint16_t addr) {
    if (addr >= 0x6000 && addr < 0x8000) {
        return (!vrc6.ppu_initialized || (vrc6.banking_mode & 0x80u))
            ? prg_ram_read(addr) : cart_cpu_bus_input;
    }
    if (addr < 0x8000) return cart_cpu_bus_input;

    size_t banks8 = C.prg_sz / PRG_BANK_8K;
    if (addr < 0xC000) {
        if (!vrc6.prg16_selected) return cart_cpu_bus_input;
        size_t bank = ((size_t)vrc6.prg16_bank * 2u + ((addr >> 13) & 1u)) % banks8;
        return C.prg[bank * PRG_BANK_8K + (addr & 0x1FFFu)];
    }
    if (addr < 0xE000) {
        if (!vrc6.prg8_selected) return cart_cpu_bus_input;
        size_t bank = vrc6.prg8_bank % banks8;
        return C.prg[bank * PRG_BANK_8K + (addr & 0x1FFFu)];
    }
    return C.prg[(banks8 - 1) * PRG_BANK_8K + (addr & 0x1FFFu)];
}

static void vrc6_cpu_write(uint16_t addr, uint8_t value) {
    if (addr >= 0x6000 && addr < 0x8000) {
        if (!vrc6.ppu_initialized || (vrc6.banking_mode & 0x80u)) prg_ram_write(addr, value);
        return;
    }
    if (addr < 0x8000) return;

    addr = vrc6_decode_register(addr);
    switch (addr & 0xF003u) {
        case 0x8000: case 0x8001: case 0x8002: case 0x8003:
            vrc6.prg16_bank = value & 0x0Fu;
            vrc6.prg16_selected = true;
            break;
        case 0x9000: case 0x9001: case 0x9002: case 0x9003:
        case 0xA000: case 0xA001: case 0xA002:
        case 0xB000: case 0xB001: case 0xB002:
            vrc6_audio_write(addr, value);
            break;
        case 0xB003:
            vrc6.banking_mode = value;
            vrc6.ppu_initialized = true;
            break;
        case 0xC000: case 0xC001: case 0xC002: case 0xC003:
            vrc6.prg8_bank = value & 0x1Fu;
            vrc6.prg8_selected = true;
            break;
        case 0xD000: case 0xD001: case 0xD002: case 0xD003:
            vrc6.chr_regs[addr & 3u] = value;
            vrc6.ppu_initialized = true;
            break;
        case 0xE000: case 0xE001: case 0xE002: case 0xE003:
            vrc6.chr_regs[4u + (addr & 3u)] = value;
            vrc6.ppu_initialized = true;
            break;
        case 0xF000:
            vrc6.irq.reload = value;
            break;
        case 0xF001:
            vrc_irq_control(&vrc6.irq, value);
            break;
        case 0xF002:
            vrc_irq_ack(&vrc6.irq);
            break;
    }
}

static uint8_t vrc6_ppu_read(uint16_t addr) {
    addr &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!vrc6.ppu_initialized
        || !shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_unmapped_read(addr);
    return C.chr[shrunk_chr_bank_offset(addr, page_size, page_count, vrc6_chr_bank(slot))];
}

static void vrc6_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!vrc6.ppu_initialized
        || !shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count)) {
        chr_ram_write(addr % C.chr_sz, value);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(addr, page_size, page_count, vrc6_chr_bank(slot)), value);
}

static Mirroring vrc6_mirr(void) {
    if (!vrc6.ppu_initialized) return C.mirr_base;
    uint8_t pages[4];
    for (unsigned nt = 0; nt < 4; ++nt) pages[nt] = vrc6_ciram_page(nt);
    if (pages[0] == 0 && pages[1] == 1 && pages[2] == 0 && pages[3] == 1) return MIRROR_VERTICAL;
    if (pages[0] == 0 && pages[1] == 0 && pages[2] == 1 && pages[3] == 1) return MIRROR_HORIZONTAL;
    if (pages[0] == 0 && pages[1] == 0 && pages[2] == 0 && pages[3] == 0) return MIRROR_SINGLE0;
    if (pages[0] == 1 && pages[1] == 1 && pages[2] == 1 && pages[3] == 1) return MIRROR_SINGLE1;
    return C.mirr_base;
}

static void vrc6_reset(void) {
    bool variant_b = vrc6.variant_b;
    memset(&vrc6, 0, sizeof(vrc6));
    vrc6.variant_b = variant_b;
    vrc6.pulse[0].frequency = vrc6.pulse[1].frequency = 1;
    vrc6.pulse[0].timer = vrc6.pulse[1].timer = 1;
    vrc6.saw.frequency = 1;
    vrc6.saw.timer = 1;
    vrc_irq_reset(&vrc6.irq);
    mapper_irq_line = false;
}

uint8_t cart_nt_read(uint16_t addr, uint8_t *nt_ram) {
    if (active_board) return board_ppu_read(active_board, addr, cart_ppu_fetch_source);
    if (cart == &mapper_m111) {
        (void)nt_ram;
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x1FFFu);
        size_t group = (m111.bank_latch & 0x20u) ? 8u : 0u;
        size_t page = group + ((off >> 10) & 7u);
        return m111_nt_ram[page * CHR_BANK_1K + (off & 0x03FFu)];
    }
    if (cart == &mapper_jy) {
        if (jy.irq_source == JY_IRQ_PPU_READ && cart_ppu_fetch_source != CART_PPU_FETCH_CPU)
            jy_irq_tick();
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned slot = (off >> 10) & 3u;
        uint16_t in = off & 0x03FFu;
        if (jy_advanced_nt()
            && (jy.disable_nt_ram
                || (jy.nt_low[slot] & 0x80u) != (jy.nt_ram_select_bit & 0x80u))) {
            size_t page = (size_t)jy.nt_low[slot] | ((size_t)jy.nt_high[slot] << 8);
            size_t offset = page * CHR_BANK_1K + in;
            return !C.chr_is_ram && offset < C.chr_sz ? C.chr[offset] : 0;
        }
        return nt_ram[(size_t)jy_ciram_page(slot) * CHR_BANK_1K + in];
    }
    if (cart == &mapper_txsrom) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        return nt_ram[(size_t)txsrom_nt[off >> 10] * 0x400u + (off & 0x03FFu)];
    }
    if (cart == &mapper_unrom512 && unrom512_four_screen_chr) {
        size_t offset = 0x6000u + ((addr - 0x2000u) & 0x1FFFu);
        if (C.chr_is_ram) return C.chr[offset];
        RamBlock *ram = chr_save_ram.size ? &chr_save_ram : &chr_work_ram;
        return ram->data[offset];
    }
    if (cart == &mapper_vrc6 && vrc6.ppu_initialized) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t in = off & 0x03FFu;
        if (vrc6.banking_mode & 0x10u) {
            size_t offset = shrunk_chr_window_offset(in, CHR_BANK_1K, vrc6_nt_chr_bank(nt));
            return C.chr[offset];
        }
        return nt_ram[(size_t)vrc6_ciram_page(nt) * 0x400u + in];
    }
    if (cart == &mapper_sunsoft4 && sunsoft4.use_chr_nt) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t base = ((size_t)sunsoft4.nt[sunsoft4_nt_reg(nt)] * CHR_BANK_1K) % C.chr_sz;
        size_t in = off & 0x03FFu;
        return base + in < C.chr_sz ? C.chr[base + in] : (uint8_t)addr;
    }
    if (cart == &mapper_mmc5) {
        mmc5_begin_ppu_read(addr);
        uint8_t value;
        if (mmc5_split_nt_read(addr, &value)) return value;
        if (mmc5_extended_attr_read(addr, true, &value)) return value;
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint16_t in  = (uint16_t)(off & 0x03FFu);
        uint8_t src = mmc5_nt_source(addr);

        switch (src) {
            case 0: return nt_ram[in];
            case 1: return nt_ram[0x400u + in];
            case 2:
                if ((mmc5.exram_mode & 0x03) >= 2) return 0x00;
                return mmc5_exram[in];
            case 3:
                if (in < 0x03C0u) return mmc5.fill_tile;
                return mmc5_fill_attr_byte();
            default:
                return nt_ram[in];
        }
    }
    if (cart == &mapper_namco) {
        uint16_t mapped = addr >= 0x3000u ? (uint16_t)(addr - 0x1000u) : addr;
        if (mapped >= 0x2000u && mapped < 0x3000u) {
            size_t chunk = mapped >> 8;
            size_t offset = namco_ppu_offset[chunk] + (mapped & 0xFFu);
            if (namco_ppu_source[chunk] == NAMCO_PPU_CIRAM)
                return nt_ram[offset & 0x07FFu];
            if (namco_ppu_source[chunk] == NAMCO_PPU_CHR)
                return offset < C.chr_sz ? C.chr[offset] : (uint8_t)addr;
        }
    }
    if (cart == &mapper_namco108 && C.mapper_no == 95 && namco108.nametables_selected) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        uint8_t page = (uint8_t)((namco108.banks[nt < 2 ? 0 : 1] >> 5) & 1u);
        return nt_ram[(size_t)page * CHR_BANK_1K + (off & 0x03FFu)];
    }

    if (cart == &mapper_rambo158) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint8_t quadrant = (uint8_t)((off >> 10) & 3u);
        uint16_t in = off & 0x03FFu;
        return nt_ram[(size_t)(rambo1.nt_map[quadrant] & 1u) * 0x400u + in];
    }

    return nt_ram[base_nt_index(addr)];
}

void cart_nt_write(uint16_t addr, uint8_t v, uint8_t *nt_ram) {
    if (active_board) {
        board_ppu_write(active_board, addr, v);
        return;
    }
    if (cart == &mapper_m111) {
        (void)nt_ram;
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x1FFFu);
        size_t group = (m111.bank_latch & 0x20u) ? 8u : 0u;
        size_t page = group + ((off >> 10) & 7u);
        m111_nt_ram[page * CHR_BANK_1K + (off & 0x03FFu)] = v;
        return;
    }
    if (cart == &mapper_jy) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned slot = (off >> 10) & 3u;
        nt_ram[(size_t)jy_ciram_page(slot) * CHR_BANK_1K + (off & 0x03FFu)] = v;
        return;
    }
    if (cart == &mapper_txsrom) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        nt_ram[(size_t)txsrom_nt[off >> 10] * 0x400u + (off & 0x03FFu)] = v;
        return;
    }
    if (cart == &mapper_unrom512 && unrom512_four_screen_chr) {
        size_t offset = 0x6000u + ((addr - 0x2000u) & 0x1FFFu);
        if (C.chr_is_ram) {
            chr_ram_write(offset, v);
        } else {
            RamBlock *ram = chr_save_ram.size ? &chr_save_ram : &chr_work_ram;
            if (ram->data[offset] != v) {
                ram->data[offset] = v;
                if (ram == &chr_save_ram && battery_enabled) chr_ram_dirty = true;
            }
        }
        return;
    }
    if (cart == &mapper_vrc6 && vrc6.ppu_initialized) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t in = off & 0x03FFu;
        if (vrc6.banking_mode & 0x10u) {
            if (!C.chr_is_ram) return;
            size_t offset = shrunk_chr_window_offset(in, CHR_BANK_1K, vrc6_nt_chr_bank(nt));
            chr_ram_write(offset, v);
            return;
        }
        nt_ram[(size_t)vrc6_ciram_page(nt) * 0x400u + in] = v;
        return;
    }
    if (cart == &mapper_sunsoft4 && sunsoft4.use_chr_nt) {
        if (!C.chr_is_ram) return;
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t base = ((size_t)sunsoft4.nt[sunsoft4_nt_reg(nt)] * CHR_BANK_1K) % C.chr_sz;
        size_t in = off & 0x03FFu;
        if (base + in < C.chr_sz) chr_ram_write(base + in, v);
        return;
    }
    if (cart == &mapper_mmc5) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint16_t in  = (uint16_t)(off & 0x03FFu);
        uint8_t src = mmc5_nt_source(addr);

        switch (src) {
            case 0:
                nt_ram[in] = v;
                return;
            case 1:
                nt_ram[0x400u + in] = v;
                return;
            case 2:
                if ((mmc5.exram_mode & 0x03) >= 2) return;
                if (mmc5_exram[in] != v) {
                    mmc5_exram[in] = v;
                    if (battery_enabled) mmc5_exram_dirty = true;
                }
                return;
            case 3:
                // Fill mode is controlled by $5106/$5107.
                return;
            default:
                nt_ram[in] = v;
                return;
        }
    }
    if (cart == &mapper_namco) {
        uint16_t mapped = addr >= 0x3000u ? (uint16_t)(addr - 0x1000u) : addr;
        if (mapped >= 0x2000u && mapped < 0x3000u) {
            size_t chunk = mapped >> 8;
            size_t offset = namco_ppu_offset[chunk] + (mapped & 0xFFu);
            if (namco_ppu_source[chunk] == NAMCO_PPU_CIRAM) {
                nt_ram[offset & 0x07FFu] = v;
                return;
            }
            if (namco_ppu_source[chunk] == NAMCO_PPU_CHR) {
                if (C.chr_is_ram && offset < C.chr_sz) chr_ram_write(offset, v);
                return;
            }
        }
    }
    if (cart == &mapper_namco108 && C.mapper_no == 95 && namco108.nametables_selected) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        uint8_t page = (uint8_t)((namco108.banks[nt < 2 ? 0 : 1] >> 5) & 1u);
        nt_ram[(size_t)page * CHR_BANK_1K + (off & 0x03FFu)] = v;
        return;
    }

    if (cart == &mapper_rambo158) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint8_t quadrant = (uint8_t)((off >> 10) & 3u);
        uint16_t in = off & 0x03FFu;
        nt_ram[(size_t)(rambo1.nt_map[quadrant] & 1u) * 0x400u + in] = v;
        return;
    }

    nt_ram[base_nt_index(addr)] = v;
}

// Mapper 7: AOROM.
static struct { uint8_t prg_bank; Mirroring mirr; } ao;
static uint8_t aorom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t b = (banks ? ao.prg_bank % banks : 0);
        return C.prg[b * PRG_BANK_32K + (a - 0x8000)];
    }
    return cart_cpu_bus_input;
}
static void aorom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        ao.prg_bank = v & 0x0F;
        ao.mirr = (v & 0x10) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    }
}
static uint8_t aorom_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void aorom_ppu_write(uint16_t a, uint8_t v) { nrom_ppu_write(a, v); }
static Mirroring aorom_mirr(void) { return ao.mirr; }
static void aorom_reset(void) { ao.prg_bank = 0; ao.mirr = MIRROR_SINGLE0; }

// Mapper 9: MMC2/PxROM.
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    uint8_t selected_chr_bank[2];
    bool chr_mapped[2];
    bool need_chr_update;
    Mirroring mirr;
} mmc2;

static void mmc2_select_chr_slot(unsigned slot) {
    uint8_t bank_idx = slot ? (uint8_t)(2 + mmc2.latch[1]) : mmc2.latch[0];
    mmc2.selected_chr_bank[slot] = mmc2.chr_banks[bank_idx];
    mmc2.chr_mapped[slot] = true;
}

static void mmc2_notify_ppu_address(uint16_t a) {
    if (mmc2.need_chr_update) {
        mmc2_select_chr_slot(0);
        mmc2_select_chr_slot(1);
        mmc2.need_chr_update = false;
    }
    if (a == 0x0FD8) {
        mmc2.latch[0] = 0;
        mmc2.need_chr_update = true;
    } else if (a == 0x0FE8) {
        mmc2.latch[0] = 1;
        mmc2.need_chr_update = true;
    } else if (a >= 0x1FD8 && a <= 0x1FDF) {
        mmc2.latch[1] = 0;
        mmc2.need_chr_update = true;
    } else if (a >= 0x1FE8 && a <= 0x1FEF) {
        mmc2.latch[1] = 1;
        mmc2.need_chr_update = true;
    }
}

static uint8_t mmc2_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000 && a < 0xA000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        size_t bank = mmc2.prg_bank % banks;
        return C.prg[bank * PRG_BANK_8K + (a - 0x8000)];
    }
    if (a >= 0xA000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        size_t slot = (size_t)((a - 0xA000) / PRG_BANK_8K);
        size_t bank = (banks + slot - (3 % banks)) % banks;
        return C.prg[bank * PRG_BANK_8K + ((a - 0xA000) & 0x1FFF)];
    }
    return cart_cpu_bus_input;
}

static void mmc2_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0xA000 && a <= 0xAFFF) mmc2.prg_bank = v & 0x0F;
    else if (a >= 0xB000 && a <= 0xBFFF) { mmc2.chr_banks[0] = v & 0x1F; mmc2_select_chr_slot(0); }
    else if (a >= 0xC000 && a <= 0xCFFF) { mmc2.chr_banks[1] = v & 0x1F; mmc2_select_chr_slot(0); }
    else if (a >= 0xD000 && a <= 0xDFFF) { mmc2.chr_banks[2] = v & 0x1F; mmc2_select_chr_slot(1); }
    else if (a >= 0xE000 && a <= 0xEFFF) { mmc2.chr_banks[3] = v & 0x1F; mmc2_select_chr_slot(1); }
    else if (a >= 0xF000) mmc2.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc2_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    size_t slot = page_size ? a / page_size : 2;
    uint8_t val = chr_unmapped_read(a);
    if (slot < 2 && mmc2.chr_mapped[slot]) {
        size_t bank = mmc2.selected_chr_bank[slot] % (C.chr_sz / page_size);
        val = C.chr[bank * page_size + (a % page_size)];
    }
    return val;
}

static void mmc2_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    size_t slot = page_size ? a / page_size : 2;
    if (slot >= 2 || !mmc2.chr_mapped[slot]) { chr_ram_write(a % C.chr_sz, v); return; }
    size_t bank = mmc2.selected_chr_bank[slot] % (C.chr_sz / page_size);
    chr_ram_write(bank * page_size + (a % page_size), v);
}

static Mirroring mmc2_mirr(void) { return mmc2.mirr; }
static void mmc2_reset(void) {
    memset(&mmc2, 0, sizeof(mmc2));
    mmc2.latch[0] = mmc2.latch[1] = 1;
    mmc2.mirr = C.mirr_base;
}

// Mapper 10: MMC4/FxROM.
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    uint8_t selected_chr_bank[2];
    bool chr_mapped[2];
    bool need_chr_update;
    Mirroring mirr;
} mmc4;

static void mmc4_select_chr_slot(unsigned slot) {
    uint8_t bank_idx = slot ? (uint8_t)(2 + mmc4.latch[1]) : mmc4.latch[0];
    mmc4.selected_chr_bank[slot] = mmc4.chr_banks[bank_idx];
    mmc4.chr_mapped[slot] = true;
}

static void mmc4_notify_ppu_address(uint16_t a) {
    if (mmc4.need_chr_update) {
        mmc4_select_chr_slot(0);
        mmc4_select_chr_slot(1);
        mmc4.need_chr_update = false;
    }
    if (a >= 0x0FD8 && a <= 0x0FDF) {
        mmc4.latch[0] = 0;
        mmc4.need_chr_update = true;
    } else if (a >= 0x0FE8 && a <= 0x0FEF) {
        mmc4.latch[0] = 1;
        mmc4.need_chr_update = true;
    } else if (a >= 0x1FD8 && a <= 0x1FDF) {
        mmc4.latch[1] = 0;
        mmc4.need_chr_update = true;
    } else if (a >= 0x1FE8 && a <= 0x1FEF) {
        mmc4.latch[1] = 1;
        mmc4.need_chr_update = true;
    }
}

static uint8_t mmc4_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000 && a < 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = mmc4.prg_bank % banks;
        return C.prg[bank * PRG_BANK_16K + (a - 0x8000)];
    }
    if (a >= 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = (banks > 0) ? banks - 1 : 0;
        return C.prg[bank * PRG_BANK_16K + (a - 0xC000)];
    }
    return cart_cpu_bus_input;
}

static void mmc4_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0xA000 && a <= 0xAFFF) mmc4.prg_bank = v & 0x0F;
    else if (a >= 0xB000 && a <= 0xBFFF) { mmc4.chr_banks[0] = v & 0x1F; mmc4_select_chr_slot(0); }
    else if (a >= 0xC000 && a <= 0xCFFF) { mmc4.chr_banks[1] = v & 0x1F; mmc4_select_chr_slot(0); }
    else if (a >= 0xD000 && a <= 0xDFFF) { mmc4.chr_banks[2] = v & 0x1F; mmc4_select_chr_slot(1); }
    else if (a >= 0xE000 && a <= 0xEFFF) { mmc4.chr_banks[3] = v & 0x1F; mmc4_select_chr_slot(1); }
    else if (a >= 0xF000) mmc4.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc4_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    size_t slot = page_size ? a / page_size : 2;
    uint8_t val = chr_unmapped_read(a);
    if (slot < 2 && mmc4.chr_mapped[slot]) {
        size_t bank = mmc4.selected_chr_bank[slot] % (C.chr_sz / page_size);
        val = C.chr[bank * page_size + (a % page_size)];
    }
    return val;
}

static void mmc4_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    size_t slot = page_size ? a / page_size : 2;
    if (slot >= 2 || !mmc4.chr_mapped[slot]) { chr_ram_write(a % C.chr_sz, v); return; }
    size_t bank = mmc4.selected_chr_bank[slot] % (C.chr_sz / page_size);
    chr_ram_write(bank * page_size + (a % page_size), v);
}

static Mirroring mmc4_mirr(void) { return mmc4.mirr; }
static void mmc4_reset(void) {
    memset(&mmc4, 0, sizeof(mmc4));
    mmc4.latch[0] = mmc4.latch[1] = 1;
    mmc4.mirr = C.mirr_base;
}

// Mappers 19/210: Namco 163, 175, and 340.
static void namco_set_variant(NamcoVariant variant) {
    if (!namco.auto_detect) return;
    if (!namco.not_340 || variant != NAMCO_VARIANT_340) namco.variant = variant;
}

static size_t namco_prg_bank(uint8_t bank) {
    size_t banks = C.prg_sz / PRG_BANK_8K;
    return banks ? (size_t)(bank & 0x3Fu) % banks : 0;
}

static void namco_map_ppu_page(unsigned slot, uint8_t value, bool ciram) {
    size_t source_size = ciram ? 0x0800u : C.chr_sz;
    size_t page_size = ciram ? CHR_BANK_1K : shrunk_chr_page_size(CHR_BANK_1K);
    if (!source_size || page_size < 0x100u || (page_size & 0xFFu) != 0) return;

    size_t page_count = source_size / page_size;
    if (!page_count) return;

    size_t page = ciram ? (size_t)(value & 1u) : (size_t)value % page_count;
    size_t start = (size_t)slot * page_size;
    size_t chunks = page_size / 0x100u;
    for (size_t i = 0; i < chunks; ++i) {
        size_t destination = start / 0x100u + i;
        if (destination >= sizeof(namco_ppu_source)) break;
        namco_ppu_source[destination] = ciram ? NAMCO_PPU_CIRAM : NAMCO_PPU_CHR;
        namco_ppu_offset[destination] = page * page_size + i * 0x100u;
    }
}

static void namco_clear_nt_ppu_mapping(void) {
    memset(namco_ppu_source + 0x20, 0, 0x10);
    memset(namco_ppu_offset + 0x20, 0, sizeof(namco_ppu_offset[0]) * 0x10);
}

static bool namco_ram_write_allowed(uint16_t address) {
    if (!default_prg_ram()->size) return false;
    if (namco.variant == NAMCO_VARIANT_163) {
        unsigned block = (unsigned)((address - 0x6000u) >> 11);
        return (namco.write_protect & 0x40u) != 0
            && (namco.write_protect & (1u << block)) == 0;
    }
    if (namco.variant == NAMCO_VARIANT_175)
        return (namco.write_protect & 0x01u) != 0;
    return false;
}

static bool namco_ram_read_allowed(void) {
    return default_prg_ram()->size
        && (namco.variant == NAMCO_VARIANT_163 || namco.variant == NAMCO_VARIANT_175);
}

static uint8_t namco_cpu_read(uint16_t address) {
    uint16_t reg = address & 0xF800u;
    if (reg == 0x4800u && namco.variant == NAMCO_VARIANT_163)
        return namco163_audio_read_data(&namco163_audio);
    if (reg == 0x5000u && namco.variant == NAMCO_VARIANT_163)
        return (uint8_t)namco.irq_counter;
    if (reg == 0x5800u && namco.variant == NAMCO_VARIANT_163)
        return (uint8_t)(namco.irq_counter >> 8);
    if (address >= 0x6000u && address < 0x8000u)
        return namco_ram_read_allowed() ? prg_ram_read(address) : cart_cpu_bus_input;
    if (address >= 0x8000u) {
        unsigned slot = (unsigned)((address - 0x8000u) / PRG_BANK_8K);
        if (slot == 3) {
            size_t bank = C.prg_sz / PRG_BANK_8K - 1;
            return C.prg[bank * PRG_BANK_8K + (address & 0x1FFFu)];
        }
        if (!namco.prg_mapped[slot]) return cart_cpu_bus_input;
        size_t bank = namco_prg_bank(namco.prg_bank[slot]);
        return C.prg[bank * PRG_BANK_8K + (address & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void namco_set_pattern_bank(unsigned slot, uint8_t value) {
    bool low_half = slot < 4;
    bool nt_mode = low_half ? namco.low_chr_nt_mode : namco.high_chr_nt_mode;
    bool ciram = namco.variant == NAMCO_VARIANT_163 && !nt_mode && value >= 0xE0u;
    namco_map_ppu_page(slot, value, ciram);
}

static void namco_set_nt_bank(unsigned slot, uint8_t value) {
    namco_map_ppu_page(8u + slot, value, value >= 0xE0u);
}

static void namco_cpu_write(uint16_t address, uint8_t value) {
    uint16_t reg = address & 0xF800u;
    if (reg == 0x4800u) {
        namco_set_variant(NAMCO_VARIANT_163);
        if (namco.variant == NAMCO_VARIANT_163
            && namco163_audio_write_data(&namco163_audio, value))
            namco163_audio_dirty = true;
        return;
    }
    if (reg == 0x5000u) {
        namco_set_variant(NAMCO_VARIANT_163);
        if (namco.variant == NAMCO_VARIANT_163) {
            namco.irq_counter = (uint16_t)((namco.irq_counter & 0xFF00u) | value);
            mapper_irq_line = false;
        }
        return;
    }
    if (reg == 0x5800u) {
        namco_set_variant(NAMCO_VARIANT_163);
        if (namco.variant == NAMCO_VARIANT_163) {
            namco.irq_counter = (uint16_t)((namco.irq_counter & 0x00FFu) | ((uint16_t)value << 8));
            mapper_irq_line = false;
        }
        return;
    }
    if (address >= 0x6000u && address < 0x8000u) {
        namco.not_340 = true;
        if (namco.variant == NAMCO_VARIANT_340) namco_set_variant(NAMCO_VARIANT_UNKNOWN);
        if (namco_ram_write_allowed(address)) {
            prg_ram_write(address, value);
        }
        return;
    }

    switch (reg) {
        case 0x8000: case 0x8800: case 0x9000: case 0x9800:
            namco_set_pattern_bank((unsigned)((reg - 0x8000u) >> 11), value);
            break;
        case 0xA000: case 0xA800: case 0xB000: case 0xB800:
            namco_set_pattern_bank((unsigned)(((reg - 0xA000u) >> 11) + 4u), value);
            break;
        case 0xC000: case 0xC800: case 0xD000: case 0xD800:
            if (reg >= 0xC800u) namco_set_variant(NAMCO_VARIANT_163);
            else if (namco.variant != NAMCO_VARIANT_163) namco_set_variant(NAMCO_VARIANT_175);
            if (namco.variant == NAMCO_VARIANT_175) {
                namco.write_protect = value;
            } else {
                namco_set_nt_bank((unsigned)((reg - 0xC000u) >> 11), value);
            }
            break;
        case 0xE000:
            if (value & 0x80u) namco_set_variant(NAMCO_VARIANT_340);
            else if ((value & 0x40u) && namco.variant != NAMCO_VARIANT_163)
                namco_set_variant(NAMCO_VARIANT_340);
            namco.prg_bank[0] = value & 0x3Fu;
            namco.prg_mapped[0] = true;
            if (namco.variant == NAMCO_VARIANT_340) {
                static const Mirroring modes[4] = {
                    MIRROR_SINGLE0, MIRROR_VERTICAL, MIRROR_SINGLE1, MIRROR_HORIZONTAL
                };
                namco.mirr = modes[value >> 6];
                namco_clear_nt_ppu_mapping();
            } else if (namco.variant == NAMCO_VARIANT_163) {
                namco163_audio_set_disabled(&namco163_audio, (value & 0x40u) != 0);
            }
            break;
        case 0xE800:
            namco.prg_bank[1] = value & 0x3Fu;
            namco.prg_mapped[1] = true;
            if (namco.variant == NAMCO_VARIANT_163) {
                namco.low_chr_nt_mode = (value & 0x40u) != 0;
                namco.high_chr_nt_mode = (value & 0x80u) != 0;
            }
            break;
        case 0xF000:
            namco.prg_bank[2] = value & 0x3Fu;
            namco.prg_mapped[2] = true;
            break;
        case 0xF800:
            namco_set_variant(NAMCO_VARIANT_163);
            if (namco.variant == NAMCO_VARIANT_163) {
                namco.write_protect = value;
                namco163_audio_write_address(&namco163_audio, value);
            }
            break;
    }
}

static uint8_t namco_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    size_t chunk = address >> 8;
    size_t offset = namco_ppu_offset[chunk] + (address & 0xFFu);
    if (namco_ppu_source[chunk] == NAMCO_PPU_CIRAM)
        return ppu_vram[offset & 0x07FFu];
    if (namco_ppu_source[chunk] == NAMCO_PPU_CHR)
        return offset < C.chr_sz ? C.chr[offset] : (uint8_t)address;
    return chr_unmapped_read(address);
}

static void namco_ppu_write(uint16_t address, uint8_t value) {
    address &= 0x1FFFu;
    size_t chunk = address >> 8;
    size_t offset = namco_ppu_offset[chunk] + (address & 0xFFu);
    if (namco_ppu_source[chunk] == NAMCO_PPU_CIRAM) {
        ppu_vram[offset & 0x07FFu] = value;
        return;
    }
    if (namco_ppu_source[chunk] == NAMCO_PPU_CHR) {
        if (C.chr_is_ram && offset < C.chr_sz) chr_ram_write(offset, value);
        return;
    }
    if (C.chr_is_ram) chr_ram_write(address % C.chr_sz, value);
}

static Mirroring namco_mirr(void) { return namco.mirr; }

static void namco_clock(int cpu_cycles) {
    if (namco.variant == NAMCO_VARIANT_163) {
        for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
            if ((namco.irq_counter & 0x8000u) && (namco.irq_counter & 0x7FFFu) != 0x7FFFu) {
                namco.irq_counter++;
                if ((namco.irq_counter & 0x7FFFu) == 0x7FFFu) mapper_irq_line = true;
            }
        }
        if (namco163_audio_clock(&namco163_audio, cpu_cycles))
            namco163_audio_dirty = true;
    }
}

static void namco_reset(void) {
    NamcoVariant variant = namco.variant;
    bool auto_detect = namco.auto_detect;
    memset(&namco, 0, sizeof(namco));
    namco.variant = variant;
    namco.auto_detect = auto_detect;
    namco.mirr = C.mirr_base;
    memset(namco_ppu_source, 0, sizeof(namco_ppu_source));
    memset(namco_ppu_offset, 0, sizeof(namco_ppu_offset));
    namco163_audio_reset(&namco163_audio);
    namco163_audio_dirty = false;
    mapper_irq_line = false;
}

// Mapper 66: GxROM.
static uint8_t gxrom_cpu_read(uint16_t address) {
    if (address >= 0x6000u && address < 0x8000u) return prg_ram_read(address);
    if (address >= 0x8000u) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t bank = banks ? (size_t)gxrom.prg_bank % banks : 0;
        return C.prg[bank * PRG_BANK_32K + (address & 0x7FFFu)];
    }
    return cart_cpu_bus_input;
}

static void gxrom_cpu_write(uint16_t address, uint8_t value) {
    if (address >= 0x6000u && address < 0x8000u) {
        prg_ram_write(address, value);
        return;
    }
    if (address >= 0x8000u) {
        gxrom.prg_bank = (value >> 4) & 3u;
        gxrom.chr_bank = value & 3u;
    }
}

static uint8_t gxrom_ppu_read(uint16_t address) {
    return discrete_chr8_read(address, gxrom.chr_bank);
}

static void gxrom_ppu_write(uint16_t address, uint8_t value) {
    discrete_chr8_write(address, gxrom.chr_bank, value);
}

static Mirroring gxrom_mirr(void) { return C.mirr_base; }
static void gxrom_reset(void) { memset(&gxrom, 0, sizeof(gxrom)); }

// Mapper 71: Codemasters/Camerica BF909x family.
static uint8_t m71_cpu_read(uint16_t address) {
    if (address >= 0x6000u && address < 0x8000u) return prg_ram_read(address);
    if (address >= 0x8000u) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = address < 0xC000u
            ? (banks ? (size_t)m71.prg_bank % banks : 0)
            : banks - 1u;
        return C.prg[bank * PRG_BANK_16K + (address & 0x3FFFu)];
    }
    return cart_cpu_bus_input;
}

static void m71_cpu_write(uint16_t address, uint8_t value) {
    if (address >= 0x6000u && address < 0x8000u) {
        prg_ram_write(address, value);
        return;
    }
    if (address < 0x8000u) return;
    if (address == 0x9000u) m71.bf9097_mode = true;
    if (address >= 0xC000u || !m71.bf9097_mode) {
        m71.prg_bank = value;
    } else {
        m71.mirr = (value & 0x10u) ? MIRROR_SINGLE0 : MIRROR_SINGLE1;
    }
}

static uint8_t m71_ppu_read(uint16_t address) {
    return discrete_chr8_read(address, 0);
}

static void m71_ppu_write(uint16_t address, uint8_t value) {
    discrete_chr8_write(address, 0, value);
}

static Mirroring m71_mirr(void) { return m71.mirr; }
static void m71_reset(void) {
    bool force = m71.force_bf9097;
    memset(&m71, 0, sizeof(m71));
    m71.force_bf9097 = force;
    m71.bf9097_mode = force;
    m71.mirr = C.mirr_base;
}

// Mapper 206: Namco 108.
static size_t namco108_prg_bank(uint16_t address) {
    size_t banks = C.prg_sz / PRG_BANK_8K;
    if (!banks) return 0;
    unsigned slot = (unsigned)((address - 0x8000u) / PRG_BANK_8K);
    if (namco108.fixed_prg) return slot % banks;
    if (slot == 0) return (size_t)namco108.banks[6] % banks;
    if (slot == 1) return (size_t)namco108.banks[7] % banks;
    if (slot == 2) return banks > 1 ? banks - 2 : 0;
    return banks - 1;
}

static uint8_t namco108_cpu_read(uint16_t address) {
    if (address >= 0x6000u && address < 0x8000u) return prg_ram_read(address);
    if (address >= 0x8000u) {
        size_t bank = namco108_prg_bank(address);
        return C.prg[bank * PRG_BANK_8K + (address & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void namco108_cpu_write(uint16_t address, uint8_t value) {
    if (address >= 0x6000u && address < 0x8000u) {
        prg_ram_write(address, value);
        return;
    }
    if (C.mapper_no == 154 && address >= 0x8000u)
        namco108.mirr = (value & 0x40u) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    if (C.mapper_no == 95 && address >= 0x8000u && (address & 1u))
        namco108.nametables_selected = true;
    if (address < 0x8000u || address >= 0xA000u) return;
    if ((address & 1u) == 0) {
        namco108.select = value & 7u;
        return;
    }

    unsigned reg = namco108.select;
    if (C.mapper_no == 88 || C.mapper_no == 154) {
        if (reg < 2) value &= 0x3Fu;
        else if (reg < 6) value |= 0x40u;
    }
    if (reg < 2) value &= 0xFEu;
    namco108.banks[reg] = value;
}

static size_t namco108_chr_bank_slot(unsigned slot) {
    if (C.mapper_no == 76) {
        return namco108.banks[2u + slot];
    }
    if (slot < 4) {
        unsigned reg = slot >> 1;
        return (size_t)(namco108.banks[reg] + (slot & 1u));
    }
    return namco108.banks[slot - 2u];
}

static uint8_t namco108_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    size_t native_page = C.mapper_no == 76 ? CHR_BANK_2K : CHR_BANK_1K;
    unsigned slot_count = C.mapper_no == 76 ? 4u : 8u;
    size_t page_size = shrunk_chr_page_size(native_page);
    unsigned slot = page_size ? (unsigned)(address / page_size) : slot_count;
    if (slot >= slot_count) return chr_unmapped_read(address);
    size_t banks = C.chr_sz / page_size;
    size_t bank = namco108_chr_bank_slot(slot) % banks;
    return C.chr[bank * page_size + (address % page_size)];
}

static void namco108_ppu_write(uint16_t address, uint8_t value) {
    if (!C.chr_is_ram) return;
    address &= 0x1FFFu;
    size_t native_page = C.mapper_no == 76 ? CHR_BANK_2K : CHR_BANK_1K;
    unsigned slot_count = C.mapper_no == 76 ? 4u : 8u;
    size_t page_size = shrunk_chr_page_size(native_page);
    unsigned slot = page_size ? (unsigned)(address / page_size) : slot_count;
    size_t offset = address % C.chr_sz;
    if (slot < slot_count) {
        size_t banks = C.chr_sz / page_size;
        size_t bank = namco108_chr_bank_slot(slot) % banks;
        offset = bank * page_size + (address % page_size);
    }
    chr_ram_write(offset, value);
}

static Mirroring namco108_mirr(void) {
    return C.mapper_no == 154 ? namco108.mirr : C.mirr_base;
}

static void namco108_reset(void) {
    bool fixed_prg = namco108.fixed_prg;
    memset(&namco108, 0, sizeof(namco108));
    namco108.fixed_prg = fixed_prg;
    namco108.banks[0] = 0;
    namco108.banks[1] = 2;
    namco108.banks[2] = 4;
    namco108.banks[3] = 5;
    namco108.banks[4] = 6;
    namco108.banks[5] = 7;
    namco108.banks[6] = 0;
    namco108.banks[7] = 1;
    if (C.mapper_no == 88 || C.mapper_no == 154)
        for (unsigned reg = 2; reg < 6; ++reg) namco108.banks[reg] |= 0x40;
    namco108.mirr = C.mirr_base;
    mapper_irq_line = false;
}

#endif // MAPPER_EXPANSION_H

