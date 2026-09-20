/*
 * mapper_mmc3.h - MMC3 family cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * MMC3, MMC6, and RAMBO-1 banking, mirroring, RAM, and IRQ behavior are implemented
 * here.
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
#ifndef MAPPER_MMC3_H
#define MAPPER_MMC3_H

// Mapper 4: MMC3/TxROM.
static struct {
    uint8_t bank_select;
    uint8_t banks[8];
    uint8_t prg_mode, chr_mode;
    Mirroring mirr;
    uint8_t irq_latch, irq_counter;
    bool irq_enabled, irq_reload;
    uint8_t ram_protect;
    bool ram_enabled; // MMC6 global enable at $8000 bit 5.
    bool a12_low;
    uint64_t a12_low_cycle;
    bool mcacc_a12_high;
    uint8_t mcacc_divider;
    bool revision_a;
} mmc3;
static uint8_t txsrom_nt[4];

typedef struct {
    uint8_t prg[2];
    uint16_t chr[8];
    uint8_t prg_mapped, chr_mapped;
    Mirroring mirr;
} TaitoBankState;

static TaitoBankState taito33, taito48;
static struct {
    uint8_t latch, counter;
    uint8_t delay;
    bool enabled, reload;
    bool a12_low;
    uint64_t a12_low_cycle;
} taito48_irq;

static void taito48_irq_clock(void);

static struct {
    uint8_t reg8000, reg_a000;
    uint8_t regs[16];
    uint8_t current_reg;
    uint8_t prg_mode, chr_mode;
    Mirroring mirr;
    uint8_t irq_counter, irq_reload;
    uint8_t cpu_divider;
    uint8_t irq_delay;
    bool irq_enabled, irq_cycle_mode, need_reload, force_clock;
    bool a12_low;
    uint64_t a12_low_cycle;
    uint8_t nt_map[4];
} rambo1;

static void rambo1_irq_clock(uint8_t delay);

typedef enum {
    JY_IRQ_CPU_CLOCK = 0,
    JY_IRQ_PPU_A12 = 1,
    JY_IRQ_PPU_READ = 2,
    JY_IRQ_CPU_WRITE = 3
} JyIrqSource;

static struct {
    uint8_t prg[4];
    uint8_t chr_low[8], chr_high[8], chr_latch[2];
    uint8_t nt_low[4], nt_high[4];
    uint8_t prg_mode, chr_mode;
    bool enable_prg_6000;
    bool chr_block_mode;
    uint8_t chr_block;
    bool mirror_chr;
    uint8_t mirroring_reg;
    bool advanced_nt;
    bool disable_nt_ram;
    uint8_t nt_ram_select_bit;
    bool irq_enabled;
    JyIrqSource irq_source;
    uint8_t irq_direction;
    bool irq_funky_mode;
    uint8_t irq_funky_reg;
    bool irq_small_prescaler;
    uint8_t irq_prescaler, irq_counter, irq_xor;
    uint8_t multiply_a, multiply_b, register_ram;
    uint16_t last_ppu_addr;
} jy;

static uint8_t jy_invert_prg(uint8_t value) {
    if ((jy.prg_mode & 3u) != 3u) return value;
    return (uint8_t)(((value & 0x01u) << 6)
                   | ((value & 0x02u) << 4)
                   | ((value & 0x04u) << 2)
                   | ((value & 0x10u) >> 2)
                   | ((value & 0x20u) >> 4)
                   | ((value & 0x40u) >> 6));
}

static size_t jy_prg_bank(uint16_t addr) {
    uint8_t regs[4];
    for (unsigned i = 0; i < 4; ++i) regs[i] = jy_invert_prg(jy.prg[i]);
    if (addr >= 0x6000 && addr < 0x8000) {
        switch (jy.prg_mode & 3u) {
            case 0: return (size_t)regs[3] * 4u + 3u;
            case 1: return (size_t)regs[3] * 2u + 1u;
            default: return regs[3];
        }
    }

    unsigned slot = (unsigned)((addr - 0x8000u) >> 13);
    switch (jy.prg_mode & 3u) {
        case 0: {
            size_t base = (jy.prg_mode & 4u) ? regs[3] : 0x3Cu;
            return base + slot;
        }
        case 1:
            if (slot < 2) return (size_t)regs[1] * 2u + slot;
            return ((jy.prg_mode & 4u) ? regs[3] : 0x3Eu) + (slot - 2u);
        default:
            if (slot < 3) return regs[slot];
            return (jy.prg_mode & 4u) ? regs[3] : 0x3Fu;
    }
}

static uint16_t jy_chr_reg(unsigned index) {
    if (jy.chr_mode >= 2 && jy.mirror_chr && (index == 2 || index == 3)) index -= 2;
    if (!jy.chr_block_mode)
        return (uint16_t)(jy.chr_low[index] | ((uint16_t)jy.chr_high[index] << 8));

    uint8_t mask, shift;
    switch (jy.chr_mode) {
        case 0: mask = 0x1F; shift = 5; break;
        case 1: mask = 0x3F; shift = 6; break;
        case 2: mask = 0x7F; shift = 7; break;
        default: mask = 0xFF; shift = 8; break;
    }
    return (uint16_t)((jy.chr_low[index] & mask) | ((uint16_t)jy.chr_block << shift));
}

static size_t jy_chr_bank(unsigned slot) {
    switch (jy.chr_mode) {
        case 0:
            return (size_t)jy_chr_reg(0) * 8u + slot;
        case 1: {
            unsigned half = slot >> 2;
            unsigned index = jy.chr_latch[half];
            return (size_t)jy_chr_reg(index) * 4u + (slot & 3u);
        }
        case 2: {
            unsigned index = (slot >> 1) * 2u;
            return (size_t)jy_chr_reg(index) * 2u + (slot & 1u);
        }
        default:
            return jy_chr_reg(slot);
    }
}

static bool jy_advanced_nt(void) {
    return (jy.advanced_nt || C.mapper_no == 211) && C.mapper_no != 90;
}

static uint8_t jy_ciram_page(unsigned slot) {
    if (jy_advanced_nt()) return jy.nt_low[slot & 3u] & 1u;
    switch (jy.mirroring_reg & 3u) {
        case 0: return slot & 1u;
        case 1: return (slot >> 1) & 1u;
        case 2: return 0;
        default: return 1;
    }
}

static void jy_irq_tick(void) {
    uint8_t mask = jy.irq_small_prescaler ? 0x07u : 0xFFu;
    uint8_t prescaler = jy.irq_prescaler & mask;
    bool clock_counter = false;
    if (jy.irq_direction == 1) {
        prescaler++;
        if ((prescaler & mask) == 0) clock_counter = true;
    } else if (jy.irq_direction == 2) {
        prescaler--;
        if (prescaler == 0) clock_counter = true;
    }
    jy.irq_prescaler = (uint8_t)((jy.irq_prescaler & (uint8_t)~mask) | (prescaler & mask));
    if (!clock_counter) return;

    if (jy.irq_direction == 1) {
        jy.irq_counter++;
        if (jy.irq_counter == 0 && jy.irq_enabled) mapper_irq_line = true;
    } else if (jy.irq_direction == 2) {
        jy.irq_counter--;
        if (jy.irq_counter == 0xFF && jy.irq_enabled) mapper_irq_line = true;
    }
}

static uint8_t jy_cpu_read(uint16_t addr) {
    if (addr >= 0x5000 && addr < 0x6000) {
        switch (addr & 0xF803u) {
            case 0x5000: return 0;
            case 0x5800: return (uint8_t)((uint16_t)jy.multiply_a * jy.multiply_b);
            case 0x5801: return (uint8_t)(((uint16_t)jy.multiply_a * jy.multiply_b) >> 8);
            case 0x5803: return jy.register_ram;
            default: return cart_cpu_bus_input;
        }
    }
    if (addr >= 0x6000 && addr < 0x8000) {
        if (!jy.enable_prg_6000) return cart_cpu_bus_input;
        size_t banks = C.prg_sz / PRG_BANK_8K;
        return banks ? C.prg[(jy_prg_bank(addr) % banks) * PRG_BANK_8K + (addr & 0x1FFFu)]
                     : cart_cpu_bus_input;
    }
    if (addr >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        return banks ? C.prg[(jy_prg_bank(addr) % banks) * PRG_BANK_8K + (addr & 0x1FFFu)]
                     : cart_cpu_bus_input;
    }
    return cart_cpu_bus_input;
}

static void jy_cpu_write(uint16_t addr, uint8_t value) {
    if (addr < 0x8000) {
        switch (addr & 0xF803u) {
            case 0x5800: jy.multiply_a = value; break;
            case 0x5801: jy.multiply_b = value; break;
            case 0x5803: jy.register_ram = value; break;
        }
        return;
    }

    switch (addr & 0xF007u) {
        case 0x8000: case 0x8001: case 0x8002: case 0x8003:
        case 0x8004: case 0x8005: case 0x8006: case 0x8007:
            jy.prg[addr & 3u] = value & 0x7Fu;
            break;
        case 0x9000: case 0x9001: case 0x9002: case 0x9003:
        case 0x9004: case 0x9005: case 0x9006: case 0x9007:
            jy.chr_low[addr & 7u] = value;
            break;
        case 0xA000: case 0xA001: case 0xA002: case 0xA003:
        case 0xA004: case 0xA005: case 0xA006: case 0xA007:
            jy.chr_high[addr & 7u] = value;
            break;
        case 0xB000: case 0xB001: case 0xB002: case 0xB003:
            jy.nt_low[addr & 3u] = value;
            break;
        case 0xB004: case 0xB005: case 0xB006: case 0xB007:
            jy.nt_high[addr & 3u] = value;
            break;
        case 0xC000:
            jy.irq_enabled = (value & 1u) != 0;
            if (!jy.irq_enabled) mapper_irq_line = false;
            break;
        case 0xC001:
            jy.irq_direction = (value >> 6) & 3u;
            jy.irq_funky_mode = (value & 0x08u) != 0;
            jy.irq_small_prescaler = (value & 0x04u) != 0;
            jy.irq_source = (JyIrqSource)(value & 3u);
            break;
        case 0xC002:
            jy.irq_enabled = false;
            mapper_irq_line = false;
            break;
        case 0xC003:
            jy.irq_enabled = true;
            break;
        case 0xC004:
            jy.irq_prescaler = value ^ jy.irq_xor;
            break;
        case 0xC005:
            jy.irq_counter = value ^ jy.irq_xor;
            break;
        case 0xC006:
            jy.irq_xor = value;
            break;
        case 0xC007:
            jy.irq_funky_reg = value;
            break;
        case 0xD000:
            jy.prg_mode = value & 7u;
            jy.chr_mode = (value >> 3) & 3u;
            jy.advanced_nt = (value & 0x20u) != 0;
            jy.disable_nt_ram = (value & 0x40u) != 0;
            jy.enable_prg_6000 = (value & 0x80u) != 0;
            break;
        case 0xD001:
            jy.mirroring_reg = value & 3u;
            break;
        case 0xD002:
            jy.nt_ram_select_bit = value & 0x80u;
            break;
        case 0xD003:
            jy.mirror_chr = (value & 0x80u) != 0;
            jy.chr_block_mode = (value & 0x20u) == 0;
            jy.chr_block = (uint8_t)(((value & 0x18u) >> 2) | (value & 1u));
            break;
    }
}

static uint8_t jy_ppu_read(uint16_t addr) {
    if (jy.irq_source == JY_IRQ_PPU_READ && cart_ppu_fetch_source != CART_PPU_FETCH_CPU)
        jy_irq_tick();
    addr &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_default_read(addr, CHR_BANK_1K);
    return C.chr[shrunk_chr_bank_offset(addr, page_size, page_count, jy_chr_bank(slot))];
}

static void jy_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count)) {
        chr_default_write(addr, CHR_BANK_1K, value);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(addr, page_size, page_count, jy_chr_bank(slot)), value);
}

static void jy_clock(int cycles) {
    if (cycles <= 0) return;
    for (int i = 0; i < cycles; ++i) {
        if (jy.irq_source == JY_IRQ_CPU_CLOCK
            || (jy.irq_source == JY_IRQ_CPU_WRITE && cart_cpu_cycle_is_write)) {
            jy_irq_tick();
        }
    }
}

static Mirroring jy_mirr(void) {
    switch (jy.mirroring_reg & 3u) {
        case 0: return MIRROR_VERTICAL;
        case 1: return MIRROR_HORIZONTAL;
        case 2: return MIRROR_SINGLE0;
        default: return MIRROR_SINGLE1;
    }
}

static void jy_reset(void) {
    memset(&jy, 0, sizeof(jy));
    jy.chr_latch[0] = 0;
    jy.chr_latch[1] = 4;
    jy.irq_source = JY_IRQ_CPU_CLOCK;
    mapper_irq_line = false;
}

void cart_notify_ppu_address(uint16_t addr, uint64_t ppu_cycle) {
    if (active_board) {
        board_notify_ppu_address(active_board, addr, ppu_cycle);
        return;
    }
    if (cart == &mapper_mmc2) {
        mmc2_notify_ppu_address(addr);
        return;
    }
    if (cart == &mapper_mmc4) {
        mmc4_notify_ppu_address(addr);
        return;
    }
    if (cart == &mapper_m96) {
        if ((m96.last_ppu_addr & 0x3000u) != 0x2000u && (addr & 0x3000u) == 0x2000u) {
            m96.inner_chr_bank = (uint8_t)((addr >> 8) & 0x03u);
            m96.chr_banking_active = true;
        }
        m96.last_ppu_addr = addr;
        return;
    }
    if (cart == &mapper_jy) {
        if (jy.irq_source == JY_IRQ_PPU_A12 && (addr & 0x1000u) && !(jy.last_ppu_addr & 0x1000u))
            jy_irq_tick();
        jy.last_ppu_addr = addr;
        if (C.mapper_no == 209) {
            switch (addr & 0x2FF8u) {
                case 0x0FD8:
                case 0x0FE8: {
                    unsigned half = (addr >> 12) & 1u;
                    jy.chr_latch[half] = (uint8_t)((addr >> 4)
                        & ((((addr >> 10) & 0x04u)) | 0x02u));
                    break;
                }
            }
        }
        return;
    }
    if (cart == &mapper_rambo1 || cart == &mapper_rambo158) {
        if (rambo1.irq_cycle_mode) return;
        if (!(addr & 0x1000)) {
            if (!rambo1.a12_low) {
                rambo1.a12_low = true;
                rambo1.a12_low_cycle = ppu_cycle;
            }
        } else {
            if (rambo1.a12_low && ppu_cycle >= rambo1.a12_low_cycle
                && ppu_cycle - rambo1.a12_low_cycle >= 30) {
                rambo1_irq_clock(2);
            }
            rambo1.a12_low = false;
        }
        return;
    }
    if (cart == &mapper_taito48) {
        uint64_t cpu_cycle = ppu_cycle / 3;
        if (!(addr & 0x1000)) {
            if (!taito48_irq.a12_low) {
                taito48_irq.a12_low = true;
                taito48_irq.a12_low_cycle = cpu_cycle;
            }
        } else {
            if (taito48_irq.a12_low && cpu_cycle >= taito48_irq.a12_low_cycle
                && cpu_cycle - taito48_irq.a12_low_cycle >= 3) {
                taito48_irq_clock();
            }
            taito48_irq.a12_low = false;
        }
        return;
    }
    if (cart != &mapper_mmc3 && cart != &mapper_txsrom) return;
    if (C.submapper == 3) {
        bool high = (addr & 0x1000) != 0;
        if (mmc3.mcacc_a12_high && !high) {
            // MC-ACC clocks on the first falling edge in each group of eight.
            if (mmc3.mcacc_divider == 0) mmc3_irq_clock();
            mmc3.mcacc_divider = (uint8_t)((mmc3.mcacc_divider + 1) & 7);
        }
        mmc3.mcacc_a12_high = high;
        return;
    }
    uint64_t cpu_cycle = ppu_cycle / 3;
    if (!(addr & 0x1000)) {
        if (!mmc3.a12_low) {
            mmc3.a12_low = true;
            mmc3.a12_low_cycle = cpu_cycle;
        }
    } else {
        if (mmc3.a12_low && cpu_cycle >= mmc3.a12_low_cycle
            && cpu_cycle - mmc3.a12_low_cycle >= 3) {
            mmc3_irq_clock();
        }
        mmc3.a12_low = false;
    }
}

static void mmc3_irq_clock(void) {
    uint8_t previous_counter = mmc3.irq_counter;
    bool explicit_reload = mmc3.irq_reload;
    if (previous_counter == 0 || explicit_reload) {
        mmc3.irq_counter = mmc3.irq_latch;
    } else {
        mmc3.irq_counter--;
    }

    bool qualified = mmc3.irq_counter == 0;
    if (mmc3.revision_a) qualified = qualified && (previous_counter != 0 || explicit_reload);
    if (qualified && mmc3.irq_enabled) {
        mapper_irq_line = true;
        MMC3_LOG("irq assert latch=%02X reload=%d", mmc3.irq_latch, explicit_reload ? 1 : 0);
    }
    mmc3.irq_reload = false;
}

static uint8_t mmc3_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        if (C.submapper == 1) {
            if (a < 0x7000 || !mmc3.ram_enabled || !(mmc3.ram_protect & 0xA0))
                return cart_cpu_bus_input;
            uint8_t read_enable = (a & 0x0200) ? 0x80 : 0x20;
            if (!(mmc3.ram_protect & read_enable)) return 0;
            return ram_read(default_prg_ram(), a & 0x03FF);
        }
        if (C.submapper == 3 || (mmc3.ram_protect & 0x80)) return prg_ram_read(a);
        return cart_cpu_bus_input;
    }
    if (a >= 0x8000) {
        size_t prg_8k_banks = C.prg_sz / PRG_BANK_8K;
        if (prg_8k_banks == 0) return cart_cpu_bus_input;

        size_t last_bank = prg_8k_banks - 1;
        size_t second_last_bank = (prg_8k_banks > 1) ? (prg_8k_banks - 2) : 0;
        size_t bank = 0;

        if (a < 0xA000) {
            bank = mmc3.prg_mode ? second_last_bank : mmc3.banks[6];
        } else if (a < 0xC000) {
            bank = mmc3.banks[7];
        } else if (a < 0xE000) {
            bank = mmc3.prg_mode ? mmc3.banks[6] : second_last_bank;
        } else {
            bank = last_bank;
        }

        bank %= prg_8k_banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFF)];
    }
    return cart_cpu_bus_input;
}

static void mmc3_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        if (C.submapper == 1) {
            uint8_t required = (a & 0x0200) ? 0xC0 : 0x30;
            if (a >= 0x7000 && mmc3.ram_enabled && (mmc3.ram_protect & required) == required)
                ram_write(default_prg_ram(), a & 0x03FF, v);
        } else if (C.submapper == 3 || (mmc3.ram_protect & 0xC0) == 0x80) {
            prg_ram_write(a, v);
        }
        return;
    }
    if (a >= 0x8000) {
        if ((a & 0xE001) == 0x8000) {
            mmc3.bank_select = v & 7;
            mmc3.prg_mode = (v >> 6) & 1;
            mmc3.chr_mode = (v >> 7) & 1;
            if (C.submapper == 1) {
                mmc3.ram_enabled = (v & 0x20) != 0;
                if (!mmc3.ram_enabled) mmc3.ram_protect = 0;
            }
            MMC3_LOG("write %04X=%02X select=%u prg_mode=%u chr_mode=%u", a, v,
                     mmc3.bank_select, mmc3.prg_mode, mmc3.chr_mode);
        } else if ((a & 0xE001) == 0x8001) {
            mmc3.banks[mmc3.bank_select] = v;
            MMC3_LOG("write %04X=%02X bank[%u]=%02X", a, v, mmc3.bank_select, v);
        } else if ((a & 0xE001) == 0xA000) {
            if (C.mirr_base != MIRROR_FOUR)
                mmc3.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            MMC3_LOG("write %04X=%02X mirr=%d", a, v, (int)mmc3.mirr);
        } else if ((a & 0xE001) == 0xA001) {
            if (C.submapper != 1 || mmc3.ram_enabled) mmc3.ram_protect = v;
        } else if ((a & 0xE001) == 0xC000) {
            mmc3.irq_latch = v;
            MMC3_LOG("write %04X=%02X irq_latch=%02X", a, v, mmc3.irq_latch);
        } else if ((a & 0xE001) == 0xC001) {
            mmc3.irq_counter = 0;
            mmc3.irq_reload = true;
            if (C.submapper == 3) mmc3.mcacc_divider = 0;
            MMC3_LOG("write %04X=%02X irq_reload=1", a, v);
        } else if ((a & 0xE001) == 0xE000) {
            mmc3.irq_enabled = false;
            mapper_irq_line = false;
            MMC3_LOG("write %04X=%02X irq_disable", a, v);
        } else if ((a & 0xE001) == 0xE001) {
            mmc3.irq_enabled = true;
            MMC3_LOG("write %04X=%02X irq_enable", a, v);
        }
    }
}

static void txsrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x8000 && (a & 0xE001) == 0x8001) {
        uint8_t nametable = v >> 7;
        if (mmc3.chr_mode == 0) {
            if (mmc3.bank_select < 2) {
                txsrom_nt[mmc3.bank_select * 2] = nametable;
                txsrom_nt[mmc3.bank_select * 2 + 1] = nametable;
            }
        } else if (mmc3.bank_select >= 2 && mmc3.bank_select <= 5) {
            txsrom_nt[mmc3.bank_select - 2] = nametable;
        }
    }
    // CIRAM routing is latched by CHR data writes; the mirroring register is disconnected.
    if (a >= 0x8000 && (a & 0xE001) == 0xA000) return;
    mmc3_cpu_write(a, v);
}

static size_t mmc3_chr_bank_for_slot(uint8_t slot) {
    size_t bank = 0;
    if (mmc3.chr_mode == 0) {
        static const uint8_t slot_map[8] = {0, 0, 1, 1, 2, 3, 4, 5};
        uint8_t reg = slot_map[slot];
        if (slot < 4) {
            uint8_t pair = (uint8_t)(mmc3.banks[reg] & 0xFE);
            bank = (size_t)(pair + (slot & 1));
        } else {
            bank = mmc3.banks[reg];
        }
    } else {
        static const uint8_t slot_map[8] = {2, 3, 4, 5, 0, 0, 1, 1};
        uint8_t reg = slot_map[slot];
        if (slot >= 4) {
            uint8_t pair = (uint8_t)(mmc3.banks[reg] & 0xFE);
            bank = (size_t)(pair + (slot & 1));
        } else {
            bank = mmc3.banks[reg];
        }
    }
    return bank;
}

static uint8_t mmc3_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_1K);
    if (!page_size) return chr_default_read(a, CHR_BANK_1K);
    size_t slot = a / page_size;
    if (slot >= 8) return chr_default_read(a, CHR_BANK_1K);
    size_t page_count = C.chr_sz / page_size;
    size_t bank = mmc3_chr_bank_for_slot((uint8_t)slot) % page_count;
    return C.chr[bank * page_size + (a % page_size)];
}

static void mmc3_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_1K);
    if (!page_size) return;
    size_t slot = a / page_size;
    if (slot >= 8) {
        chr_default_write(a, CHR_BANK_1K, v);
        return;
    }
    size_t page_count = C.chr_sz / page_size;
    size_t bank = mmc3_chr_bank_for_slot((uint8_t)slot) % page_count;
    chr_ram_write(bank * page_size + (a % page_size), v);
}

static Mirroring mmc3_mirr(void) { return mmc3.mirr; }
static void mmc3_reset(void) {
    memset(&mmc3, 0, sizeof(mmc3));
    const uint8_t initial_banks[8] = {0, 2, 4, 5, 6, 7, 0, 1};
    memcpy(mmc3.banks, initial_banks, sizeof(initial_banks));
    mmc3.mirr = C.mirr_base;
    mmc3.revision_a = mmc3_revision_a_profile
                   && !(C.mapper_no == 4 && (C.submapper == 1 || C.submapper == 3));
    if (C.submapper == 3 && C.mirr_base != MIRROR_FOUR) mmc3.mirr = MIRROR_VERTICAL;
    mapper_irq_line = false;
}

static void txsrom_reset(void) {
    mmc3_reset();
    for (unsigned page = 0; page < 4; ++page) {
        switch (C.mirr_base) {
            case MIRROR_HORIZONTAL: txsrom_nt[page] = (uint8_t)(page >> 1); break;
            case MIRROR_VERTICAL: txsrom_nt[page] = (uint8_t)(page & 1); break;
            case MIRROR_FOUR: txsrom_nt[page] = (uint8_t)page; break;
            case MIRROR_SINGLE1: txsrom_nt[page] = 1; break;
            default: txsrom_nt[page] = 0; break;
        }
    }
}

// Mappers 33 and 48: Taito TC0190/TC0690 family.
static uint8_t taito_cpu_read(const TaitoBankState *state, uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        if (!banks) return cart_cpu_bus_input;
        size_t slot = (a - 0x8000u) >> 13;
        size_t bank;
        if (slot < 2) {
            if (!(state->prg_mapped & (1u << slot))) return cart_cpu_bus_input;
            bank = state->prg[slot];
        }
        else if (slot == 2) bank = banks > 1 ? banks - 2 : 0;
        else bank = banks - 1;
        bank %= banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static uint8_t taito_ppu_read(const TaitoBankState *state, uint16_t a) {
    a &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_default_read(a, CHR_BANK_1K);
    if (!(state->chr_mapped & (1u << slot))) return chr_default_read(a, CHR_BANK_1K);
    return C.chr[shrunk_chr_bank_offset(a, page_size, page_count, state->chr[slot])];
}

static void taito_ppu_write(const TaitoBankState *state, uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(state->chr_mapped & (1u << slot))) {
        chr_default_write(a, CHR_BANK_1K, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, state->chr[slot]), v);
}

static void taito_reset_banks(TaitoBankState *state) {
    state->prg[0] = 0;
    state->prg[1] = 1;
    for (unsigned i = 0; i < 8; ++i) state->chr[i] = (uint16_t)i;
    state->mirr = C.mirr_base;
    state->prg_mapped = 0;
    state->chr_mapped = C.chr_is_ram ? 0xFF : 0;
}

static uint8_t taito33_cpu_read(uint16_t a) { return taito_cpu_read(&taito33, a); }
static void taito33_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a < 0x8000) return;
    switch (a & 0xA003u) {
        case 0x8000:
            taito33.prg[0] = v & 0x3Fu;
            taito33.prg_mapped |= 1;
            taito33.mirr = (v & 0x40u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            break;
        case 0x8001:
            taito33.prg[1] = v & 0x3Fu;
            taito33.prg_mapped |= 2;
            break;
        case 0x8002:
        case 0x8003: {
            unsigned slot = (unsigned)(a & 3u) * 2u - 4u;
            taito33.chr[slot] = (uint16_t)v * 2u;
            taito33.chr[slot + 1] = (uint16_t)v * 2u + 1u;
            taito33.chr_mapped |= (uint8_t)(3u << slot);
        } break;
        case 0xA000: case 0xA001: case 0xA002: case 0xA003:
            taito33.chr[4u + (a & 3u)] = v;
            taito33.chr_mapped |= (uint8_t)(1u << (4u + (a & 3u)));
            break;
    }
}
static uint8_t taito33_ppu_read(uint16_t a) { return taito_ppu_read(&taito33, a); }
static void taito33_ppu_write(uint16_t a, uint8_t v) { taito_ppu_write(&taito33, a, v); }
static Mirroring taito33_mirr(void) { return taito33.mirr; }
static void taito33_reset(void) {
    taito_reset_banks(&taito33);
    mapper_irq_line = false;
}

static void taito48_irq_clock(void) {
    if (taito48_irq.counter == 0 || taito48_irq.reload) {
        taito48_irq.counter = taito48_irq.latch;
        taito48_irq.reload = false;
    } else {
        taito48_irq.counter--;
    }
    if (taito48_irq.counter == 0 && taito48_irq.enabled)
        taito48_irq.delay = C.submapper == 1 ? 6 : 22;
}

static void taito48_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0 && taito48_irq.delay) {
        taito48_irq.delay--;
        if (!taito48_irq.delay) mapper_irq_line = true;
    }
}

static uint8_t taito48_cpu_read(uint16_t a) { return taito_cpu_read(&taito48, a); }
static void taito48_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a < 0x8000) return;
    switch (a & 0xE003u) {
        case 0x8000:
            taito48.prg[0] = v & 0x3Fu;
            taito48.prg_mapped |= 1;
            break;
        case 0x8001:
            taito48.prg[1] = v & 0x3Fu;
            taito48.prg_mapped |= 2;
            break;
        case 0x8002:
        case 0x8003: {
            unsigned slot = (unsigned)(a & 3u) * 2u - 4u;
            taito48.chr[slot] = (uint16_t)v * 2u;
            taito48.chr[slot + 1] = (uint16_t)v * 2u + 1u;
            taito48.chr_mapped |= (uint8_t)(3u << slot);
        } break;
        case 0xA000: case 0xA001: case 0xA002: case 0xA003:
            taito48.chr[4u + (a & 3u)] = v;
            taito48.chr_mapped |= (uint8_t)(1u << (4u + (a & 3u)));
            break;
        case 0xC000:
            mapper_irq_line = false;
            taito48_irq.latch = (uint8_t)((v ^ 0xFFu) + (C.submapper == 1 ? 1u : 0u));
            break;
        case 0xC001:
            mapper_irq_line = false;
            taito48_irq.counter = 0;
            taito48_irq.reload = true;
            break;
        case 0xC002:
            taito48_irq.enabled = true;
            break;
        case 0xC003:
            taito48_irq.enabled = false;
            mapper_irq_line = false;
            break;
        case 0xE000:
            taito48.mirr = (v & 0x40u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            break;
    }
}
static uint8_t taito48_ppu_read(uint16_t a) { return taito_ppu_read(&taito48, a); }
static void taito48_ppu_write(uint16_t a, uint8_t v) { taito_ppu_write(&taito48, a, v); }
static Mirroring taito48_mirr(void) { return taito48.mirr; }
static void taito48_reset(void) {
    taito_reset_banks(&taito48);
    memset(&taito48_irq, 0, sizeof(taito48_irq));
    mapper_irq_line = false;
}

// Mappers 64 and 158: RAMBO-1.
static size_t rambo1_prg_bank(uint16_t a) {
    size_t banks = C.prg_sz / PRG_BANK_8K;
    if (!banks) return 0;
    unsigned slot = (unsigned)((a - 0x8000u) >> 13);
    size_t bank;
    if (slot == 3) {
        bank = banks - 1;
    } else if (slot == 1) {
        bank = rambo1.regs[7];
    } else if (rambo1.prg_mode == 0) {
        bank = slot == 0 ? rambo1.regs[6] : rambo1.regs[15];
    } else {
        bank = slot == 0 ? rambo1.regs[15] : rambo1.regs[6];
    }
    return bank % banks;
}

static size_t rambo1_chr_bank(unsigned physical_slot) {
    unsigned slot = physical_slot ^ (rambo1.chr_mode ? 4u : 0u);
    switch (slot) {
        case 0: return rambo1.regs[0];
        case 1: return (rambo1.reg8000 & 0x20u) ? rambo1.regs[8] : (rambo1.regs[0] | 1u);
        case 2: return rambo1.regs[1];
        case 3: return (rambo1.reg8000 & 0x20u) ? rambo1.regs[9] : (rambo1.regs[1] | 1u);
        case 4: return rambo1.regs[2];
        case 5: return rambo1.regs[3];
        case 6: return rambo1.regs[4];
        default: return rambo1.regs[5];
    }
}

static uint8_t rambo1_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        if (!banks) return cart_cpu_bus_input;
        size_t bank = rambo1_prg_bank(a);
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static uint8_t rambo1_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_default_read(a, CHR_BANK_1K);
    return C.chr[shrunk_chr_bank_offset(a, page_size, page_count, rambo1_chr_bank(slot))];
}

static void rambo1_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)) {
        chr_default_write(a, CHR_BANK_1K, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, rambo1_chr_bank(slot)), v);
}

static void rambo1_irq_clock(uint8_t delay) {
    if (rambo1.need_reload) {
        rambo1.irq_counter = (uint8_t)(rambo1.irq_reload + (rambo1.irq_reload <= 1 ? 1u : 2u));
        rambo1.need_reload = false;
    } else if (rambo1.irq_counter == 0) {
        rambo1.irq_counter = (uint8_t)(rambo1.irq_reload + 1u);
    }
    rambo1.irq_counter--;
    if (rambo1.irq_counter == 0 && rambo1.irq_enabled) rambo1.irq_delay = delay;
}

static void rambo1_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0) {
        if (rambo1.irq_delay && --rambo1.irq_delay == 0) mapper_irq_line = true;
        if (rambo1.irq_cycle_mode || rambo1.force_clock) {
            rambo1.cpu_divider = (uint8_t)((rambo1.cpu_divider + 1u) & 3u);
            if (rambo1.cpu_divider == 0) {
                rambo1_irq_clock(1);
                rambo1.force_clock = false;
            }
        }
    }
}

static void rambo158_update_nametable(uint8_t value) {
    uint8_t nametable = value >> 7;
    unsigned reg = rambo1.current_reg & 7u;
    if (rambo1.chr_mode) {
        if (reg >= 2 && reg <= 5) rambo1.nt_map[reg - 2] = nametable;
    } else if (reg == 0) {
        rambo1.nt_map[0] = nametable;
        rambo1.nt_map[1] = nametable;
    } else if (reg == 1) {
        rambo1.nt_map[2] = nametable;
        rambo1.nt_map[3] = nametable;
    }
}

static void rambo1_cpu_write_common(uint16_t a, uint8_t v, bool mapper158) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a < 0x8000) return;
    uint16_t reg = a & 0xE001u;
    if (mapper158 && reg == 0x8001) rambo158_update_nametable(v);
    if (mapper158 && reg == 0xA000) return;
    switch (reg) {
        case 0x8000:
            rambo1.reg8000 = v;
            rambo1.current_reg = v & 0x0Fu;
            rambo1.prg_mode = (v >> 6) & 1u;
            rambo1.chr_mode = (v >> 7) & 1u;
            break;
        case 0x8001:
            rambo1.regs[rambo1.current_reg] = v;
            break;
        case 0xA000:
            rambo1.reg_a000 = v;
            rambo1.mirr = (v & 1u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            break;
        case 0xC000:
            rambo1.irq_reload = v;
            break;
        case 0xC001:
            if (rambo1.irq_cycle_mode && !(v & 1u)) rambo1.force_clock = true;
            rambo1.irq_cycle_mode = (v & 1u) != 0;
            if (rambo1.irq_cycle_mode) rambo1.cpu_divider = 0;
            rambo1.need_reload = true;
            break;
        case 0xE000:
            rambo1.irq_enabled = false;
            mapper_irq_line = false;
            break;
        case 0xE001:
            rambo1.irq_enabled = true;
            break;
    }
}

static void rambo1_cpu_write(uint16_t a, uint8_t v) { rambo1_cpu_write_common(a, v, false); }
static void rambo158_cpu_write(uint16_t a, uint8_t v) { rambo1_cpu_write_common(a, v, true); }
static Mirroring rambo1_mirr(void) { return rambo1.mirr; }

static void rambo1_reset(void) {
    memset(&rambo1, 0, sizeof(rambo1));
    const uint8_t initial_banks[10] = {0, 2, 4, 5, 6, 7, 0, 1, 8, 9};
    memcpy(rambo1.regs, initial_banks, sizeof(initial_banks));
    rambo1.regs[15] = 2;
    rambo1.mirr = MIRROR_VERTICAL;
    rambo1.nt_map[0] = 0;
    rambo1.nt_map[1] = 1;
    rambo1.nt_map[2] = 0;
    rambo1.nt_map[3] = 1;
    mapper_irq_line = false;
}

#endif // MAPPER_MMC3_H

