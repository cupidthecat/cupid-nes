/*
 * mapper_jaleco_irem.h - Jaleco and Irem cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * Jaleco and Irem board state, bank switching, mirroring, and IRQ behavior are
 * implemented here.
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
#ifndef MAPPER_JALECO_IREM_H
#define MAPPER_JALECO_IREM_H

// Mappers 72, 78, 87, 92, 101, and 140: Jaleco discrete boards.
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
    bool prg_flag;
    bool chr_flag;
    Mirroring mirr;
} jaleco_discrete;

static uint8_t jaleco_discrete_cpu_read(uint16_t a) {
    if (a >= 0x6000u && a < 0x8000u) return prg_ram_read(a);
    if (a < 0x8000u) return cart_cpu_bus_input;

    if (C.mapper_no == 87 || C.mapper_no == 101) {
        return C.prg[a - 0x8000u];
    }
    if (C.mapper_no == 140) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        if (!banks) return cart_cpu_bus_input;
        size_t bank = jaleco_discrete.prg_bank % banks;
        return C.prg[bank * PRG_BANK_32K + (a & 0x7FFFu)];
    }

    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank;
    if (C.mapper_no == 92) {
        bank = a < 0xC000u ? 0 : jaleco_discrete.prg_bank % banks;
    } else {
        bank = a < 0xC000u ? jaleco_discrete.prg_bank % banks : banks - 1;
    }
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void jaleco_discrete_cpu_write(uint16_t a, uint8_t value) {
    if (C.mapper_no == 87 || C.mapper_no == 101 || C.mapper_no == 140) {
        if (a < 0x6000u || a > 0x7FFFu) return;
        if (C.mapper_no == 87) {
            jaleco_discrete.chr_bank = (uint8_t)(((value & 0x01u) << 1) | ((value & 0x02u) >> 1));
        } else if (C.mapper_no == 101) {
            jaleco_discrete.chr_bank = value;
        } else {
            jaleco_discrete.prg_bank = (value >> 4) & 0x03u;
            jaleco_discrete.chr_bank = value & 0x0Fu;
        }
        return;
    }

    if (a >= 0x6000u && a < 0x8000u) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000u) return;
    if (C.mapper_no == 72 || C.mapper_no == 92) {
        bool prg_flag = (value & 0x80u) != 0;
        bool chr_flag = (value & 0x40u) != 0;
        if (!jaleco_discrete.prg_flag && prg_flag)
            jaleco_discrete.prg_bank = C.mapper_no == 92 ? value & 0x0Fu : value & 0x07u;
        if (!jaleco_discrete.chr_flag && chr_flag)
            jaleco_discrete.chr_bank = value & 0x0Fu;
        jaleco_discrete.prg_flag = prg_flag;
        jaleco_discrete.chr_flag = chr_flag;
        return;
    }

    jaleco_discrete.prg_bank = value & 0x07u;
    jaleco_discrete.chr_bank = (value >> 4) & 0x0Fu;
    if (C.submapper == 3)
        jaleco_discrete.mirr = (value & 0x08u) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
    else
        jaleco_discrete.mirr = (value & 0x08u) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
}

static uint8_t jaleco_discrete_ppu_read(uint16_t a) {
    return discrete_chr8_read(a, jaleco_discrete.chr_bank);
}

static void jaleco_discrete_ppu_write(uint16_t a, uint8_t value) {
    discrete_chr8_write(a, jaleco_discrete.chr_bank, value);
}

static Mirroring jaleco_discrete_mirr(void) { return jaleco_discrete.mirr; }

static void jaleco_discrete_reset(void) {
    memset(&jaleco_discrete, 0, sizeof(jaleco_discrete));
    jaleco_discrete.mirr = C.mirr_base;
    if (C.mapper_no == 92)
        jaleco_discrete.prg_bank = (uint8_t)(C.prg_sz / PRG_BANK_16K - 1u);
}

// Mapper 97: Irem TAM-S1.
static struct {
    uint8_t upper_prg_bank;
    Mirroring mirr;
} irem97;

static uint8_t irem97_cpu_read(uint16_t a) {
    if (a >= 0x6000u && a < 0x8000u) return prg_ram_read(a);
    if (a < 0x8000u) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank = a < 0xC000u ? banks - 1 : irem97.upper_prg_bank % banks;
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void irem97_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000u && a < 0x8000u) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000u) return;
    irem97.upper_prg_bank = value & 0x0Fu;
    switch (value >> 6) {
        case 0: irem97.mirr = MIRROR_SINGLE0; break;
        case 1: irem97.mirr = MIRROR_HORIZONTAL; break;
        case 2: irem97.mirr = MIRROR_VERTICAL; break;
        case 3: irem97.mirr = MIRROR_SINGLE1; break;
    }
}

static uint8_t irem97_ppu_read(uint16_t a) {
    return discrete_chr8_read(a, 0);
}

static void irem97_ppu_write(uint16_t a, uint8_t value) {
    discrete_chr8_write(a, 0, value);
}

static Mirroring irem97_mirr(void) { return irem97.mirr; }

static void irem97_power_on(void) {
    size_t banks = C.prg_sz / PRG_BANK_16K;
    irem97.upper_prg_bank = (uint8_t)(banks - 1);
    irem97.mirr = C.mirr_base;
}

// Mapper 32: Irem G-101.
static struct {
    uint8_t prg_banks[2];
    uint8_t chr_banks[8];
    uint8_t prg_mapped, chr_mapped;
    uint8_t prg_mode;
    Mirroring mirr;
} irem32;

static uint8_t irem32_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        unsigned slot = (a - 0x8000u) >> 13;
        size_t bank;
        if (slot == 3) bank = banks - 1;
        else if (slot == 2 && irem32.prg_mode == 0) bank = banks - 2;
        else if (slot == 0 && irem32.prg_mode != 0) bank = banks - 2;
        else {
            unsigned reg = slot == 1 ? 1 : 0;
            if (!(irem32.prg_mapped & (1u << reg))) return cart_cpu_bus_input;
            bank = irem32.prg_banks[reg] % banks;
        }
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void irem32_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a < 0x8000) return;
    switch (a & 0xF000u) {
        case 0x8000:
            irem32.prg_banks[0] = v & 0x1F;
            irem32.prg_mapped |= 1;
            break;
        case 0x9000:
            irem32.prg_mode = (C.submapper == 1) ? 0 : (uint8_t)((v >> 1) & 1u);
            irem32.prg_mapped = 3;
            irem32.mirr = (v & 1u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            break;
        case 0xA000:
            irem32.prg_banks[1] = v & 0x1F;
            irem32.prg_mapped |= 2;
            break;
        case 0xB000:
            irem32.chr_banks[a & 7u] = v;
            irem32.chr_mapped |= (uint8_t)(1u << (a & 7u));
            break;
    }
}

static uint8_t irem32_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(irem32.chr_mapped & (1u << slot)))
        return chr_unmapped_read(a);
    return C.chr[shrunk_chr_bank_offset(a, page_size, page_count, irem32.chr_banks[slot])];
}

static void irem32_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(irem32.chr_mapped & (1u << slot))) {
        chr_ram_write(a % C.chr_sz, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, irem32.chr_banks[slot]), v);
}

static Mirroring irem32_mirr(void) { return irem32.mirr; }

static void irem32_reset(void) {
    memset(&irem32, 0, sizeof(irem32));
    irem32.mirr = C.submapper == 1 ? MIRROR_SINGLE0 : C.mirr_base;
    mapper_irq_line = false;
}

// Mapper 65: Irem H-3001.
static struct {
    uint8_t prg_banks[3];
    uint8_t chr_banks[8];
    uint8_t chr_mapped;
    uint16_t irq_counter;
    uint16_t irq_reload;
    bool irq_enabled;
    Mirroring mirr;
} irem65;

static uint8_t irem65_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        unsigned slot = (a - 0x8000u) >> 13;
        size_t bank = slot == 3 ? banks - 1 : irem65.prg_banks[slot] % banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void irem65_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    switch (a) {
        case 0x8000: irem65.prg_banks[0] = v; break;
        case 0x9001: irem65.mirr = (v & 0x80u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL; break;
        case 0x9003:
            irem65.irq_enabled = (v & 0x80u) != 0;
            mapper_irq_line = false;
            break;
        case 0x9004:
            irem65.irq_counter = irem65.irq_reload;
            mapper_irq_line = false;
            break;
        case 0x9005:
            irem65.irq_reload = (uint16_t)((irem65.irq_reload & 0x00FFu) | ((uint16_t)v << 8));
            break;
        case 0x9006:
            irem65.irq_reload = (uint16_t)((irem65.irq_reload & 0xFF00u) | v);
            break;
        case 0xA000: irem65.prg_banks[1] = v; break;
        case 0xB000: case 0xB001: case 0xB002: case 0xB003:
        case 0xB004: case 0xB005: case 0xB006: case 0xB007:
            irem65.chr_banks[a & 7u] = v;
            irem65.chr_mapped |= (uint8_t)(1u << (a & 7u));
            break;
        case 0xC000: irem65.prg_banks[2] = v; break;
    }
}

static uint8_t irem65_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(irem65.chr_mapped & (1u << slot)))
        return chr_unmapped_read(a);
    return C.chr[shrunk_chr_bank_offset(a, page_size, page_count, irem65.chr_banks[slot])];
}

static void irem65_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(irem65.chr_mapped & (1u << slot))) {
        chr_ram_write(a % C.chr_sz, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, irem65.chr_banks[slot]), v);
}

static void irem65_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0 && irem65.irq_enabled) {
        irem65.irq_counter--;
        if (irem65.irq_counter == 0) {
            irem65.irq_enabled = false;
            mapper_irq_line = true;
        }
    }
}

static Mirroring irem65_mirr(void) { return irem65.mirr; }

static void irem65_reset(void) {
    memset(&irem65, 0, sizeof(irem65));
    irem65.prg_banks[0] = 0;
    irem65.prg_banks[1] = 1;
    irem65.prg_banks[2] = 0xFE;
    irem65.mirr = C.mirr_base;
    mapper_irq_line = false;
}

typedef enum {
    VRC2A, VRC2B, VRC2C, VRC4A, VRC4B, VRC4C, VRC4D, VRC4E, VRC4F,
    VRC4_27, VRC4_183
} Vrc24Variant;

static struct {
    Vrc24Variant variant;
    bool heuristics;
    uint8_t prg[2];
    uint16_t chr[8];
    uint8_t prg_mode;
    uint8_t latch;
    uint8_t expansion_prg_bank;
    Mirroring mirr;
    VrcIrq irq;
} vrc24;

static bool vrc24_select_variant(void) {
    vrc24.heuristics = C.submapper == 0 && C.mapper_no != 22
                    && C.mapper_no != 27 && C.mapper_no != 183;
    switch (C.mapper_no) {
        case 21:
            vrc24.variant = C.submapper == 2 ? VRC4C : VRC4A;
            return C.submapper <= 2;
        case 22:
            vrc24.variant = VRC2A;
            return C.submapper == 0;
        case 23:
            if (C.submapper == 1) vrc24.variant = VRC4F;
            else if (C.submapper == 2) vrc24.variant = VRC4E;
            else vrc24.variant = VRC2B;
            return C.submapper <= 3;
        case 25:
            if (C.submapper == 2) vrc24.variant = VRC4D;
            else if (C.submapper == 3) vrc24.variant = VRC2C;
            else vrc24.variant = VRC4B;
            return C.submapper <= 3;
        case 27:
            vrc24.variant = VRC4_27;
            return C.submapper == 0;
        case 183:
            vrc24.variant = VRC4_183;
            return C.submapper == 0;
        default:
            return false;
    }
}

static bool vrc24_explicit_vrc2(void) {
    return !vrc24.heuristics && vrc24.variant <= VRC2C;
}

static bool vrc24_has_irq(void) {
    return (vrc24.heuristics && C.mapper_no != 22) || vrc24.variant >= VRC4A;
}

static uint16_t vrc24_translate(uint16_t addr) {
    unsigned a0 = 0, a1 = 0;
    if (vrc24.heuristics) {
        switch (C.mapper_no) {
            case 21:
                a0 = ((addr >> 1) & 1u) | ((addr >> 6) & 1u);
                a1 = ((addr >> 2) & 1u) | ((addr >> 7) & 1u);
                break;
            case 23:
                a0 = (addr & 1u) | ((addr >> 2) & 1u);
                a1 = ((addr >> 1) & 1u) | ((addr >> 3) & 1u);
                break;
            case 25:
                a0 = ((addr >> 1) & 1u) | ((addr >> 3) & 1u);
                a1 = (addr & 1u) | ((addr >> 2) & 1u);
                break;
        }
    } else {
        switch (vrc24.variant) {
            case VRC2A: case VRC2C: case VRC4B:
                a0 = (addr >> 1) & 1u; a1 = addr & 1u; break;
            case VRC2B: case VRC4F: case VRC4_27:
                a0 = addr & 1u; a1 = (addr >> 1) & 1u; break;
            case VRC4A:
                a0 = (addr >> 1) & 1u; a1 = (addr >> 2) & 1u; break;
            case VRC4C:
                a0 = (addr >> 6) & 1u; a1 = (addr >> 7) & 1u; break;
            case VRC4D:
                a0 = (addr >> 3) & 1u; a1 = (addr >> 2) & 1u; break;
            case VRC4E: case VRC4_183:
                a0 = (addr >> 2) & 1u; a1 = (addr >> 3) & 1u; break;
        }
    }
    return (uint16_t)((addr & 0xFF00u) | (a1 << 1) | a0);
}

static size_t vrc24_chr_bank(unsigned slot, size_t page_count) {
    size_t bank = vrc24.chr[slot];
    if (vrc24.variant == VRC2A) bank >>= 1;
    return bank % page_count;
}

static uint8_t vrc24_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a < 0x8000) {
        if (vrc24.variant == VRC4_183) {
            size_t banks = C.prg_sz / PRG_BANK_8K;
            size_t bank = vrc24.expansion_prg_bank % banks;
            return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
        }
        if (vrc24_explicit_vrc2() && !prg_work_ram.size && !prg_save_ram.size) {
            if (a <= 0x6FFF) return (uint8_t)((cart_cpu_bus_input & 0xFEu) | vrc24.latch);
            return cart_cpu_bus_input;
        }
        return prg_ram_read(a);
    }
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        unsigned slot = (a - 0x8000u) >> 13;
        size_t bank;
        if (slot == 3) bank = banks - 1;
        else if (slot == 1) bank = vrc24.prg[1] % banks;
        else if ((slot == 0 && vrc24.prg_mode) || (slot == 2 && !vrc24.prg_mode)) bank = banks - 2;
        else bank = vrc24.prg[0] % banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void vrc24_cpu_write(uint16_t a, uint8_t value) {
    if (a < 0x8000) {
        if (a < 0x6000) return;
        if (vrc24.variant == VRC4_183) {
            vrc24.expansion_prg_bank = (uint8_t)(a & 0x0Fu);
        } else if (vrc24_explicit_vrc2() && !prg_work_ram.size && !prg_save_ram.size) {
            if (a <= 0x6FFF) vrc24.latch = value & 1u;
        } else {
            prg_ram_write(a, value);
        }
        return;
    }
    uint16_t reg = vrc24_translate(a) & 0xF00Fu;
    if (reg >= 0x8000 && reg <= 0x8006) {
        vrc24.prg[0] = value & 0x1Fu;
    } else if ((vrc24.variant <= VRC2C && reg >= 0x9000 && reg <= 0x9003)
            || (vrc24.variant >= VRC4A && reg >= 0x9000 && reg <= 0x9001)) {
        uint8_t mask = vrc24_explicit_vrc2() ? 1u : 3u;
        switch (value & mask) {
            case 0: vrc24.mirr = MIRROR_VERTICAL; break;
            case 1: vrc24.mirr = MIRROR_HORIZONTAL; break;
            case 2: vrc24.mirr = MIRROR_SINGLE0; break;
            case 3: vrc24.mirr = MIRROR_SINGLE1; break;
        }
    } else if (vrc24.variant >= VRC4A && reg >= 0x9002 && reg <= 0x9003) {
        vrc24.prg_mode = (value >> 1) & 1u;
    } else if (reg >= 0xA000 && reg <= 0xA006) {
        vrc24.prg[1] = value & 0x1Fu;
    } else if (reg >= 0xB000 && reg <= 0xE006) {
        unsigned bank = (unsigned)((((reg >> 12) & 7u) - 3u) * 2u + ((reg >> 1) & 1u));
        if (reg & 1u) vrc24.chr[bank] = (uint16_t)((vrc24.chr[bank] & 0x00Fu) | ((value & 0x1Fu) << 4));
        else vrc24.chr[bank] = (uint16_t)((vrc24.chr[bank] & 0x1F0u) | (value & 0x0Fu));
    } else if (reg == 0xF000) {
        vrc24.irq.reload = (uint8_t)((vrc24.irq.reload & 0xF0u) | (value & 0x0Fu));
    } else if (reg == 0xF001) {
        vrc24.irq.reload = (uint8_t)((vrc24.irq.reload & 0x0Fu) | ((value & 0x0Fu) << 4));
    } else if (reg == 0xF002) {
        vrc_irq_control(&vrc24.irq, value);
    } else if (reg == 0xF003) {
        vrc_irq_ack(&vrc24.irq);
    }
}

static uint8_t vrc24_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_unmapped_read(a);
    size_t bank = vrc24_chr_bank(slot, page_count);
    return C.chr[bank * page_size + (a % page_size)];
}

static void vrc24_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)) {
        chr_ram_write(a % C.chr_sz, value);
        return;
    }
    size_t bank = vrc24_chr_bank(slot, page_count);
    chr_ram_write(bank * page_size + (a % page_size), value);
}

static void vrc24_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0) vrc_irq_clock(&vrc24.irq);
}

static Mirroring vrc24_mirr(void) { return vrc24.mirr; }

static void vrc24_reset(void) {
    Vrc24Variant variant = vrc24.variant;
    bool heuristics = vrc24.heuristics;
    memset(&vrc24, 0, sizeof(vrc24));
    vrc24.variant = variant;
    vrc24.heuristics = heuristics;
    vrc24.mirr = C.mirr_base;
    vrc_irq_reset(&vrc24.irq);
    mapper_irq_line = false;
}

// Mapper 85: Konami VRC7.
static struct {
    uint8_t prg[3];
    uint8_t chr[8];
    bool prg_selected[3];
    bool chr_selected[8];
    uint8_t control;
    Mirroring mirr;
    VrcIrq irq;
    Vrc7Fm fm;
} vrc7;

static uint16_t vrc7_decode_register(uint16_t addr) {
    uint16_t audio = addr & 0xF038u;
    if (audio == 0x9010u || audio == 0x9030u) return audio;
    if ((addr & 0x20u) != 0) return 0xFFFFu;

    bool secondary;
    if (C.submapper == 1) secondary = (addr & 0x08u) != 0;
    else if (C.submapper == 2) secondary = (addr & 0x10u) != 0;
    else secondary = (addr & 0x18u) != 0;
    return (uint16_t)((addr & 0xF000u) | (secondary ? 0x0008u : 0u));
}

static void vrc7_update_control(uint8_t value) {
    vrc7.control = value;
    switch (value & 3u) {
        case 0: vrc7.mirr = MIRROR_VERTICAL; break;
        case 1: vrc7.mirr = MIRROR_HORIZONTAL; break;
        case 2: vrc7.mirr = MIRROR_SINGLE0; break;
        case 3: vrc7.mirr = MIRROR_SINGLE1; break;
    }
    vrc7_fm_set_muted(&vrc7.fm, (value & 0x40u) != 0);
}

static uint8_t vrc7_cpu_read(uint16_t addr) {
    if (addr >= 0x6000 && addr < 0x8000) {
        if (!(vrc7.control & 0x80u)) return cart_cpu_bus_input;
        return prg_ram_read(addr);
    }
    if (addr >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        unsigned slot = (addr - 0x8000u) >> 13;
        if (slot == 3) {
            return C.prg[(banks - 1) * PRG_BANK_8K + (addr & 0x1FFFu)];
        }
        if (!vrc7.prg_selected[slot]) return cart_cpu_bus_input;
        size_t bank = vrc7.prg[slot] % banks;
        return C.prg[bank * PRG_BANK_8K + (addr & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void vrc7_cpu_write(uint16_t addr, uint8_t value) {
    if (addr >= 0x6000 && addr < 0x8000) {
        if (vrc7.control & 0x80u) prg_ram_write(addr, value);
        return;
    }
    if (addr < 0x8000) return;

    switch (vrc7_decode_register(addr)) {
        case 0x8000:
            vrc7.prg[0] = value & 0x3Fu;
            vrc7.prg_selected[0] = true;
            break;
        case 0x8008:
            vrc7.prg[1] = value & 0x3Fu;
            vrc7.prg_selected[1] = true;
            break;
        case 0x9000:
            vrc7.prg[2] = value & 0x3Fu;
            vrc7.prg_selected[2] = true;
            break;
        case 0x9010:
            vrc7_fm_write_address(&vrc7.fm, value);
            break;
        case 0x9030:
            vrc7_fm_write_data(&vrc7.fm, value);
            break;
        case 0xA000: vrc7.chr[0] = value; vrc7.chr_selected[0] = true; break;
        case 0xA008: vrc7.chr[1] = value; vrc7.chr_selected[1] = true; break;
        case 0xB000: vrc7.chr[2] = value; vrc7.chr_selected[2] = true; break;
        case 0xB008: vrc7.chr[3] = value; vrc7.chr_selected[3] = true; break;
        case 0xC000: vrc7.chr[4] = value; vrc7.chr_selected[4] = true; break;
        case 0xC008: vrc7.chr[5] = value; vrc7.chr_selected[5] = true; break;
        case 0xD000: vrc7.chr[6] = value; vrc7.chr_selected[6] = true; break;
        case 0xD008: vrc7.chr[7] = value; vrc7.chr_selected[7] = true; break;
        case 0xE000:
            vrc7_update_control(value);
            break;
        case 0xE008:
            vrc7.irq.reload = value;
            break;
        case 0xF000:
            vrc_irq_control(&vrc7.irq, value);
            break;
        case 0xF008:
            vrc_irq_ack(&vrc7.irq);
            break;
        default:
            break;
    }
}

static uint8_t vrc7_ppu_read(uint16_t addr) {
    addr &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !vrc7.chr_selected[slot])
        return chr_unmapped_read(addr);
    return C.chr[shrunk_chr_bank_offset(addr, page_size, page_count, vrc7.chr[slot])];
}

static void vrc7_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(addr, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !vrc7.chr_selected[slot]) {
        chr_ram_write(addr % C.chr_sz, value);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(addr, page_size, page_count, vrc7.chr[slot]), value);
}

static void vrc7_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) vrc_irq_clock(&vrc7.irq);
    vrc7_fm_clock(&vrc7.fm, cpu_cycles, nes_timing()->cpu_hz);
}

static Mirroring vrc7_mirr(void) { return vrc7.mirr; }

static float vrc7_expansion_output(void) { return vrc7_fm_output(&vrc7.fm); }

static void vrc7_shutdown(void) { vrc7_fm_destroy(&vrc7.fm); }

static void vrc7_console_reset(void) { vrc7_fm_reset_chip(&vrc7.fm); }

static void vrc7_reset(void) {
    Vrc7Fm fm = vrc7.fm;
    memset(&vrc7, 0, sizeof(vrc7));
    vrc7.fm = fm;
    vrc7.mirr = MIRROR_VERTICAL;
    vrc_irq_reset(&vrc7.irq);
    vrc7_fm_reset(&vrc7.fm);
    mapper_irq_line = false;
}

// VS System mapper 99. The board uses four 8KB CPU pages and one 8KB CHR page.
// OUT0 bit 2 selects the alternate CHR bank and, on the 40KB single-system
// layout, the alternate $8000-$9FFF PRG page.
static uint8_t m99_cpu_read(uint16_t addr) {
    if (addr >= 0x6000 && addr < 0x8000)
        return vs_shared_ram_access_allowed() ? prg_ram_read(addr) : cart_cpu_bus_input;
    if (addr < 0x8000) return cart_cpu_bus_input;
    if (C.prg_sz < PRG_BANK_8K)
        return repeated_prg_window_read(addr, 0x8000, PRG_BANK_32K);

    unsigned slot = (addr - 0x8000u) >> 13;
    size_t bank;
    if (vs_dual_system() && C.prg_sz == 0xC000) {
        if (slot == 0) return cart_cpu_bus_input;
        bank = (vs_active_side() ? 3u : 0u) + slot - 1u;
    } else if (vs_dual_system()) {
        bank = (vs_active_side() ? 4u : 0u) + slot;
    } else if (slot == 0 && C.prg_sz > 0x8000 && vs_system_type() == VS_TYPE_DEFAULT) {
        bank = vs_prg_chr_select_bit() ? 4u : 0u;
    } else {
        bank = slot;
    }
    size_t banks = C.prg_sz / PRG_BANK_8K;
    bank %= banks;
    return C.prg[bank * PRG_BANK_8K + (addr & 0x1FFFu)];
}

static void m99_cpu_write(uint16_t addr, uint8_t value) {
    if (addr >= 0x6000 && addr < 0x8000 && vs_shared_ram_access_allowed())
        prg_ram_write(addr, value);
}

static uint8_t m99_ppu_read(uint16_t addr) {
    addr &= 0x1FFF;
    size_t page_size = C.chr_sz < CHR_BANK_8K ? C.chr_sz : CHR_BANK_8K;
    if (!page_size) return (uint8_t)addr;
    size_t bank = (vs_dual_system() ? vs_active_side() * 2u : 0u) + vs_prg_chr_select_bit();
    size_t pages = C.chr_sz / page_size;
    bank %= pages;
    if (!C.chr_is_ram && addr >= page_size) return (uint8_t)addr;
    return C.chr[bank * page_size + (addr % page_size)];
}

static void m99_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram || !C.chr_sz) return;
    addr &= 0x1FFF;
    size_t page_size = C.chr_sz < CHR_BANK_8K ? C.chr_sz : CHR_BANK_8K;
    size_t pages = C.chr_sz / page_size;
    size_t bank = ((vs_dual_system() ? vs_active_side() * 2u : 0u)
                 + vs_prg_chr_select_bit()) % pages;
    chr_ram_write(bank * page_size + (addr % page_size), value);
}

static Mirroring m99_mirr(void) { return C.mirr_base; }

#endif // MAPPER_JALECO_IREM_H

