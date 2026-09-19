/*
 * mapper_konami_sunsoft.h - Konami and Sunsoft cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * This private header contains state and behavior for supported Konami and Sunsoft
 * cartridge boards.
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
#ifndef MAPPER_KONAMI_SUNSOFT_H
#define MAPPER_KONAMI_SUNSOFT_H

// Mappers 75/151: VRC1.
static struct {
    uint8_t prg[3];
    uint8_t chr[2];
    bool prg_mapped[3];
    bool chr_mapped[2];
    Mirroring mirr;
} vrc1;

static uint8_t vrc1_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;

    size_t banks = C.prg_sz / PRG_BANK_8K;
    unsigned slot = (unsigned)((a - 0x8000u) >> 13);
    size_t bank;
    if (slot == 3) {
        bank = banks - 1;
    } else {
        if (!vrc1.prg_mapped[slot]) return cart_cpu_bus_input;
        bank = vrc1.prg[slot] % banks;
    }
    return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
}

static void vrc1_map_chr(void) {
    vrc1.chr_mapped[0] = true;
    vrc1.chr_mapped[1] = true;
}

static void vrc1_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000) return;

    switch (a & 0xF000u) {
        case 0x8000:
            vrc1.prg[0] = value;
            vrc1.prg_mapped[0] = true;
            break;
        case 0x9000:
            if (C.mirr_base != MIRROR_FOUR)
                vrc1.mirr = (value & 1u) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
            vrc1.chr[0] = (uint8_t)((vrc1.chr[0] & 0x0Fu) | ((value & 0x02u) << 3));
            vrc1.chr[1] = (uint8_t)((vrc1.chr[1] & 0x0Fu) | ((value & 0x04u) << 2));
            vrc1_map_chr();
            break;
        case 0xA000:
            vrc1.prg[1] = value;
            vrc1.prg_mapped[1] = true;
            break;
        case 0xC000:
            vrc1.prg[2] = value;
            vrc1.prg_mapped[2] = true;
            break;
        case 0xE000:
            vrc1.chr[0] = (uint8_t)((vrc1.chr[0] & 0x10u) | (value & 0x0Fu));
            vrc1_map_chr();
            break;
        case 0xF000:
            vrc1.chr[1] = (uint8_t)((vrc1.chr[1] & 0x10u) | (value & 0x0Fu));
            vrc1_map_chr();
            break;
        default:
            break;
    }
}

static uint8_t vrc1_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 2u;
    if (slot >= 2 || !vrc1.chr_mapped[slot]) return chr_unmapped_read(a);
    size_t banks = C.chr_sz / page_size;
    size_t bank = vrc1.chr[slot] % banks;
    return C.chr[bank * page_size + (a % page_size)];
}

static void vrc1_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 2u;
    size_t offset = a % C.chr_sz;
    if (slot < 2 && vrc1.chr_mapped[slot]) {
        size_t banks = C.chr_sz / page_size;
        size_t bank = vrc1.chr[slot] % banks;
        offset = bank * page_size + (a % page_size);
    }
    chr_ram_write(offset, value);
}

static Mirroring vrc1_mirr(void) { return vrc1.mirr; }

static void vrc1_reset(void) {
    memset(&vrc1, 0, sizeof(vrc1));
    vrc1.mirr = C.mirr_base;
}

// Mapper 73: VRC3.
static struct {
    uint16_t reload;
    uint16_t counter;
    uint8_t prg_bank;
    bool prg_mapped;
    bool enabled;
    bool enable_after_ack;
    bool eight_bit;
} vrc3;

static uint8_t vrc3_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;

    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank;
    if (a < 0xC000) {
        if (!vrc3.prg_mapped) return cart_cpu_bus_input;
        bank = vrc3.prg_bank % banks;
    } else {
        bank = banks - 1;
    }
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void vrc3_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000) return;

    switch (a & 0xF000u) {
        case 0x8000:
            vrc3.reload = (uint16_t)((vrc3.reload & 0xFFF0u) | (value & 0x0Fu));
            break;
        case 0x9000:
            vrc3.reload = (uint16_t)((vrc3.reload & 0xFF0Fu) | ((uint16_t)(value & 0x0Fu) << 4));
            break;
        case 0xA000:
            vrc3.reload = (uint16_t)((vrc3.reload & 0xF0FFu) | ((uint16_t)(value & 0x0Fu) << 8));
            break;
        case 0xB000:
            vrc3.reload = (uint16_t)((vrc3.reload & 0x0FFFu) | ((uint16_t)(value & 0x0Fu) << 12));
            break;
        case 0xC000:
            vrc3.enable_after_ack = (value & 0x01u) != 0;
            vrc3.enabled = (value & 0x02u) != 0;
            vrc3.eight_bit = (value & 0x04u) != 0;
            if (vrc3.enabled) vrc3.counter = vrc3.reload;
            mapper_irq_line = false;
            break;
        case 0xD000:
            mapper_irq_line = false;
            vrc3.enabled = vrc3.enable_after_ack;
            break;
        case 0xF000:
            vrc3.prg_bank = value & 0x07u;
            vrc3.prg_mapped = true;
            break;
        default:
            break;
    }
}

static uint8_t vrc3_ppu_read(uint16_t a) {
    return discrete_chr8_read(a, 0);
}

static void vrc3_ppu_write(uint16_t a, uint8_t value) {
    discrete_chr8_write(a, 0, value);
}

static void vrc3_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0 && vrc3.enabled) {
        if (vrc3.eight_bit) {
            uint8_t low = (uint8_t)vrc3.counter;
            low++;
            if (low == 0) {
                low = (uint8_t)vrc3.reload;
                mapper_irq_line = true;
            }
            vrc3.counter = (uint16_t)((vrc3.counter & 0xFF00u) | low);
        } else {
            vrc3.counter++;
            if (vrc3.counter == 0) {
                vrc3.counter = vrc3.reload;
                mapper_irq_line = true;
            }
        }
    }
}

static Mirroring vrc3_mirr(void) { return C.mirr_base; }

static void vrc3_reset(void) {
    memset(&vrc3, 0, sizeof(vrc3));
    mapper_irq_line = false;
}

// Mapper 67: Sunsoft 3.
static struct {
    uint8_t chr[4];
    bool chr_mapped[4];
    uint8_t prg_bank;
    bool prg_mapped;
    bool irq_write_low;
    bool irq_enabled;
    uint16_t irq_counter;
    Mirroring mirr;
} sunsoft3;

static uint8_t sunsoft3_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank;
    if (a < 0xC000) {
        if (!sunsoft3.prg_mapped) return cart_cpu_bus_input;
        bank = sunsoft3.prg_bank % banks;
    } else {
        bank = banks - 1;
    }
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void sunsoft3_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000) return;
    switch (a & 0xF800u) {
        case 0x8800: case 0x9800: case 0xA800: case 0xB800: {
            unsigned slot = (unsigned)((a >> 12) - 8);
            sunsoft3.chr[slot] = value;
            sunsoft3.chr_mapped[slot] = true;
            break;
        }
        case 0xC800:
            if (sunsoft3.irq_write_low)
                sunsoft3.irq_counter = (uint16_t)((sunsoft3.irq_counter & 0xFF00u) | value);
            else
                sunsoft3.irq_counter = (uint16_t)((sunsoft3.irq_counter & 0x00FFu) | ((uint16_t)value << 8));
            sunsoft3.irq_write_low = !sunsoft3.irq_write_low;
            break;
        case 0xD800:
            sunsoft3.irq_enabled = (value & 0x10u) != 0;
            sunsoft3.irq_write_low = false;
            mapper_irq_line = false;
            break;
        case 0xE800:
            switch (value & 3u) {
                case 0: sunsoft3.mirr = MIRROR_VERTICAL; break;
                case 1: sunsoft3.mirr = MIRROR_HORIZONTAL; break;
                case 2: sunsoft3.mirr = MIRROR_SINGLE0; break;
                case 3: sunsoft3.mirr = MIRROR_SINGLE1; break;
            }
            break;
        case 0xF800:
            sunsoft3.prg_bank = value;
            sunsoft3.prg_mapped = true;
            break;
        default:
            break;
    }
}

static uint8_t sunsoft3_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_2K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 4u;
    if (slot >= 4 || !sunsoft3.chr_mapped[slot]) return chr_unmapped_read(a);
    size_t offset;
    if (!chr_bank_slot_offset(a, CHR_BANK_2K, 4, sunsoft3.chr, &offset)) {
        return chr_unmapped_read(a);
    }
    return C.chr[offset];
}

static void sunsoft3_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_2K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 4u;
    size_t offset;
    if (slot >= 4 || !sunsoft3.chr_mapped[slot]
        || !chr_bank_slot_offset(a, CHR_BANK_2K, 4, sunsoft3.chr, &offset)) {
        offset = a % C.chr_sz;
    }
    chr_ram_write(offset, value);
}

static void sunsoft3_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0 && sunsoft3.irq_enabled) {
        sunsoft3.irq_counter--;
        if (sunsoft3.irq_counter == 0xFFFFu) {
            sunsoft3.irq_enabled = false;
            mapper_irq_line = true;
        }
    }
}

static Mirroring sunsoft3_mirr(void) { return sunsoft3.mirr; }

static void sunsoft3_reset(void) {
    memset(&sunsoft3, 0, sizeof(sunsoft3));
    sunsoft3.mirr = C.mirr_base;
    mapper_irq_line = false;
}

// Mapper 68: Sunsoft 4.
static struct {
    uint8_t chr[4];
    bool chr_mapped[4];
    uint8_t nt[2];
    uint8_t prg_bank;
    bool use_chr_nt;
    bool ram_enabled;
    bool external_prg;
    uint32_t license_timer;
    Mirroring mirr;
} sunsoft4;

#define SUNSOFT4_LICENSE_CYCLES (1024u * 105u)

static size_t sunsoft4_lower_prg_bank(void) {
    size_t banks = C.prg_sz / PRG_BANK_16K;
    if (!sunsoft4.external_prg) return (size_t)(sunsoft4.prg_bank & 7u) % banks;
    size_t external = banks > 8 ? banks - 8 : 0;
    if (!external) return (size_t)(sunsoft4.prg_bank & 7u) % banks;
    return 8u + ((size_t)(sunsoft4.prg_bank & 7u) % external);
}

static uint8_t sunsoft4_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF)
        return sunsoft4.ram_enabled ? prg_ram_read(a) : cart_cpu_bus_input;
    if (a < 0x8000) return cart_cpu_bus_input;
    if (a < 0xC000) {
        if (sunsoft4.external_prg && sunsoft4.license_timer == 0) return cart_cpu_bus_input;
        size_t bank = sunsoft4_lower_prg_bank();
        return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
    }
    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank = 7u % banks;
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void sunsoft4_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        sunsoft4.license_timer = SUNSOFT4_LICENSE_CYCLES;
        if (sunsoft4.ram_enabled) prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000) return;
    switch (a & 0xF000u) {
        case 0x8000: case 0x9000: case 0xA000: case 0xB000: {
            unsigned slot = (unsigned)((a >> 12) - 8);
            sunsoft4.chr[slot] = value;
            sunsoft4.chr_mapped[slot] = true;
            break;
        }
        case 0xC000:
            sunsoft4.nt[0] = value | 0x80u;
            break;
        case 0xD000:
            sunsoft4.nt[1] = value | 0x80u;
            break;
        case 0xE000:
            switch (value & 3u) {
                case 0: sunsoft4.mirr = MIRROR_VERTICAL; break;
                case 1: sunsoft4.mirr = MIRROR_HORIZONTAL; break;
                case 2: sunsoft4.mirr = MIRROR_SINGLE0; break;
                case 3: sunsoft4.mirr = MIRROR_SINGLE1; break;
            }
            sunsoft4.use_chr_nt = (value & 0x10u) != 0;
            break;
        case 0xF000:
            sunsoft4.prg_bank = value & 7u;
            sunsoft4.external_prg = (value & 0x08u) == 0 && C.prg_sz > 8u * PRG_BANK_16K;
            sunsoft4.ram_enabled = (value & 0x10u) != 0;
            break;
        default:
            break;
    }
}

static uint8_t sunsoft4_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_2K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 4u;
    if (slot >= 4 || !sunsoft4.chr_mapped[slot]) return chr_unmapped_read(a);
    size_t offset;
    if (!chr_bank_slot_offset(a, CHR_BANK_2K, 4, sunsoft4.chr, &offset)) {
        return chr_unmapped_read(a);
    }
    return C.chr[offset];
}

static void sunsoft4_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_2K);
    unsigned slot = page_size ? (unsigned)(a / page_size) : 4u;
    size_t offset;
    if (slot >= 4 || !sunsoft4.chr_mapped[slot]
        || !chr_bank_slot_offset(a, CHR_BANK_2K, 4, sunsoft4.chr, &offset)) {
        offset = a % C.chr_sz;
    }
    chr_ram_write(offset, value);
}

static unsigned sunsoft4_nt_reg(unsigned nt) {
    switch (sunsoft4.mirr) {
        case MIRROR_VERTICAL: return nt & 1u;
        case MIRROR_HORIZONTAL: return (nt >> 1) & 1u;
        case MIRROR_SINGLE1: return 1u;
        case MIRROR_SINGLE0:
        default: return 0u;
    }
}

static void sunsoft4_clock(int cpu_cycles) {
    if (cpu_cycles <= 0 || sunsoft4.license_timer == 0) return;
    uint32_t elapsed = (uint32_t)cpu_cycles;
    sunsoft4.license_timer = elapsed >= sunsoft4.license_timer
        ? 0 : sunsoft4.license_timer - elapsed;
}

static Mirroring sunsoft4_mirr(void) { return sunsoft4.mirr; }

static void sunsoft4_reset(void) {
    memset(&sunsoft4, 0, sizeof(sunsoft4));
    sunsoft4.mirr = C.mirr_base;
}

// Mappers 89, 93, and 184: discrete Sunsoft boards.
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
    bool mapped;
    Mirroring mirr;
} sunsoft89;

static uint8_t sunsoft89_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_16K;
    if (a < 0xC000) {
        if (!sunsoft89.mapped) return cart_cpu_bus_input;
        size_t bank = sunsoft89.prg_bank % banks;
        return C.prg[bank * PRG_BANK_16K + (a & 0x3FFFu)];
    }
    return C.prg[(banks - 1) * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void sunsoft89_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, value); return; }
    if (a < 0x8000) return;
    sunsoft89.prg_bank = (value >> 4) & 7u;
    sunsoft89.chr_bank = (uint8_t)((value & 7u) | ((value & 0x80u) >> 4));
    sunsoft89.mirr = (value & 8u) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    sunsoft89.mapped = true;
}

static uint8_t sunsoft89_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!sunsoft89.mapped) return chr_unmapped_read(a);
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    if (!page_size || a >= page_size) return chr_unmapped_read(a);
    size_t banks = C.chr_sz / page_size;
    size_t bank = sunsoft89.chr_bank % banks;
    return C.chr[bank * page_size + a];
}

static void sunsoft89_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t offset = a % C.chr_sz;
    if (sunsoft89.mapped) {
        size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
        if (page_size && a < page_size) {
            size_t bank = sunsoft89.chr_bank % (C.chr_sz / page_size);
            offset = bank * page_size + a;
        }
    }
    chr_ram_write(offset, value);
}

static Mirroring sunsoft89_mirr(void) { return sunsoft89.mirr; }
static void sunsoft89_reset(void) { memset(&sunsoft89, 0, sizeof(sunsoft89)); sunsoft89.mirr = C.mirr_base; }

static struct {
    uint8_t prg_bank;
    bool prg_mapped;
    bool chr_enabled;
    bool chr_startup_aliases;
} sunsoft93;

static uint8_t sunsoft93_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_16K;
    if (a < 0xC000) {
        if (!sunsoft93.prg_mapped) return cart_cpu_bus_input;
        return C.prg[(sunsoft93.prg_bank % banks) * PRG_BANK_16K + (a & 0x3FFFu)];
    }
    return C.prg[(banks - 1) * PRG_BANK_16K + (a & 0x3FFFu)];
}

static void sunsoft93_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, value); return; }
    if (a < 0x8000) return;
    sunsoft93.prg_bank = (value >> 4) & 7u;
    sunsoft93.prg_mapped = true;
    sunsoft93.chr_enabled = (value & 1u) != 0;
    if (!sunsoft93.chr_enabled) sunsoft93.chr_startup_aliases = false;
}

static uint8_t sunsoft93_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!sunsoft93.chr_enabled) return (uint8_t)a;
    if (sunsoft93.chr_startup_aliases) return C.chr[a % C.chr_sz];
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    return page_size && a < page_size ? C.chr[a] : (uint8_t)a;
}

static void sunsoft93_ppu_write(uint16_t a, uint8_t value) {
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    if (sunsoft93.chr_startup_aliases) {
        chr_ram_write(a % C.chr_sz, value);
        return;
    }
    if (sunsoft93.chr_enabled && C.chr_is_ram && page_size && a < page_size)
        chr_ram_write(a, value);
}

static Mirroring sunsoft93_mirr(void) { return C.mirr_base; }
static void sunsoft93_reset(void) {
    memset(&sunsoft93, 0, sizeof(sunsoft93));
    sunsoft93.chr_enabled = C.chr_is_ram;
    sunsoft93.chr_startup_aliases = C.chr_is_ram;
}

static struct { uint8_t chr[2]; bool mapped; } sunsoft184;

static uint8_t sunsoft184_cpu_read(uint16_t a) {
    if (a >= 0x8000) return C.prg[a - 0x8000u];
    if (a >= 0x6000) return prg_ram_read(a);
    return cart_cpu_bus_input;
}

static void sunsoft184_cpu_write(uint16_t a, uint8_t value) {
    if (a < 0x6000 || a > 0x7FFF) return;
    sunsoft184.chr[0] = value & 7u;
    sunsoft184.chr[1] = (uint8_t)(4u | ((value >> 4) & 3u));
    sunsoft184.mapped = true;
}

static uint8_t sunsoft184_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!sunsoft184.mapped) return chr_unmapped_read(a);
    size_t offset;
    if (!chr_bank_slot_offset(a, CHR_BANK_4K, 2, sunsoft184.chr, &offset))
        return chr_unmapped_read(a);
    return C.chr[offset];
}

static void sunsoft184_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t offset;
    if (!sunsoft184.mapped
        || !chr_bank_slot_offset(a, CHR_BANK_4K, 2, sunsoft184.chr, &offset)) {
        offset = a % C.chr_sz;
    }
    chr_ram_write(offset, value);
}

static Mirroring sunsoft184_mirr(void) { return C.mirr_base; }
static void sunsoft184_reset(void) { memset(&sunsoft184, 0, sizeof(sunsoft184)); }

#endif // MAPPER_KONAMI_SUNSOFT_H

