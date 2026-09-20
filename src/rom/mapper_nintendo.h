/*
 * mapper_nintendo.h - Nintendo cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * This file implements bank switching, mirroring, and board state for supported
 * Nintendo cartridges.
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
#ifndef MAPPER_NINTENDO_H
#define MAPPER_NINTENDO_H

// Mapper 0: NROM.
static uint8_t repeated_prg_window_read(uint16_t address, uint16_t start, size_t window_size) {
    if (address < start || !C.prg_sz) return cart_cpu_bus_input;
    size_t offset = (size_t)(address - start);
    if (offset >= window_size) return cart_cpu_bus_input;
    size_t mapped = (window_size / C.prg_sz) * C.prg_sz;
    if (offset >= mapped) return cart_cpu_bus_input;
    return C.prg[offset % C.prg_sz];
}

static uint8_t nrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        if (C.prg_sz < PRG_BANK_16K)
            return repeated_prg_window_read(a, 0x8000, PRG_BANK_32K);
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t slot = (size_t)(a - 0x8000u) / PRG_BANK_16K;
        return C.prg[(slot % banks) * PRG_BANK_16K + (a & 0x3FFFu)];
    }
    return cart_cpu_bus_input;
}
static void nrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) prg_ram_write(a, v);
}
static uint8_t nrom_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (C.chr_is_ram) return chr_default_read(a, CHR_BANK_8K);
    size_t mapped = C.chr_sz < CHR_BANK_8K ? C.chr_sz : CHR_BANK_8K;
    return a < mapped ? chr_read_byte(a) : (uint8_t)a;
}
static void nrom_ppu_write(uint16_t a, uint8_t v) {
    chr_default_write(a, CHR_BANK_8K, v);
}
static Mirroring nrom_mirr(void) { return C.mirr_base; }

// Mapper 1: MMC1/SxROM.
static struct {
    uint8_t shift_reg;
    uint8_t shift_count;
    uint8_t control;
    uint8_t chr_bank0, chr_bank1;
    uint8_t prg_bank;
    bool last_chr_bank1;
    bool has_write_cycle;
    uint64_t last_write_cycle;
    Mirroring mirr;
} mmc1;

static void mmc1_write_control(uint8_t v) {
    mmc1.control = v & 0x1F;
    switch (v & 3) {
        case 0: mmc1.mirr = MIRROR_SINGLE0; break;
        case 1: mmc1.mirr = MIRROR_SINGLE1; break;
        case 2: mmc1.mirr = MIRROR_VERTICAL; break;
        case 3: mmc1.mirr = MIRROR_HORIZONTAL; break;
    }
}

static uint8_t mmc1_extra_register(void) {
    return (mmc1.last_chr_bank1 && (mmc1.control & 0x10)) ? mmc1.chr_bank1 : mmc1.chr_bank0;
}

static RamBlock *mmc1_ram_location(uint16_t a, size_t *offset) {
    uint8_t extra = mmc1_extra_register();
    size_t total = C.ram.prg_ram + C.ram.prg_nvram;
    size_t bank = 0;
    RamBlock *ram = default_prg_ram();
    if (C.ram.prg_ram == 0x2000 && C.ram.prg_nvram == 0x2000) {
        // SOROM uses CHR bit 3 to select its battery or work RAM chip.
        ram = (extra & 8) ? &prg_work_ram : &prg_save_ram;
    } else if (total > 0x4000) {
        bank = (extra >> 2) & 3; // SXROM: four 8KB banks.
    } else if (total > 0x2000) {
        bank = (extra >> 2) & 1;
    }
    *offset = bank * PRG_BANK_8K + (a & 0x1FFF);
    return ram;
}

static uint8_t mmc1_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        if (!C.mmc1a && (mmc1.prg_bank & 0x10)) return cart_cpu_bus_input;
        size_t offset;
        RamBlock *ram = mmc1_ram_location(a, &offset);
        return ram_read(ram, offset);
    }
    if (a >= 0x8000) {
        uint8_t prg_mode = (mmc1.control >> 2) & 3;
        size_t slot = (a - 0x8000) >> 14;
        size_t bank;
        uint8_t extra = mmc1_extra_register();
        size_t outer = C.prg_sz == 0x80000 ? (extra & 0x10) : 0;
        if (C.submapper == 5) {
            bank = slot; // Fixed-PRG MMC1 boards.
        } else if (prg_mode < 2) {
            bank = ((mmc1.prg_bank & 0x0E) + slot) | outer;
        } else if (prg_mode == 2) {
            bank = (slot ? (mmc1.prg_bank & 0x0F) : 0) | outer;
        } else {
            bank = (slot ? 0x0F : (mmc1.prg_bank & 0x0F)) | outer;
        }
        size_t banks = C.prg_sz / PRG_BANK_16K;
        if (!banks) return cart_cpu_bus_input;
        bank %= banks;
        return C.prg[bank * PRG_BANK_16K + (a & 0x3FFF)];
    }
    return cart_cpu_bus_input;
}

static void mmc1_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        if (C.mmc1a || !(mmc1.prg_bank & 0x10)) {
            size_t offset;
            RamBlock *ram = mmc1_ram_location(a, &offset);
            ram_write(ram, offset, v);
        }
        return;
    }
    if (a >= 0x8000) {
        uint64_t write_cycle = cpu_get_bus_cycle();
        bool consecutive = mmc1.has_write_cycle
                        && write_cycle - mmc1.last_write_cycle < 2;
        mmc1.last_write_cycle = write_cycle;
        mmc1.has_write_cycle = true;
        if (consecutive && !(v & 0x80)) return;
        if (v & 0x80) {
            mmc1.shift_reg = 0;
            mmc1.shift_count = 0;
            mmc1_write_control(mmc1.control | 0x0C);
        } else {
            mmc1.shift_reg = ((v & 1) << 4) | (mmc1.shift_reg >> 1);
            mmc1.shift_count++;
            if (mmc1.shift_count == 5) {
                uint8_t reg = (a >> 13) & 3;
                if (reg == 0) mmc1_write_control(mmc1.shift_reg);
                else if (reg == 1) {
                    mmc1.chr_bank0 = mmc1.shift_reg;
                    mmc1.last_chr_bank1 = false;
                } else if (reg == 2) {
                    mmc1.chr_bank1 = mmc1.shift_reg;
                    mmc1.last_chr_bank1 = true;
                } else mmc1.prg_bank = mmc1.shift_reg;
                mmc1.shift_reg = 0;
                mmc1.shift_count = 0;
            }
        }
    }
}

static bool mmc1_chr_offset(uint16_t a, size_t *offset) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    if (!page_size || !offset) return false;
    size_t slot = a / page_size;
    if (slot >= 2) return false;
    size_t bank = (mmc1.control & 0x10)
        ? (slot ? mmc1.chr_bank1 : mmc1.chr_bank0)
        : (size_t)(mmc1.chr_bank0 & 0x1E) + slot;
    size_t page_count = C.chr_sz / page_size;
    *offset = (bank % page_count) * page_size + (a % page_size);
    return true;
}

static uint8_t mmc1_ppu_read(uint16_t a) {
    size_t offset;
    return mmc1_chr_offset(a, &offset) ? chr_read_byte(offset) : chr_default_read(a, CHR_BANK_4K);
}

static void mmc1_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    size_t offset;
    chr_ram_write(mmc1_chr_offset(a, &offset) ? offset : chr_default_offset(a, CHR_BANK_4K), v);
}

static Mirroring mmc1_mirr(void) { return mmc1.mirr; }
static void mmc1_reset(void) {
    mmc1.shift_reg = 0;
    mmc1.shift_count = 0;
    mmc1_write_control(0x0C);
    mmc1.chr_bank0 = 0;
    mmc1.chr_bank1 = 0;
    mmc1.prg_bank = 0;
    mmc1.last_chr_bank1 = false;
    mmc1.has_write_cycle = false;
    mmc1.last_write_cycle = 0;
}

// Mapper 105: NES-EVENT competition board.
static struct {
    uint8_t init_state;
    uint32_t irq_counter;
    bool irq_enabled;
} m105;

static void m105_update_state(void) {
    if (m105.init_state == 0 && !(mmc1.chr_bank0 & 0x10)) m105.init_state = 1;
    else if (m105.init_state == 1 && (mmc1.chr_bank0 & 0x10)) m105.init_state = 2;

    if (mmc1.chr_bank0 & 0x10) {
        m105.irq_enabled = false;
        m105.irq_counter = 0;
        mapper_irq_line = false;
    } else {
        m105.irq_enabled = true;
    }
}

static size_t m105_prg_bank(uint16_t a) {
    size_t slot = (a - 0x8000u) >> 14;
    if (m105.init_state != 2) return slot;
    if (!(mmc1.chr_bank0 & 0x08)) return (size_t)(mmc1.chr_bank0 & 0x06) + slot;

    uint8_t prg = (uint8_t)((mmc1.prg_bank & 0x07) | 0x08);
    uint8_t mode = (mmc1.control >> 2) & 3;
    if (mode < 2) return (size_t)(prg & 0x0E) + slot;
    if (mode == 2) return slot ? prg : 0x08;
    return slot ? 0x0F : prg;
}

static uint8_t m105_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a < 0x8000) {
        if (mmc1.prg_bank & 0x10) return cart_cpu_bus_input;
        return prg_ram_read(a);
    }
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        if (!banks) return cart_cpu_bus_input;
        size_t bank = m105_prg_bank(a) % banks;
        return C.prg[bank * PRG_BANK_16K + (a & 0x3FFF)];
    }
    return cart_cpu_bus_input;
}

static void m105_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a < 0x8000) {
        if (!(mmc1.prg_bank & 0x10)) prg_ram_write(a, v);
        return;
    }
    if (a < 0x8000) return;
    uint8_t previous_count = mmc1.shift_count;
    mmc1_cpu_write(a, v);
    if ((v & 0x80) || (previous_count == 4 && mmc1.shift_count == 0))
        m105_update_state();
}

static uint8_t m105_ppu_read(uint16_t a) {
    // Competition control bits do not bank the board's fixed CHR RAM window.
    return C.chr_is_ram ? nrom_ppu_read(a) : (uint8_t)a;
}

static void m105_clock(int cpu_cycles) {
    if (!m105.irq_enabled || cpu_cycles <= 0) return;
    uint32_t limit = 0x20000000u | ((uint32_t)(cart_dip_value & 0x0Fu) << 25);
    if (m105.irq_counter >= limit) {
        ++m105.irq_counter;
        m105.irq_enabled = false;
        mapper_irq_line = true;
        return;
    }
    uint32_t remaining = limit - m105.irq_counter;
    if ((uint32_t)cpu_cycles >= remaining) {
        m105.irq_counter = limit;
        m105.irq_enabled = false;
        mapper_irq_line = true;
    } else {
        m105.irq_counter += (uint32_t)cpu_cycles;
    }
}

static void m105_reset(void) {
    mmc1_reset();
    memset(&m105, 0, sizeof(m105));
    mmc1.chr_bank0 = 0x10;
    m105_update_state();
}

// Mapper 232: Codemasters BF9096 multicart board.
static struct { uint8_t block, page; } m232;

static uint8_t m232_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a < 0x8000) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_16K;
    if (!banks) return cart_cpu_bus_input;
    size_t bank = ((size_t)m232.block << 2) | (a < 0xC000 ? m232.page : 3u);
    bank %= banks;
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFF)];
}

static void m232_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a < 0x8000) {
        prg_ram_write(a, v);
        return;
    }
    if (a < 0x8000) return;
    if (a >= 0xC000) {
        m232.page = v & 3u;
    } else if (C.submapper == 1) {
        m232.block = (uint8_t)(((v >> 4) & 1u) | ((v >> 2) & 2u));
    } else {
        m232.block = (v >> 3) & 3u;
    }
}

static uint8_t m232_ppu_read(uint16_t a) { return discrete_chr8_read(a, 0); }
static void m232_ppu_write(uint16_t a, uint8_t v) {
    discrete_chr8_write(a, 0, v);
}

static bool shrunk_chr_slot_geometry(uint16_t address, size_t native_page_size,
                                     unsigned slot_count, unsigned *slot,
                                     size_t *page_size, size_t *page_count) {
    if (!slot || !page_size || !page_count) return false;
    *page_size = shrunk_chr_page_size(native_page_size);
    if (!*page_size) return false;
    *slot = (unsigned)((address & 0x1FFFu) / *page_size);
    if (*slot >= slot_count) return false;
    *page_count = C.chr_sz / *page_size;
    return *page_count != 0;
}

static size_t shrunk_chr_bank_offset(uint16_t address, size_t page_size,
                                     size_t page_count, size_t bank) {
    return (bank % page_count) * page_size + ((address & 0x1FFFu) % page_size);
}

static Mirroring m232_mirr(void) { return C.mirr_base; }
static void m232_reset(void) { memset(&m232, 0, sizeof(m232)); }

// Mapper 96: Bandai Oeka Kids board.
static struct {
    uint8_t prg_bank;
    uint8_t outer_chr_bank;
    uint8_t inner_chr_bank;
    uint16_t last_ppu_addr;
    bool chr_banking_active;
} m96;

static uint8_t m96_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a < 0x8000) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_32K;
    if (!banks) return cart_cpu_bus_input;
    size_t bank = m96.prg_bank % banks;
    return C.prg[bank * PRG_BANK_32K + (a & 0x7FFFu)];
}

static void m96_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a < 0x8000) {
        prg_ram_write(a, v);
        return;
    }
    if (a < 0x8000) return;
    m96.prg_bank = v & 0x03u;
    m96.outer_chr_bank = v & 0x04u;
    m96.chr_banking_active = true;
}

static size_t m96_chr_bank(unsigned slot) {
    return slot == 0
        ? (size_t)(m96.outer_chr_bank | m96.inner_chr_bank)
        : (size_t)(m96.outer_chr_bank | 0x03u);
}

static uint8_t m96_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!C.chr_sz) return (uint8_t)a;
    if (!m96.chr_banking_active) return chr_default_read(a, CHR_BANK_4K);
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_4K, 2, &slot, &page_size, &page_count))
        return chr_default_read(a, CHR_BANK_4K);
    return chr_read_byte(shrunk_chr_bank_offset(a, page_size, page_count, m96_chr_bank(slot)));
}

static void m96_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram || !C.chr_sz) return;
    a &= 0x1FFFu;
    unsigned slot;
    size_t page_size, page_count;
    if (!m96.chr_banking_active
        || !shrunk_chr_slot_geometry(a, CHR_BANK_4K, 2, &slot, &page_size, &page_count)) {
        chr_default_write(a, CHR_BANK_4K, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, m96_chr_bank(slot)), v);
}

static Mirroring m96_mirr(void) { return C.mirr_base; }
static void m96_reset(void) { memset(&m96, 0, sizeof(m96)); }

// Discrete boards select complete PRG/CHR pages. Trailing partial pages are not
// folded into a selected bank. A PRG image smaller than a 32 KiB window is
// repeated only as many whole copies as fit in that window.
static uint8_t discrete_prg32_read(uint16_t a, uint8_t bank) {
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t offset = a - 0x8000u;
    if (C.prg_sz < PRG_BANK_32K) {
        return repeated_prg_window_read(a, 0x8000, PRG_BANK_32K);
    }
    size_t banks = C.prg_sz / PRG_BANK_32K;
    if (!banks) return cart_cpu_bus_input;
    return C.prg[(bank % banks) * PRG_BANK_32K + offset];
}

static uint8_t discrete_chr8_read(uint16_t a, uint8_t bank) {
    a &= 0x1FFFu;
    if (C.chr_sz < CHR_BANK_8K) {
        if (C.chr_is_ram) return chr_default_read(a, CHR_BANK_8K);
        return a < C.chr_sz ? chr_read_byte(a) : (uint8_t)a;
    }
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (!banks) return (uint8_t)a;
    return chr_read_byte((bank % banks) * CHR_BANK_8K + a);
}

static void discrete_chr8_write(uint16_t a, uint8_t bank, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    if (C.chr_sz < CHR_BANK_8K) {
        chr_default_write(a, CHR_BANK_8K, value);
        return;
    }
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (!banks) return;
    chr_ram_write((bank % banks) * CHR_BANK_8K + a, value);
}

// UxROM and the mapper 94/180 register-wiring variants.
static struct { uint8_t bank; } ux;
static uint8_t uxrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000 && C.prg_sz < PRG_BANK_16K)
        return repeated_prg_window_read(a, 0x8000, PRG_BANK_32K);
    if (a >= 0x8000 && a <= 0xBFFF) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t b = C.mapper_no == 180 || banks == 0 ? 0 : ux.bank % banks;
        return C.prg[b * PRG_BANK_16K + (a - 0x8000)];
    }
    if (a >= 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t bank = C.mapper_no == 180 ? ux.bank % banks : banks - 1;
        return C.prg[bank * PRG_BANK_16K + (a - 0xC000)];
    }
    return cart_cpu_bus_input;
}
static void uxrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) ux.bank = C.mapper_no == 94 ? (v >> 2) & 7 : v;
}
static uint8_t uxrom_ppu_read(uint16_t a) { return discrete_chr8_read(a, 0); }
static void uxrom_ppu_write(uint16_t a, uint8_t v) { discrete_chr8_write(a, 0, v); }
static Mirroring uxrom_mirr(void) { return C.mirr_base; }
static void uxrom_reset(void) { ux.bank = 0; }

// Mapper 3: CNROM.
static struct { uint8_t chr_bank; } cn;
static uint8_t cnrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return C.prg[a - 0x8000u];
    return cart_cpu_bus_input;
}
static void cnrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
        size_t banks = page_size ? C.chr_sz / page_size : 0;
        if (banks) cn.chr_bank = v % banks;
    }
}
static uint8_t cnrom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    if (!page_size || a >= page_size) return chr_default_read(a, CHR_BANK_8K);
    size_t bank = cn.chr_bank % (C.chr_sz / page_size);
    return chr_read_byte(bank * page_size + a);
}
static void cnrom_ppu_write(uint16_t a, uint8_t v) {
    if (C.chr_is_ram) {
        a &= 0x1FFFu;
        size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
        if (!page_size || a >= page_size) {
            chr_default_write(a, CHR_BANK_8K, v);
            return;
        }
        size_t bank = cn.chr_bank % (C.chr_sz / page_size);
        chr_ram_write(bank * page_size + a, v);
    }
}
static Mirroring cnrom_mirr(void) { return C.mirr_base; }
static void cnrom_reset(void) { cn.chr_bank = 0; }

// Mapper 185: protected CNROM.
static bool cnrom185_chr_enabled;
static bool cnrom185_initial_ram_mapping;

static uint8_t cnrom185_cpu_read(uint16_t a) {
    return cnrom_cpu_read(a);
}

static void cnrom185_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000u && a <= 0x7FFFu) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000u) return;

    if (C.submapper == 0) {
        cnrom185_chr_enabled = (value & 0x0Fu) != 0 && value != 0x13u;
    } else {
        cnrom185_chr_enabled = (value & 0x03u) == (C.submapper - 4u);
    }
    if (!cnrom185_chr_enabled) cnrom185_initial_ram_mapping = false;
}

static uint8_t cnrom185_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!cnrom185_chr_enabled) return (uint8_t)a | 0x01u;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    size_t coverage = cnrom185_initial_ram_mapping && page_size
                    ? CHR_BANK_8K / page_size * page_size : page_size;
    return page_size && a < coverage ? chr_read_byte(a % page_size) : (uint8_t)a;
}

static void cnrom185_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram || !cnrom185_chr_enabled) return;
    a &= 0x1FFFu;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    size_t coverage = cnrom185_initial_ram_mapping && page_size
                    ? CHR_BANK_8K / page_size * page_size : page_size;
    if (page_size && a < coverage) chr_ram_write(a % page_size, value);
}

static Mirroring cnrom185_mirr(void) { return C.mirr_base; }

static void cnrom185_reset(void) {
    cnrom185_chr_enabled = true;
    cnrom185_initial_ram_mapping = C.chr_is_ram;
}

#endif // MAPPER_NINTENDO_H

