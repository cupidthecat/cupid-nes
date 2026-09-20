/*
 * mapper_discrete.h - Discrete cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * Banking, mirroring, IRQ, and board state for discrete logic cartridges live here.
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
#ifndef MAPPER_DISCRETE_H
#define MAPPER_DISCRETE_H

// Color Dreams, including mapper 144's ROM-driven D0 line.
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
} colordreams;

static uint8_t colordreams_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return discrete_prg32_read(a, colordreams.prg_bank);
    return cart_cpu_bus_input;
}

static void colordreams_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        if (C.mapper_no == 144) v |= colordreams_cpu_read(a) & 1;
        colordreams.prg_bank = v & 0x0F;
        colordreams.chr_bank = (v >> 4) & 0x0F;
    }
}

static uint8_t colordreams_ppu_read(uint16_t a) {
    return discrete_chr8_read(a, colordreams.chr_bank);
}

static void colordreams_ppu_write(uint16_t a, uint8_t v) {
    discrete_chr8_write(a, colordreams.chr_bank, v);
}

static Mirroring colordreams_mirr(void) { return C.mirr_base; }
static void colordreams_reset(void) {
    colordreams.prg_bank = 0;
    colordreams.chr_bank = 0;
}

// NINA-03/06 and the mapper 113 multicart wiring.
static struct {
    uint8_t prg_bank, chr_bank;
    Mirroring mirr;
} nina;

static uint8_t nina_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return discrete_prg32_read(a, nina.prg_bank);
    return cart_cpu_bus_input;
}

static void nina_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        prg_ram_write(a, v);
    } else if ((a & 0xE100u) == 0x4100u) {
        if (C.mapper_no == 113) {
            nina.prg_bank = (v >> 3) & 7;
            nina.chr_bank = (v & 7) | ((v >> 3) & 8);
            nina.mirr = v & 0x80 ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
        } else {
            nina.prg_bank = (v >> 3) & 1;
            nina.chr_bank = v & 7;
        }
    }
}

static uint8_t nina_ppu_read(uint16_t a) {
    return discrete_chr8_read(a, nina.chr_bank);
}

static void nina_ppu_write(uint16_t a, uint8_t v) {
    discrete_chr8_write(a, nina.chr_bank, v);
}

static Mirroring nina_mirr(void) { return nina.mirr; }

static void nina_reset(void) {
    nina.prg_bank = nina.chr_bank = 0;
    nina.mirr = C.mirr_base;
}

// Mapper 13: CPROM.
static struct {
    uint8_t chr_bank;
    bool chr_bank_mapped;
} cprom;

static uint8_t cprom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return C.prg[a - 0x8000u];
    return cart_cpu_bus_input;
}

static void cprom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        cprom.chr_bank = v & 0x03;
        cprom.chr_bank_mapped = true;
    }
}

static uint8_t cprom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    if (!page_size) return chr_default_read(a, CHR_BANK_4K);
    size_t slot = a / page_size;
    if (slot == 0) return C.chr[a % page_size];
    if (slot == 1 && cprom.chr_bank_mapped) {
        size_t bank = cprom.chr_bank % (C.chr_sz / page_size);
        return C.chr[bank * page_size + (a % page_size)];
    }
    return chr_default_read(a, CHR_BANK_4K);
}

static void cprom_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    size_t slot = page_size ? a / page_size : 2;
    if (slot == 0) {
        chr_ram_write(a % page_size, v);
    } else if (slot == 1 && cprom.chr_bank_mapped) {
        size_t bank = cprom.chr_bank % (C.chr_sz / page_size);
        chr_ram_write(bank * page_size + (a % page_size), v);
    } else {
        chr_default_write(a, CHR_BANK_4K, v);
    }
}

static Mirroring cprom_mirr(void) { return MIRROR_VERTICAL; }
static void cprom_reset(void) { cprom.chr_bank = 0; cprom.chr_bank_mapped = false; }

// Mapper 15: 100-in-1 Contra Function 16.
static struct {
    uint16_t prg_banks[4];
    uint8_t mode;
    Mirroring mirr;
} m15;

static uint8_t m15_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        if (!banks) return cart_cpu_bus_input;
        size_t bank = m15.prg_banks[(a - 0x8000) >> 13];
        return C.prg[(bank % banks) * PRG_BANK_8K + (a & 0x1FFF)];
    }
    return cart_cpu_bus_input;
}

static void m15_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        uint8_t bank = (uint8_t)((v & 0x7F) << 1);
        uint8_t sub = v >> 7;
        m15.mode = a & 3;
        m15.mirr = (v & 0x40) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
        if (m15.mode == 0) {
            for (unsigned slot = 0; slot < 4; ++slot)
                m15.prg_banks[slot] = (uint16_t)((bank + slot) ^ sub);
        } else if (m15.mode == 2) {
            for (unsigned slot = 0; slot < 4; ++slot)
                m15.prg_banks[slot] = bank | sub;
        } else {
            bank |= sub;
            m15.prg_banks[0] = bank;
            m15.prg_banks[1] = (uint16_t)(bank + 1);
            if (m15.mode == 1) bank |= 0x0E;
            m15.prg_banks[2] = bank;
            m15.prg_banks[3] = (uint16_t)(bank + 1);
        }
    }
}

static uint8_t m15_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void m15_ppu_write(uint16_t a, uint8_t v) {
    if (m15.mode == 1 || m15.mode == 2) nrom_ppu_write(a, v);
}
static Mirroring m15_mirr(void) { return m15.mirr; }
static void m15_reset(void) {
    m15_cpu_write(0x8000, 0);
}

// Bandai FCG and LZ93D50 boards, including SRAM and Datach peripherals.
static struct {
    int mapper;
    uint8_t chr_regs[8], chr_banks[8], chr_mapped;
    uint8_t prg_bank, outer_bank;
    bool prg_selected, ram_enabled, irq_enabled;
    uint16_t irq_counter, irq_reload;
    Mirroring mirroring;
    uint8_t barcode[160];
    unsigned barcode_length, barcode_cycles;
} bandai;

static uint8_t bandai_cpu_read(uint16_t address) {
    if (address >= 0x8000) {
        if (address < 0xC000 && !bandai.prg_selected) return cart_cpu_bus_input;
        size_t banks = C.prg_sz / PRG_BANK_16K;
        if (!banks) return cart_cpu_bus_input;
        unsigned page = (address < 0xC000 ? bandai.prg_bank : 15u) | bandai.outer_bank;
        return C.prg[((size_t)page % banks) * PRG_BANK_16K + (address & 0x3FFF)];
    }
    if (address < 0x6000) return cart_cpu_bus_input;
    if (bandai.mapper == 153)
        return bandai.ram_enabled ? prg_ram_read(address) : cart_cpu_bus_input;
    uint8_t output = cart_cpu_bus_input & 0xE7;
    if (bandai.mapper == 157 && bandai.barcode_cycles / 1000 < bandai.barcode_length)
        output |= bandai.barcode[bandai.barcode_cycles / 1000];
    if (bandai_eeprom[0].capacity && bandai_eeprom[0].output
        && (!bandai_eeprom[1].capacity || bandai_eeprom[1].output)) output |= 0x10;
    return output;
}

static bool bandai_has_chr_ram(void) {
    return C.chr_is_ram || C.ram.chr_ram || C.ram.chr_nvram;
}

static void bandai_cpu_write(uint16_t address, uint8_t value) {
    if (address < 0x6000) return;
    if (address < 0x8000 && bandai.mapper != 16) {
        if (bandai.mapper == 153 && bandai.ram_enabled) prg_ram_write(address, value);
        return;
    }
    if (bandai.mapper == 16
        && ((C.submapper == 4 && address >= 0x8000)
            || (C.submapper == 5 && address < 0x8000))) return;
    unsigned reg = address & 15;
    if (reg < 8) {
        bandai.chr_regs[reg] = value;
        if (bandai.mapper == 153 || C.prg_sz >= 0x80000) {
            bandai.outer_bank = 0;
            for (unsigned i = 0; i < 8; ++i)
                bandai.outer_bank |= (bandai.chr_regs[i] & 1u) << 4;
            bandai.prg_selected = true;
        } else if (!bandai_has_chr_ram() && bandai.mapper != 157) {
            bandai.chr_banks[reg] = value;
            bandai.chr_mapped |= (uint8_t)(1u << reg);
        }
        if (bandai.mapper == 157 && reg < 4)
            eeprom24_write(&bandai_eeprom[1], (value & 8) != 0, bandai_eeprom[1].sda);
        return;
    }
    bool direct_counter = bandai.mapper == 16 && C.submapper == 4;
    switch (reg) {
        case 8:
            bandai.prg_bank = value & 15;
            bandai.prg_selected = true;
            break;
        case 9: {
            static const Mirroring modes[] = {
                MIRROR_VERTICAL, MIRROR_HORIZONTAL, MIRROR_SINGLE0, MIRROR_SINGLE1
            };
            bandai.mirroring = modes[value & 3];
            break;
        }
        case 10:
            bandai.irq_enabled = (value & 1) != 0;
            if (!direct_counter) bandai.irq_counter = bandai.irq_reload;
            mapper_irq_line = false;
            break;
        case 11:
            if (direct_counter) bandai.irq_counter = (bandai.irq_counter & 0xFF00u) | value;
            else bandai.irq_reload = (bandai.irq_reload & 0xFF00u) | value;
            break;
        case 12:
            if (direct_counter) bandai.irq_counter = (uint16_t)((bandai.irq_counter & 0xFFu) | ((unsigned)value << 8));
            else bandai.irq_reload = (uint16_t)((bandai.irq_reload & 0xFFu) | ((unsigned)value << 8));
            break;
        case 13:
            if (bandai.mapper == 153) {
                bandai.ram_enabled = (value & 0x20) != 0;
            } else {
                eeprom24_write(&bandai_eeprom[0], (value & 0x20) != 0, (value & 0x40) != 0);
                eeprom24_write(&bandai_eeprom[1], bandai_eeprom[1].scl, (value & 0x40) != 0);
            }
            break;
        default:
            break;
    }
}

static uint8_t bandai_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    if (C.chr_is_ram) return chr_default_read(address, CHR_BANK_1K);
    if (C.ram.chr_ram || C.ram.chr_nvram) return (uint8_t)address;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(address, CHR_BANK_1K, 8, &slot, &page_size, &page_count))
        return chr_default_read(address, CHR_BANK_1K);
    if (!(bandai.chr_mapped & (1u << slot))) return (uint8_t)address;
    return C.chr[shrunk_chr_bank_offset(address, page_size, page_count, bandai.chr_banks[slot])];
}

static void bandai_ppu_write(uint16_t address, uint8_t value) {
    if (C.chr_is_ram) chr_default_write(address, CHR_BANK_1K, value);
}

static void bandai_clock(int cycles) {
    if (cycles <= 0) return;
    if (bandai.irq_enabled) {
        // The output asserts on the clock after zero, before the counter wraps.
        if ((unsigned)cycles > bandai.irq_counter) mapper_irq_line = true;
        bandai.irq_counter = (uint16_t)(bandai.irq_counter - (unsigned)cycles);
    }
    unsigned end = bandai.barcode_length * 1000;
    if (bandai.barcode_cycles < end) {
        unsigned remaining = end - bandai.barcode_cycles;
        bandai.barcode_cycles += (unsigned)cycles < remaining ? (unsigned)cycles : remaining;
    }
}

static Mirroring bandai_mirroring(void) { return bandai.mirroring; }

static void bandai_init(int mapper, unsigned standard_eeprom, unsigned extra_eeprom) {
    memset(&bandai, 0, sizeof(bandai));
    bandai.mapper = mapper;
    bandai.mirroring = C.mirr_base;
    bandai.ram_enabled = true;
    bandai.chr_mapped = 0;
    for (unsigned i = 0; i < 8; ++i) bandai.chr_banks[i] = (uint8_t)i;
    eeprom24_init(&bandai_eeprom[0], standard_eeprom);
    eeprom24_init(&bandai_eeprom[1], extra_eeprom);
}

/* NSF/NSFe music execution environment.  The player maps a tiny reset/IRQ
   trampoline at $4100, eight 4 KiB program windows, and the same expansion
   audio implementations used by cartridge mappers. */
static uint32_t nsf_play_reload(void) {
    uint16_t speed = nes_timing()->region == NES_REGION_NTSC
                   ? nsf_player.metadata.play_speed_ntsc : nsf_player.metadata.play_speed_pal;
    double cycles = (double)speed * nes_timing()->cpu_hz / 1000000.0;
    uint16_t reload = (uint16_t)(uint32_t)cycles;
    return reload;
}

static size_t nsf_program_page(uint8_t bank) {
    size_t pages = C.prg_sz / 0x1000u;
    return pages ? (size_t)bank % pages : 0;
}

static uint8_t nsf_program_read(uint16_t address, unsigned slot) {
    if (!C.prg || C.prg_sz < 0x1000u || slot >= 10) return cart_cpu_bus_input;
    size_t page = nsf_program_page(nsf_player.banks[slot]);
    return C.prg[page * 0x1000u + (address & 0x0FFFu)];
}

static void nsf_program_write(uint16_t address, unsigned slot, uint8_t value) {
    if (!C.prg || C.prg_sz < 0x1000u || slot >= 8) return;
    size_t page = nsf_program_page(nsf_player.banks[slot]);
    C.prg[page * 0x1000u + (address & 0x0FFFu)] = value;
}

static uint8_t nsf_cpu_read(uint16_t address) {
    if (address >= 0x4100 && address < 0x4200) {
        size_t offset = address - 0x4100u;
        return offset < sizeof(nsf_player.bios) ? nsf_player.bios[offset] : 0;
    }
    if ((nsf_player.metadata.sound_chips & NSF_SOUND_FDS)
        && address >= 0x4040 && address <= 0x4097)
        return fds_nsf_audio_read(address, cart_cpu_bus_input);
    if ((nsf_player.metadata.sound_chips & NSF_SOUND_NAMCO163)
        && address >= 0x4800 && address <= 0x4FFF)
        return namco163_audio_read_data(&namco163_audio);
    if ((nsf_player.metadata.sound_chips & NSF_SOUND_MMC5) && address == 0x5205) {
        uint16_t product = (uint16_t)nsf_player.mmc5_multiplier[0] * nsf_player.mmc5_multiplier[1];
        return (uint8_t)product;
    }
    if ((nsf_player.metadata.sound_chips & NSF_SOUND_MMC5) && address == 0x5206) {
        uint16_t product = (uint16_t)nsf_player.mmc5_multiplier[0] * nsf_player.mmc5_multiplier[1];
        return (uint8_t)(product >> 8);
    }
    if ((nsf_player.metadata.sound_chips & NSF_SOUND_MMC5)
        && address >= 0x5C00 && address <= 0x5FFF)
        return prg_work_ram.data ? prg_work_ram.data[0x3000u + (address - 0x5C00u)] : cart_cpu_bus_input;
    if (address >= 0x6000 && address < 0x8000) {
        unsigned slot = (address - 0x6000u) >> 12;
        if (nsf_player.lower_program[slot]) return nsf_program_read(address, slot);
        return prg_work_ram.data ? prg_work_ram.data[address - 0x6000u] : cart_cpu_bus_input;
    }
    if (address >= 0x8000) {
        if (address >= 0xFFFC) {
            static const uint8_t vectors[4] = {0x00, 0x41, 0x10, 0x41};
            return vectors[address - 0xFFFCu];
        }
        return nsf_program_read(address, 2u + ((address - 0x8000u) >> 12));
    }
    return cart_cpu_bus_input;
}

static void nsf_cpu_write(uint16_t address, uint8_t value) {
    uint8_t chips = nsf_player.metadata.sound_chips;
    if ((chips & NSF_SOUND_FDS) && address >= 0x4040 && address <= 0x408A) {
        fds_nsf_audio_write(address, value);
        return;
    }
    if ((chips & NSF_SOUND_MMC5) && address >= 0x5000 && address <= 0x5015) {
        mmc5_cpu_write(address, value);
        return;
    }
    if ((chips & NSF_SOUND_NAMCO163)
        && ((address >= 0x4800 && address <= 0x4FFF) || address >= 0xF800)) {
        if (address >= 0xF800) namco163_audio_write_address(&namco163_audio, value);
        else (void)namco163_audio_write_data(&namco163_audio, value);
        return;
    }
    if ((chips & NSF_SOUND_SUNSOFT5B) && address >= 0xC000) {
        sunsoft5b_write(&sunsoft5b_audio, address, value);
        return;
    }
    if ((chips & NSF_SOUND_VRC7) && address == 0x9010) {
        vrc7_fm_write_address(&nsf_vrc7, value);
        return;
    }
    if ((chips & NSF_SOUND_VRC7) && address == 0x9030) {
        vrc7_fm_write_data(&nsf_vrc7, value);
        return;
    }
    if ((chips & NSF_SOUND_VRC6)
        && ((address >= 0x9000 && address <= 0x9003)
            || (address >= 0xA000 && address <= 0xA002)
            || (address >= 0xB000 && address <= 0xB002))) {
        vrc6_audio_write(address, value);
        return;
    }
    if (address == 0x4100) {
        nsf_player.play_counter = nsf_play_reload();
        mapper_irq_line = false;
        return;
    }
    if ((chips & NSF_SOUND_MMC5) && address == 0x5205) {
        nsf_player.mmc5_multiplier[0] = value;
        return;
    }
    if ((chips & NSF_SOUND_MMC5) && address == 0x5206) {
        nsf_player.mmc5_multiplier[1] = value;
        return;
    }
    if (address >= 0x5FF6 && address <= 0x5FFF) {
        unsigned slot = address - 0x5FF6u;
        nsf_player.banks[slot] = value;
        if (slot < 2) nsf_player.lower_program[slot] = true;
        return;
    }
    if ((chips & NSF_SOUND_MMC5) && address >= 0x5C00 && address <= 0x5FFF) {
        if (prg_work_ram.data) prg_work_ram.data[0x3000u + (address - 0x5C00u)] = value;
        return;
    }
    if (address >= 0x6000 && address < 0x8000) {
        unsigned slot = (address - 0x6000u) >> 12;
        if (nsf_player.lower_program[slot]) nsf_program_write(address, slot, value);
        else if (prg_work_ram.data) prg_work_ram.data[address - 0x6000u] = value;
        return;
    }
    if ((chips & NSF_SOUND_FDS) && address >= 0x8000 && address <= 0xDFFF)
        nsf_program_write(address, 2u + ((address - 0x8000u) >> 12), value);
}

static void nsf_clock(int cpu_cycles) {
    uint8_t chips = nsf_player.metadata.sound_chips;
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        if (nsf_player.play_counter && --nsf_player.play_counter == 0) {
            nsf_player.play_counter = nsf_play_reload();
            mapper_irq_line = true;
        }
        if (chips & NSF_SOUND_MMC5) {
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
        }
        if (chips & NSF_SOUND_VRC6) {
            if (!vrc6.halt_audio) {
                vrc6_clock_pulse(&vrc6.pulse[0]);
                vrc6_clock_pulse(&vrc6.pulse[1]);
                vrc6_clock_saw();
            }
        }
    }
    if (chips & NSF_SOUND_VRC7) vrc7_fm_clock(&nsf_vrc7, cpu_cycles, nes_timing()->cpu_hz);
    if (chips & NSF_SOUND_NAMCO163) (void)namco163_audio_clock(&namco163_audio, cpu_cycles);
    if (chips & NSF_SOUND_SUNSOFT5B) sunsoft5b_clock(&sunsoft5b_audio, cpu_cycles);
    if (chips & NSF_SOUND_FDS) fds_nsf_audio_clock(cpu_cycles);
}

static void nsf_reset(bool soft_reset) {
    if (!soft_reset) nsf_player.song = nsf_player.metadata.starting_song;
    nsf_player.play_counter = 0;
    mapper_irq_line = false;
    if (nsf_player.metadata.sound_chips & NSF_SOUND_MMC5) mmc5_reset();
    if (nsf_player.metadata.sound_chips & NSF_SOUND_VRC6) { vrc6.variant_b = false; vrc6_reset(); }
    if (nsf_player.metadata.sound_chips & NSF_SOUND_VRC7) vrc7_fm_reset(&nsf_vrc7);
    if (nsf_player.metadata.sound_chips & NSF_SOUND_NAMCO163) namco163_audio_reset(&namco163_audio);
    if (nsf_player.metadata.sound_chips & NSF_SOUND_SUNSOFT5B) sunsoft5b_reset(&sunsoft5b_audio);
    if (nsf_player.metadata.sound_chips & NSF_SOUND_FDS) fds_nsf_audio_reset();
}

static void nsf_after_reset(void) {
    cpu_clear_internal_ram();
    if (prg_work_ram.data) memset(prg_work_ram.data, 0, prg_work_ram.size);
    for (uint16_t address = 0x4000; address < 0x4013; ++address) apu_write(address, 0);
    apu_write(0x4015, 0);
    apu_write(0x4015, 0x0F);
    apu_write(0x4017, 0x40);
    ppu_reg_write(0x2000, 0);
    ppu_reg_write(0x2001, 0);

    memset(nsf_player.banks, 0, sizeof(nsf_player.banks));
    memset(nsf_player.lower_program, 0, sizeof(nsf_player.lower_program));
    uint8_t setup[8];
    memcpy(setup, nsf_player.metadata.bank_setup, sizeof(setup));
    if (!nsf_player.explicit_banking) {
        memset(setup, 0, sizeof(setup));
        int start_bank = nsf_player.metadata.load_address / 0x1000;
        size_t pages = C.prg_sz / 0x1000u;
        for (size_t i = 0; i < pages && start_bank + (int)i <= 0x0F; ++i) {
            int slot = start_bank + (int)i - 8;
            if (slot >= 0 && slot < 8) setup[slot] = (uint8_t)i;
        }
    }
    for (unsigned i = 0; i < 8; ++i) nsf_player.banks[i + 2] = setup[i];
    if (nsf_player.metadata.sound_chips & NSF_SOUND_FDS) {
        nsf_player.banks[0] = setup[6];
        nsf_player.banks[1] = setup[7];
        nsf_player.lower_program[0] = true;
        nsf_player.lower_program[1] = true;
    }
    cpu.a = nsf_player.song;
    cpu.x = nes_timing()->region == NES_REGION_NTSC ? 0 : 1;
    cpu.y = 0;
    cpu.sp = 0xFD;
    nsf_player.track_start_cycle = cpu_total_cycles;
}

static uint8_t nsf_ppu_read(uint16_t address) { return C.chr[address & 0x1FFFu]; }
static void nsf_ppu_write(uint16_t address, uint8_t value) { C.chr[address & 0x1FFFu] = value; }
static Mirroring nsf_mirroring(void) { return MIRROR_HORIZONTAL; }

int mapper_init_nsf(const NsfImage *image, uint8_t *program, size_t program_size,
                    uint8_t *chr, size_t chr_size) {
    if (!image || !program || program_size < 0x1000 || (program_size & 0x0FFFu)
        || !chr || chr_size < 0x2000) return -1;
    RamBlock new_work = { (uint8_t *)calloc(1, 0x4000), 0x4000 };
    if (!new_work.data) return -1;
    Vrc7Fm new_fm = {0};
    bool need_vrc7 = (image->metadata.sound_chips & NSF_SOUND_VRC7) != 0;
    if (need_vrc7 && !vrc7_fm_init(&new_fm)) { free(new_work.data); return -1; }

    mapper_shutdown();
    C.prg = program; C.prg_sz = program_size;
    C.chr = chr; C.chr_sz = chr_size; C.chr_is_ram = true;
    C.mapper_no = 65531; C.mirr_base = MIRROR_HORIZONTAL;
    prg_work_ram = new_work;
    memset(&nsf_player, 0, sizeof(nsf_player));
    nsf_player.metadata = image->metadata;
    for (unsigned i = 0; i < 8; ++i)
        if (image->metadata.bank_setup[i]) nsf_player.explicit_banking = true;
    static const uint8_t bios[0x20] = {
        0x58,0x20,0x00,0x00,0x8D,0x00,0x41,0x58,0x4C,0x08,0x41,0,0,0,0,0,
        0x8D,0x00,0x41,0x20,0x00,0x00,0x58,0x40,0,0,0,0,0,0,0,0
    };
    memcpy(nsf_player.bios, bios, sizeof(bios));
    nsf_player.bios[2] = (uint8_t)image->metadata.init_address;
    nsf_player.bios[3] = (uint8_t)(image->metadata.init_address >> 8);
    nsf_player.bios[0x14] = (uint8_t)image->metadata.play_address;
    nsf_player.bios[0x15] = (uint8_t)(image->metadata.play_address >> 8);
    if (need_vrc7) nsf_vrc7 = new_fm;
    build_mapper(&mapper_nsf, nsf_cpu_read, nsf_cpu_write,
                 nsf_ppu_read, nsf_ppu_write, NULL, nsf_mirroring);
    mapper_nsf.clock = nsf_clock;
    cart = &mapper_nsf;
    nsf_reset(false);
    return 65531;
}

bool cart_nsf_active(void) { return cart == &mapper_nsf; }
unsigned cart_nsf_current_track(void) { return cart == &mapper_nsf ? nsf_player.song : 0; }
bool cart_nsf_select_track(unsigned track) {
    if (cart != &mapper_nsf || track >= nsf_player.metadata.total_songs) return false;
    nsf_player.song = (uint8_t)track;
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    return true;
}

static void barcode_pattern(uint8_t *bits, unsigned *length, unsigned pattern, unsigned width) {
    for (unsigned bit = width; bit > 0; --bit)
        bits[(*length)++] = (pattern & (1u << (bit - 1))) ? 0 : 8;
}

bool cart_set_barcode(const char *digits) {
    if (cart != &mapper_bandai || bandai.mapper != 157 || !digits) return false;
    size_t count = strlen(digits);
    if (count != 8 && count != 13) return false;
    for (size_t i = 0; i < count; ++i)
        if (digits[i] < '0' || digits[i] > '9') return false;
    static const uint8_t left[] = {0x0D, 0x19, 0x13, 0x3D, 0x23, 0x31, 0x2F, 0x3B, 0x37, 0x0B};
    static const uint8_t parity[] = {0x3F, 0x34, 0x32, 0x31, 0x2C, 0x26, 0x23, 0x2A, 0x29, 0x25};
    uint8_t bits[160];
    unsigned length = 33;
    memset(bits, 8, length);
    barcode_pattern(bits, &length, 5, 3);
    unsigned left_count = count == 13 ? 6 : 4;
    for (unsigned i = 0; i < left_count; ++i) {
        unsigned digit = (unsigned)(digits[i + (count == 13 ? 1 : 0)] - '0');
        unsigned pattern = left[digit];
        if (count == 13 && !(parity[digits[0] - '0'] & (1u << (5 - i)))) {
            unsigned reversed = 0;
            for (unsigned bit = 0; bit < 7; ++bit) reversed = (reversed << 1) | ((pattern >> bit) & 1);
            pattern = reversed ^ 0x7F;
        }
        barcode_pattern(bits, &length, pattern, 7);
    }
    barcode_pattern(bits, &length, 0x0A, 5);
    for (unsigned i = count == 13 ? 7 : 4; i + 1 < count; ++i)
        barcode_pattern(bits, &length, left[digits[i] - '0'] ^ 0x7Fu, 7);
    unsigned sum = 0;
    for (unsigned i = 0; i + 1 < count; ++i)
        sum += (unsigned)(digits[i] - '0') * (((i & 1) != (count == 13)) ? 1u : 3u);
    unsigned checksum = (10 - sum % 10) % 10;
    barcode_pattern(bits, &length, left[checksum] ^ 0x7Fu, 7);
    barcode_pattern(bits, &length, 5, 3);
    memset(bits + length, 8, 32);
    length += 32;
    memcpy(bandai.barcode, bits, length);
    bandai.barcode_length = length;
    bandai.barcode_cycles = 0;
    return true;
}

// Mapper 28: Action 53.
static struct {
    uint8_t selected_reg;
    uint8_t regs[4];
    uint8_t mirroring_bit;
    size_t prg_bank[2];
    uint8_t chr_bank;
    bool chr_selected;
    bool prg_selected;
    Mirroring mirr;
} m28;

static void m28_update_state(void) {
    m28.prg_selected = true;
    uint8_t mirroring = m28.regs[2] & 0x03;
    if (!(mirroring & 0x02)) mirroring = m28.mirroring_bit;
    switch (mirroring) {
        case 0: m28.mirr = MIRROR_SINGLE0; break;
        case 1: m28.mirr = MIRROR_SINGLE1; break;
        case 2: m28.mirr = MIRROR_VERTICAL; break;
        default: m28.mirr = MIRROR_HORIZONTAL; break;
    }

    unsigned game_size = (m28.regs[2] >> 4) & 0x03;
    bool prg_16k = (m28.regs[2] & 0x08) != 0;
    bool slot_select = (m28.regs[2] & 0x04) != 0;
    unsigned prg_select = m28.regs[1] & 0x0F;
    unsigned outer = (unsigned)m28.regs[3] << 1;
    static const unsigned outer_mask[4] = {0x1FE, 0x1FC, 0x1F8, 0x1F0};
    static const unsigned inner_mask[4] = {0x01, 0x03, 0x07, 0x0F};
    size_t banks = C.prg_sz / PRG_BANK_16K;

    if (!banks) {
        m28.prg_bank[0] = 0;
        m28.prg_bank[1] = 0;
    } else if (prg_16k) {
        unsigned selected = (outer & outer_mask[game_size]) | (prg_select & inner_mask[game_size]);
        unsigned fixed = (outer & 0x1FE) | (slot_select ? 1u : 0u);
        m28.prg_bank[slot_select ? 0 : 1] = selected % banks;
        m28.prg_bank[slot_select ? 1 : 0] = fixed % banks;
    } else {
        unsigned selected = prg_select << 1;
        unsigned base = (outer & outer_mask[game_size]) | (selected & inner_mask[game_size]);
        m28.prg_bank[0] = base % banks;
        m28.prg_bank[1] = ((outer & outer_mask[game_size])
            | ((selected | 1u) & inner_mask[game_size])) % banks;
    }
    m28.chr_bank = m28.regs[0] & 0x03;
    m28.chr_selected = true;
}

static uint8_t m28_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        if (a < 0xC000 && !m28.prg_selected) return cart_cpu_bus_input;
        size_t slot = (a >> 14) & 1u;
        size_t banks = C.prg_sz / PRG_BANK_16K;
        if (!banks) return cart_cpu_bus_input;
        size_t bank = m28.prg_bank[slot] % banks;
        return C.prg[bank * PRG_BANK_16K + (a & 0x3FFF)];
    }
    return cart_cpu_bus_input;
}

// Mapper 18: Jaleco SS88006.
static struct {
    uint8_t prg_banks[3];
    uint8_t chr_banks[8];
    uint8_t prg_mapped, chr_mapped;
    uint8_t irq_reload[4];
    uint16_t irq_counter;
    uint8_t irq_counter_size;
    bool irq_enabled;
    Mirroring mirr;
} jaleco18;

static const uint16_t jaleco18_irq_mask[4] = {0xFFFF, 0x0FFF, 0x00FF, 0x000F};

static void jaleco18_update_nibble(uint8_t *reg, uint8_t value, bool upper) {
    value &= 0x0F;
    if (upper) *reg = (uint8_t)((*reg & 0x0F) | (value << 4));
    else *reg = (uint8_t)((*reg & 0xF0) | value);
}

static uint8_t jaleco18_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_8K;
        unsigned slot = (a - 0x8000u) >> 13;
        if (slot < 3 && !(jaleco18.prg_mapped & (1u << slot))) return cart_cpu_bus_input;
        size_t bank = slot == 3 ? banks - 1 : jaleco18.prg_banks[slot] % banks;
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void m28_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x5000 && a <= 0x5FFF) {
        m28.selected_reg = (uint8_t)(((v & 0x80) >> 6) | (v & 0x01));
        return;
    }
    if (a >= 0x6000 && a <= 0x7FFF) {
        prg_ram_write(a, v);
        return;
    }
    if (a < 0x8000) return;

    if (m28.selected_reg <= 1) m28.mirroring_bit = (v >> 4) & 1;
    else if (m28.selected_reg == 2) m28.mirroring_bit = v & 1;
    m28.regs[m28.selected_reg] = v;
    m28_update_state();
}

static uint8_t m28_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    if (!C.chr_sz) return (uint8_t)a;
    if (!C.chr_is_ram && !m28.chr_selected) return (uint8_t)a;
    return discrete_chr8_read(a, m28.chr_bank);
}

static void m28_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_sz) return;
    discrete_chr8_write(a, m28.chr_bank, v);
}

static Mirroring m28_mirr(void) { return m28.mirr; }

static void m28_power_on(void) {
    memset(&m28, 0, sizeof(m28));
    m28.mirr = C.mirr_base;
    m28.prg_bank[0] = 0;
    m28.prg_bank[1] = C.prg_sz / PRG_BANK_16K - 1;
}

// Mapper 30: UNROM 512.
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
    uint8_t flash_cycle;
    FlashMode flash_mode;
    bool software_id;
    bool flash_writable;
    bool chr_selected;
    bool mirroring_bit_enabled;
    bool led_variant;
    Mirroring mirr;
} m30;

static void m30_flash_reset_command(void) {
    m30.flash_mode = FLASH_WAITING;
    m30.flash_cycle = 0;
}

static int m30_flash_read(size_t physical) {
    if (!m30.software_id) return -1;
    switch (physical & 0x1FF) {
        case 0: return 0xBF;
        case 1: return 0xB7;
        default: return 0xFF;
    }
}

static void m30_flash_program(size_t physical, uint8_t value) {
    if (physical >= C.prg_sz) return;
    uint8_t programmed = C.prg[physical] & value;
    if (programmed != C.prg[physical]) {
        C.prg[physical] = programmed;
        flash_dirty = true;
    }
}

static void m30_flash_erase_sector(size_t physical) {
    size_t offset = physical & 0x7F000u;
    if (offset + 0x1000 > C.prg_sz) return;
    for (size_t i = 0; i < 0x1000; ++i) {
        if (C.prg[offset + i] != 0xFF) {
            memset(C.prg + offset, 0xFF, 0x1000);
            flash_dirty = true;
            break;
        }
    }
}

static void m30_flash_chip_erase(void) {
    for (size_t i = 0; i < C.prg_sz; ++i) {
        if (C.prg[i] != 0xFF) {
            memset(C.prg, 0xFF, C.prg_sz);
            flash_dirty = true;
            return;
        }
    }
}

static void m30_flash_write(size_t physical, uint8_t value) {
    unsigned command_addr = (unsigned)(physical & 0x7FFFu);
    if (m30.flash_mode == FLASH_PROGRAM) {
        m30_flash_program(physical, value);
        m30_flash_reset_command();
        return;
    }
    if (m30.flash_mode == FLASH_ERASE) {
        if (m30.flash_cycle == 3 && command_addr == 0x5555 && value == 0xAA) {
            m30.flash_cycle = 4;
            return;
        }
        if (m30.flash_cycle == 4 && command_addr == 0x2AAA && value == 0x55) {
            m30.flash_cycle = 5;
            return;
        }
        if (m30.flash_cycle == 5) {
            if (command_addr == 0x5555 && value == 0x10) m30_flash_chip_erase();
            else if (value == 0x30) m30_flash_erase_sector(physical);
        }
        m30_flash_reset_command();
        return;
    }

    if (m30.flash_cycle == 0) {
        if (command_addr == 0x5555 && value == 0xAA) m30.flash_cycle = 1;
        else if (value == 0xF0) {
            m30_flash_reset_command();
            m30.software_id = false;
        }
        return;
    }
    if (m30.flash_cycle == 1 && command_addr == 0x2AAA && value == 0x55) {
        m30.flash_cycle = 2;
        return;
    }
    if (m30.flash_cycle == 2 && command_addr == 0x5555) {
        m30.flash_cycle = 3;
        switch (value) {
            case 0x80:
                m30.flash_mode = FLASH_ERASE;
                m30.flash_cycle = 3;
                return;
            case 0x90:
                m30_flash_reset_command();
                m30.software_id = true;
                return;
            case 0xA0:
                m30.flash_mode = FLASH_PROGRAM;
                m30.flash_cycle = 3;
                return;
            case 0xF0:
                m30_flash_reset_command();
                m30.software_id = false;
                return;
            default:
                return;
        }
    }
    m30.flash_cycle = 0;
}

static void m30_latch(uint8_t value) {
    size_t chr_page_size = C.chr_sz < CHR_BANK_8K ? C.chr_sz : CHR_BANK_8K;
    size_t chr_banks = chr_page_size ? C.chr_sz / chr_page_size : 1;
    m30.prg_bank = value & 0x1F;
    m30.chr_bank = (uint8_t)(((value >> 5) & 0x03) % chr_banks);
    m30.chr_selected = true;
    if (m30.mirroring_bit_enabled) {
        if (C.submapper == 3)
            m30.mirr = (value & 0x80) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
        else
            m30.mirr = (value & 0x80) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    }
}

static uint8_t m30_cpu_read(uint16_t a) {
    if (a >= 0x6000u && a < 0x8000u) return prg_ram_read(a);
    if (a < 0x8000) return cart_cpu_bus_input;
    if (m30.flash_writable) {
        int flash_value = m30_flash_read(a);
        if (flash_value >= 0) return (uint8_t)flash_value;
    }
    if (C.prg_sz < PRG_BANK_16K)
        return repeated_prg_window_read(a, 0x8000, PRG_BANK_32K);
    size_t banks = C.prg_sz / PRG_BANK_16K;
    size_t bank = a < 0xC000 ? m30.prg_bank % banks : banks - 1;
    return C.prg[bank * PRG_BANK_16K + (a & 0x3FFF)];
}

static void m30_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000u && a < 0x8000u) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000) return;
    if (m30.led_variant && a < 0xC000) return;
    if (!m30.flash_writable || a >= 0xC000) {
        m30_latch(value);
        return;
    }
    size_t physical = (size_t)m30.prg_bank * PRG_BANK_16K + (a & 0x3FFF);
    m30_flash_write(physical, value);
}

static uint8_t m30_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    if (!C.chr_sz) return (uint8_t)a;
    if (!C.chr_is_ram && !m30.chr_selected) return (uint8_t)a;
    return discrete_chr8_read(a, m30.chr_bank);
}

static void m30_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_sz) return;
    discrete_chr8_write(a, m30.chr_bank, value);
}

static Mirroring m30_mirr(void) { return m30.mirr; }

static void m30_power_on(const iNESHeader *h) {
    memset(&m30, 0, sizeof(m30));
    m30.flash_writable = (h->flags6 & 0x02) != 0;
    m30.led_variant = C.submapper == 4;
    m30.mirr = C.mirr_base;
    m30.mirroring_bit_enabled = false;
    unrom512_four_screen_chr = false;

    if (C.submapper == 3) {
        m30.mirroring_bit_enabled = true;
        m30.mirr = MIRROR_VERTICAL;
    } else {
        switch (h->flags6 & 0x09) {
            case 0x00: m30.mirr = MIRROR_HORIZONTAL; break;
            case 0x01: m30.mirr = MIRROR_VERTICAL; break;
            case 0x08:
                m30.mirr = MIRROR_SINGLE0;
                m30.mirroring_bit_enabled = true;
                break;
            case 0x09:
                m30.mirr = MIRROR_FOUR;
                unrom512_four_screen_chr = (C.chr_is_ram ? C.chr_sz
                    : chr_work_ram.size + chr_save_ram.size) >= 0x8000;
                break;
        }
    }
}

// Mapper 111: GTROM with 512 KiB writable flash, 16 KiB CHR RAM and
// cartridge nametable RAM.
static void m111_flash_reset_command(void) {
    m111.flash_mode = FLASH_WAITING;
    m111.flash_cycle = 0;
}

static int m111_flash_read(size_t physical) {
    if (!m111.software_id) return -1;
    switch (physical & 0x1FFu) {
        case 0: return 0xBF;
        case 1: return 0xB7;
        default: return 0xFF;
    }
}

static void m111_flash_program(size_t physical, uint8_t value) {
    if (physical >= C.prg_sz) return;
    uint8_t programmed = C.prg[physical] & value;
    if (programmed == C.prg[physical]) return;
    C.prg[physical] = programmed;
    flash_dirty = true;
}

static void m111_flash_erase_sector(size_t physical) {
    size_t offset = physical & 0x7F000u;
    if (offset + 0x1000 > C.prg_sz) return;
    for (size_t i = 0; i < 0x1000; ++i) {
        if (C.prg[offset + i] == 0xFF) continue;
        memset(C.prg + offset, 0xFF, 0x1000);
        flash_dirty = true;
        return;
    }
}

static void m111_flash_chip_erase(void) {
    for (size_t i = 0; i < C.prg_sz; ++i) {
        if (C.prg[i] == 0xFF) continue;
        memset(C.prg, 0xFF, C.prg_sz);
        flash_dirty = true;
        return;
    }
}

static void m111_flash_write(size_t physical, uint8_t value) {
    unsigned command_addr = (unsigned)(physical & 0x7FFFu);
    if (m111.flash_mode == FLASH_PROGRAM) {
        m111_flash_program(physical, value);
        m111_flash_reset_command();
        return;
    }
    if (m111.flash_mode == FLASH_ERASE) {
        if (m111.flash_cycle == 3 && command_addr == 0x5555 && value == 0xAA) {
            m111.flash_cycle = 4;
            return;
        }
        if (m111.flash_cycle == 4 && command_addr == 0x2AAA && value == 0x55) {
            m111.flash_cycle = 5;
            return;
        }
        if (m111.flash_cycle == 5) {
            if (command_addr == 0x5555 && value == 0x10) m111_flash_chip_erase();
            else if (value == 0x30) m111_flash_erase_sector(physical);
        }
        m111_flash_reset_command();
        return;
    }

    if (m111.flash_cycle == 0) {
        if (command_addr == 0x5555 && value == 0xAA) m111.flash_cycle = 1;
        else if (value == 0xF0) {
            m111_flash_reset_command();
            m111.software_id = false;
        }
        return;
    }
    if (m111.flash_cycle == 1 && command_addr == 0x2AAA && value == 0x55) {
        m111.flash_cycle = 2;
        return;
    }
    if (m111.flash_cycle == 2 && command_addr == 0x5555) {
        m111.flash_cycle = 3;
        switch (value) {
            case 0x80:
                m111.flash_mode = FLASH_ERASE;
                return;
            case 0x90:
                m111_flash_reset_command();
                m111.software_id = true;
                return;
            case 0xA0:
                m111.flash_mode = FLASH_PROGRAM;
                return;
            case 0xF0:
                m111_flash_reset_command();
                m111.software_id = false;
                return;
            default:
                return;
        }
    }
    m111.flash_cycle = 0;
}

static void m111_latch(uint8_t value) {
    m111.bank_latch = value;
}

static bool m111_is_register(uint16_t addr) {
    return (addr >= 0x5000 && addr <= 0x5FFF)
        || (addr >= 0x7000 && addr <= 0x7FFF);
}

static uint8_t m111_cpu_read(uint16_t addr) {
    if (m111_is_register(addr)) {
        m111_latch(cart_cpu_bus_input);
        return prg_ram_internal_read(addr);
    }
    if (addr >= 0x6000u && addr < 0x7000u) return prg_ram_read(addr);
    if (addr < 0x8000) return cart_cpu_bus_input;

    int flash_value = m111_flash_read(addr);
    if (flash_value >= 0) return (uint8_t)flash_value;
    if (C.prg_sz < PRG_BANK_32K)
        return repeated_prg_window_read(addr, 0x8000, PRG_BANK_32K);
    size_t banks = C.prg_sz / PRG_BANK_32K;
    size_t bank = (m111.bank_latch & 0x0Fu) % banks;
    size_t physical = bank * PRG_BANK_32K + (addr & 0x7FFFu);
    return C.prg[physical];
}

static void m111_cpu_write(uint16_t addr, uint8_t value) {
    if (m111_is_register(addr)) {
        m111_latch(value);
        return;
    }
    if (addr >= 0x6000u && addr < 0x7000u) {
        prg_ram_write(addr, value);
        return;
    }
    if (addr < 0x8000) return;
    size_t bank = m111.bank_latch & 0x0Fu;
    size_t physical = bank * PRG_BANK_32K + (addr & 0x7FFFu);
    m111_flash_write(physical, value);
}

static uint8_t m111_ppu_read(uint16_t addr) {
    addr &= 0x1FFFu;
    if (!C.chr_sz) return (uint8_t)addr;
    return discrete_chr8_read(addr, (m111.bank_latch >> 4) & 1u);
}

static void m111_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_sz) return;
    discrete_chr8_write(addr, (m111.bank_latch >> 4) & 1u, value);
}

static Mirroring m111_mirr(void) { return MIRROR_FOUR; }

static void m111_reset(void) {
    m111.bank_latch = 0;
    m111.software_id = false;
    m111_flash_reset_command();
}

static void jaleco18_reload_irq(void) {
    jaleco18.irq_counter = (uint16_t)(jaleco18.irq_reload[0]
        | ((uint16_t)jaleco18.irq_reload[1] << 4)
        | ((uint16_t)jaleco18.irq_reload[2] << 8)
        | ((uint16_t)jaleco18.irq_reload[3] << 12));
}

static void jaleco18_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a < 0x8000) return;

    uint16_t reg = a & 0xF003u;
    bool upper = (a & 1u) != 0;
    v &= 0x0F;
    if (reg <= 0x8003 || reg == 0x9000 || reg == 0x9001) {
        unsigned slot = reg >= 0x9000 ? 2 : (reg >> 1) & 1;
        jaleco18_update_nibble(&jaleco18.prg_banks[slot], v, upper);
        jaleco18.prg_mapped |= (uint8_t)(1u << slot);
        return;
    }
    if (reg >= 0xA000 && reg <= 0xD003) {
        unsigned slot = ((reg >> 12) - 0xA) * 2 + ((reg >> 1) & 1);
        jaleco18_update_nibble(&jaleco18.chr_banks[slot], v, upper);
        jaleco18.chr_mapped |= (uint8_t)(1u << slot);
        return;
    }
    switch (reg) {
        case 0xE000: case 0xE001: case 0xE002: case 0xE003:
            jaleco18.irq_reload[reg & 3u] = v;
            break;
        case 0xF000:
            mapper_irq_line = false;
            jaleco18_reload_irq();
            break;
        case 0xF001:
            mapper_irq_line = false;
            jaleco18.irq_enabled = (v & 0x01u) != 0;
            if (v & 0x08u) jaleco18.irq_counter_size = 3;
            else if (v & 0x04u) jaleco18.irq_counter_size = 2;
            else if (v & 0x02u) jaleco18.irq_counter_size = 1;
            else jaleco18.irq_counter_size = 0;
            break;
        case 0xF002:
            jaleco18.mirr = (Mirroring)(v & 3u);
            break;
        case 0xF003:
            break;
    }
}

static uint8_t jaleco18_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(jaleco18.chr_mapped & (1u << slot)))
        return chr_default_read(a, CHR_BANK_1K);
    return C.chr[shrunk_chr_bank_offset(a, page_size, page_count, jaleco18.chr_banks[slot])];
}

static void jaleco18_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    unsigned slot;
    size_t page_size, page_count;
    if (!shrunk_chr_slot_geometry(a, CHR_BANK_1K, 8, &slot, &page_size, &page_count)
        || !(jaleco18.chr_mapped & (1u << slot))) {
        chr_default_write(a, CHR_BANK_1K, v);
        return;
    }
    chr_ram_write(shrunk_chr_bank_offset(a, page_size, page_count, jaleco18.chr_banks[slot]), v);
}

static void jaleco18_clock(int cpu_cycles) {
    while (cpu_cycles-- > 0 && jaleco18.irq_enabled) {
        uint16_t mask = jaleco18_irq_mask[jaleco18.irq_counter_size];
        uint16_t counter = (uint16_t)(jaleco18.irq_counter & mask);
        counter--;
        if (counter == 0) mapper_irq_line = true;
        jaleco18.irq_counter = (uint16_t)((jaleco18.irq_counter & (uint16_t)~mask) | (counter & mask));
    }
}

static Mirroring jaleco18_mirr(void) { return jaleco18.mirr; }

static void jaleco18_reset(void) {
    memset(&jaleco18, 0, sizeof(jaleco18));
    jaleco18.mirr = C.mirr_base;
    mapper_irq_line = false;
}

#endif // MAPPER_DISCRETE_H

