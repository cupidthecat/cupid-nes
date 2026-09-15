/*
 * mapper.c - NES cartridge mapper implementations
 * 
 * Author: @frankischilling
 * 
 * This file implements various NES cartridge mappers (0-15) including NROM, MMC1, UxROM,
 * CNROM, MMC3, MMC5, AxROM, MMC2, MMC4, Color Dreams, CPROM, and others. Each mapper
 * handles bank switching, PRG-ROM/CHR-ROM mapping, and mirroring control according to
 * the specific hardware behavior.
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "mapper.h"

extern uint64_t cpu_total_cycles;
extern uint64_t cpu_get_bus_cycle(void);

#define PRG_BANK_8K  0x2000
#define PRG_BANK_16K 0x4000
#define PRG_BANK_32K 0x8000
#define CHR_BANK_1K  0x0400
#define CHR_BANK_2K  0x0800
#define CHR_BANK_4K  0x1000
#define CHR_BANK_8K  0x2000

// ----------------- Cart wiring/state -----------------
typedef struct {
    uint8_t *prg; size_t prg_sz;
    uint8_t *chr; size_t chr_sz;
    bool chr_is_ram;
    uint8_t submapper;
    bool bus_conflicts;
    bool nes2;
    RomRamSizes ram;
    Mirroring mirr_base;
} CartCommon;

static CartCommon C;
static Mapper mapper_nrom, mapper_mmc1, mapper_uxrom, mapper_cnrom, mapper_mmc3;
static Mapper mapper_mmc5, mapper_aorom, mapper_mmc2, mapper_mmc4, mapper_colordreams;
static Mapper mapper_cprom, mapper_100in1;
Mapper *cart = NULL;
static bool mapper_irq_line = false;
static uint8_t cart_cpu_bus_input = 0xFF;
static CartPpuFetchSource cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
static void mmc3_irq_clock(void);
static void mmc5_scanline_clock(void);
static void mmc5_vblank_start(void);

#ifdef PPU_DEBUG_LOG
static uint32_t mmc3_log_count = 0;
static const uint32_t mmc3_log_limit = 3000;
#define MMC3_LOG(fmt, ...) do { \
    if (mmc3_log_count < mmc3_log_limit) { \
        fprintf(stderr, "[MMC3 cpu=%llu] " fmt "\n", \
                (unsigned long long)cpu_total_cycles, ##__VA_ARGS__); \
        mmc3_log_count++; \
    } \
} while (0)
#else
#define MMC3_LOG(...) do {} while (0)
#endif

typedef struct {
    uint8_t *data;
    size_t size;
} RamBlock;

static RamBlock prg_work_ram, prg_save_ram;
static bool prg_ram_dirty = false;
static bool chr_ram_dirty = false;
static bool battery_enabled = false;
static char *battery_save_path = NULL;
static char *chr_save_path = NULL;

void cart_apply_trainer(const uint8_t trainer[512]) {
    if (!trainer) return;
    RamBlock *ram = prg_work_ram.size >= 0x2000 ? &prg_work_ram : &prg_save_ram;
    if (ram->size >= 0x2000) {
        memcpy(ram->data + 0x1000, trainer, 512);
        if (ram == &prg_save_ram && battery_enabled) prg_ram_dirty = true;
    }
}

static inline uint16_t base_nt_index(uint16_t addr) {
    uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
    uint16_t nt  = (uint16_t)((off >> 10) & 3u);
    uint16_t in  = (uint16_t)(off & 0x03FFu);

    switch (cart_get_mirroring()) {
        case MIRROR_HORIZONTAL: return (uint16_t)(((nt & 2u) ? 0x400u : 0x000u) + in);
        case MIRROR_VERTICAL:   return (uint16_t)(((nt & 1u) ? 0x400u : 0x000u) + in);
        case MIRROR_SINGLE0:    return in;
        case MIRROR_SINGLE1:    return (uint16_t)(0x400u + in);
        case MIRROR_FOUR:       return (uint16_t)(nt * 0x400u + in);
        default:                return in;
    }
}

static uint8_t ram_read(const RamBlock *ram, size_t offset) {
    return ram && ram->size ? ram->data[offset % ram->size] : cart_cpu_bus_input;
}

static void ram_write(RamBlock *ram, size_t offset, uint8_t value) {
    if (!ram || !ram->size) return;
    size_t index = offset % ram->size;
    if (ram->data[index] == value) return;
    ram->data[index] = value;
    if (ram == &prg_save_ram && battery_enabled) prg_ram_dirty = true;
}

static RamBlock *default_prg_ram(void) {
    return prg_save_ram.size ? &prg_save_ram : &prg_work_ram;
}

static inline uint8_t prg_ram_read(uint16_t addr) {
    return ram_read(default_prg_ram(), addr - 0x6000u);
}

static inline void prg_ram_write(uint16_t addr, uint8_t value) {
    ram_write(default_prg_ram(), addr - 0x6000u, value);
}

static void chr_ram_write(size_t index, uint8_t value) {
    if (!C.chr_is_ram || index >= C.chr_sz || C.chr[index] == value) return;
    C.chr[index] = value;
    if (index < C.ram.chr_nvram && battery_enabled) chr_ram_dirty = true;
}

static char *build_save_path(const char *rom_path, const char *suffix) {
    const char *last_slash = strrchr(rom_path, '/');
    const char *last_backslash = strrchr(rom_path, '\\');
    const char *sep = last_slash;
    if (last_backslash && (!sep || last_backslash > sep)) sep = last_backslash;

    const char *last_dot = strrchr(rom_path, '.');
    if (last_dot && sep && last_dot < sep) last_dot = NULL;

    size_t stem_len = last_dot ? (size_t)(last_dot - rom_path) : strlen(rom_path);
    size_t suffix_len = strlen(suffix) + 1;
    if (stem_len > SIZE_MAX - suffix_len) return NULL;
    char *save_path = (char*)malloc(stem_len + suffix_len);
    if (!save_path) return NULL;

    memcpy(save_path, rom_path, stem_len);
    memcpy(save_path + stem_len, suffix, suffix_len);
    return save_path;
}

static void flush_battery(const char *path, const uint8_t *data, size_t size, bool *dirty) {
    if (!battery_enabled || !path || !size || !*dirty) return;
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        perror("battery save open");
        return;
    }

    size_t written = fwrite(data, 1, size, fp);
    int close_result = fclose(fp);

    if (written != size || close_result != 0) {
        fprintf(stderr, "Failed to write battery save '%s' (%zu/%zu bytes)\n",
                path, written, size);
        return;
    }
    *dirty = false;
}

void cart_battery_flush(void) {
    flush_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size, &prg_ram_dirty);
    flush_battery(chr_save_path, C.chr, C.ram.chr_nvram, &chr_ram_dirty);
}

void cart_battery_shutdown(void) {
    cart_battery_flush();
    free(battery_save_path);
    free(chr_save_path);
    battery_save_path = NULL;
    chr_save_path = NULL;
    battery_enabled = false;
    prg_ram_dirty = false;
    chr_ram_dirty = false;
}

static void load_battery(const char *path, uint8_t *data, size_t size) {
    if (!path || !size) return;
    memset(data, 0, size);
    FILE *fp = fopen(path, "rb");
    if (!fp) return; // first run/no prior save
    (void)fread(data, 1, size, fp); // Short saves leave the remaining bytes zero.
    if (ferror(fp)) fprintf(stderr, "Failed to read battery save '%s'\n", path);
    fclose(fp);
}

void cart_battery_configure(const char *rom_path, bool has_battery) {
    cart_battery_shutdown();
    if (!has_battery || !rom_path) return;
    if (prg_save_ram.size) battery_save_path = build_save_path(rom_path, ".sav");
    if (C.ram.chr_nvram) chr_save_path = build_save_path(rom_path, ".chr.sav");
    if ((prg_save_ram.size && !battery_save_path) || (C.ram.chr_nvram && !chr_save_path)) {
        fprintf(stderr, "Failed to allocate battery save path\n");
        cart_battery_shutdown();
        return;
    }
    battery_enabled = battery_save_path || chr_save_path;
    load_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size);
    load_battery(chr_save_path, C.chr, C.ram.chr_nvram);
}

void mapper_shutdown(void) {
    cart_battery_shutdown();
    free(prg_work_ram.data);
    free(prg_save_ram.data);
    prg_work_ram = (RamBlock){0};
    prg_save_ram = (RamBlock){0};
    cart = NULL;
    memset(&C, 0, sizeof(C));
    mapper_irq_line = false;
    cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
}

// Helpers
static inline Mirroring base_mirr(void) { return C.mirr_base; }
Mirroring cart_get_mirroring(void) { return cart && cart->get_mirroring ? cart->get_mirroring() : base_mirr(); }
void cart_set_mirroring(Mirroring m) { C.mirr_base = m; }
uint8_t cart_cpu_read(uint16_t a) { return cart ? cart->cpu_read(a) : 0xFF; }
uint8_t cart_cpu_read_bus(uint16_t a, uint8_t open_bus) {
    if (!cart) return open_bus;
    // Read paths decide which registers and RAM chips drive the data lines.
    // Scope the input so nested callbacks restore their caller's bus latch.
    uint8_t previous_bus = cart_cpu_bus_input;
    cart_cpu_bus_input = open_bus;
    uint8_t value = cart->cpu_read(a);
    cart_cpu_bus_input = previous_bus;
    if (cart == &mapper_mmc5 && a == 0x5204) value |= open_bus & 0x3F;
    return value;
}
void cart_cpu_write(uint16_t a, uint8_t v) {
    if (!cart) return;
    if (a >= 0x8000 && C.bus_conflicts) v &= cart->cpu_read(a);
    cart->cpu_write(a, v);
}
uint8_t cart_ppu_read(uint16_t a) { return cart ? cart->ppu_read(a) : 0x00; }
void cart_ppu_write(uint16_t a, uint8_t v) { if (cart) cart->ppu_write(a, v); }
void cart_set_ppu_fetch_source(CartPpuFetchSource src) { cart_ppu_fetch_source = src; }
bool cart_irq_pending(void) { return mapper_irq_line; }
void cart_irq_ack(void) {
    mapper_irq_line = false;
}
void cart_notify_scanline(void) {
    // MMC3 clocks from qualified PPU A12 edges, not scanline completion.
}

void cart_notify_scanline_early(void) {
    if (cart == &mapper_mmc5) {
        mmc5_scanline_clock();
    }
}

void cart_notify_vblank_start(void) {
    if (cart == &mapper_mmc5) {
        mmc5_vblank_start();
    }
}

// ----------------- Mapper 0 (NROM) -------------------
static uint8_t nrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        return C.prg[(a - 0x8000) % C.prg_sz];
    }
    return cart_cpu_bus_input;
}
static void nrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) prg_ram_write(a, v);
}
static uint8_t nrom_ppu_read(uint16_t a) { return C.chr[(a & 0x1FFF) % C.chr_sz]; }
static void nrom_ppu_write(uint16_t a, uint8_t v) {
    chr_ram_write((a & 0x1FFF) % C.chr_sz, v);
}
static Mirroring nrom_mirr(void) { return C.mirr_base; }

// ----------------- Mapper 1 (MMC1/SxROM) -----------------
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
        if (mmc1.prg_bank & 0x10) return cart_cpu_bus_input;
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
        return C.prg[(bank * PRG_BANK_16K + (a & 0x3FFF)) % C.prg_sz];
    }
    return cart_cpu_bus_input;
}

static void mmc1_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) {
        if (!(mmc1.prg_bank & 0x10)) {
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

static size_t mmc1_chr_offset(uint16_t a) {
    a &= 0x1FFF;
    size_t bank;
    if (mmc1.control & 0x10) {
        bank = a < 0x1000 ? mmc1.chr_bank0 : mmc1.chr_bank1;
    } else {
        bank = (mmc1.chr_bank0 & 0x1E) + (a >> 12);
    }
    return (bank * CHR_BANK_4K + (a & 0x0FFF)) % C.chr_sz;
}

static uint8_t mmc1_ppu_read(uint16_t a) {
    return C.chr[mmc1_chr_offset(a)];
}

static void mmc1_ppu_write(uint16_t a, uint8_t v) {
    chr_ram_write(mmc1_chr_offset(a), v);
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

// ----------------- Mapper 2 (UxROM) -----------------
static struct { uint8_t bank; } ux;
static uint8_t uxrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000 && a <= 0xBFFF) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t b = (banks == 0) ? 0 : (ux.bank % banks);
        return C.prg[b * PRG_BANK_16K + (a - 0x8000)];
    }
    if (a >= 0xC000) {
        size_t banks = C.prg_sz / PRG_BANK_16K;
        size_t last = (banks ? banks : 1) - 1;
        return C.prg[last * PRG_BANK_16K + (a - 0xC000)];
    }
    return cart_cpu_bus_input;
}
static void uxrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) ux.bank = v;
}
static uint8_t uxrom_ppu_read(uint16_t a) { return nrom_ppu_read(a); }
static void uxrom_ppu_write(uint16_t a, uint8_t v) { nrom_ppu_write(a, v); }
static Mirroring uxrom_mirr(void) { return C.mirr_base; }
static void uxrom_reset(void) { ux.bank = 0; }

// ----------------- Mapper 3 (CNROM) -----------------
static struct { uint8_t chr_bank; } cn;
static uint8_t cnrom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return C.prg[(a - 0x8000) % C.prg_sz];
    return cart_cpu_bus_input;
}
static void cnrom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        size_t banks = C.chr_sz / CHR_BANK_8K;
        if (banks) cn.chr_bank = v % banks;
    }
}
static uint8_t cnrom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t off = (size_t)cn.chr_bank * CHR_BANK_8K + a;
    return C.chr[off % C.chr_sz];
}
static void cnrom_ppu_write(uint16_t a, uint8_t v) {
    if (C.chr_is_ram) {
        size_t off = (size_t)cn.chr_bank * CHR_BANK_8K + (a & 0x1FFF);
        chr_ram_write(off % C.chr_sz, v);
    }
}
static Mirroring cnrom_mirr(void) { return C.mirr_base; }
static void cnrom_reset(void) { cn.chr_bank = 0; }

// ----------------- Mapper 4 (MMC3/TxROM) -----------------
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
} mmc3;

void cart_notify_ppu_address(uint16_t addr, uint64_t ppu_cycle) {
    if (cart != &mapper_mmc3) return;
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
    if (mmc3.irq_counter == 0 || mmc3.irq_reload) {
        mmc3.irq_counter = mmc3.irq_latch;
        mmc3.irq_reload = false;
    } else {
        mmc3.irq_counter--;
    }

    if (mmc3.irq_counter == 0 && mmc3.irq_enabled) {
        mapper_irq_line = true;
        MMC3_LOG("irq assert latch=%02X reload=%d", mmc3.irq_latch, mmc3.irq_reload ? 1 : 0);
    }
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
        return (mmc3.ram_protect & 0x80) ? prg_ram_read(a) : cart_cpu_bus_input;
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
        } else if ((mmc3.ram_protect & 0xC0) == 0x80) {
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

static uint8_t mmc3_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return nrom_ppu_read(a);

    uint8_t slot = (uint8_t)(a >> 10);
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

    bank %= chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc3_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) { nrom_ppu_write(a, v); return; }

    uint8_t slot = (uint8_t)(a >> 10);
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

    bank %= chr_1k_banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FF), v);
}

static Mirroring mmc3_mirr(void) { return mmc3.mirr; }
static void mmc3_reset(void) {
    memset(&mmc3, 0, sizeof(mmc3));
    const uint8_t initial_banks[8] = {0, 2, 4, 5, 6, 7, 0, 1};
    memcpy(mmc3.banks, initial_banks, sizeof(initial_banks));
    mmc3.mirr = C.mirr_base;
    mapper_irq_line = false;
}

// ----------------- Mapper 5 (MMC5/ExROM) -----------------
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
    bool irq_enabled;        // $5204 bit7
    bool irq_pending;
    Mirroring mirr;
    uint8_t exram[0x400];
} mmc5;

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

    // MMC5 supports per-nametable mapping (including EXRAM/fill), but this emulator
    // currently exposes classic mirroring modes. Choose the closest classic mode.
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
    if (cart_ppu_fetch_source == CART_PPU_FETCH_BG) {
        // MMC5 uses $5128-$512B only for background in 8x16 sprite mode.
        use_bg_set = true;
    } else if (cart_ppu_fetch_source == CART_PPU_FETCH_CPU) {
        // PPUDATA CHR reads/writes use the most recently written CHR register set.
        use_bg_set = mmc5.chr_last_set_b;
    }

    uint16_t reg = 0;
    uint8_t mode = (uint8_t)(mmc5.chr_mode & 0x03);
    uint8_t sub = 0;

    if (!use_bg_set) {
        switch (mode) {
            case 0: reg = mmc5.chr_regs_a[7]; sub = slot; break;                         // 8KB
            case 1: reg = (slot < 4) ? mmc5.chr_regs_a[3] : mmc5.chr_regs_a[7]; sub = (uint8_t)(slot & 0x03u); break; // 4KB
            case 2: reg = mmc5.chr_regs_a[1 + ((slot >> 1) * 2)]; sub = (uint8_t)(slot & 0x01u); break;               // 2KB
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

static void mmc5_scanline_clock(void) {
    if (!mmc5.in_frame) {
        mmc5.in_frame = true;
        mmc5.scanline_counter = 0;
    } else {
        mmc5.scanline_counter++;
    }

    // MMC5 sets pending on match regardless of enable; /IRQ is gated by enable.
    if (mmc5.irq_scanline != 0 && mmc5.scanline_counter == mmc5.irq_scanline) {
        mmc5.irq_pending = true;
        if (mmc5.irq_enabled) mapper_irq_line = true;
    }
}

static void mmc5_vblank_start(void) {
    mmc5.in_frame = false;
    mmc5.scanline_counter = 0;
    mmc5.irq_pending = false;
    mapper_irq_line = false;
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
    if (C.nes2 && (ram->size == 0x10000 || ram->size == 0x20000)) {
        bank &= 0x0F; // NES 2.0 can describe a single 64KB/128KB chip.
    } else {
        bank &= 7;
        if (C.nes2) {
            if (C.ram.prg_ram == 0x2000 && C.ram.prg_nvram == 0x2000) {
                ram = (bank & 4) ? &prg_work_ram : &prg_save_ram;
            } else if (bank >= 4) {
                return NULL; // The second RAM socket is empty.
            }
        }
    }
    *offset = bank * PRG_BANK_8K + (a & 0x1FFF);
    return ram;
}

static uint8_t mmc5_cpu_read(uint16_t a) {
    if (a == 0xFFFA || a == 0xFFFB) mmc5_vblank_start();
    if (a >= 0x5C00 && a <= 0x5FFF) {
        uint8_t mode = (uint8_t)(mmc5.exram_mode & 0x03);
        if (mode == 0 || mode == 1) return cart_cpu_bus_input;
        return mmc5.exram[a - 0x5C00u];
    }
    if (a == 0x5204) {
        uint8_t status = 0;
        if (mmc5.irq_pending) status |= 0x80;
        if (mmc5.in_frame) status |= 0x40;
        mmc5.irq_pending = false;
        mapper_irq_line = false;
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

    if (mmc5_ram_mapped(a)) {
        size_t offset = 0;
        RamBlock *ram = mmc5_ram_location(a, &offset);
        return ram_read(ram, offset);
    }
    if (a >= 0x8000) {
        size_t banks = mmc5_prg_bank_count_8k();
        if (banks == 0) return cart_cpu_bus_input;

        size_t slot = (size_t)((a - 0x8000u) >> 13); // 0..3
        size_t bank = mmc5_map_prg_slot_to_bank(slot);
        return C.prg[bank * PRG_BANK_8K + (a & 0x1FFF)];
    }
    return cart_cpu_bus_input;
}

static void mmc5_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x5C00 && a <= 0x5FFF) {
        uint8_t mode = (uint8_t)(mmc5.exram_mode & 0x03);
        if (mode != 3) mmc5.exram[a - 0x5C00u] = (mode <= 1 && !mmc5.in_frame) ? 0 : v;
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
        mmc5.chr_last_set_b = true;
    } else if (a == 0x5130) {
        mmc5.chr_upper = v & 0x03;
    } else if (a == 0x5203) {
        mmc5.irq_scanline = v;
    } else if (a == 0x5204) {
        mmc5.irq_enabled = (v & 0x80) != 0;
        // Writing only toggles enable; pending state is retained.
        mapper_irq_line = (mmc5.irq_enabled && mmc5.irq_pending);
    } else if (a == 0x5205) {
        mmc5.mul_a = v;
    } else if (a == 0x5206) {
        mmc5.mul_b = v;
    }
}

static uint8_t mmc5_ppu_read(uint16_t a) {
    a &= 0x1FFF;
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
    mmc5.mirr = C.mirr_base;
    mmc5.prg_mode = 3;
    mmc5.chr_mode = 3;
    mmc5.irq_enabled = false;
    mmc5.irq_pending = false;
    mmc5.in_frame = false;
    mmc5.scanline_counter = 0;
    mmc5.exram_mode = 0;
    mmc5.prg_ram_protect1 = 0;
    mmc5.prg_ram_protect2 = 0;
    mmc5.fill_tile = 0;
    mmc5.fill_attr = 0;
    mmc5.chr_last_set_b = false;

    mmc5.prg_regs[3] = 0xFF; // Reset vectors are in the last ROM bank.

    for (int i = 0; i < 8; ++i) mmc5.chr_regs_a[i] = (uint16_t)i;
    for (int i = 0; i < 4; ++i) mmc5.chr_regs_b[i] = (uint16_t)i;
}

uint8_t cart_nt_read(uint16_t addr, uint8_t *nt_ram) {
    if (cart == &mapper_mmc5) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint16_t in  = (uint16_t)(off & 0x03FFu);
        uint8_t src = mmc5_nt_source(addr);

        switch (src) {
            case 0: return nt_ram[in];
            case 1: return nt_ram[0x400u + in];
            case 2:
                if ((mmc5.exram_mode & 0x03) >= 2) return 0x00;
                return mmc5.exram[in];
            case 3:
                if (in < 0x03C0u) return mmc5.fill_tile;
                return mmc5_fill_attr_byte();
            default:
                return nt_ram[in];
        }
    }

    return nt_ram[base_nt_index(addr)];
}

void cart_nt_write(uint16_t addr, uint8_t v, uint8_t *nt_ram) {
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
                mmc5.exram[in] = v;
                return;
            case 3:
                // Fill mode is controlled by $5106/$5107.
                return;
            default:
                nt_ram[in] = v;
                return;
        }
    }

    nt_ram[base_nt_index(addr)] = v;
}

// ----------------- Mapper 7 (AOROM) -----------------
static struct { uint8_t prg_bank; Mirroring mirr; } ao;
static uint8_t aorom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t b = (banks ? ao.prg_bank % banks : 0);
        return C.prg[(b * PRG_BANK_32K + (a - 0x8000)) % C.prg_sz];
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

// ----------------- Mapper 9 (MMC2/PxROM) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    Mirroring mirr;
} mmc2;

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
    else if (a >= 0xB000 && a <= 0xBFFF) mmc2.chr_banks[0] = v & 0x1F;
    else if (a >= 0xC000 && a <= 0xCFFF) mmc2.chr_banks[1] = v & 0x1F;
    else if (a >= 0xD000 && a <= 0xDFFF) mmc2.chr_banks[2] = v & 0x1F;
    else if (a >= 0xE000 && a <= 0xEFFF) mmc2.chr_banks[3] = v & 0x1F;
    else if (a >= 0xF000) mmc2.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc2_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) return nrom_ppu_read(a);
    
    uint8_t bank_idx = (a < 0x1000) ? mmc2.latch[0] : (2 + mmc2.latch[1]);
    size_t bank = mmc2.chr_banks[bank_idx] % chr_4k_banks;
    uint8_t val = C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)];
    
    // Latch update
    if (a == 0x0FD8) mmc2.latch[0] = 0;
    else if (a == 0x0FE8) mmc2.latch[0] = 1;
    else if (a >= 0x1FD8 && a <= 0x1FDF) mmc2.latch[1] = 0;
    else if (a >= 0x1FE8 && a <= 0x1FEF) mmc2.latch[1] = 1;
    
    return val;
}

static void mmc2_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) { nrom_ppu_write(a, v); return; }
    uint8_t bank_idx = (a < 0x1000) ? mmc2.latch[0] : (2 + mmc2.latch[1]);
    size_t bank = mmc2.chr_banks[bank_idx] % chr_4k_banks;
    chr_ram_write(bank * CHR_BANK_4K + (a & 0x0FFF), v);
}

static Mirroring mmc2_mirr(void) { return mmc2.mirr; }
static void mmc2_reset(void) {
    memset(&mmc2, 0, sizeof(mmc2));
    mmc2.latch[0] = mmc2.latch[1] = 1;
    mmc2.mirr = C.mirr_base;
}

// ----------------- Mapper 10 (MMC4/FxROM) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_banks[4];
    uint8_t latch[2];
    Mirroring mirr;
} mmc4;

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
    else if (a >= 0xB000 && a <= 0xBFFF) mmc4.chr_banks[0] = v & 0x1F;
    else if (a >= 0xC000 && a <= 0xCFFF) mmc4.chr_banks[1] = v & 0x1F;
    else if (a >= 0xD000 && a <= 0xDFFF) mmc4.chr_banks[2] = v & 0x1F;
    else if (a >= 0xE000 && a <= 0xEFFF) mmc4.chr_banks[3] = v & 0x1F;
    else if (a >= 0xF000) mmc4.mirr = (v & 1) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL;
}

static uint8_t mmc4_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) return nrom_ppu_read(a);
    
    uint8_t bank_idx = (a < 0x1000) ? mmc4.latch[0] : (2 + mmc4.latch[1]);
    size_t bank = mmc4.chr_banks[bank_idx] % chr_4k_banks;
    uint8_t val = C.chr[bank * CHR_BANK_4K + (a & 0x0FFF)];
    
    // MMC4 decodes all eight addresses in both pattern-table latch ranges.
    if (a >= 0x0FD8 && a <= 0x0FDF) mmc4.latch[0] = 0;
    else if (a >= 0x0FE8 && a <= 0x0FEF) mmc4.latch[0] = 1;
    else if (a >= 0x1FD8 && a <= 0x1FDF) mmc4.latch[1] = 0;
    else if (a >= 0x1FE8 && a <= 0x1FEF) mmc4.latch[1] = 1;
    
    return val;
}

static void mmc4_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_4k_banks = C.chr_sz / CHR_BANK_4K;
    if (chr_4k_banks == 0) { nrom_ppu_write(a, v); return; }
    uint8_t bank_idx = (a < 0x1000) ? mmc4.latch[0] : (2 + mmc4.latch[1]);
    size_t bank = mmc4.chr_banks[bank_idx] % chr_4k_banks;
    chr_ram_write(bank * CHR_BANK_4K + (a & 0x0FFF), v);
}

static Mirroring mmc4_mirr(void) { return mmc4.mirr; }
static void mmc4_reset(void) {
    memset(&mmc4, 0, sizeof(mmc4));
    mmc4.latch[0] = mmc4.latch[1] = 1;
    mmc4.mirr = C.mirr_base;
}

// ----------------- Mapper 11 (Color Dreams) -----------------
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
} colordreams;

static uint8_t colordreams_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t banks = C.prg_sz / PRG_BANK_32K;
        size_t bank = (banks > 0) ? (colordreams.prg_bank % banks) : 0;
        return C.prg[(bank * PRG_BANK_32K + (a - 0x8000)) % C.prg_sz];
    }
    return cart_cpu_bus_input;
}

static void colordreams_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) {
        colordreams.prg_bank = v & 0x03;
        colordreams.chr_bank = (v >> 4) & 0x0F;
    }
}

static uint8_t colordreams_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (banks == 0) return nrom_ppu_read(a);
    size_t bank = colordreams.chr_bank % banks;
    return C.chr[bank * CHR_BANK_8K + a];
}

static void colordreams_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (banks == 0) { nrom_ppu_write(a, v); return; }
    size_t bank = colordreams.chr_bank % banks;
    chr_ram_write(bank * CHR_BANK_8K + a, v);
}

static Mirroring colordreams_mirr(void) { return C.mirr_base; }
static void colordreams_reset(void) {
    colordreams.prg_bank = 0;
    colordreams.chr_bank = 0;
}

// ----------------- Mapper 13 (CPROM) -----------------
static struct {
    uint8_t chr_bank;
} cprom;

static uint8_t cprom_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) return C.prg[(a - 0x8000) % C.prg_sz];
    return cart_cpu_bus_input;
}

static void cprom_cpu_write(uint16_t a, uint8_t v) {
    if (a >= 0x6000 && a <= 0x7FFF) { prg_ram_write(a, v); return; }
    if (a >= 0x8000) cprom.chr_bank = v & 0x03;
}

static uint8_t cprom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    if (a < 0x1000) {
        // Fixed first 4KB
        return C.chr[a % C.chr_sz];
    } else {
        // Switchable second 4KB
        size_t off = (size_t)cprom.chr_bank * CHR_BANK_4K + (a - 0x1000);
        return C.chr[off % C.chr_sz];
    }
}

static void cprom_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    if (a < 0x1000) {
        chr_ram_write(a % C.chr_sz, v);
    } else {
        size_t off = (size_t)cprom.chr_bank * CHR_BANK_4K + (a - 0x1000);
        chr_ram_write(off % C.chr_sz, v);
    }
}

static Mirroring cprom_mirr(void) { return MIRROR_VERTICAL; }
static void cprom_reset(void) { cprom.chr_bank = 0; }

// ----------------- Mapper 15 (100-in-1 Contra Function 16) -----------------
static struct {
    uint16_t prg_banks[4];
    uint8_t mode;
    Mirroring mirr;
} m15;

static uint8_t m15_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        size_t bank = m15.prg_banks[(a - 0x8000) >> 13];
        return C.prg[(bank * PRG_BANK_8K + (a & 0x1FFF)) % C.prg_sz];
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

// ----------------- Init/Factory ----------------------
static bool ram_geometry_supported(int mapper_no, bool nes2, const RomRamSizes *ram,
                                   bool chr_is_ram, size_t chr_sz) {
    size_t prg_total = ram->prg_ram + ram->prg_nvram;
    size_t chr_total = ram->chr_ram + ram->chr_nvram;
    if (prg_total && (prg_total & (prg_total - 1))) return false;

    // Only these boards have implemented selection between separate PRG RAM chips.
    bool split_prg = ram->prg_ram && ram->prg_nvram;
    if (split_prg && !((mapper_no == 1 || mapper_no == 5)
        && ram->prg_ram == 0x2000 && ram->prg_nvram == 0x2000)) return false;
    if (mapper_no == 1) {
        if (prg_total > 0x8000) return false;
    } else if (mapper_no == 5) {
        if (nes2) {
            if (!split_prg && prg_total > 0x2000 && prg_total != 0x8000
                && prg_total != 0x10000 && prg_total != 0x20000) return false;
        } else if (prg_total > 0x10000) {
            return false; // Legacy bank registers address at most eight 8KB pages.
        }
    } else if (prg_total > 0x2000) {
        return false;
    }

    // No supported board currently selects separate CHR ROM/RAM or two RAM chips.
    if ((!chr_is_ram && chr_total) || (ram->chr_ram && ram->chr_nvram)) return false;
    if (nes2 && chr_is_ram && chr_total != chr_sz) return false;
    size_t chr_limit;
    switch (mapper_no) {
        case 1: case 9: case 10: case 11: chr_limit = 0x20000; break;
        case 3: chr_limit = 0x200000; break;
        case 4: chr_limit = 0x40000; break;
        case 5: chr_limit = 0x800000; break;
        case 13: chr_limit = 0x4000; break;
        default: chr_limit = 0x2000; break;
    }
    return chr_total <= chr_limit;
}

static void build_mapper(Mapper *m,
    uint8_t(*cr)(uint16_t), void(*cw)(uint16_t,uint8_t),
    uint8_t(*pr)(uint16_t), void(*pw)(uint16_t,uint8_t),
    void(*rst)(void), Mirroring(*gm)(void))
{
    m->cpu_read = cr; m->cpu_write = cw;
    m->ppu_read = pr; m->ppu_write = pw;
    m->reset = rst; m->clock = NULL;
    m->get_mirroring = gm;
}

int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz)
{
    if (!h || !prg || prg_sz < PRG_BANK_16K || !chr || !chr_sz) return -1;
    int mapper_no = rom_mapper_number(h);
    bool nes2 = (h->flags7 & 0x0C) == 0x08;
    uint8_t submapper = nes2 ? h->prg_ram_size >> 4 : 0;
    switch (mapper_no) {
        case 0: case 1: case 2: case 3: case 4: case 5:
        case 7: case 9: case 10: case 11: case 13: case 15:
            break;
        default:
            fprintf(stderr, "Unsupported mapper: %d\n", mapper_no);
            return -1;
    }
    if (submapper && !((mapper_no == 1 && submapper == 5)
        || (mapper_no == 4 && submapper == 1)
        || ((mapper_no == 2 || mapper_no == 3 || mapper_no == 7) && submapper <= 2))) {
        fprintf(stderr, "Unsupported mapper/submapper: %d/%u\n", mapper_no, submapper);
        return -1;
    }
    RomRamSizes ram;
    rom_ram_sizes(h, &ram);
    bool chr_is_ram = h->chr_rom_chunks == 0 && (!nes2 || (h->flags9 & 0xF0) == 0);
    if (!ram_geometry_supported(mapper_no, nes2, &ram, chr_is_ram, chr_sz)
        || (mapper_no == 4 && submapper == 1 && ram.prg_ram + ram.prg_nvram != 0x400)) {
        fprintf(stderr, "Unsupported RAM layout for mapper %d (PRG %zu+%zu, CHR %zu+%zu)\n",
                mapper_no, ram.prg_ram, ram.prg_nvram, ram.chr_ram, ram.chr_nvram);
        return -1;
    }
    if ((ram.prg_nvram || ram.chr_nvram) && !(h->flags6 & 2)) {
        fprintf(stderr, "Nonvolatile RAM declared without the battery flag\n");
        return -1;
    }
    RamBlock new_work = {NULL, ram.prg_ram};
    RamBlock new_save = {NULL, ram.prg_nvram};
    if (new_work.size) new_work.data = (uint8_t *)calloc(1, new_work.size);
    if (new_save.size) new_save.data = (uint8_t *)calloc(1, new_save.size);
    if ((new_work.size && !new_work.data) || (new_save.size && !new_save.data)) {
        free(new_work.data);
        free(new_save.data);
        fprintf(stderr, "Cartridge RAM allocation failed\n");
        return -1;
    }

    // Finish all fallible setup before releasing the previous cartridge's RAM.
    mapper_shutdown();
    prg_work_ram = new_work;
    prg_save_ram = new_save;
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz;
    C.chr_is_ram = chr_is_ram;
    C.ram = ram;
    C.nes2 = nes2;
    C.submapper = submapper;
    C.bus_conflicts = submapper == 2 && (mapper_no == 2 || mapper_no == 3 || mapper_no == 7);
    
    // iNES flags6:
    // bit 0 = 1 -> VERTICAL mirroring, 0 -> HORIZONTAL mirroring
    // bit 3 = 1 -> four-screen (overrides bit 0)
    Mirroring mir;
    if (h->flags6 & 0x08) {
        mir = MIRROR_FOUR;
    } else {
        mir = (h->flags6 & 0x01) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
    }
    cart_set_mirroring(mir);

    switch(mapper_no) {
        case 0:
            build_mapper(&mapper_nrom, nrom_cpu_read, nrom_cpu_write,
                        nrom_ppu_read, nrom_ppu_write, NULL, nrom_mirr);
            cart = &mapper_nrom;
            break;
        case 1:
            build_mapper(&mapper_mmc1, mmc1_cpu_read, mmc1_cpu_write,
                        mmc1_ppu_read, mmc1_ppu_write, mmc1_reset, mmc1_mirr);
            cart = &mapper_mmc1;
            break;
        case 2:
            build_mapper(&mapper_uxrom, uxrom_cpu_read, uxrom_cpu_write,
                        uxrom_ppu_read, uxrom_ppu_write, uxrom_reset, uxrom_mirr);
            cart = &mapper_uxrom;
            break;
        case 3:
            build_mapper(&mapper_cnrom, cnrom_cpu_read, cnrom_cpu_write,
                        cnrom_ppu_read, cnrom_ppu_write, cnrom_reset, cnrom_mirr);
            cart = &mapper_cnrom;
            break;
        case 4:
            build_mapper(&mapper_mmc3, mmc3_cpu_read, mmc3_cpu_write,
                        mmc3_ppu_read, mmc3_ppu_write, mmc3_reset, mmc3_mirr);
            cart = &mapper_mmc3;
            break;
        case 5:
            build_mapper(&mapper_mmc5, mmc5_cpu_read, mmc5_cpu_write,
                        mmc5_ppu_read, mmc5_ppu_write, mmc5_reset, mmc5_mirr);
            cart = &mapper_mmc5;
            break;
        case 7:
            build_mapper(&mapper_aorom, aorom_cpu_read, aorom_cpu_write,
                        aorom_ppu_read, aorom_ppu_write, aorom_reset, aorom_mirr);
            cart = &mapper_aorom;
            break;
        case 9:
            build_mapper(&mapper_mmc2, mmc2_cpu_read, mmc2_cpu_write,
                        mmc2_ppu_read, mmc2_ppu_write, mmc2_reset, mmc2_mirr);
            cart = &mapper_mmc2;
            break;
        case 10:
            build_mapper(&mapper_mmc4, mmc4_cpu_read, mmc4_cpu_write,
                        mmc4_ppu_read, mmc4_ppu_write, mmc4_reset, mmc4_mirr);
            cart = &mapper_mmc4;
            break;
        case 11:
            build_mapper(&mapper_colordreams, colordreams_cpu_read, colordreams_cpu_write,
                        colordreams_ppu_read, colordreams_ppu_write, colordreams_reset, colordreams_mirr);
            cart = &mapper_colordreams;
            break;
        case 13:
            build_mapper(&mapper_cprom, cprom_cpu_read, cprom_cpu_write,
                        cprom_ppu_read, cprom_ppu_write, cprom_reset, cprom_mirr);
            cart = &mapper_cprom;
            break;
        case 15:
            build_mapper(&mapper_100in1, m15_cpu_read, m15_cpu_write,
                        m15_ppu_read, m15_ppu_write, m15_reset, m15_mirr);
            cart = &mapper_100in1;
            break;
    }
    
    if (cart && cart->reset) cart->reset();
    return mapper_no;
}
