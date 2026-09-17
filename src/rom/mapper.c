/*
 * mapper.c - NES cartridge mapper implementations
 *
 * Author: @frankischilling
 *
 * This file implements the supported cartridge mappers, including PRG and CHR banking,
 * mirroring, mapper IRQs, cartridge RAM, bus conflicts, MMC5 expansion features, and
 * mapper specific audio and save behavior.
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
#include <errno.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif
#include "mapper.h"
#include "eeprom.h"
#include "sunsoft5b.h"
#include "../system/timing.h"

extern uint64_t cpu_total_cycles;
extern uint64_t cpu_get_bus_cycle(void);

#define PRG_BANK_8K  0x2000
#define PRG_BANK_16K 0x4000
#define PRG_BANK_32K 0x8000
#define CHR_BANK_1K  0x0400
#define CHR_BANK_2K  0x0800
#define CHR_BANK_4K  0x1000
#define CHR_BANK_8K  0x2000

// Cartridge wiring and shared mapper state.
typedef struct {
    uint8_t *prg; size_t prg_sz;
    uint8_t *chr; size_t chr_sz;
    bool chr_is_ram;
    uint16_t mapper_no;
    uint8_t submapper;
    bool mmc1a;
    bool bus_conflicts;
    bool nes2;
    RomRamSizes ram;
    Mirroring mirr_base;
} CartCommon;

static CartCommon C;
static Mapper mapper_nrom, mapper_mmc1, mapper_uxrom, mapper_cnrom, mapper_mmc3, mapper_tqrom, mapper_txsrom;
static Mapper mapper_mmc5, mapper_aorom, mapper_mmc2, mapper_mmc4, mapper_colordreams;
static Mapper mapper_cprom, mapper_100in1, mapper_bandai, mapper_action53, mapper_unrom512;
static Mapper mapper_taito33, mapper_taito48, mapper_jaleco18, mapper_irem32, mapper_irem65;
static Mapper mapper_rambo1, mapper_rambo158;
static Mapper mapper_vrc24;
static Mapper mapper_sunsoft69;
Mapper *cart = NULL;
static bool mapper_irq_line = false;
static uint8_t cart_cpu_bus_input = 0xFF;
static CartPpuFetchSource cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
static void mmc3_irq_clock(void);

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
static uint8_t mmc5_exram[0x400];
static uint8_t tqrom_chr_ram[CHR_BANK_8K];
static bool mmc5_exram_dirty = false;
static bool battery_enabled = false;
static char *battery_save_path = NULL;
static char *chr_save_path = NULL;
static Eeprom24 bandai_eeprom[2];
static char *eeprom_save_path[2];
static char *flash_save_path = NULL;
static bool flash_dirty = false;
static bool unrom512_four_screen_chr = false;
static RamBlock *mmc5_ram_location(uint16_t a, size_t *offset);
static Sunsoft5B sunsoft5b_audio;

void cart_apply_trainer(const uint8_t trainer[512]) {
    if (!trainer) return;
    if (cart == &mapper_mmc5) {
        size_t offset = 0;
        RamBlock *ram = mmc5_ram_location(0x7000, &offset);
        if (ram && offset <= ram->size && 512 <= ram->size - offset) {
            memcpy(ram->data + offset, trainer, 512);
            if (ram == &prg_save_ram && battery_enabled) prg_ram_dirty = true;
        }
        return;
    }
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

static void flush_mmc5_battery(void) {
    if (!battery_enabled || !battery_save_path || (!prg_ram_dirty && !mmc5_exram_dirty)) return;
    FILE *fp = fopen(battery_save_path, "wb");
    if (!fp) {
        perror("battery save open");
        return;
    }
    size_t prg_written = prg_save_ram.size
        ? fwrite(prg_save_ram.data, 1, prg_save_ram.size, fp) : 0;
    size_t exram_written = fwrite(mmc5_exram, 1, sizeof(mmc5_exram), fp);
    int close_result = fclose(fp);
    if (prg_written != prg_save_ram.size || exram_written != sizeof(mmc5_exram)
        || close_result != 0) {
        fprintf(stderr, "Failed to write battery save '%s'\n", battery_save_path);
        return;
    }
    prg_ram_dirty = false;
    mmc5_exram_dirty = false;
}

static void flush_flash_battery(void) {
    if (!battery_enabled || !flash_save_path || !flash_dirty) return;
    size_t path_size = strlen(flash_save_path);
    if (path_size > SIZE_MAX - 32) return;
    char *temporary = malloc(path_size + 32);
    if (!temporary) return;
    static unsigned serial;
    FILE *file = NULL;
    for (unsigned attempt = 0; attempt < 100; ++attempt) {
        snprintf(temporary, path_size + 32, "%s.tmp-%u", flash_save_path, serial++);
        file = fopen(temporary, "wbx");
        if (file || errno != EEXIST) break;
    }
    if (!file) {
        fprintf(stderr, "Cannot create temporary flash save for '%s'\n", flash_save_path);
        free(temporary);
        return;
    }
    size_t written = fwrite(C.prg, 1, C.prg_sz, file);
    int closed = fclose(file);
    bool replaced = false;
    if (written == C.prg_sz && closed == 0) {
#ifdef _WIN32
        replaced = MoveFileExA(temporary, flash_save_path,
                              MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        replaced = rename(temporary, flash_save_path) == 0;
#endif
    }
    if (replaced) flash_dirty = false;
    else {
        fprintf(stderr, "Cannot replace flash save '%s'; changes remain unsaved\n", flash_save_path);
        remove(temporary);
    }
    free(temporary);
}

void cart_battery_flush(void) {
    if (cart == &mapper_unrom512)
        flush_flash_battery();
    else if (cart == &mapper_mmc5) flush_mmc5_battery();
    else flush_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size, &prg_ram_dirty);
    flush_battery(chr_save_path, C.chr, C.ram.chr_nvram, &chr_ram_dirty);
    for (unsigned i = 0; i < 2; ++i)
        flush_battery(eeprom_save_path[i], bandai_eeprom[i].bytes,
                      bandai_eeprom[i].capacity, &bandai_eeprom[i].dirty);
}

void cart_battery_shutdown(void) {
    cart_battery_flush();
    free(battery_save_path);
    free(chr_save_path);
    free(flash_save_path);
    battery_save_path = NULL;
    chr_save_path = NULL;
    for (unsigned i = 0; i < 2; ++i) {
        free(eeprom_save_path[i]);
        eeprom_save_path[i] = NULL;
        bandai_eeprom[i].dirty = false;
    }
    flash_save_path = NULL;
    battery_enabled = false;
    prg_ram_dirty = false;
    chr_ram_dirty = false;
    mmc5_exram_dirty = false;
    flash_dirty = false;
}

static void load_battery(const char *path, uint8_t *data, size_t size) {
    if (!path || !size) return;
    memset(data, 0, size);
    FILE *fp = fopen(path, "rb");
    if (!fp) return; // first run/no prior save
    size_t bytes_read = fread(data, 1, size, fp); // Short saves leave the remaining bytes zero.
    if (bytes_read < size && ferror(fp))
        fprintf(stderr, "Failed to read battery save '%s' (%zu/%zu bytes)\n", path, bytes_read, size);
    fclose(fp);
}

static void load_mmc5_battery(const char *path) {
    if (prg_save_ram.size) memset(prg_save_ram.data, 0, prg_save_ram.size);
    memset(mmc5_exram, 0, sizeof(mmc5_exram));
    if (!path) return;
    FILE *fp = fopen(path, "rb");
    if (!fp) return;
    size_t prg_read = prg_save_ram.size
        ? fread(prg_save_ram.data, 1, prg_save_ram.size, fp) : 0;
    size_t exram_read = prg_read == prg_save_ram.size
        ? fread(mmc5_exram, 1, sizeof(mmc5_exram), fp) : 0;
    if ((prg_read < prg_save_ram.size || exram_read < sizeof(mmc5_exram)) && ferror(fp))
        fprintf(stderr, "Failed to read battery save '%s' (%zu/%zu bytes)\n",
                path, prg_read + exram_read, prg_save_ram.size + sizeof(mmc5_exram));
    fclose(fp);
}

static void load_flash_battery(const char *path) {
    if (!path || !C.prg || !C.prg_sz) return;
    FILE *fp = fopen(path, "rb");
    if (!fp) return;
    size_t bytes_read = fread(C.prg, 1, C.prg_sz, fp);
    if (bytes_read < C.prg_sz && ferror(fp))
        fprintf(stderr, "Failed to read flash save '%s' (%zu/%zu bytes)\n",
                path, bytes_read, C.prg_sz);
    fclose(fp);
}

void cart_battery_configure(const char *rom_path, bool has_battery) {
    cart_battery_shutdown();
    bool serial_storage = bandai_eeprom[0].capacity || bandai_eeprom[1].capacity;
    if (!rom_path || (!has_battery && !serial_storage)) return;
    if (has_battery && cart == &mapper_unrom512)
        flash_save_path = build_save_path(rom_path, ".flash.sav");
    if (has_battery && prg_save_ram.size) battery_save_path = build_save_path(rom_path, ".sav");
    if (has_battery && C.ram.chr_nvram) chr_save_path = build_save_path(rom_path, ".chr.sav");
    bool paths_valid = (!has_battery || !prg_save_ram.size || battery_save_path)
                    && (!has_battery || !C.ram.chr_nvram || chr_save_path)
                    && (!has_battery || cart != &mapper_unrom512 || flash_save_path);
    for (unsigned i = 0; i < 2; ++i) {
        if (!bandai_eeprom[i].capacity) continue;
        eeprom_save_path[i] = build_save_path(rom_path,
            bandai_eeprom[i].capacity == 128 ? ".eeprom128" : ".eeprom256");
        if (!eeprom_save_path[i]) paths_valid = false;
    }
    if (!paths_valid) {
        fprintf(stderr, "Failed to allocate battery save path\n");
        cart_battery_shutdown();
        return;
    }
    battery_enabled = battery_save_path || chr_save_path || flash_save_path
                    || eeprom_save_path[0] || eeprom_save_path[1];
    if (cart == &mapper_unrom512) load_flash_battery(flash_save_path);
    else if (cart == &mapper_mmc5) load_mmc5_battery(battery_save_path);
    else load_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size);
    load_battery(chr_save_path, C.chr, C.ram.chr_nvram);
    for (unsigned i = 0; i < 2; ++i)
        load_battery(eeprom_save_path[i], bandai_eeprom[i].bytes, bandai_eeprom[i].capacity);
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
    memset(mmc5_exram, 0, sizeof(mmc5_exram));
    memset(tqrom_chr_ram, 0, sizeof(tqrom_chr_ram));
    memset(bandai_eeprom, 0, sizeof(bandai_eeprom));
    mmc5_exram_dirty = false;
    unrom512_four_screen_chr = false;
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
    // Kept for older PPU callers; MMC5 derives scanlines from physical reads.
}

void cart_notify_vblank_start(void) {
    // MMC5 leaves the in-frame state when three CPU clocks pass without a PPU read.
}

// Mapper 0: NROM.
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
        return C.prg[(bank * PRG_BANK_16K + (a & 0x3FFF)) % C.prg_sz];
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

// Mapper 2: UxROM.
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

// Mapper 3: CNROM.
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

void cart_notify_ppu_address(uint16_t addr, uint64_t ppu_cycle) {
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
    if (cart != &mapper_mmc3 && cart != &mapper_tqrom && cart != &mapper_txsrom) return;
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

static size_t mmc3_chr_bank(uint16_t a) {
    a &= 0x1FFF;
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
    return bank;
}

static uint8_t mmc3_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return nrom_ppu_read(a);

    size_t bank = mmc3_chr_bank(a) % chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc3_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) { nrom_ppu_write(a, v); return; }

    size_t bank = mmc3_chr_bank(a) % chr_1k_banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FF), v);
}

static bool tqrom_chr_uses_ram(size_t bank) {
    return bank >= 0x40 && bank <= 0x7F;
}

static size_t tqrom_chr_ram_offset(uint16_t a, size_t bank) {
    size_t ram_bank = (bank - 0x40) % (sizeof(tqrom_chr_ram) / CHR_BANK_1K);
    return ram_bank * CHR_BANK_1K + (a & 0x03FF);
}

static uint8_t tqrom_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t bank = mmc3_chr_bank(a);
    if (tqrom_chr_uses_ram(bank)) return tqrom_chr_ram[tqrom_chr_ram_offset(a, bank)];

    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return 0;
    bank %= chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void tqrom_ppu_write(uint16_t a, uint8_t v) {
    a &= 0x1FFF;
    size_t bank = mmc3_chr_bank(a);
    if (!tqrom_chr_uses_ram(bank)) return;
    tqrom_chr_ram[tqrom_chr_ram_offset(a, bank)] = v;
}

static Mirroring mmc3_mirr(void) { return mmc3.mirr; }
static void mmc3_reset(void) {
    memset(&mmc3, 0, sizeof(mmc3));
    const uint8_t initial_banks[8] = {0, 2, 4, 5, 6, 7, 0, 1};
    memcpy(mmc3.banks, initial_banks, sizeof(initial_banks));
    mmc3.mirr = C.mirr_base;
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
    if (!(state->chr_mapped & (1u << (a >> 10)))) return (uint8_t)a;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) return nrom_ppu_read(a);
    size_t bank = state->chr[a >> 10] % banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void taito_ppu_write(const TaitoBankState *state, uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) { nrom_ppu_write(a, v); return; }
    size_t bank = state->chr[a >> 10] % banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), v);
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

static size_t rambo1_chr_bank(uint16_t a) {
    unsigned physical_slot = (unsigned)((a & 0x1FFFu) >> 10);
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
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) return nrom_ppu_read(a);
    size_t bank = rambo1_chr_bank(a) % banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void rambo1_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) { nrom_ppu_write(a, v); return; }
    size_t bank = rambo1_chr_bank(a) % banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), v);
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
    if (cart == &mapper_mmc5) {
        unsigned pulse = (unsigned)mmc5.pulse[0].output + (unsigned)mmc5.pulse[1].output;
        unsigned raw = pulse * 3u + mmc5.pcm_output;
        // Expansion audio is mixed linearly; the common mixer uses a gain of 14
        // against the native nonlinear mixer's 5000-unit reference scale.
        return -(float)raw * (14.0f / 5000.0f);
    }
    if (cart == &mapper_sunsoft69) return sunsoft5b_output(&sunsoft5b_audio);
    return 0.0f;
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
    return C.chr_sz ? C.chr[offset % C.chr_sz] : 0;
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

uint8_t cart_nt_read(uint16_t addr, uint8_t *nt_ram) {
    if (cart == &mapper_txsrom) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        return nt_ram[(size_t)txsrom_nt[off >> 10] * 0x400u + (off & 0x03FFu)];
    }
    if (cart == &mapper_unrom512 && unrom512_four_screen_chr) {
        size_t offset = 0x6000u + ((addr - 0x2000u) & 0x1FFFu);
        return C.chr[offset];
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

    if (cart == &mapper_rambo158) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        uint8_t quadrant = (uint8_t)((off >> 10) & 3u);
        uint16_t in = off & 0x03FFu;
        return nt_ram[(size_t)(rambo1.nt_map[quadrant] & 1u) * 0x400u + in];
    }

    return nt_ram[base_nt_index(addr)];
}

void cart_nt_write(uint16_t addr, uint8_t v, uint8_t *nt_ram) {
    if (cart == &mapper_txsrom) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        nt_ram[(size_t)txsrom_nt[off >> 10] * 0x400u + (off & 0x03FFu)] = v;
        return;
    }
    if (cart == &mapper_unrom512 && unrom512_four_screen_chr) {
        size_t offset = 0x6000u + ((addr - 0x2000u) & 0x1FFFu);
        chr_ram_write(offset, v);
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

// Mapper 9: MMC2/PxROM.
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

// Mapper 10: MMC4/FxROM.
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

// Mapper 69: Sunsoft FME-7 / 5B.
static struct {
    uint8_t command;
    uint8_t work_ram_value;
    uint8_t prg_bank[3];
    bool prg_mapped[3];
    uint8_t chr_bank[8];
    bool chr_mapped[8];
    uint16_t irq_counter;
    bool irq_enabled;
    bool irq_counter_enabled;
    Mirroring mirr;
} sunsoft69;

static size_t sunsoft69_prg_bank(uint8_t bank) {
    size_t banks = C.prg_sz / PRG_BANK_8K;
    return banks ? (size_t)(bank & 0x3Fu) % banks : 0;
}

static size_t sunsoft69_ram_offset(uint16_t address) {
    return ((size_t)(sunsoft69.work_ram_value & 0x3Fu) * PRG_BANK_8K)
        + (address & (PRG_BANK_8K - 1u));
}

static uint8_t sunsoft69_cpu_read(uint16_t address) {
    if (address >= 0x6000 && address < 0x8000) {
        if (sunsoft69.work_ram_value & 0x40u) {
            if (!(sunsoft69.work_ram_value & 0x80u)) return cart_cpu_bus_input;
            RamBlock *ram = default_prg_ram();
            if (!ram->size) return cart_cpu_bus_input;
            return ram_read(ram, sunsoft69_ram_offset(address));
        }
        size_t bank = sunsoft69_prg_bank(sunsoft69.work_ram_value);
        return C.prg[bank * PRG_BANK_8K + (address & 0x1FFFu)];
    }
    if (address >= 0x8000) {
        unsigned slot = (unsigned)((address - 0x8000u) / PRG_BANK_8K);
        size_t bank;
        if (slot == 3) {
            bank = C.prg_sz / PRG_BANK_8K - 1;
        } else {
            if (!sunsoft69.prg_mapped[slot]) return cart_cpu_bus_input;
            bank = sunsoft69_prg_bank(sunsoft69.prg_bank[slot]);
        }
        return C.prg[bank * PRG_BANK_8K + (address & 0x1FFFu)];
    }
    return cart_cpu_bus_input;
}

static void sunsoft69_apply_command(uint8_t value) {
    switch (sunsoft69.command) {
        case 0: case 1: case 2: case 3:
        case 4: case 5: case 6: case 7:
            sunsoft69.chr_bank[sunsoft69.command] = value;
            sunsoft69.chr_mapped[sunsoft69.command] = true;
            break;
        case 8:
            sunsoft69.work_ram_value = value;
            break;
        case 9: case 10: case 11: {
            unsigned slot = sunsoft69.command - 9u;
            sunsoft69.prg_bank[slot] = value & 0x3Fu;
            sunsoft69.prg_mapped[slot] = true;
            break;
        }
        case 12:
            switch (value & 3u) {
                case 0: sunsoft69.mirr = MIRROR_VERTICAL; break;
                case 1: sunsoft69.mirr = MIRROR_HORIZONTAL; break;
                case 2: sunsoft69.mirr = MIRROR_SINGLE0; break;
                default: sunsoft69.mirr = MIRROR_SINGLE1; break;
            }
            break;
        case 13:
            sunsoft69.irq_enabled = (value & 0x01u) != 0;
            sunsoft69.irq_counter_enabled = (value & 0x80u) != 0;
            mapper_irq_line = false;
            break;
        case 14:
            sunsoft69.irq_counter = (uint16_t)((sunsoft69.irq_counter & 0xFF00u) | value);
            break;
        case 15:
            sunsoft69.irq_counter = (uint16_t)((sunsoft69.irq_counter & 0x00FFu) | ((uint16_t)value << 8));
            break;
    }
}

static void sunsoft69_cpu_write(uint16_t address, uint8_t value) {
    if (address >= 0x6000 && address < 0x8000) {
        if ((sunsoft69.work_ram_value & 0xC0u) == 0xC0u) {
            RamBlock *ram = default_prg_ram();
            if (ram->size) ram_write(ram, sunsoft69_ram_offset(address), value);
        }
        return;
    }
    switch (address & 0xE000u) {
        case 0x8000:
            sunsoft69.command = value & 0x0Fu;
            break;
        case 0xA000:
            sunsoft69_apply_command(value);
            break;
        case 0xC000:
        case 0xE000:
            sunsoft5b_write(&sunsoft5b_audio, address, value);
            break;
    }
}

static size_t sunsoft69_chr_index(uint16_t address) {
    unsigned slot = (address & 0x1FFFu) / CHR_BANK_1K;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = banks ? (size_t)sunsoft69.chr_bank[slot] % banks : 0;
    return bank * CHR_BANK_1K + (address & 0x03FFu);
}

static uint8_t sunsoft69_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    unsigned slot = address / CHR_BANK_1K;
    if (!sunsoft69.chr_mapped[slot])
        return C.chr_is_ram ? C.chr[address % C.chr_sz] : (uint8_t)address;
    return C.chr[sunsoft69_chr_index(address)];
}

static void sunsoft69_ppu_write(uint16_t address, uint8_t value) {
    if (!C.chr_is_ram) return;
    address &= 0x1FFFu;
    unsigned slot = address / CHR_BANK_1K;
    size_t index = sunsoft69.chr_mapped[slot] ? sunsoft69_chr_index(address) : address % C.chr_sz;
    chr_ram_write(index, value);
}

static Mirroring sunsoft69_mirr(void) { return sunsoft69.mirr; }

static void sunsoft69_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        if (sunsoft69.irq_counter_enabled) {
            sunsoft69.irq_counter--;
            if (sunsoft69.irq_counter == 0xFFFFu && sunsoft69.irq_enabled)
                mapper_irq_line = true;
        }
    }
    sunsoft5b_clock(&sunsoft5b_audio, cpu_cycles);
}

static void sunsoft69_reset(void) {
    memset(&sunsoft69, 0, sizeof(sunsoft69));
    sunsoft69.mirr = C.mirr_base;
    if (C.chr_is_ram) {
        for (unsigned slot = 0; slot < 8; ++slot) sunsoft69.chr_bank[slot] = (uint8_t)slot;
    }
    mapper_irq_line = false;
    sunsoft5b_reset(&sunsoft5b_audio);
}

// Mapper 11: Color Dreams.
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

// Mapper 13: CPROM.
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

// Mapper 15: 100-in-1 Contra Function 16.
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
        unsigned page = (address < 0xC000 ? bandai.prg_bank : 15u) | bandai.outer_bank;
        return C.prg[((size_t)page * PRG_BANK_16K + (address & 0x3FFF)) % C.prg_sz];
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
        } else if (!C.chr_is_ram && bandai.mapper != 157) {
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
    unsigned slot = (address >> 10) & 7;
    if (!(bandai.chr_mapped & (1u << slot))) return (uint8_t)address;
    size_t offset = (size_t)bandai.chr_banks[slot] * CHR_BANK_1K + (address & 0x03FF);
    return C.chr[offset % C.chr_sz];
}

static void bandai_ppu_write(uint16_t address, uint8_t value) {
    if (C.chr_is_ram) chr_ram_write(address & 0x1FFF, value);
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
    bandai.chr_mapped = C.chr_is_ram ? 0xFF : 0;
    for (unsigned i = 0; i < 8; ++i) bandai.chr_banks[i] = (uint8_t)i;
    eeprom24_init(&bandai_eeprom[0], standard_eeprom);
    eeprom24_init(&bandai_eeprom[1], extra_eeprom);
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

    if (prg_16k) {
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
}

static uint8_t m28_cpu_read(uint16_t a) {
    if (a >= 0x6000 && a <= 0x7FFF) return prg_ram_read(a);
    if (a >= 0x8000) {
        if (a < 0xC000 && !m28.prg_selected) return cart_cpu_bus_input;
        size_t slot = (a >> 14) & 1u;
        size_t offset = m28.prg_bank[slot] * PRG_BANK_16K + (a & 0x3FFF);
        return C.prg[offset % C.prg_sz];
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
    size_t banks = C.chr_sz / CHR_BANK_8K;
    size_t bank = m28.chr_bank % banks;
    return C.chr[bank * CHR_BANK_8K + a];
}

static void m28_ppu_write(uint16_t a, uint8_t v) {
    a &= 0x1FFF;
    size_t banks = C.chr_sz / CHR_BANK_8K;
    size_t bank = m28.chr_bank % banks;
    chr_ram_write(bank * CHR_BANK_8K + a, v);
}

static Mirroring m28_mirr(void) { return m28.mirr; }

static void m28_power_on(void) {
    memset(&m28, 0, sizeof(m28));
    m28.mirr = C.mirr_base;
    m28.prg_bank[0] = 0;
    m28.prg_bank[1] = C.prg_sz / PRG_BANK_16K - 1;
}

// Mapper 30: UNROM 512.
typedef enum {
    FLASH_WAITING = 0,
    FLASH_PROGRAM,
    FLASH_ERASE
} FlashMode;

static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
    uint8_t flash_cycle;
    FlashMode flash_mode;
    bool software_id;
    bool flash_writable;
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
    size_t chr_banks = C.chr_sz / CHR_BANK_8K;
    m30.prg_bank = value & 0x1F;
    m30.chr_bank = (uint8_t)(((value >> 5) & 0x03) % chr_banks);
    if (m30.mirroring_bit_enabled) {
        if (C.submapper == 3)
            m30.mirr = (value & 0x80) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
        else
            m30.mirr = (value & 0x80) ? MIRROR_SINGLE1 : MIRROR_SINGLE0;
    }
}

static uint8_t m30_cpu_read(uint16_t a) {
    if (a < 0x8000) return cart_cpu_bus_input;
    size_t bank = a < 0xC000 ? m30.prg_bank : C.prg_sz / PRG_BANK_16K - 1;
    size_t physical = bank * PRG_BANK_16K + (a & 0x3FFF);
    if (m30.flash_writable) {
        int flash_value = m30_flash_read(physical);
        if (flash_value >= 0) return (uint8_t)flash_value;
    }
    return C.prg[physical % C.prg_sz];
}

static void m30_cpu_write(uint16_t a, uint8_t value) {
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
    return C.chr[(size_t)m30.chr_bank * CHR_BANK_8K + a];
}

static void m30_ppu_write(uint16_t a, uint8_t value) {
    a &= 0x1FFF;
    chr_ram_write((size_t)m30.chr_bank * CHR_BANK_8K + a, value);
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
                unrom512_four_screen_chr = C.chr_sz >= 0x8000;
                break;
        }
    }
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
    if (!(jaleco18.chr_mapped & (1u << (a >> 10))))
        return C.chr_is_ram ? C.chr[a % C.chr_sz] : (uint8_t)a;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = jaleco18.chr_banks[a >> 10] % banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void jaleco18_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    if (!(jaleco18.chr_mapped & (1u << (a >> 10)))) {
        chr_ram_write(a % C.chr_sz, v);
        return;
    }
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = jaleco18.chr_banks[a >> 10] % banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), v);
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
    if (!(irem32.chr_mapped & (1u << (a >> 10))))
        return C.chr_is_ram ? C.chr[a % C.chr_sz] : (uint8_t)a;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = irem32.chr_banks[a >> 10] % banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void irem32_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    if (!(irem32.chr_mapped & (1u << (a >> 10)))) {
        chr_ram_write(a % C.chr_sz, v);
        return;
    }
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = irem32.chr_banks[a >> 10] % banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), v);
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
    if (!(irem65.chr_mapped & (1u << (a >> 10))))
        return C.chr_is_ram ? C.chr[a % C.chr_sz] : (uint8_t)a;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = irem65.chr_banks[a >> 10] % banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void irem65_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    if (!(irem65.chr_mapped & (1u << (a >> 10)))) {
        chr_ram_write(a % C.chr_sz, v);
        return;
    }
    size_t banks = C.chr_sz / CHR_BANK_1K;
    size_t bank = irem65.chr_banks[a >> 10] % banks;
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), v);
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

typedef struct {
    uint8_t reload;
    uint8_t counter;
    int16_t prescaler;
    bool enabled;
    bool enabled_after_ack;
    bool cycle_mode;
} VrcIrq;

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

static void vrc_irq_reset(VrcIrq *irq) {
    memset(irq, 0, sizeof(*irq));
}

static void vrc_irq_clock(VrcIrq *irq) {
    if (!irq->enabled) return;
    irq->prescaler -= 3;
    if (!irq->cycle_mode && irq->prescaler > 0) return;
    if (irq->counter == 0xFF) {
        irq->counter = irq->reload;
        mapper_irq_line = true;
    } else {
        irq->counter++;
    }
    irq->prescaler += 341;
}

static void vrc_irq_control(VrcIrq *irq, uint8_t value) {
    irq->enabled_after_ack = (value & 0x01u) != 0;
    irq->enabled = (value & 0x02u) != 0;
    irq->cycle_mode = (value & 0x04u) != 0;
    if (irq->enabled) {
        irq->counter = irq->reload;
        irq->prescaler = 341;
    }
    mapper_irq_line = false;
}

static void vrc_irq_ack(VrcIrq *irq) {
    irq->enabled = irq->enabled_after_ack;
    mapper_irq_line = false;
}

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

static size_t vrc24_chr_bank(unsigned slot) {
    size_t bank = vrc24.chr[slot];
    if (vrc24.variant == VRC2A) bank >>= 1;
    return bank % (C.chr_sz / CHR_BANK_1K);
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
    size_t bank = vrc24_chr_bank(a >> 10);
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void vrc24_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFF;
    size_t bank = vrc24_chr_bank(a >> 10);
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), value);
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

// Mapper selection and initialization.
static bool vrc24_submapper_supported(int mapper_no, uint8_t submapper) {
    switch (mapper_no) {
        case 21: return submapper <= 2;
        case 22: return submapper == 0;
        case 23: case 25: return submapper <= 3;
        case 27: case 183: return submapper == 0;
        default: return false;
    }
}

static bool bandai_layout(int mapper, uint8_t submapper, bool nes2,
                          size_t prg_bytes, size_t chr_bytes, bool chr_is_ram,
                          RomRamSizes *ram, unsigned eeprom_sizes[2]) {
    if (prg_bytes > 0x80000 || (prg_bytes & (prg_bytes - 1)) != 0) return false;
    if ((mapper == 153 || mapper == 157 || prg_bytes == 0x80000) && !chr_is_ram) return false;
    if ((chr_is_ram && chr_bytes != CHR_BANK_8K)
        || (!chr_is_ram && (chr_bytes > 0x40000 || (chr_bytes % CHR_BANK_1K) != 0))) return false;
    if (mapper == 153)
        return (!ram->prg_ram || ram->prg_ram == PRG_BANK_8K)
            && (!ram->prg_nvram || ram->prg_nvram == PRG_BANK_8K);

    unsigned serial_bytes = mapper == 159 || mapper == 157 ? 128 : submapper == 4 ? 0 : 256;
    if (nes2) {
        if (ram->prg_ram || (ram->prg_nvram && ram->prg_nvram != serial_bytes)) return false;
    } else if (ram->prg_ram + ram->prg_nvram > PRG_BANK_8K) {
        return false;
    }
    if (mapper == 157) {
        eeprom_sizes[0] = 256;
        eeprom_sizes[1] = !nes2 || ram->prg_nvram == 128 ? 128 : 0;
    } else if (mapper == 159) {
        eeprom_sizes[0] = 128;
    } else if (submapper == 0 || (submapper == 5 && ram->prg_nvram == 256)) {
        eeprom_sizes[0] = 256;
    }
    // The header describes the serial device, not an addressable PRG-RAM chip.
    ram->prg_ram = ram->prg_nvram = 0;
    return true;
}

static bool ram_geometry_supported(int mapper_no, bool nes2, const RomRamSizes *ram,
                                   bool chr_is_ram, size_t chr_sz) {
    size_t prg_total = ram->prg_ram + ram->prg_nvram;
    size_t chr_total = ram->chr_ram + ram->chr_nvram;
    if (prg_total && (prg_total & (prg_total - 1))) return false;

    // Only these boards have implemented selection between separate PRG RAM chips.
    bool split_prg = ram->prg_ram && ram->prg_nvram;
    if (split_prg && !((mapper_no == 1 || mapper_no == 155 || mapper_no == 5)
        && ram->prg_ram == 0x2000 && ram->prg_nvram == 0x2000)) return false;
    if (mapper_no == 1 || mapper_no == 155) {
        if (prg_total > 0x8000) return false;
    } else if (mapper_no == 5) {
        if (nes2) {
            if (!split_prg && prg_total > 0x2000 && prg_total != 0x8000
                && prg_total != 0x10000 && prg_total != 0x20000) return false;
        } else if (prg_total > 0x10000) {
            return false; // Legacy bank registers address at most eight 8KB pages.
        }
    } else if (mapper_no == 69) {
        if (split_prg || prg_total > 0x80000) return false;
    } else if (mapper_no == 30) {
        if (prg_total != 0) return false;
    } else if (prg_total > 0x2000) {
        return false;
    }

    if (mapper_no == 119) {
        if (chr_is_ram || chr_sz == 0) return false;
        if (nes2 && (ram->chr_ram != sizeof(tqrom_chr_ram) || ram->chr_nvram != 0)) return false;
    } else {
        // Other supported boards do not select separate CHR ROM/RAM or two RAM chips.
        if ((!chr_is_ram && chr_total) || (ram->chr_ram && ram->chr_nvram)) return false;
    }
    if (nes2 && chr_is_ram && chr_total != chr_sz) return false;
    size_t chr_limit;
    switch (mapper_no) {
        case 1: case 9: case 10: case 11: case 155: chr_limit = 0x20000; break;
        case 3: chr_limit = 0x200000; break;
        case 4: case 118: chr_limit = 0x40000; break;
        case 33: case 48: chr_limit = 0x80000; break;
        case 64: case 158: chr_limit = 0x40000; break;
        case 5: chr_limit = 0x100000; break;
        case 13: chr_limit = 0x4000; break;
        case 28: case 30: chr_limit = 0x8000; break;
        case 18: case 32: case 65: chr_limit = 0x40000; break;
        case 21: case 23: case 25: case 27: case 183: chr_limit = 0x80000; break;
        case 22: chr_limit = 0x40000; break;
        case 69: chr_limit = 0x40000; break;
        case 119: chr_limit = sizeof(tqrom_chr_ram); break;
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
        case 7: case 9: case 10: case 11: case 13: case 15: case 28: case 30: case 118: case 119: case 155:
        case 16: case 153: case 157: case 159:
        case 18: case 32: case 33: case 48: case 64: case 65: case 158:
        case 21: case 22: case 23: case 25: case 27: case 183:
        case 69:
            break;
        default:
            fprintf(stderr, "Unsupported mapper: %d\n", mapper_no);
            return -1;
    }
    if (submapper && !((mapper_no == 1 && submapper == 5)
        || vrc24_submapper_supported(mapper_no, submapper)
        || (mapper_no == 4 && (submapper == 1 || submapper == 3))
        || (mapper_no == 16 && (submapper == 4 || submapper == 5))
        || (mapper_no == 48 && submapper == 1)
        || (mapper_no == 32 && submapper == 1)
        || ((mapper_no == 2 || mapper_no == 3 || mapper_no == 7) && submapper <= 2)
        || (mapper_no == 30 && submapper <= 4))) {
        fprintf(stderr, "Unsupported mapper/submapper: %d/%u\n", mapper_no, submapper);
        return -1;
    }
    RomRamSizes ram;
    rom_ram_sizes(h, &ram);
    if ((ram.prg_nvram || ram.chr_nvram) && !(h->flags6 & 2)) {
        fprintf(stderr, "Nonvolatile RAM declared without the battery flag\n");
        return -1;
    }
    bool chr_is_ram = h->chr_rom_chunks == 0 && (!nes2 || (h->flags9 & 0xF0) == 0);
    unsigned eeprom_sizes[2] = {0, 0};
    bool is_bandai = mapper_no == 16 || mapper_no == 153 || mapper_no == 157 || mapper_no == 159;
    if (is_bandai && !bandai_layout(mapper_no, submapper, nes2, prg_sz, chr_sz,
                                    chr_is_ram, &ram, eeprom_sizes)) {
        fprintf(stderr, "Unsupported cartridge layout for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 5 && (prg_sz > 0x100000 || (!chr_is_ram && chr_sz > 0x100000))) {
        fprintf(stderr, "Unsupported ROM size for mapper 5\n");
        return -1;
    }
    if ((mapper_no == 33 || mapper_no == 48)
        && (prg_sz > 0x80000 || prg_sz % PRG_BANK_8K != 0
            || chr_sz % CHR_BANK_1K != 0 || (!chr_is_ram && chr_sz > 0x80000))) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if ((mapper_no == 64 || mapper_no == 158)
        && (prg_sz > 0x200000 || prg_sz % PRG_BANK_8K != 0
            || chr_sz > 0x40000 || chr_sz % CHR_BANK_1K != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 118 && (prg_sz > 0x200000 || prg_sz % PRG_BANK_8K != 0
        || chr_sz > 0x40000 || chr_sz % CHR_BANK_1K != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper 118\n");
        return -1;
    }
    if (mapper_no == 119 && (chr_is_ram || chr_sz > 0x40000 || (chr_sz % CHR_BANK_1K) != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper 119\n");
        return -1;
    }
    if (mapper_no == 28 && ((prg_sz % PRG_BANK_16K) != 0 || prg_sz > 0x800000
        || !chr_is_ram || (chr_sz != 0x2000 && chr_sz != 0x4000 && chr_sz != 0x8000))) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 28\n");
        return -1;
    }
    if (mapper_no == 30 && (prg_sz > 0x80000 || (prg_sz & (prg_sz - 1)) != 0
        || !chr_is_ram || (chr_sz != 0x2000 && chr_sz != 0x4000 && chr_sz != 0x8000)
        || ((h->flags6 & 0x09) == 0x09 && submapper != 3 && chr_sz != 0x8000))) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 30\n");
        return -1;
    }
    if (mapper_no == 18 && (prg_sz > 0x200000 || (prg_sz % PRG_BANK_8K) != 0
        || chr_sz > 0x40000 || (chr_sz % CHR_BANK_1K) != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper 18\n");
        return -1;
    }
    if ((mapper_no == 32 || mapper_no == 65)
        && (prg_sz > (mapper_no == 32 ? 0x40000u : 0x200000u)
            || (prg_sz % PRG_BANK_8K) != 0
            || chr_sz > 0x40000 || (chr_sz % CHR_BANK_1K) != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (vrc24_submapper_supported(mapper_no, submapper)
        && (prg_sz > 0x40000 || (prg_sz % PRG_BANK_8K) != 0
            || chr_sz > (mapper_no == 22 ? 0x40000u : 0x80000u)
            || (chr_sz % CHR_BANK_1K) != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 69 && (prg_sz > 0x80000 || (prg_sz % PRG_BANK_8K) != 0
        || chr_sz > 0x40000 || (chr_sz % CHR_BANK_1K) != 0)) {
        fprintf(stderr, "Unsupported ROM size for mapper 69\n");
        return -1;
    }
    if (!ram_geometry_supported(mapper_no, nes2, &ram, chr_is_ram, chr_sz)
        || (mapper_no == 4 && submapper == 1 && ram.prg_ram + ram.prg_nvram != 0x400)) {
        fprintf(stderr, "Unsupported RAM layout for mapper %d (PRG %zu+%zu, CHR %zu+%zu)\n",
                mapper_no, ram.prg_ram, ram.prg_nvram, ram.chr_ram, ram.chr_nvram);
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
    C.mapper_no = (uint16_t)mapper_no;
    C.ram = ram;
    C.nes2 = nes2;
    C.submapper = submapper;
    C.mmc1a = mapper_no == 155;
    C.bus_conflicts = mapper_no == 11
        || (submapper == 2 && (mapper_no == 2 || mapper_no == 3 || mapper_no == 7 || mapper_no == 30))
        || (mapper_no == 30 && submapper == 0 && !(h->flags6 & 2));
    
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
        case 1: case 155:
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
            mapper_mmc5.clock = mmc5_clock;
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
        case 21: case 22: case 23: case 25: case 27: case 183:
            memset(&vrc24, 0, sizeof(vrc24));
            if (!vrc24_select_variant()) return -1;
            build_mapper(&mapper_vrc24, vrc24_cpu_read, vrc24_cpu_write,
                        vrc24_ppu_read, vrc24_ppu_write, vrc24_reset, vrc24_mirr);
            if (vrc24_has_irq()) mapper_vrc24.clock = vrc24_clock;
            cart = &mapper_vrc24;
            break;
        case 28:
            build_mapper(&mapper_action53, m28_cpu_read, m28_cpu_write,
                        m28_ppu_read, m28_ppu_write, NULL, m28_mirr);
            cart = &mapper_action53;
            m28_power_on();
            break;
        case 33:
            build_mapper(&mapper_taito33, taito33_cpu_read, taito33_cpu_write,
                        taito33_ppu_read, taito33_ppu_write, taito33_reset, taito33_mirr);
            cart = &mapper_taito33;
            break;
        case 48:
            build_mapper(&mapper_taito48, taito48_cpu_read, taito48_cpu_write,
                        taito48_ppu_read, taito48_ppu_write, taito48_reset, taito48_mirr);
            mapper_taito48.clock = taito48_clock;
            cart = &mapper_taito48;
            break;
        case 30:
            build_mapper(&mapper_unrom512, m30_cpu_read, m30_cpu_write,
                        m30_ppu_read, m30_ppu_write, NULL, m30_mirr);
            cart = &mapper_unrom512;
            m30_power_on(h);
            break;
        case 18:
            build_mapper(&mapper_jaleco18, jaleco18_cpu_read, jaleco18_cpu_write,
                        jaleco18_ppu_read, jaleco18_ppu_write, jaleco18_reset, jaleco18_mirr);
            mapper_jaleco18.clock = jaleco18_clock;
            cart = &mapper_jaleco18;
            break;
        case 32:
            build_mapper(&mapper_irem32, irem32_cpu_read, irem32_cpu_write,
                        irem32_ppu_read, irem32_ppu_write, irem32_reset, irem32_mirr);
            cart = &mapper_irem32;
            break;
        case 65:
            build_mapper(&mapper_irem65, irem65_cpu_read, irem65_cpu_write,
                        irem65_ppu_read, irem65_ppu_write, irem65_reset, irem65_mirr);
            mapper_irem65.clock = irem65_clock;
            cart = &mapper_irem65;
            break;
        case 64:
            build_mapper(&mapper_rambo1, rambo1_cpu_read, rambo1_cpu_write,
                        rambo1_ppu_read, rambo1_ppu_write, rambo1_reset, rambo1_mirr);
            mapper_rambo1.clock = rambo1_clock;
            cart = &mapper_rambo1;
            break;
        case 118:
            build_mapper(&mapper_txsrom, mmc3_cpu_read, txsrom_cpu_write,
                         mmc3_ppu_read, mmc3_ppu_write, txsrom_reset, mmc3_mirr);
            cart = &mapper_txsrom;
            break;
        case 69:
            build_mapper(&mapper_sunsoft69, sunsoft69_cpu_read, sunsoft69_cpu_write,
                         sunsoft69_ppu_read, sunsoft69_ppu_write, sunsoft69_reset, sunsoft69_mirr);
            mapper_sunsoft69.clock = sunsoft69_clock;
            cart = &mapper_sunsoft69;
            break;
        case 119:
            build_mapper(&mapper_tqrom, mmc3_cpu_read, mmc3_cpu_write,
                        tqrom_ppu_read, tqrom_ppu_write, mmc3_reset, mmc3_mirr);
            cart = &mapper_tqrom;
            break;
        case 16: case 153: case 157: case 159:
            build_mapper(&mapper_bandai, bandai_cpu_read, bandai_cpu_write,
                         bandai_ppu_read, bandai_ppu_write, NULL, bandai_mirroring);
            mapper_bandai.clock = bandai_clock;
            bandai_init(mapper_no, eeprom_sizes[0], eeprom_sizes[1]);
            cart = &mapper_bandai;
            break;
        case 158:
            build_mapper(&mapper_rambo158, rambo1_cpu_read, rambo158_cpu_write,
                        rambo1_ppu_read, rambo1_ppu_write, rambo1_reset, rambo1_mirr);
            mapper_rambo158.clock = rambo1_clock;
            cart = &mapper_rambo158;
            break;
    }
    
    if (cart && cart->reset) cart->reset();
    return mapper_no;
}
