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
#include "board.h"
#include "eeprom.h"
#include "namco163.h"
#include "sunsoft5b.h"
#include "../ppu/ppu.h"
#include "vrc7_audio.h"
#include "fds.h"
#include "../system/timing.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"

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
static Mapper mapper_nrom, mapper_mmc1, mapper_m105, mapper_m232, mapper_m96, mapper_uxrom;
static Mapper mapper_cnrom, mapper_cnrom_protect, mapper_mmc3, mapper_tqrom, mapper_txsrom;
static Mapper mapper_mmc5, mapper_aorom, mapper_mmc2, mapper_mmc4, mapper_colordreams;
static Mapper mapper_cprom, mapper_100in1, mapper_bandai, mapper_action53, mapper_unrom512, mapper_m111, mapper_fds;
static Mapper mapper_taito33, mapper_taito48, mapper_taito_x1005, mapper_taito_x1017;
static Mapper mapper_jaleco18, mapper_jaleco_discrete, mapper_irem32, mapper_irem65, mapper_irem77, mapper_irem97;
static Mapper mapper_rambo1, mapper_rambo158;
static Mapper mapper_vrc1, mapper_vrc3, mapper_vrc6, mapper_vrc24, mapper_vrc7;
static Mapper mapper_sunsoft3, mapper_sunsoft4, mapper_sunsoft89, mapper_sunsoft93, mapper_sunsoft184;
static Mapper mapper_sunsoft69, mapper_namco, mapper_m34, mapper_gxrom, mapper_m71, mapper_namco108;
static Mapper mapper_vs99, mapper_jy, mapper_nina;
static Mapper mapper_board;
static CartridgeBoard *active_board = NULL;
Mapper *cart = NULL;
static bool mapper_irq_line = false;
static uint8_t cart_cpu_bus_input = 0xFF;
static CartPpuFetchSource cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
static bool mmc3_revision_a_profile = false;
static unsigned cart_dip_value = 0;
static bool cart_cpu_cycle_is_write = false;
static void mmc3_irq_clock(void);
static size_t namco_chr_bank(uint8_t bank);
static float vrc7_expansion_output(void);
static void vrc7_shutdown(void);

typedef struct {
    uint8_t reload;
    uint8_t counter;
    int16_t prescaler;
    bool enabled;
    bool enabled_after_ack;
    bool cycle_mode;
} VrcIrq;

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

typedef enum {
    FLASH_WAITING = 0,
    FLASH_PROGRAM,
    FLASH_ERASE
} FlashMode;

static struct {
    uint8_t bank_latch;
    uint8_t flash_cycle;
    FlashMode flash_mode;
    bool software_id;
} m111;

static RamBlock prg_work_ram, prg_save_ram;
static RamBlock chr_work_ram, chr_save_ram;
static uint8_t *chr_nvram_data(void);
static void mmc2_notify_ppu_address(uint16_t address);
static void mmc4_notify_ppu_address(uint16_t address);
static bool prg_ram_dirty = false;
static bool chr_ram_dirty = false;
static uint8_t mmc5_exram[0x400];
static uint8_t mmc3_mixed_chr_ram[CHR_BANK_8K];
static size_t mmc3_mixed_chr_first_bank;
static size_t mmc3_mixed_chr_last_bank;
static size_t mmc3_mixed_chr_ram_size;
static uint8_t irem77_chr_ram[CHR_BANK_8K];
static bool mmc5_exram_dirty = false;
static bool battery_enabled = false;
static char *battery_save_path = NULL;
static char *chr_save_path = NULL;
static Eeprom24 bandai_eeprom[2];
static char *eeprom_save_path[2];
static char *flash_save_path = NULL;
static bool flash_dirty = false;
static bool unrom512_four_screen_chr = false;
static uint8_t m111_nt_ram[0x4000];
static RamBlock *mmc5_ram_location(uint16_t a, size_t *offset);
static Sunsoft5B sunsoft5b_audio;
static Namco163Audio namco163_audio;
static bool namco163_audio_dirty = false;

typedef enum {
    NAMCO_VARIANT_163,
    NAMCO_VARIANT_175,
    NAMCO_VARIANT_340,
    NAMCO_VARIANT_UNKNOWN
} NamcoVariant;

static struct {
    NamcoVariant variant;
    bool auto_detect;
    bool not_340;
    uint8_t write_protect;
    bool low_chr_nt_mode;
    bool high_chr_nt_mode;
    uint16_t irq_counter;
    uint8_t prg_bank[3];
    bool prg_mapped[3];
    uint8_t chr_bank[8];
    bool chr_mapped[8];
    bool chr_ciram[8];
    uint8_t chr_ciram_page[8];
    uint8_t nt_bank[4];
    bool nt_mapped[4];
    bool nt_ciram[4];
    uint8_t nt_ciram_page[4];
    Mirroring mirr;
} namco;

static struct {
    bool nina;
    uint8_t prg_bank;
    uint8_t chr_bank[2];
    bool chr_mapped[2];
} m34;

static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
} gxrom;

static struct {
    uint8_t prg_bank;
    bool bf9097_mode;
    bool force_bf9097;
    Mirroring mirr;
} m71;

static struct {
    uint8_t select;
    uint8_t banks[8];
    bool fixed_prg;
    bool nametables_selected;
    Mirroring mirr;
} namco108;

void cart_apply_trainer(const uint8_t trainer[512]) {
    if (active_board) {
        board_apply_trainer(active_board, trainer);
        return;
    }
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

static bool namco_has_audio(void) {
    return cart == &mapper_namco && namco.variant == NAMCO_VARIANT_163;
}

static bool cart_has_flash_storage(void) {
    return cart == &mapper_unrom512 || cart == &mapper_m111;
}

static void flush_namco_battery(void) {
    bool audio = namco_has_audio();
    if (!battery_enabled || !battery_save_path
        || (!prg_ram_dirty && (!audio || !namco163_audio_dirty))) return;
    FILE *fp = fopen(battery_save_path, "wb");
    if (!fp) {
        perror("battery save open");
        return;
    }
    size_t prg_written = prg_save_ram.size
        ? fwrite(prg_save_ram.data, 1, prg_save_ram.size, fp) : 0;
    size_t audio_written = audio
        ? fwrite(namco163_audio_ram(&namco163_audio), 1, NAMCO163_RAM_SIZE, fp) : 0;
    int close_result = fclose(fp);
    if (prg_written != prg_save_ram.size
        || (audio && audio_written != NAMCO163_RAM_SIZE) || close_result != 0) {
        fprintf(stderr, "Failed to write battery save '%s'\n", battery_save_path);
        return;
    }
    prg_ram_dirty = false;
    if (audio) namco163_audio_dirty = false;
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
    if (active_board) {
        board_battery_flush(active_board);
        return;
    }
    if (cart_has_flash_storage())
        flush_flash_battery();
    else if (cart == &mapper_mmc5) flush_mmc5_battery();
    else if (cart == &mapper_namco) flush_namco_battery();
    else flush_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size, &prg_ram_dirty);
    flush_battery(chr_save_path, chr_nvram_data(), C.ram.chr_nvram, &chr_ram_dirty);
    for (unsigned i = 0; i < 2; ++i)
        flush_battery(eeprom_save_path[i], bandai_eeprom[i].bytes,
                      bandai_eeprom[i].capacity, &bandai_eeprom[i].dirty);
}

void cart_battery_shutdown(void) {
    cart_battery_flush();
    if (active_board) board_battery_configure(active_board, NULL);
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
    namco163_audio_dirty = false;
    flash_dirty = false;
}

static void load_battery(const char *path, uint8_t *data, size_t size) {
    if (!path || !size) return;
    FILE *fp = fopen(path, "rb");
    if (!fp) return; // first run/no prior save
    // Preserve initialized memory, including trainer bytes beyond a short save.
    size_t bytes_read = fread(data, 1, size, fp);
    if (bytes_read < size && ferror(fp))
        fprintf(stderr, "Failed to read battery save '%s' (%zu/%zu bytes)\n", path, bytes_read, size);
    fclose(fp);
}

static void load_mmc5_battery(const char *path) {
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

static void load_namco_battery(const char *path) {
    if (namco_has_audio()) memset(namco163_audio_ram(&namco163_audio), 0, NAMCO163_RAM_SIZE);
    if (!path) return;
    FILE *fp = fopen(path, "rb");
    if (!fp) return;
    size_t prg_read = prg_save_ram.size
        ? fread(prg_save_ram.data, 1, prg_save_ram.size, fp) : 0;
    size_t audio_read = 0;
    if (namco_has_audio() && prg_read == prg_save_ram.size)
        audio_read = fread(namco163_audio_ram(&namco163_audio), 1, NAMCO163_RAM_SIZE, fp);
    if ((prg_read < prg_save_ram.size
        || (namco_has_audio() && audio_read < NAMCO163_RAM_SIZE)) && ferror(fp))
        fprintf(stderr, "Failed to read battery save '%s'\n", path);
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
    if (active_board) {
        board_battery_configure(active_board, rom_path);
        return;
    }
    cart_battery_shutdown();
    bool serial_storage = bandai_eeprom[0].capacity || bandai_eeprom[1].capacity;
    bool persistent_flash = cart == &mapper_m111 || (has_battery && cart == &mapper_unrom512);
    if (!rom_path || (!has_battery && !serial_storage && !persistent_flash)) return;
    if (persistent_flash)
        flash_save_path = build_save_path(rom_path, ".flash.sav");
    if (has_battery && (prg_save_ram.size || namco_has_audio()))
        battery_save_path = build_save_path(rom_path, ".sav");
    if (has_battery && C.ram.chr_nvram) chr_save_path = build_save_path(rom_path, ".chr.sav");
    bool paths_valid = (!has_battery || (!prg_save_ram.size && !namco_has_audio()) || battery_save_path)
                    && (!has_battery || !C.ram.chr_nvram || chr_save_path)
                    && (!persistent_flash || flash_save_path);
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
    if (cart_has_flash_storage()) load_flash_battery(flash_save_path);
    else if (cart == &mapper_mmc5) load_mmc5_battery(battery_save_path);
    else if (cart == &mapper_namco) load_namco_battery(battery_save_path);
    else load_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size);
    load_battery(chr_save_path, chr_nvram_data(), C.ram.chr_nvram);
    for (unsigned i = 0; i < 2; ++i)
        load_battery(eeprom_save_path[i], bandai_eeprom[i].bytes, bandai_eeprom[i].capacity);
}

void mapper_shutdown(void) {
    cart_battery_shutdown();
    board_destroy(active_board);
    active_board = NULL;
    if (cart == &mapper_vrc7) vrc7_shutdown();
    if (cart == &mapper_fds) fds_shutdown();
    free(prg_work_ram.data);
    free(prg_save_ram.data);
    free(chr_work_ram.data);
    free(chr_save_ram.data);
    prg_work_ram = (RamBlock){0};
    prg_save_ram = (RamBlock){0};
    chr_work_ram = (RamBlock){0};
    chr_save_ram = (RamBlock){0};
    cart = NULL;
    memset(&C, 0, sizeof(C));
    mapper_irq_line = false;
    cart_cpu_cycle_is_write = false;
    cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
    memset(mmc5_exram, 0, sizeof(mmc5_exram));
    memset(mmc3_mixed_chr_ram, 0, sizeof(mmc3_mixed_chr_ram));
    mmc3_mixed_chr_first_bank = 0;
    mmc3_mixed_chr_last_bank = 0;
    mmc3_mixed_chr_ram_size = 0;
    memset(irem77_chr_ram, 0, sizeof(irem77_chr_ram));
    memset(bandai_eeprom, 0, sizeof(bandai_eeprom));
    mmc5_exram_dirty = false;
    unrom512_four_screen_chr = false;
    memset(m111_nt_ram, 0, sizeof(m111_nt_ram));
}

// Helpers
static inline Mirroring base_mirr(void) { return C.mirr_base; }
Mirroring cart_get_mirroring(void) { return cart && cart->get_mirroring ? cart->get_mirroring() : base_mirr(); }
void cart_set_mirroring(Mirroring m) {
    C.mirr_base = m;
    if (active_board) board_set_mirroring(active_board, m);
}
bool cart_set_mmc3_revision_name(const char *name) {
    if (!name) return false;
    if (strcmp(name, "standard") == 0) {
        mmc3_revision_a_profile = false;
        return true;
    }
    if (strcmp(name, "a") == 0) {
        mmc3_revision_a_profile = true;
        return true;
    }
    return false;
}
const char *cart_mmc3_revision_name(void) {
    return mmc3_revision_a_profile ? "a" : "standard";
}
bool cart_set_dip_switches(unsigned value) {
    if (value > 0xFFu) return false;
    cart_dip_value = value;
    return true;
}
unsigned cart_dip_switches(void) { return cart_dip_value; }
static uint8_t repeated_prg_window_read(uint16_t address, uint16_t start, size_t window_size);
static uint8_t discrete_chr8_read(uint16_t address, uint8_t bank);
static void discrete_chr8_write(uint16_t address, uint8_t bank, uint8_t value);

static size_t mapper_prg_page_size(uint16_t mapper_no) {
    switch (mapper_no) {
        case 4: case 5: case 9: case 15: case 18: case 19:
        case 21: case 22: case 23: case 24: case 25: case 26: case 27:
        case 32: case 33: case 48: case 64: case 65: case 69: case 74:
        case 75: case 76: case 80: case 82: case 85: case 88: case 90:
        case 95: case 99: case 118: case 119: case 151: case 154: case 158:
        case 183: case 191: case 192: case 194: case 195: case 206: case 207:
        case 209: case 210: case 211:
            return PRG_BANK_8K;
        case 0: case 1: case 2: case 10: case 16: case 28: case 30:
        case 67: case 68: case 71: case 72: case 73: case 78: case 89:
        case 92: case 93: case 94: case 97: case 105: case 153: case 155:
        case 157: case 159: case 180: case 232:
            return PRG_BANK_16K;
        case 3: case 7: case 11: case 13: case 34: case 66: case 77:
        case 79: case 87: case 96: case 101: case 111: case 113: case 140:
        case 144: case 146: case 184: case 185:
            return PRG_BANK_32K;
        default:
            return 0;
    }
}

static size_t mapper_chr_page_size(uint16_t mapper_no) {
    switch (mapper_no) {
        case 4: case 5: case 16: case 18: case 19: case 21: case 22: case 23:
        case 24: case 25: case 26: case 27: case 32: case 33: case 48: case 64:
        case 65: case 69: case 74: case 80: case 82: case 85: case 88: case 90:
        case 95: case 118: case 119: case 153: case 154: case 157: case 158:
        case 159: case 183: case 191: case 192: case 194: case 195: case 206:
        case 207: case 209: case 210: case 211:
            return CHR_BANK_1K;
        case 67: case 68: case 76: case 77:
            return CHR_BANK_2K;
        case 1: case 9: case 10: case 13: case 34: case 75: case 96:
        case 105: case 151: case 155: case 184:
            return CHR_BANK_4K;
        case 0: case 2: case 3: case 7: case 11: case 15: case 28: case 30:
        case 66: case 71: case 72: case 73: case 78: case 79: case 87: case 89:
        case 92: case 93: case 94: case 97: case 99: case 101: case 111:
        case 113: case 140: case 144: case 146: case 180: case 185: case 232:
            return CHR_BANK_8K;
        default:
            return 0;
    }
}

static bool mapper_has_shrinking_chr_window(uint16_t mapper_no) {
    switch (mapper_no) {
        case 0: case 1: case 2: case 3: case 7: case 9: case 10: case 11:
        case 13: case 15: case 34: case 66: case 67: case 68: case 69: case 71:
        case 72: case 73: case 75: case 76: case 78: case 79: case 87: case 88:
        case 89: case 92: case 93: case 94: case 95: case 97: case 101: case 105:
        case 113: case 140: case 144: case 146: case 151: case 154: case 155:
        case 180: case 184: case 185: case 206: case 232:
            return true;
        default:
            return false;
    }
}

static size_t shrunk_chr_page_size(size_t native_page_size) {
    if (!C.chr_sz) return 0;
    return C.chr_sz < native_page_size ? C.chr_sz : native_page_size;
}

static bool chr_bank_slot_offset(uint16_t address, size_t native_page_size,
                                 unsigned slot_count, const uint8_t *banks,
                                 size_t *offset) {
    size_t page_size = shrunk_chr_page_size(native_page_size);
    if (!page_size || !banks || !offset) return false;
    size_t slot = (address & 0x1FFFu) / page_size;
    if (slot >= slot_count) return false;
    size_t page_count = C.chr_sz / page_size;
    if (!page_count) return false;
    *offset = ((size_t)banks[slot] % page_count) * page_size
            + ((address & 0x1FFFu) % page_size);
    return true;
}

static uint8_t chr_unmapped_read(uint16_t address) {
    address &= 0x1FFFu;
    return C.chr_is_ram ? C.chr[address % C.chr_sz] : (uint8_t)address;
}

static bool small_prg_window_read(uint16_t address, uint8_t *value) {
    if (active_board || !value || address < 0x8000 || !C.prg || !C.prg_sz || C.prg_sz >= PRG_BANK_32K)
        return false;
    if (C.mapper_no == 5 || C.mapper_no == 30 || C.mapper_no == 99 || C.mapper_no == 111)
        return false;
    size_t page_size = mapper_prg_page_size(C.mapper_no);
    if (!page_size || page_size <= C.prg_sz) return false;
    *value = repeated_prg_window_read(address, 0x8000, PRG_BANK_32K);
    return true;
}

bool cart_set_karaoke_input(CartKaraokeInput input, bool pressed) {
    if (C.mapper_no != 188 || (unsigned)input >= CART_KARAOKE_INPUT_COUNT) return false;
    return board_set_mapper_input(active_board, (unsigned)input, pressed);
}
uint8_t cart_cpu_read(uint16_t a) {
    if (active_board) return board_cpu_read(active_board, a, 0xFF);
    if (cart == &mapper_fds) return fds_cpu_read_bus(a, 0xFF);
    uint8_t small;
    if (small_prg_window_read(a, &small)) return small;
    return cart ? cart->cpu_read(a) : 0xFF;
}
uint8_t cart_cpu_read_bus(uint16_t a, uint8_t open_bus) {
    if (!cart) return open_bus;
    if (cart == &mapper_fds) return fds_cpu_read_bus(a, open_bus);
    // Read paths decide which registers and RAM chips drive the data lines.
    // Scope the input so nested callbacks restore their caller's bus latch.
    uint8_t previous_bus = cart_cpu_bus_input;
    cart_cpu_bus_input = open_bus;
    uint8_t value;
    if (!small_prg_window_read(a, &value)) value = cart->cpu_read(a);
    cart_cpu_bus_input = previous_bus;
    if (cart == &mapper_mmc5 && a == 0x5204) value |= open_bus & 0x3F;
    return value;
}
bool cart_read_cpu_register(uint16_t address, uint8_t *value) {
    return board_read_cpu_register(active_board, address, value);
}

void cart_observe_cpu_write(uint16_t address, uint8_t value) {
    board_observe_cpu_write(active_board, address, value);
}

void cart_cpu_write(uint16_t a, uint8_t v) {
    if (!cart) return;
    if (a >= 0x8000 && C.bus_conflicts) {
        uint8_t rom_value;
        if (!small_prg_window_read(a, &rom_value)) rom_value = cart->cpu_read(a);
        v &= rom_value;
    }
    cart->cpu_write(a, v);
}
void cart_clock_cpu_cycle(bool write_cycle) {
    if (!cart || !cart->clock) return;
    cart_cpu_cycle_is_write = write_cycle;
    cart->clock(1);
    cart_cpu_cycle_is_write = false;
}
uint8_t cart_ppu_read(uint16_t a) { return cart ? cart->ppu_read(a) : 0x00; }
void cart_ppu_write(uint16_t a, uint8_t v) { if (cart) cart->ppu_write(a, v); }
void cart_set_ppu_fetch_source(CartPpuFetchSource src) { cart_ppu_fetch_source = src; }
bool cart_irq_pending(void) {
    return mapper_irq_line || board_irq_pending(active_board)
        || (cart == &mapper_fds && fds_irq_pending());
}
void cart_irq_ack(void) {
    mapper_irq_line = false;
    board_irq_ack(active_board);
}
void cart_console_reset(bool soft_reset) { board_reset(active_board, soft_reset); }
void cart_after_console_reset(void) { board_after_reset(active_board); }
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
static uint8_t *chr_nvram_data(void) {
    return chr_save_ram.size ? chr_save_ram.data : C.chr;
}

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
    if (C.chr_is_ram) return C.chr[a % C.chr_sz];
    size_t mapped = C.chr_sz < CHR_BANK_8K ? C.chr_sz : CHR_BANK_8K;
    return a < mapped ? C.chr[a] : (uint8_t)a;
}
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
    return mmc1_chr_offset(a, &offset) ? C.chr[offset] : chr_unmapped_read(a);
}

static void mmc1_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    size_t offset;
    chr_ram_write(mmc1_chr_offset(a, &offset) ? offset : ((a & 0x1FFFu) % C.chr_sz), v);
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

static size_t m96_chr_bank(uint16_t a) {
    a &= 0x1FFFu;
    if (!m96.chr_banking_active) return (a >> 12) & 1u;
    return a < 0x1000u
        ? (size_t)(m96.outer_chr_bank | m96.inner_chr_bank)
        : (size_t)(m96.outer_chr_bank | 0x03u);
}

static uint8_t m96_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    size_t banks = C.chr_sz / CHR_BANK_4K;
    if (!banks) return 0;
    size_t bank = m96_chr_bank(a) % banks;
    return C.chr[bank * CHR_BANK_4K + (a & 0x0FFFu)];
}

static void m96_ppu_write(uint16_t a, uint8_t v) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    size_t banks = C.chr_sz / CHR_BANK_4K;
    if (!banks) return;
    size_t bank = m96_chr_bank(a) % banks;
    chr_ram_write(bank * CHR_BANK_4K + (a & 0x0FFFu), v);
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
        // CHR RAM is mapped across the full pattern-table window before mapper
        // initialization. A smaller CHR ROM leaves the upper addresses open.
        if (C.chr_is_ram) return C.chr[a % C.chr_sz];
        return a < C.chr_sz ? C.chr[a] : (uint8_t)a;
    }
    size_t banks = C.chr_sz / CHR_BANK_8K;
    if (!banks) return (uint8_t)a;
    return C.chr[(bank % banks) * CHR_BANK_8K + a];
}

static void discrete_chr8_write(uint16_t a, uint8_t bank, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    if (C.chr_sz < CHR_BANK_8K) {
        chr_ram_write(a % C.chr_sz, value);
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
    if (!page_size || a >= page_size) return chr_unmapped_read(a);
    size_t bank = cn.chr_bank % (C.chr_sz / page_size);
    return C.chr[bank * page_size + a];
}
static void cnrom_ppu_write(uint16_t a, uint8_t v) {
    if (C.chr_is_ram) {
        a &= 0x1FFFu;
        size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
        if (!page_size || a >= page_size) {
            chr_ram_write(a % C.chr_sz, v);
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
}

static uint8_t cnrom185_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!cnrom185_chr_enabled) return (uint8_t)a | 0x01u;
    size_t page_size = shrunk_chr_page_size(CHR_BANK_8K);
    return page_size && a < page_size ? C.chr[a] : (uint8_t)a;
}

static void cnrom185_ppu_write(uint16_t a, uint8_t value) {
    (void)a;
    (void)value;
}

static Mirroring cnrom185_mirr(void) { return C.mirr_base; }

static void cnrom185_reset(void) {
    cnrom185_chr_enabled = true;
}

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

static size_t jy_chr_bank(uint16_t addr) {
    unsigned slot = (unsigned)((addr & 0x1FFFu) >> 10);
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
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) return 0;
    size_t bank = jy_chr_bank(addr) % banks;
    return C.chr[bank * CHR_BANK_1K + (addr & 0x03FFu)];
}

static void jy_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFFu;
    size_t banks = C.chr_sz / CHR_BANK_1K;
    if (!banks) return;
    size_t bank = jy_chr_bank(addr) % banks;
    chr_ram_write(bank * CHR_BANK_1K + (addr & 0x03FFu), value);
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

static bool mmc3_mixed_chr_uses_ram(size_t bank) {
    return mmc3_mixed_chr_ram_size
        && bank >= mmc3_mixed_chr_first_bank && bank <= mmc3_mixed_chr_last_bank;
}

static size_t mmc3_mixed_chr_ram_offset(uint16_t a, size_t bank) {
    size_t ram_bank = (bank - mmc3_mixed_chr_first_bank)
                    % (mmc3_mixed_chr_ram_size / CHR_BANK_1K);
    return ram_bank * CHR_BANK_1K + (a & 0x03FF);
}

static uint8_t mmc3_mixed_chr_ppu_read(uint16_t a) {
    a &= 0x1FFF;
    size_t bank = mmc3_chr_bank(a);
    if (mmc3_mixed_chr_uses_ram(bank))
        return mmc3_mixed_chr_ram[mmc3_mixed_chr_ram_offset(a, bank)];

    size_t chr_1k_banks = C.chr_sz / CHR_BANK_1K;
    if (chr_1k_banks == 0) return 0;
    bank %= chr_1k_banks;
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FF)];
}

static void mmc3_mixed_chr_ppu_write(uint16_t a, uint8_t v) {
    a &= 0x1FFF;
    size_t bank = mmc3_chr_bank(a);
    if (!mmc3_mixed_chr_uses_ram(bank)) return;
    mmc3_mixed_chr_ram[mmc3_mixed_chr_ram_offset(a, bank)] = v;
}

static size_t mmc3_mixed_chr_expected_ram(int mapper_no) {
    switch (mapper_no) {
        case 74: case 191: case 194: return 0x0800;
        case 192: case 195: return 0x1000;
        case 119: return 0x2000;
        default: return 0;
    }
}

static bool mmc3_mixed_chr_configure(int mapper_no) {
    mmc3_mixed_chr_ram_size = mmc3_mixed_chr_expected_ram(mapper_no);
    switch (mapper_no) {
        case 74:  mmc3_mixed_chr_first_bank = 0x08; mmc3_mixed_chr_last_bank = 0x09; break;
        case 119: mmc3_mixed_chr_first_bank = 0x40; mmc3_mixed_chr_last_bank = 0x7F; break;
        case 191: mmc3_mixed_chr_first_bank = 0x80; mmc3_mixed_chr_last_bank = 0xFF; break;
        case 192: mmc3_mixed_chr_first_bank = 0x08; mmc3_mixed_chr_last_bank = 0x0B; break;
        case 194: mmc3_mixed_chr_first_bank = 0x00; mmc3_mixed_chr_last_bank = 0x01; break;
        case 195: mmc3_mixed_chr_first_bank = 0x00; mmc3_mixed_chr_last_bank = 0x03; break;
        default: return false;
    }
    return true;
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

// Mappers 80 and 207: Taito X1-005.
static struct {
    uint8_t prg[3];
    uint8_t chr[8];
    uint8_t prg_mapped;
    uint8_t chr_mapped;
    uint8_t ram_permission;
    uint8_t nt_page[4];
    uint8_t nt_mapped;
    Mirroring mirr;
} taito_x1005;

static bool taito_x1005_ram_enabled(void) {
    return taito_x1005.ram_permission == 0xA3u;
}

static uint8_t taito_x1005_cpu_read(uint16_t a) {
    if (a >= 0x7F00u && a <= 0x7FFFu) {
        if (!taito_x1005_ram_enabled()) return cart_cpu_bus_input;
        return ram_read(default_prg_ram(), a - 0x7F00u);
    }
    if (a < 0x8000u) return cart_cpu_bus_input;

    size_t banks = C.prg_sz / PRG_BANK_8K;
    unsigned slot = (unsigned)((a - 0x8000u) >> 13);
    size_t bank;
    if (slot == 3) {
        bank = banks - 1;
    } else {
        if (!(taito_x1005.prg_mapped & (1u << slot))) return cart_cpu_bus_input;
        bank = taito_x1005.prg[slot] % banks;
    }
    return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
}

static void taito_x1005_ram_write(uint16_t a, uint8_t value) {
    if (!taito_x1005_ram_enabled()) return;
    size_t offset = a - 0x7F00u;
    // The physical 128-byte RAM is mirrored across the 256-byte CPU window.
    ram_write(default_prg_ram(), offset, value);
    ram_write(default_prg_ram(), offset ^ 0x80u, value);
}

static void taito_x1005_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x7F00u && a <= 0x7FFFu) {
        taito_x1005_ram_write(a, value);
        return;
    }
    if (a < 0x7EF0u || a > 0x7EFFu) return;

    switch (a) {
        case 0x7EF0:
            taito_x1005.chr[0] = value;
            taito_x1005.chr[1] = (uint8_t)(value + 1u);
            taito_x1005.chr_mapped |= 0x03u;
            if (C.mapper_no == 207) {
                uint8_t page = value >> 7;
                taito_x1005.nt_page[0] = page;
                taito_x1005.nt_page[1] = page;
                taito_x1005.nt_mapped |= 0x03u;
            }
            break;
        case 0x7EF1:
            taito_x1005.chr[2] = value;
            taito_x1005.chr[3] = (uint8_t)(value + 1u);
            taito_x1005.chr_mapped |= 0x0Cu;
            if (C.mapper_no == 207) {
                uint8_t page = value >> 7;
                taito_x1005.nt_page[2] = page;
                taito_x1005.nt_page[3] = page;
                taito_x1005.nt_mapped |= 0x0Cu;
            }
            break;
        case 0x7EF2: case 0x7EF3: case 0x7EF4: case 0x7EF5: {
            unsigned slot = 4u + (unsigned)(a - 0x7EF2u);
            taito_x1005.chr[slot] = value;
            taito_x1005.chr_mapped |= (uint8_t)(1u << slot);
            break;
        }
        case 0x7EF6: case 0x7EF7:
            if (C.mapper_no != 207)
                taito_x1005.mirr = (value & 1u) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
            break;
        case 0x7EF8: case 0x7EF9:
            taito_x1005.ram_permission = value;
            break;
        case 0x7EFA: case 0x7EFB:
            taito_x1005.prg[0] = value;
            taito_x1005.prg_mapped |= 0x01u;
            break;
        case 0x7EFC: case 0x7EFD:
            taito_x1005.prg[1] = value;
            taito_x1005.prg_mapped |= 0x02u;
            break;
        case 0x7EFE: case 0x7EFF:
            taito_x1005.prg[2] = value;
            taito_x1005.prg_mapped |= 0x04u;
            break;
    }
}

static uint8_t taito_x1005_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    unsigned slot = a >> 10;
    if (!(taito_x1005.chr_mapped & (1u << slot)))
        return C.chr_is_ram ? C.chr[a % C.chr_sz] : (uint8_t)a;
    size_t bank = taito_x1005.chr[slot] % (C.chr_sz / CHR_BANK_1K);
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void taito_x1005_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    unsigned slot = a >> 10;
    if (!(taito_x1005.chr_mapped & (1u << slot))) {
        chr_ram_write(a % C.chr_sz, value);
        return;
    }
    size_t bank = taito_x1005.chr[slot] % (C.chr_sz / CHR_BANK_1K);
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), value);
}

static Mirroring taito_x1005_mirr(void) { return taito_x1005.mirr; }

static void taito_x1005_reset(void) {
    memset(&taito_x1005, 0, sizeof(taito_x1005));
    taito_x1005.mirr = C.mirr_base;
}

// Mapper 82: Taito X1-017.
static struct {
    uint8_t prg[3];
    uint8_t chr[6];
    uint8_t prg_mapped;
    bool chr_mapped;
    uint8_t chr_mode;
    uint8_t ram_permission[3];
    Mirroring mirr;
} taito_x1017;

static bool taito_x1017_ram_location(uint16_t a, size_t *offset, unsigned *region) {
    if (a >= 0x6000u && a <= 0x67FFu) {
        *offset = a - 0x6000u;
        *region = 0;
        return true;
    }
    if (a >= 0x6800u && a <= 0x6FFFu) {
        *offset = a - 0x6000u;
        *region = 1;
        return true;
    }
    if (a >= 0x7000u && a <= 0x73FFu) {
        *offset = a - 0x6000u;
        *region = 2;
        return true;
    }
    return false;
}

static bool taito_x1017_ram_enabled(unsigned region) {
    static const uint8_t key[3] = {0xCA, 0x69, 0x84};
    return region < 3 && taito_x1017.ram_permission[region] == key[region];
}

static uint8_t taito_x1017_cpu_read(uint16_t a) {
    size_t offset;
    unsigned region;
    if (taito_x1017_ram_location(a, &offset, &region))
        return taito_x1017_ram_enabled(region) ? ram_read(default_prg_ram(), offset) : cart_cpu_bus_input;
    if (a < 0x8000u) return cart_cpu_bus_input;

    size_t banks = C.prg_sz / PRG_BANK_8K;
    unsigned slot = (unsigned)((a - 0x8000u) >> 13);
    size_t bank;
    if (slot == 3) {
        bank = banks - 1;
    } else {
        if (!(taito_x1017.prg_mapped & (1u << slot))) return cart_cpu_bus_input;
        bank = taito_x1017.prg[slot] % banks;
    }
    return C.prg[bank * PRG_BANK_8K + (a & 0x1FFFu)];
}

static void taito_x1017_cpu_write(uint16_t a, uint8_t value) {
    size_t offset;
    unsigned region;
    if (taito_x1017_ram_location(a, &offset, &region)) {
        if (taito_x1017_ram_enabled(region)) ram_write(default_prg_ram(), offset, value);
        return;
    }
    if (a < 0x7EF0u || a > 0x7EFFu) return;

    if (a <= 0x7EF5u) {
        taito_x1017.chr[a - 0x7EF0u] = value;
        taito_x1017.chr_mapped = true;
        return;
    }
    if (a == 0x7EF6u) {
        taito_x1017.mirr = (value & 1u) ? MIRROR_VERTICAL : MIRROR_HORIZONTAL;
        taito_x1017.chr_mode = (value >> 1) & 1u;
        taito_x1017.chr_mapped = true;
        return;
    }
    if (a >= 0x7EF7u && a <= 0x7EF9u) {
        taito_x1017.ram_permission[a - 0x7EF7u] = value;
        return;
    }
    if (a >= 0x7EFAu && a <= 0x7EFCu) {
        unsigned slot = a - 0x7EFAu;
        taito_x1017.prg[slot] = value >> 2;
        taito_x1017.prg_mapped |= (uint8_t)(1u << slot);
    }
}

static size_t taito_x1017_chr_bank(unsigned slot) {
    if (taito_x1017.chr_mode == 0) {
        if (slot < 4) return (taito_x1017.chr[slot >> 1] & 0xFEu) + (slot & 1u);
        return taito_x1017.chr[slot - 2u];
    }
    if (slot < 4) return taito_x1017.chr[slot + 2u];
    return (taito_x1017.chr[(slot - 4u) >> 1] & 0xFEu) + (slot & 1u);
}

static uint8_t taito_x1017_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (!taito_x1017.chr_mapped)
        return C.chr_is_ram ? C.chr[a % C.chr_sz] : (uint8_t)a;
    size_t bank = taito_x1017_chr_bank(a >> 10) % (C.chr_sz / CHR_BANK_1K);
    return C.chr[bank * CHR_BANK_1K + (a & 0x03FFu)];
}

static void taito_x1017_ppu_write(uint16_t a, uint8_t value) {
    if (!C.chr_is_ram) return;
    a &= 0x1FFFu;
    if (!taito_x1017.chr_mapped) {
        chr_ram_write(a % C.chr_sz, value);
        return;
    }
    size_t bank = taito_x1017_chr_bank(a >> 10) % (C.chr_sz / CHR_BANK_1K);
    chr_ram_write(bank * CHR_BANK_1K + (a & 0x03FFu), value);
}

static Mirroring taito_x1017_mirr(void) { return taito_x1017.mirr; }

static void taito_x1017_reset(void) {
    memset(&taito_x1017, 0, sizeof(taito_x1017));
    taito_x1017.mirr = C.mirr_base;
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

static size_t vrc6_chr_bank(uint16_t addr) {
    unsigned slot = (addr >> 10) & 7u;
    if (!vrc6.ppu_initialized) return C.chr_is_ram ? slot : SIZE_MAX;
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
    size_t bank = vrc6_chr_bank(addr);
    if (bank == SIZE_MAX) return (uint8_t)addr;
    bank %= C.chr_sz / CHR_BANK_1K;
    return C.chr[bank * CHR_BANK_1K + (addr & 0x03FFu)];
}

static void vrc6_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFFu;
    size_t bank = vrc6_chr_bank(addr);
    if (bank == SIZE_MAX) return;
    bank %= C.chr_sz / CHR_BANK_1K;
    chr_ram_write(bank * CHR_BANK_1K + (addr & 0x03FFu), value);
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
        return C.chr[offset];
    }
    if (cart == &mapper_vrc6 && vrc6.ppu_initialized) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t in = off & 0x03FFu;
        if (vrc6.banking_mode & 0x10u) {
            size_t bank = vrc6_nt_chr_bank(nt) % (C.chr_sz / CHR_BANK_1K);
            return C.chr[bank * CHR_BANK_1K + in];
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
    if (cart == &mapper_taito_x1005 && C.mapper_no == 207) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        if (taito_x1005.nt_mapped & (1u << nt))
            return nt_ram[(size_t)taito_x1005.nt_page[nt] * CHR_BANK_1K + (off & 0x03FFu)];
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
    if (cart == &mapper_namco
        && (namco.variant == NAMCO_VARIANT_163 || namco.variant == NAMCO_VARIANT_340)) {
        unsigned slot = (unsigned)(((addr - 0x2000u) & 0x0FFFu) >> 10);
        uint16_t in = (uint16_t)((addr - 0x2000u) & 0x03FFu);
        if (namco.nt_mapped[slot]) {
            if (namco.nt_ciram[slot])
                return nt_ram[(size_t)namco.nt_ciram_page[slot] * CHR_BANK_1K + in];
            size_t bank = namco_chr_bank(namco.nt_bank[slot]);
            return C.chr[bank * CHR_BANK_1K + in];
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
        chr_ram_write(offset, v);
        return;
    }
    if (cart == &mapper_vrc6 && vrc6.ppu_initialized) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        size_t in = off & 0x03FFu;
        if (vrc6.banking_mode & 0x10u) {
            if (!C.chr_is_ram) return;
            size_t bank = vrc6_nt_chr_bank(nt) % (C.chr_sz / CHR_BANK_1K);
            chr_ram_write(bank * CHR_BANK_1K + in, v);
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
    if (cart == &mapper_taito_x1005 && C.mapper_no == 207) {
        uint16_t off = (uint16_t)((addr - 0x2000u) & 0x0FFFu);
        unsigned nt = (off >> 10) & 3u;
        if (taito_x1005.nt_mapped & (1u << nt)) {
            nt_ram[(size_t)taito_x1005.nt_page[nt] * CHR_BANK_1K + (off & 0x03FFu)] = v;
            return;
        }
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
    if (cart == &mapper_namco
        && (namco.variant == NAMCO_VARIANT_163 || namco.variant == NAMCO_VARIANT_340)) {
        unsigned slot = (unsigned)(((addr - 0x2000u) & 0x0FFFu) >> 10);
        uint16_t in = (uint16_t)((addr - 0x2000u) & 0x03FFu);
        if (namco.nt_mapped[slot]) {
            if (namco.nt_ciram[slot]) {
                nt_ram[(size_t)namco.nt_ciram_page[slot] * CHR_BANK_1K + in] = v;
            } else if (C.chr_is_ram) {
                size_t bank = namco_chr_bank(namco.nt_bank[slot]);
                chr_ram_write(bank * CHR_BANK_1K + in, v);
            }
            return;
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

static size_t namco_chr_bank(uint8_t bank) {
    size_t banks = C.chr_sz / CHR_BANK_1K;
    return banks ? (size_t)bank % banks : 0;
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
    namco.chr_mapped[slot] = true;
    namco.chr_ciram[slot] = namco.variant == NAMCO_VARIANT_163 && !nt_mode && value >= 0xE0u;
    namco.chr_ciram_page[slot] = value & 1u;
    namco.chr_bank[slot] = value;
}

static void namco_set_nt_bank(unsigned slot, uint8_t value) {
    namco.nt_mapped[slot] = true;
    namco.nt_ciram[slot] = value >= 0xE0u;
    namco.nt_ciram_page[slot] = value & 1u;
    namco.nt_bank[slot] = value;
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
                memset(namco.nt_mapped, 0, sizeof(namco.nt_mapped));
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
    unsigned slot = address / CHR_BANK_1K;
    if (!namco.chr_mapped[slot])
        return C.chr_is_ram ? C.chr[address % C.chr_sz] : (uint8_t)address;
    if (namco.chr_ciram[slot])
        return ppu_vram[(size_t)namco.chr_ciram_page[slot] * CHR_BANK_1K + (address & 0x03FFu)];
    size_t bank = namco_chr_bank(namco.chr_bank[slot]);
    return C.chr[bank * CHR_BANK_1K + (address & 0x03FFu)];
}

static void namco_ppu_write(uint16_t address, uint8_t value) {
    address &= 0x1FFFu;
    unsigned slot = address / CHR_BANK_1K;
    if (namco.chr_mapped[slot] && namco.chr_ciram[slot]) {
        ppu_vram[(size_t)namco.chr_ciram_page[slot] * CHR_BANK_1K + (address & 0x03FFu)] = value;
        return;
    }
    if (!C.chr_is_ram) return;
    size_t index = namco.chr_mapped[slot]
        ? namco_chr_bank(namco.chr_bank[slot]) * CHR_BANK_1K + (address & 0x03FFu)
        : address % C.chr_sz;
    chr_ram_write(index, value);
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
    namco163_audio_reset(&namco163_audio);
    namco163_audio_dirty = false;
    mapper_irq_line = false;
}

// Mapper 34: BNROM and NINA-001.
static size_t m34_prg_bank(void) {
    size_t banks = C.prg_sz / PRG_BANK_32K;
    return banks ? (size_t)m34.prg_bank % banks : 0;
}

static uint8_t m34_cpu_read(uint16_t address) {
    if (address >= 0x6000u && address < 0x8000u)
        return prg_ram_read(address);
    if (address >= 0x8000u) {
        size_t bank = m34_prg_bank();
        return C.prg[bank * PRG_BANK_32K + (address & 0x7FFFu)];
    }
    return cart_cpu_bus_input;
}

static void m34_cpu_write(uint16_t address, uint8_t value) {
    if (address >= 0x6000u && address < 0x8000u) {
        prg_ram_write(address, value);
        if (!m34.nina) return;
        switch (address) {
            case 0x7FFD:
                m34.prg_bank = value;
                break;
            case 0x7FFE:
                m34.chr_bank[0] = value;
                m34.chr_mapped[0] = true;
                break;
            case 0x7FFF:
                m34.chr_bank[1] = value;
                m34.chr_mapped[1] = true;
                break;
        }
        return;
    }
    if (!m34.nina && address >= 0x8000u) m34.prg_bank = value;
}

static uint8_t m34_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    if (!m34.nina) return discrete_chr8_read(address, 0);
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    unsigned slot = page_size ? (unsigned)(address / page_size) : 2u;
    if (slot >= 2 || !m34.chr_mapped[slot]) return chr_unmapped_read(address);
    size_t banks = C.chr_sz / page_size;
    size_t bank = m34.chr_bank[slot] % banks;
    return C.chr[bank * page_size + (address % page_size)];
}

static void m34_ppu_write(uint16_t address, uint8_t value) {
    if (!C.chr_is_ram) return;
    address &= 0x1FFFu;
    if (!m34.nina) {
        discrete_chr8_write(address, 0, value);
        return;
    }
    size_t page_size = shrunk_chr_page_size(CHR_BANK_4K);
    unsigned slot = page_size ? (unsigned)(address / page_size) : 2u;
    size_t offset = address % C.chr_sz;
    if (slot < 2 && m34.chr_mapped[slot]) {
        size_t banks = C.chr_sz / page_size;
        size_t bank = m34.chr_bank[slot] % banks;
        offset = bank * page_size + (address % page_size);
    }
    chr_ram_write(offset, value);
}

static Mirroring m34_mirr(void) { return C.mirr_base; }

static void m34_reset(void) {
    bool nina = m34.nina;
    memset(&m34, 0, sizeof(m34));
    m34.nina = nina;
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

static uint8_t sunsoft69_prg_rom_read(uint16_t address, uint16_t window_start,
                                      size_t window_size, uint8_t bank) {
    if (C.prg_sz < PRG_BANK_8K) {
        if (window_start == 0x8000)
            return repeated_prg_window_read(address, 0x8000, PRG_BANK_32K);
        return repeated_prg_window_read(address, window_start, window_size);
    }
    size_t banks = C.prg_sz / PRG_BANK_8K;
    if (!banks) return cart_cpu_bus_input;
    size_t selected = (size_t)(bank & 0x3Fu) % banks;
    return C.prg[selected * PRG_BANK_8K + (address & 0x1FFFu)];
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
        return sunsoft69_prg_rom_read(address, 0x6000, PRG_BANK_8K,
                                     sunsoft69.work_ram_value);
    }
    if (address >= 0x8000) {
        if (C.prg_sz < PRG_BANK_8K)
            return repeated_prg_window_read(address, 0x8000, PRG_BANK_32K);
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
    size_t page_size = C.chr_sz < CHR_BANK_1K ? C.chr_sz : CHR_BANK_1K;
    unsigned slot = page_size ? (unsigned)((address & 0x1FFFu) / page_size) : 0;
    size_t banks = page_size ? C.chr_sz / page_size : 0;
    size_t bank = banks ? (size_t)sunsoft69.chr_bank[slot] % banks : 0;
    return bank * page_size + ((address & 0x1FFFu) % page_size);
}

static uint8_t sunsoft69_ppu_read(uint16_t address) {
    address &= 0x1FFFu;
    size_t page_size = C.chr_sz < CHR_BANK_1K ? C.chr_sz : CHR_BANK_1K;
    unsigned slot = page_size ? (unsigned)(address / page_size) : 8u;
    if (slot >= 8 || !sunsoft69.chr_mapped[slot])
        return C.chr_is_ram ? C.chr[address % C.chr_sz] : (uint8_t)address;
    return C.chr[sunsoft69_chr_index(address)];
}

static void sunsoft69_ppu_write(uint16_t address, uint8_t value) {
    if (!C.chr_is_ram) return;
    address &= 0x1FFFu;
    size_t page_size = C.chr_sz < CHR_BANK_1K ? C.chr_sz : CHR_BANK_1K;
    unsigned slot = page_size ? (unsigned)(address / page_size) : 8u;
    size_t index = slot < 8 && sunsoft69.chr_mapped[slot]
        ? sunsoft69_chr_index(address) : address % C.chr_sz;
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
    if (!page_size) return chr_unmapped_read(a);
    size_t slot = a / page_size;
    if (slot == 0) return C.chr[a % page_size];
    if (slot == 1 && cprom.chr_bank_mapped) {
        size_t bank = cprom.chr_bank % (C.chr_sz / page_size);
        return C.chr[bank * page_size + (a % page_size)];
    }
    return chr_unmapped_read(a);
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
        chr_ram_write(a % C.chr_sz, v);
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
        return 0;
    }
    if (addr < 0x8000) return cart_cpu_bus_input;

    size_t banks = C.prg_sz / PRG_BANK_32K;
    if (!banks) return cart_cpu_bus_input;
    size_t bank = (m111.bank_latch & 0x0Fu) % banks;
    size_t physical = bank * PRG_BANK_32K + (addr & 0x7FFFu);
    int flash_value = m111_flash_read(physical);
    if (flash_value >= 0) return (uint8_t)flash_value;
    return C.prg[physical];
}

static void m111_cpu_write(uint16_t addr, uint8_t value) {
    if (m111_is_register(addr)) {
        m111_latch(value);
        return;
    }
    if (addr < 0x8000) return;
    size_t banks = C.prg_sz / PRG_BANK_32K;
    if (!banks) return;
    size_t bank = (m111.bank_latch & 0x0Fu) % banks;
    size_t physical = bank * PRG_BANK_32K + (addr & 0x7FFFu);
    m111_flash_write(physical, value);
}

static uint8_t m111_ppu_read(uint16_t addr) {
    addr &= 0x1FFFu;
    size_t bank = (m111.bank_latch >> 4) & 1u;
    return C.chr[bank * CHR_BANK_8K + addr];
}

static void m111_ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x1FFFu;
    size_t bank = (m111.bank_latch >> 4) & 1u;
    chr_ram_write(bank * CHR_BANK_8K + addr, value);
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

// Mapper 77: Irem LROG017.
static struct {
    uint8_t prg_bank;
    uint8_t chr_bank;
} irem77;

static uint8_t irem77_cpu_read(uint16_t a) {
    if (a >= 0x6000u && a < 0x8000u) return prg_ram_read(a);
    if (a < 0x8000u) return cart_cpu_bus_input;
    size_t banks = C.prg_sz / PRG_BANK_32K;
    size_t bank = irem77.prg_bank % banks;
    return C.prg[bank * PRG_BANK_32K + (a & 0x7FFFu)];
}

static void irem77_cpu_write(uint16_t a, uint8_t value) {
    if (a >= 0x6000u && a < 0x8000u) {
        prg_ram_write(a, value);
        return;
    }
    if (a < 0x8000u) return;
    irem77.prg_bank = value & 0x0Fu;
    irem77.chr_bank = (value >> 4) & 0x0Fu;
}

static uint8_t irem77_ppu_read(uint16_t a) {
    a &= 0x1FFFu;
    if (a < 0x0800u) {
        size_t banks = C.chr_sz / 0x0800u;
        size_t bank = irem77.chr_bank % banks;
        return C.chr[bank * 0x0800u + a];
    }
    return irem77_chr_ram[a - 0x0800u];
}

static void irem77_ppu_write(uint16_t a, uint8_t value) {
    a &= 0x1FFFu;
    if (a < 0x0800u) return;
    irem77_chr_ram[a - 0x0800u] = value;
}

static Mirroring irem77_mirr(void) { return MIRROR_FOUR; }

static void irem77_power_on(void) {
    memset(&irem77, 0, sizeof(irem77));
    memset(irem77_chr_ram, 0, sizeof(irem77_chr_ram));
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

static size_t vrc7_chr_bank(unsigned slot) {
    if (!vrc7.chr_selected[slot])
        return C.chr_is_ram ? slot % (C.chr_sz / CHR_BANK_1K) : SIZE_MAX;
    return vrc7.chr[slot] % (C.chr_sz / CHR_BANK_1K);
}

static uint8_t vrc7_ppu_read(uint16_t addr) {
    addr &= 0x1FFF;
    unsigned slot = addr >> 10;
    size_t bank = vrc7_chr_bank(slot);
    if (bank == SIZE_MAX) return (uint8_t)addr;
    return C.chr[bank * CHR_BANK_1K + (addr & 0x03FFu)];
}

static void vrc7_ppu_write(uint16_t addr, uint8_t value) {
    if (!C.chr_is_ram) return;
    addr &= 0x1FFF;
    size_t bank = vrc7_chr_bank(addr >> 10);
    if (bank == SIZE_MAX) return;
    chr_ram_write(bank * CHR_BANK_1K + (addr & 0x03FFu), value);
}

static void vrc7_clock(int cpu_cycles) {
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) vrc_irq_clock(&vrc7.irq);
    vrc7_fm_clock(&vrc7.fm, cpu_cycles, nes_timing()->cpu_hz);
}

static Mirroring vrc7_mirr(void) { return vrc7.mirr; }

static float vrc7_expansion_output(void) { return vrc7_fm_output(&vrc7.fm); }

static void vrc7_shutdown(void) { vrc7_fm_destroy(&vrc7.fm); }

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
    size_t offset = bank * PRG_BANK_8K + (addr & 0x1FFFu);
    return offset < C.prg_sz ? C.prg[offset] : cart_cpu_bus_input;
}

static void m99_cpu_write(uint16_t addr, uint8_t value) {
    if (addr >= 0x6000 && addr < 0x8000 && vs_shared_ram_access_allowed())
        prg_ram_write(addr, value);
}

static uint8_t m99_ppu_read(uint16_t addr) {
    addr &= 0x1FFF;
    size_t bank = (vs_dual_system() ? vs_active_side() * 2u : 0u) + vs_prg_chr_select_bit();
    size_t offset = bank * CHR_BANK_8K + addr;
    return offset < C.chr_sz ? C.chr[offset] : (uint8_t)addr;
}

static void m99_ppu_write(uint16_t addr, uint8_t value) {
    (void)addr;
    (void)value;
}

static Mirroring m99_mirr(void) { return C.mirr_base; }

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
                          size_t chr_bytes, bool chr_is_ram,
                          RomRamSizes *ram, unsigned eeprom_sizes[2]) {
    if ((chr_is_ram && chr_bytes != CHR_BANK_8K)
        || (!chr_is_ram && chr_bytes < CHR_BANK_1K)) return false;
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
    if (mapper_no != 1 && mapper_no != 5 && mapper_no != 82 && mapper_no != 155
        && prg_total && (prg_total & (prg_total - 1))) return false;

    bool split_prg = ram->prg_ram && ram->prg_nvram;
    if (split_prg && mapper_no != 1 && mapper_no != 5 && mapper_no != 155) return false;
    if (mapper_no == 1 || mapper_no == 105 || mapper_no == 155) {
        if (prg_total > 0x8000) return false;
    } else if (mapper_no == 5) {
        if (nes2) {
            const size_t supported[] = {0, 0x2000, 0x4000, 0x8000, 0x10000, 0x20000};
            bool work_ok = false, save_ok = false;
            for (size_t i = 0; i < sizeof(supported) / sizeof(supported[0]); ++i) {
                if (ram->prg_ram == supported[i]) work_ok = true;
                if (ram->prg_nvram == supported[i]) save_ok = true;
            }
            if (!work_ok || !save_ok) return false;
        } else if (prg_total > 0x10000) {
            return false; // Legacy bank registers address at most eight 8KB pages.
        }
    } else if (mapper_no == 80 || mapper_no == 207) {
        if (split_prg || prg_total != 0x100) return false;
    } else if (mapper_no == 82) {
        if (split_prg || prg_total != 0x1400) return false;
    } else if (mapper_no == 69) {
        if (split_prg || prg_total > 0x80000) return false;
    } else if (mapper_no == 19 || mapper_no == 210) {
        if (split_prg || prg_total > 0x2000) return false;
    } else if (mapper_no == 90 || mapper_no == 111 || mapper_no == 209 || mapper_no == 211) {
        if (prg_total != 0) return false;
    } else if (mapper_no == 30) {
        if (prg_total != 0) return false;
    } else if (prg_total > 0x2000) {
        return false;
    }

    size_t mixed_chr_ram = mmc3_mixed_chr_expected_ram(mapper_no);
    if (mapper_no == 96) {
        if (!chr_is_ram || chr_sz != 0x8000) return false;
        if (nes2 && (ram->chr_ram != 0x8000 || ram->chr_nvram != 0)) return false;
    } else if (mapper_no == 111) {
        if (!chr_is_ram || chr_sz != 0x4000) return false;
        if (nes2 && (ram->chr_ram != 0x4000 || ram->chr_nvram != 0)) return false;
    } else if (mixed_chr_ram) {
        if (chr_is_ram || chr_sz == 0) return false;
        if (nes2 && (ram->chr_ram != mixed_chr_ram || ram->chr_nvram != 0)) return false;
    } else if (mapper_no == 77) {
        if (ram->chr_nvram) return false;
        if (nes2 && !chr_is_ram && ram->chr_ram != CHR_BANK_8K) return false;
    } else {
        bool unmapped_chr_storage = !chr_is_ram && chr_total
            && (mapper_no == 0 || mapper_no == 1 || mapper_no == 5 || mapper_no == 155);
        if ((!chr_is_ram && chr_total && !unmapped_chr_storage)
            || (chr_is_ram && ram->chr_ram && ram->chr_nvram)) return false;
    }
    if (nes2 && chr_is_ram && chr_total != chr_sz) return false;
    size_t chr_limit;
    switch (mapper_no) {
        case 1: case 9: case 10: case 11: case 89: case 105: case 113: case 144: case 155:
            chr_limit = 0x20000; break;
        case 79: case 146: chr_limit = 0x10000; break;
        case 3: chr_limit = 0x200000; break;
        case 4: case 24: case 26: case 118: chr_limit = 0x40000; break;
        case 33: case 48: case 67: case 68: chr_limit = 0x80000; break;
        case 80: case 82: case 207: chr_limit = 0x40000; break;
        case 72: case 78: case 92: case 140: chr_limit = 0x20000; break;
        case 87: chr_limit = 0x8000; break;
        case 101: chr_limit = 0x200000; break;
        case 64: case 158: chr_limit = 0x40000; break;
        case 5: chr_limit = 0x100000; break;
        case 13: chr_limit = 0x4000; break;
        case 93: chr_limit = CHR_BANK_8K; break;
        case 184: chr_limit = 0x8000; break;
        case 28: case 30: case 96: chr_limit = 0x8000; break;
        case 18: case 32: case 65: chr_limit = 0x40000; break;
        case 73: chr_limit = CHR_BANK_8K; break;
        case 75: case 151: chr_limit = 0x20000; break;
        case 21: case 23: case 25: case 27: case 183: chr_limit = 0x80000; break;
        case 22: chr_limit = 0x40000; break;
        case 19: case 69: case 95: case 206: case 210: chr_limit = 0x40000; break;
        case 76: chr_limit = 0x80000; break;
        case 88: case 154: chr_limit = 0x20000; break;
        case 90: case 209: case 211: chr_limit = 0x200000; break;
        case 111: chr_limit = 0x4000; break;
        case 85: chr_limit = 0x40000; break;
        case 74: case 191: case 194: chr_limit = 0x0800; break;
        case 192: case 195: chr_limit = 0x1000; break;
        case 119: chr_limit = 0x2000; break;
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

static uint8_t fds_mapper_cpu_read(uint16_t addr) { return fds_cpu_read_bus(addr, 0xFF); }
static void fds_mapper_cpu_write(uint16_t addr, uint8_t value) { fds_cpu_write(addr, value); }
static uint8_t fds_mapper_ppu_read(uint16_t addr) { return fds_ppu_read(addr); }
static void fds_mapper_ppu_write(uint16_t addr, uint8_t value) { fds_ppu_write(addr, value); }
static Mirroring fds_mapper_mirroring(void) { return fds_mirroring(); }

int mapper_init_fds(FdsImage *image) {
    if (!image) return -1;
    mapper_shutdown();
    memset(&C, 0, sizeof(C));
    build_mapper(&mapper_fds, fds_mapper_cpu_read, fds_mapper_cpu_write,
                 fds_mapper_ppu_read, fds_mapper_ppu_write, fds_reset, fds_mapper_mirroring);
    mapper_fds.clock = fds_clock_cpu;
    cart = &mapper_fds;
    fds_activate(image);
    fds_reset();
    return 0;
}

static uint8_t board_mapper_cpu_read(uint16_t addr) {
    return board_cpu_read(active_board, addr, cart_cpu_bus_input);
}
static void board_mapper_cpu_write(uint16_t addr, uint8_t value) {
    board_cpu_write(active_board, addr, value);
}
static uint8_t board_mapper_ppu_read(uint16_t addr) {
    return board_ppu_read(active_board, addr, cart_ppu_fetch_source);
}
static void board_mapper_ppu_write(uint16_t addr, uint8_t value) {
    board_ppu_write(active_board, addr, value);
}
static void board_mapper_reset(void) { board_reset(active_board, true); }
static void board_mapper_clock(int cycles) {
    for (int i = 0; i < cycles; ++i) board_clock_cpu(active_board, cart_cpu_cycle_is_write);
}
static Mirroring board_mapper_mirroring(void) { return board_mirroring(active_board); }

static int activate_prepared_board(CartridgeBoard *prepared, uint16_t mapper_no,
                                   uint8_t *prg, size_t prg_sz,
                                   uint8_t *chr, size_t chr_sz) {
    if (!prepared) return -1;
    mapper_shutdown();
    active_board = prepared;
    C.mapper_no = mapper_no;
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz;
    C.mirr_base = board_mirroring(active_board);
    build_mapper(&mapper_board, board_mapper_cpu_read, board_mapper_cpu_write,
                 board_mapper_ppu_read, board_mapper_ppu_write,
                 board_mapper_reset, board_mapper_mirroring);
    mapper_board.clock = board_mapper_clock;
    cart = &mapper_board;
    return C.mapper_no;
}

int mapper_init_studybox(CartridgeBoard *prepared) {
    return activate_prepared_board(prepared, BOARD_STUDYBOX_MAPPER_ID, NULL, 0, NULL, 0) < 0 ? -1 : 0;
}

int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz)
{
    if (h && board_handles_header(h)) {
        CartridgeBoard *prepared = board_create(h, prg, prg_sz, chr, chr_sz);
        if (!prepared) return -1;
        uint16_t mapper_no = board_is_fcns_header(h) ? BOARD_FCNS_MAPPER_ID
                                                     : (uint16_t)rom_mapper_number(h);
        return activate_prepared_board(prepared, mapper_no, prg, prg_sz, chr, chr_sz);
    }
    if (!h || !prg || !prg_sz || !chr || !chr_sz) return -1;
    int mapper_no = rom_mapper_number(h);
    bool nes2 = (h->flags7 & 0x0C) == 0x08;
    uint8_t submapper = nes2 ? h->prg_ram_size >> 4 : 0;
    switch (mapper_no) {
        case 0: case 1: case 2: case 3: case 4: case 5:
        case 7: case 9: case 10: case 11: case 13: case 15: case 28: case 30: case 74: case 111: case 118: case 119: case 155:
        case 16: case 153: case 157: case 159:
        case 18: case 32: case 33: case 34: case 48: case 64: case 65: case 158:
        case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 183:
        case 19: case 66: case 67: case 68: case 69: case 71: case 72: case 73: case 75: case 76: case 77: case 78:
        case 80: case 82: case 85: case 87: case 88: case 89: case 92: case 93: case 95: case 96: case 97: case 99: case 101:
        case 140: case 151: case 154: case 184: case 185: case 206: case 207: case 210:
        case 90: case 105: case 191: case 192: case 194: case 195: case 209: case 211: case 232:
        case 79: case 94: case 113: case 144: case 146: case 180:
            break;
        default:
            fprintf(stderr, "Unsupported mapper: %d\n", mapper_no);
            return -1;
    }
    // Only boards with explicit small-window mapping may accept PRG images
    // below 16 KiB. Other mapper paths still index complete 8/16/32 KiB
    // pages and require their normal minimum until those paths are converted.
    bool small_prg_supported = mapper_no == 0 || mapper_no == 11 || mapper_no == 69
        || mapper_no == 79 || mapper_no == 113 || mapper_no == 144 || mapper_no == 146;
    if (prg_sz < PRG_BANK_16K && !small_prg_supported) {
        fprintf(stderr, "Unsupported PRG size for mapper %d\n", mapper_no);
        return -1;
    }
    bool ignores_submapper = mapper_no == 79 || mapper_no == 94 || mapper_no == 113
        || mapper_no == 144 || mapper_no == 146 || mapper_no == 180;
    if (submapper && !(ignores_submapper
        || (mapper_no == 1 && submapper == 5)
        || vrc24_submapper_supported(mapper_no, submapper)
        || (mapper_no == 85 && submapper <= 2)
        || (mapper_no == 4 && (submapper == 1 || submapper == 3))
        || (mapper_no == 16 && (submapper == 4 || submapper == 5))
        || (mapper_no == 34 && submapper <= 2)
        || (mapper_no == 48 && submapper == 1)
        || (mapper_no == 32 && submapper == 1)
        || (mapper_no == 71 && submapper == 1)
        || (mapper_no == 232 && submapper == 1)
        || (mapper_no == 78 && (submapper == 1 || submapper == 3))
        || (mapper_no == 185 && submapper >= 4 && submapper <= 7)
        || (mapper_no == 206 && submapper == 1)
        || (mapper_no == 210 && submapper <= 2)
        || ((mapper_no == 2 || mapper_no == 3 || mapper_no == 7) && submapper <= 2)
        || (mapper_no == 30 && submapper <= 4))) {
        fprintf(stderr, "Unsupported mapper/submapper: %d/%u\n", mapper_no, submapper);
        return -1;
    }
    RomRamSizes ram;
    rom_ram_sizes(h, &ram);
    bool is_jy = mapper_no == 90 || mapper_no == 209 || mapper_no == 211;
    if (is_jy && !nes2) ram.prg_ram = ram.prg_nvram = 0;
    if (mapper_no == 99 && !nes2 && !(h->flags6 & 2)) {
        ram.prg_ram = 0x800;
    }
    if ((ram.prg_nvram || ram.chr_nvram) && !(h->flags6 & 2)) {
        fprintf(stderr, "Nonvolatile RAM declared without the battery flag\n");
        return -1;
    }
    if (mapper_no == 80 || mapper_no == 82 || mapper_no == 207) {
        size_t expected = mapper_no == 82 ? 0x1400u : 0x100u;
        size_t declared = ram.prg_ram + ram.prg_nvram;
        if (nes2 && ((ram.prg_ram && ram.prg_nvram) || (declared && declared != expected))) {
            fprintf(stderr, "Unsupported RAM layout for mapper %d\n", mapper_no);
            return -1;
        }
        ram.prg_ram = (h->flags6 & 2) ? 0 : expected;
        ram.prg_nvram = (h->flags6 & 2) ? expected : 0;
    }
    bool chr_is_ram = h->chr_rom_chunks == 0 && (!nes2 || (h->flags9 & 0xF0) == 0);
    size_t chr_page_size = mapper_chr_page_size((uint16_t)mapper_no);
    if (!chr_is_ram && chr_page_size && chr_sz < chr_page_size
        && !mapper_has_shrinking_chr_window((uint16_t)mapper_no)) {
        fprintf(stderr, "Unsupported CHR-ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    bool mapper34_nina = mapper_no == 34
        && (submapper == 1 || (submapper == 0 && !chr_is_ram));
    unsigned eeprom_sizes[2] = {0, 0};
    bool is_bandai = mapper_no == 16 || mapper_no == 153 || mapper_no == 157 || mapper_no == 159;
    if (is_bandai && !bandai_layout(mapper_no, submapper, nes2, chr_sz,
                                    chr_is_ram, &ram, eeprom_sizes)) {
        fprintf(stderr, "Unsupported cartridge layout for mapper %d\n", mapper_no);
        return -1;
    }
    if ((mapper_no == 33 || mapper_no == 48)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if ((mapper_no == 80 || mapper_no == 82 || mapper_no == 207)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 77
        && (chr_is_ram || chr_sz < CHR_BANK_2K
            || ram.chr_nvram || (nes2 && ram.chr_ram != CHR_BANK_8K))) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 77\n");
        return -1;
    }
    if (mapper_no == 185 && chr_is_ram) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 185\n");
        return -1;
    }
    if ((mapper_no == 64 || mapper_no == 158)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 118 && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper 118\n");
        return -1;
    }
    if (mmc3_mixed_chr_expected_ram(mapper_no)
        && (chr_is_ram || chr_sz < CHR_BANK_1K)) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 28
        && (!chr_is_ram || (chr_sz != 0x2000 && chr_sz != 0x4000 && chr_sz != 0x8000))) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 28\n");
        return -1;
    }
    if (mapper_no == 30 && (prg_sz > 0x80000 || (prg_sz & (prg_sz - 1)) != 0
        || !chr_is_ram || (chr_sz != 0x2000 && chr_sz != 0x4000 && chr_sz != 0x8000)
        || ((h->flags6 & 0x09) == 0x09 && submapper != 3 && chr_sz != 0x8000))) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 30\n");
        return -1;
    }
    if (mapper_no == 18 && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper 18\n");
        return -1;
    }
    if ((mapper_no == 32 || mapper_no == 65)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (vrc24_submapper_supported(mapper_no, submapper)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if ((mapper_no == 19 || mapper_no == 210)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if ((mapper_no == 24 || mapper_no == 26)
        && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 68 && (h->flags6 & 0x08)) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 68\n");
        return -1;
    }
    if (mapper_no == 85 && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM size for mapper 85\n");
        return -1;
    }
    if (mapper_no == 96 && (prg_sz != 0x20000 || !chr_is_ram || chr_sz != 0x8000)) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 96\n");
        return -1;
    }
    if (mapper_no == 206 && submapper == 1 && prg_sz != PRG_BANK_32K) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 206\n");
        return -1;
    }
    if (mapper_no == 99 && (chr_is_ram
        || (prg_sz != 0x8000 && prg_sz != 0xA000 && prg_sz != 0xC000 && prg_sz != 0x10000)
        || (chr_sz != 0x2000 && chr_sz != 0x4000 && chr_sz != 0x8000))) {
        fprintf(stderr, "Unsupported ROM size for mapper 99\n");
        return -1;
    }
    if (is_jy && chr_sz < CHR_BANK_1K) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper %d\n", mapper_no);
        return -1;
    }
    if (mapper_no == 105
        && chr_sz < CHR_BANK_4K) {
        fprintf(stderr, "Unsupported ROM size for mapper 105\n");
        return -1;
    }
    if (mapper_no == 111
        && (prg_sz > 0x80000 || (prg_sz % PRG_BANK_32K) != 0
            || !chr_is_ram || chr_sz != 0x4000)) {
        fprintf(stderr, "Unsupported ROM/RAM size for mapper 111\n");
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
    RamBlock new_chr_work = {NULL, !chr_is_ram ? ram.chr_ram : 0};
    RamBlock new_chr_save = {NULL, !chr_is_ram ? ram.chr_nvram : 0};
    if (new_work.size) new_work.data = (uint8_t *)calloc(1, new_work.size);
    if (new_save.size) new_save.data = (uint8_t *)calloc(1, new_save.size);
    if (new_chr_work.size) new_chr_work.data = (uint8_t *)calloc(1, new_chr_work.size);
    if (new_chr_save.size) new_chr_save.data = (uint8_t *)calloc(1, new_chr_save.size);
    if ((new_work.size && !new_work.data) || (new_save.size && !new_save.data)
        || (new_chr_work.size && !new_chr_work.data)
        || (new_chr_save.size && !new_chr_save.data)) {
        free(new_work.data);
        free(new_save.data);
        free(new_chr_work.data);
        free(new_chr_save.data);
        fprintf(stderr, "Cartridge RAM allocation failed\n");
        return -1;
    }

    Vrc7Fm new_fm = {0};
    if (mapper_no == 85 && !vrc7_fm_init(&new_fm)) {
        free(new_work.data);
        free(new_save.data);
        free(new_chr_work.data);
        free(new_chr_save.data);
        fprintf(stderr, "VRC7 audio allocation failed\n");
        return -1;
    }

    nes_initialize_power_on_ram(new_work.data, new_work.size, 0);
    nes_initialize_power_on_ram(new_save.data, new_save.size, 0);
    nes_initialize_power_on_ram(new_chr_work.data, new_chr_work.size, 0);
    nes_initialize_power_on_ram(new_chr_save.data, new_chr_save.size, 0);

    // Finish all fallible setup before releasing the previous cartridge's RAM.
    mapper_shutdown();
    prg_work_ram = new_work;
    prg_save_ram = new_save;
    chr_work_ram = new_chr_work;
    chr_save_ram = new_chr_save;
    C.prg = prg; C.prg_sz = prg_sz;
    C.chr = chr; C.chr_sz = chr_sz;
    C.chr_is_ram = chr_is_ram;
    C.mapper_no = (uint16_t)mapper_no;
    C.ram = ram;
    C.nes2 = nes2;
    C.submapper = submapper;
    C.mmc1a = mapper_no == 155;
    C.bus_conflicts = mapper_no == 11 || mapper_no == 144
        || mapper_no == 72 || mapper_no == 78 || mapper_no == 92
        || mapper_no == 77 || mapper_no == 96 || mapper_no == 185
        || (mapper_no == 34 && !mapper34_nina)
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
        case 2: case 94: case 180:
            build_mapper(&mapper_uxrom, uxrom_cpu_read, uxrom_cpu_write,
                        uxrom_ppu_read, uxrom_ppu_write, uxrom_reset, uxrom_mirr);
            cart = &mapper_uxrom;
            break;
        case 3:
            build_mapper(&mapper_cnrom, cnrom_cpu_read, cnrom_cpu_write,
                        cnrom_ppu_read, cnrom_ppu_write, cnrom_reset, cnrom_mirr);
            cart = &mapper_cnrom;
            break;
        case 185:
            build_mapper(&mapper_cnrom_protect, cnrom185_cpu_read, cnrom185_cpu_write,
                         cnrom185_ppu_read, cnrom185_ppu_write,
                         cnrom185_reset, cnrom185_mirr);
            cart = &mapper_cnrom_protect;
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
        case 11: case 144:
            build_mapper(&mapper_colordreams, colordreams_cpu_read, colordreams_cpu_write,
                        colordreams_ppu_read, colordreams_ppu_write, colordreams_reset, colordreams_mirr);
            cart = &mapper_colordreams;
            break;
        case 79: case 113: case 146:
            build_mapper(&mapper_nina, nina_cpu_read, nina_cpu_write,
                         nina_ppu_read, nina_ppu_write, nina_reset, nina_mirr);
            cart = &mapper_nina;
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
        case 24: case 26:
            memset(&vrc6, 0, sizeof(vrc6));
            vrc6.variant_b = mapper_no == 26;
            build_mapper(&mapper_vrc6, vrc6_cpu_read, vrc6_cpu_write,
                        vrc6_ppu_read, vrc6_ppu_write, vrc6_reset, vrc6_mirr);
            mapper_vrc6.clock = vrc6_clock;
            cart = &mapper_vrc6;
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
        case 34:
            m34.nina = mapper34_nina;
            build_mapper(&mapper_m34, m34_cpu_read, m34_cpu_write,
                         m34_ppu_read, m34_ppu_write, m34_reset, m34_mirr);
            cart = &mapper_m34;
            break;
        case 66:
            build_mapper(&mapper_gxrom, gxrom_cpu_read, gxrom_cpu_write,
                         gxrom_ppu_read, gxrom_ppu_write, gxrom_reset, gxrom_mirr);
            cart = &mapper_gxrom;
            break;
        case 71:
            m71.force_bf9097 = submapper == 1;
            build_mapper(&mapper_m71, m71_cpu_read, m71_cpu_write,
                         m71_ppu_read, m71_ppu_write, m71_reset, m71_mirr);
            cart = &mapper_m71;
            break;
        case 72: case 78: case 87: case 92: case 101: case 140:
            build_mapper(&mapper_jaleco_discrete, jaleco_discrete_cpu_read, jaleco_discrete_cpu_write,
                         jaleco_discrete_ppu_read, jaleco_discrete_ppu_write,
                         jaleco_discrete_reset, jaleco_discrete_mirr);
            cart = &mapper_jaleco_discrete;
            break;
        case 67:
            build_mapper(&mapper_sunsoft3, sunsoft3_cpu_read, sunsoft3_cpu_write,
                         sunsoft3_ppu_read, sunsoft3_ppu_write, sunsoft3_reset, sunsoft3_mirr);
            mapper_sunsoft3.clock = sunsoft3_clock;
            cart = &mapper_sunsoft3;
            sunsoft3_reset();
            break;
        case 68:
            build_mapper(&mapper_sunsoft4, sunsoft4_cpu_read, sunsoft4_cpu_write,
                         sunsoft4_ppu_read, sunsoft4_ppu_write, sunsoft4_reset, sunsoft4_mirr);
            mapper_sunsoft4.clock = sunsoft4_clock;
            cart = &mapper_sunsoft4;
            sunsoft4_reset();
            break;
        case 89:
            build_mapper(&mapper_sunsoft89, sunsoft89_cpu_read, sunsoft89_cpu_write,
                         sunsoft89_ppu_read, sunsoft89_ppu_write, sunsoft89_reset, sunsoft89_mirr);
            cart = &mapper_sunsoft89;
            sunsoft89_reset();
            break;
        case 93:
            build_mapper(&mapper_sunsoft93, sunsoft93_cpu_read, sunsoft93_cpu_write,
                         sunsoft93_ppu_read, sunsoft93_ppu_write, sunsoft93_reset, sunsoft93_mirr);
            cart = &mapper_sunsoft93;
            sunsoft93_reset();
            break;
        case 184:
            build_mapper(&mapper_sunsoft184, sunsoft184_cpu_read, sunsoft184_cpu_write,
                         sunsoft184_ppu_read, sunsoft184_ppu_write, sunsoft184_reset, sunsoft184_mirr);
            cart = &mapper_sunsoft184;
            sunsoft184_reset();
            break;
        case 73:
            build_mapper(&mapper_vrc3, vrc3_cpu_read, vrc3_cpu_write,
                         vrc3_ppu_read, vrc3_ppu_write, vrc3_reset, vrc3_mirr);
            mapper_vrc3.clock = vrc3_clock;
            cart = &mapper_vrc3;
            vrc3_reset();
            break;
        case 75: case 151:
            build_mapper(&mapper_vrc1, vrc1_cpu_read, vrc1_cpu_write,
                         vrc1_ppu_read, vrc1_ppu_write, vrc1_reset, vrc1_mirr);
            cart = &mapper_vrc1;
            vrc1_reset();
            break;
        case 76: case 88: case 95: case 154: case 206:
            namco108.fixed_prg = mapper_no == 206 && submapper == 1;
            build_mapper(&mapper_namco108, namco108_cpu_read, namco108_cpu_write,
                         namco108_ppu_read, namco108_ppu_write, namco108_reset, namco108_mirr);
            cart = &mapper_namco108;
            namco108_reset();
            break;
        case 232:
            build_mapper(&mapper_m232, m232_cpu_read, m232_cpu_write,
                         m232_ppu_read, m232_ppu_write, m232_reset, m232_mirr);
            cart = &mapper_m232;
            break;
        case 48:
            build_mapper(&mapper_taito48, taito48_cpu_read, taito48_cpu_write,
                        taito48_ppu_read, taito48_ppu_write, taito48_reset, taito48_mirr);
            mapper_taito48.clock = taito48_clock;
            cart = &mapper_taito48;
            break;
        case 80: case 207:
            build_mapper(&mapper_taito_x1005, taito_x1005_cpu_read, taito_x1005_cpu_write,
                         taito_x1005_ppu_read, taito_x1005_ppu_write,
                         taito_x1005_reset, taito_x1005_mirr);
            cart = &mapper_taito_x1005;
            taito_x1005_reset();
            break;
        case 82:
            build_mapper(&mapper_taito_x1017, taito_x1017_cpu_read, taito_x1017_cpu_write,
                         taito_x1017_ppu_read, taito_x1017_ppu_write,
                         taito_x1017_reset, taito_x1017_mirr);
            cart = &mapper_taito_x1017;
            taito_x1017_reset();
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
        case 77:
            build_mapper(&mapper_irem77, irem77_cpu_read, irem77_cpu_write,
                         irem77_ppu_read, irem77_ppu_write, NULL, irem77_mirr);
            cart = &mapper_irem77;
            irem77_power_on();
            break;
        case 97:
            build_mapper(&mapper_irem97, irem97_cpu_read, irem97_cpu_write,
                         irem97_ppu_read, irem97_ppu_write, NULL, irem97_mirr);
            cart = &mapper_irem97;
            irem97_power_on();
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
        case 19: case 210:
            if (mapper_no == 19) namco.variant = NAMCO_VARIANT_163;
            else if (submapper == 1) namco.variant = NAMCO_VARIANT_175;
            else if (submapper == 2) namco.variant = NAMCO_VARIANT_340;
            else namco.variant = NAMCO_VARIANT_UNKNOWN;
            namco.auto_detect = (mapper_no == 210 && submapper == 0)
                             || (mapper_no == 19 && !nes2);
            build_mapper(&mapper_namco, namco_cpu_read, namco_cpu_write,
                         namco_ppu_read, namco_ppu_write, namco_reset, namco_mirr);
            mapper_namco.clock = namco_clock;
            cart = &mapper_namco;
            break;
        case 85:
            vrc7.fm = new_fm;
            build_mapper(&mapper_vrc7, vrc7_cpu_read, vrc7_cpu_write,
                        vrc7_ppu_read, vrc7_ppu_write, vrc7_reset, vrc7_mirr);
            mapper_vrc7.clock = vrc7_clock;
            cart = &mapper_vrc7;
            break;
        case 96:
            build_mapper(&mapper_m96, m96_cpu_read, m96_cpu_write,
                         m96_ppu_read, m96_ppu_write, m96_reset, m96_mirr);
            cart = &mapper_m96;
            break;
        case 90: case 209: case 211:
            build_mapper(&mapper_jy, jy_cpu_read, jy_cpu_write,
                         jy_ppu_read, jy_ppu_write, jy_reset, jy_mirr);
            mapper_jy.clock = jy_clock;
            cart = &mapper_jy;
            break;
        case 99:
            build_mapper(&mapper_vs99, m99_cpu_read, m99_cpu_write,
                         m99_ppu_read, m99_ppu_write, NULL, m99_mirr);
            cart = &mapper_vs99;
            break;
        case 105:
            build_mapper(&mapper_m105, m105_cpu_read, m105_cpu_write,
                         m105_ppu_read, nrom_ppu_write, m105_reset, mmc1_mirr);
            mapper_m105.clock = m105_clock;
            cart = &mapper_m105;
            break;
        case 111:
            build_mapper(&mapper_m111, m111_cpu_read, m111_cpu_write,
                         m111_ppu_read, m111_ppu_write, m111_reset, m111_mirr);
            cart = &mapper_m111;
            break;
        case 74: case 119: case 191: case 192: case 194: case 195:
            if (!mmc3_mixed_chr_configure(mapper_no)) return -1;
            build_mapper(&mapper_tqrom, mmc3_cpu_read, mmc3_cpu_write,
                        mmc3_mixed_chr_ppu_read, mmc3_mixed_chr_ppu_write,
                        mmc3_reset, mmc3_mirr);
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
