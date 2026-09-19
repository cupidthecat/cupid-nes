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
#include "nsf.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
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
static Mapper mapper_cnrom, mapper_cnrom_protect, mapper_mmc3, mapper_txsrom;
static Mapper mapper_mmc5, mapper_aorom, mapper_mmc2, mapper_mmc4, mapper_colordreams;
static Mapper mapper_cprom, mapper_100in1, mapper_bandai, mapper_action53, mapper_unrom512, mapper_m111, mapper_fds;
static Mapper mapper_taito33, mapper_taito48;
static Mapper mapper_jaleco18, mapper_jaleco_discrete, mapper_irem32, mapper_irem65, mapper_irem97;
static Mapper mapper_rambo1, mapper_rambo158;
static Mapper mapper_vrc1, mapper_vrc3, mapper_vrc6, mapper_vrc24, mapper_vrc7;
static Mapper mapper_sunsoft3, mapper_sunsoft4, mapper_sunsoft89, mapper_sunsoft93, mapper_sunsoft184;
static Mapper mapper_namco, mapper_gxrom, mapper_m71, mapper_namco108;
static Mapper mapper_vs99, mapper_jy, mapper_nina;
static Mapper mapper_board, mapper_nsf;
static CartridgeBoard *active_board = NULL;
Mapper *cart = NULL;
static bool mapper_irq_line = false;
static uint8_t cart_cpu_bus_input = 0xFF;
static CartPpuFetchSource cart_ppu_fetch_source = CART_PPU_FETCH_CPU;
static bool mmc3_revision_a_profile = false;
static unsigned cart_dip_value = 0;
static bool cart_cpu_cycle_is_write = false;
static void mmc3_irq_clock(void);
static float vrc7_expansion_output(void);
static void vrc7_shutdown(void);
static void nsf_reset(bool soft_reset);
static void nsf_after_reset(void);
static void build_mapper(Mapper *m,
    uint8_t(*cr)(uint16_t), void(*cw)(uint16_t,uint8_t),
    uint8_t(*pr)(uint16_t), void(*pw)(uint16_t,uint8_t),
    void(*rst)(void), Mirroring(*gm)(void));

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

static struct {
    NsfMetadata metadata;
    uint8_t bios[0x20];
    uint8_t banks[10];
    bool lower_program[2];
    bool explicit_banking;
    uint8_t song;
    uint32_t play_counter;
    uint64_t track_start_cycle;
    uint8_t mmc5_multiplier[2];
} nsf_player;
static Vrc7Fm nsf_vrc7;

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
    Mirroring mirr;
} namco;

enum {
    NAMCO_PPU_NONE = 0,
    NAMCO_PPU_CHR,
    NAMCO_PPU_CIRAM
};
static uint8_t namco_ppu_source[0x30];
static size_t namco_ppu_offset[0x30];

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

static size_t prg_ram_window_coverage(const RamBlock *ram) {
    if (!ram || !ram->size) return 0;
    size_t page = ram->size < PRG_BANK_8K ? ram->size : PRG_BANK_8K;
    if (page & 0xFFu) return 0;
    // The fixed window contains complete physical pages. A trailing partial
    // copy remains open bus, and RAM smaller than one bus page is unmapped.
    return (PRG_BANK_8K / page) * page;
}

static inline uint8_t prg_ram_read(uint16_t addr) {
    RamBlock *ram = default_prg_ram();
    size_t offset = addr - 0x6000u;
    return offset < prg_ram_window_coverage(ram) ? ram_read(ram, offset) : cart_cpu_bus_input;
}

static inline void prg_ram_write(uint16_t addr, uint8_t value) {
    RamBlock *ram = default_prg_ram();
    size_t offset = addr - 0x6000u;
    if (offset < prg_ram_window_coverage(ram)) ram_write(ram, offset, value);
}

static uint8_t prg_ram_internal_read(uint16_t addr) {
    if (addr < 0x6000u || addr > 0x7FFFu) return 0;
    RamBlock *ram = default_prg_ram();
    size_t offset = addr - 0x6000u;
    if (offset >= prg_ram_window_coverage(ram)) return 0;
    return ram->data[offset % ram->size];
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
    if (cart_has_flash_storage()) {
        flush_flash_battery();
        flush_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size, &prg_ram_dirty);
    } else if (cart == &mapper_mmc5) flush_mmc5_battery();
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
    if (cart_has_flash_storage()) {
        load_flash_battery(flash_save_path);
        load_battery(battery_save_path, prg_save_ram.data, prg_save_ram.size);
    } else if (cart == &mapper_mmc5) load_mmc5_battery(battery_save_path);
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
    if (cart == &mapper_nsf && (nsf_player.metadata.sound_chips & NSF_SOUND_VRC7))
        vrc7_fm_destroy(&nsf_vrc7);
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
        case 32: case 33: case 48: case 64: case 65:
        case 75: case 76: case 80: case 82: case 85: case 88: case 90:
        case 95: case 99: case 118: case 151: case 154: case 158:
        case 183: case 206: case 207:
        case 209: case 210: case 211:
            return PRG_BANK_8K;
        case 0: case 1: case 2: case 10: case 16: case 28: case 30:
        case 67: case 68: case 71: case 72: case 73: case 78: case 89:
        case 92: case 93: case 94: case 97: case 105: case 153: case 155:
        case 157: case 159: case 180: case 232:
            return PRG_BANK_16K;
        case 3: case 7: case 11: case 13: case 66:
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
        case 65: case 80: case 82: case 85: case 88: case 90:
        case 95: case 118: case 153: case 154: case 157: case 158:
        case 159: case 183: case 206:
        case 207: case 209: case 210: case 211:
            return CHR_BANK_1K;
        case 67: case 68: case 76:
            return CHR_BANK_2K;
        case 1: case 9: case 10: case 13: case 75: case 96:
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
        case 0: case 1: case 2: case 3: case 4: case 7: case 9: case 10: case 11:
        case 13: case 15: case 16: case 18: case 19: case 21: case 22: case 23:
        case 24:
        case 25: case 26: case 27: case 28: case 30:
        case 32: case 33: case 48: case 64: case 65: case 66: case 67:
        case 68: case 71: case 72: case 73: case 75: case 76:
        case 78:
        case 79: case 80: case 82: case 85: case 87: case 88: case 89: case 90:
        case 92:
        case 93: case 94: case 95: case 96: case 97: case 99: case 101: case 105:
        case 111: case 113: case 118:
        case 140: case 144: case 146: case 151: case 153: case 154: case 155:
        case 157: case 158: case 159: case 180: case 183: case 184: case 185:
        case 206: case 207: case 209:
        case 210: case 211: case 232:
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

uint8_t *cart_cpu_ram_8k(void) {
    return board_cpu_ram_8k(active_board);
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
    if (cart == &mapper_fds) fds_irq_ack();
}
void cart_console_reset(bool soft_reset) {
    if (cart == &mapper_nsf) nsf_reset(soft_reset);
    else board_reset(active_board, soft_reset);
}
void cart_after_console_reset(void) {
    if (cart == &mapper_nsf) nsf_after_reset();
    else board_after_reset(active_board);
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

#include "mapper_nintendo.inc"
#include "mapper_konami_sunsoft.inc"
#include "mapper_mmc3.inc"
#include "mapper_mmc5.inc"
#include "mapper_expansion.inc"
#include "mapper_discrete.inc"
#include "mapper_jaleco_irem.inc"
#include "mapper_factory.inc"
