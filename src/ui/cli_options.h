/*
 * cli_options.h - Shared application option parser and desktop reference
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_CLI_OPTIONS_H
#define CUPID_CLI_OPTIONS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
    FRONTEND_CLI_UNKNOWN = -1,
    FRONTEND_CLI_MOVIE,
    FRONTEND_CLI_TAS,
    FRONTEND_CLI_REGION,
    FRONTEND_CLI_CONSOLE,
    FRONTEND_CLI_CPU_REVISION,
    FRONTEND_CLI_CPU_TEST_MODE,
    FRONTEND_CLI_APU_DISABLE_NOISE_MODE,
    FRONTEND_CLI_APU_SWAP_DUTY_CYCLES,
    FRONTEND_CLI_EPSM_ADPCM,
    FRONTEND_CLI_FCNS_KANJI,
    FRONTEND_CLI_GAME_DB,
    FRONTEND_CLI_DATA_DIR,
    FRONTEND_CLI_NO_GAME_DB_OVERRIDES,
    FRONTEND_CLI_STARTUP_PHASE,
    FRONTEND_CLI_STARTUP_SEED,
    FRONTEND_CLI_RAM_POWER_ON,
    FRONTEND_CLI_POWER_ON_SEED,
    FRONTEND_CLI_RANDOM_VBLANK,
    FRONTEND_CLI_PPU_REVISION,
    FRONTEND_CLI_PPU_OAM_ROW_CORRUPTION,
    FRONTEND_CLI_PPU_STARTUP_RESTRICTION,
    FRONTEND_CLI_PPU_OAM_DECAY,
    FRONTEND_CLI_PPU_SPRITE_EVAL_WRAP_BUG,
    FRONTEND_CLI_PPU_DISABLE_OAMDATA_READ,
    FRONTEND_CLI_PPU_DISABLE_PALETTE_READBACK,
    FRONTEND_CLI_PPU_RESET_SUPPRESSION,
    FRONTEND_CLI_VIDEO_FILTER,
    FRONTEND_CLI_MMC3_REVISION,
    FRONTEND_CLI_CART_DIP,
    FRONTEND_CLI_ADAPTER,
    FRONTEND_CLI_PORT1,
    FRONTEND_CLI_PORT2,
    FRONTEND_CLI_EXPANSION,
    FRONTEND_CLI_ZAPPER_RADIUS,
    FRONTEND_CLI_VS_DIP,
    FRONTEND_CLI_BARCODE,
    FRONTEND_CLI_BARCODE_BATTLER,
    FRONTEND_CLI_TAPE_PLAY,
    FRONTEND_CLI_TAPE_RECORD,
    FRONTEND_CLI_FDS_BIOS,
    FRONTEND_CLI_STUDYBOX_BIOS,
    FRONTEND_CLI_FDS_SIDE,
    FRONTEND_CLI_FDS_EJECT,
    FRONTEND_CLI_FDS_WRITE_PROTECT,
    FRONTEND_CLI_HELP,
    FRONTEND_CLI_COUNT
} FrontendCliId;

typedef struct {
    FrontendCliId id;
    const char *name, *alias, *values, *default_value, *description;
    bool takes_value, choices;
} FrontendCliOption;

typedef struct {
    const char *rom_path;
    const char *movie_path;
    bool open_tas_editor;
    bool movie_launch_failed;
    const char *barcode;
    const char *barcode_battler;
    const char *tape_play_path;
    const char *tape_record_path;
    const char *fds_bios_path;
    const char *studybox_bios_path;
    size_t fds_frontend_side;
    bool fds_side_set;
    bool fds_start_ejected;
    bool fds_start_write_protected;
    bool fds_write_protect_cli;
    bool vs_dip_set;
    uint16_t vs_dips;
    uint8_t input_overrides;
    bool startup_phase_set;
    bool startup_seed_set;
    unsigned startup_cpu_offset;
    unsigned startup_ppu_phase;
    uint32_t startup_seed;
    bool power_on_seed_set;
    uint32_t power_on_seed;
    const char *epsm_adpcm_path;
    const char *fcns_kanji_path;
    const char *game_db_path;
    const char *data_dir_override;
    bool disable_game_db_overrides;
    bool ntsc_composite_requested;
    uint32_t frontend_cli_overrides;
    bool help;
} FrontendCliOptions;

size_t frontend_cli_count(void);
const FrontendCliOption *frontend_cli_at(size_t index);
FrontendCliId frontend_cli_find(const char *text);
bool frontend_cli_accepts(const FrontendCliOption *option, const char *value);
bool frontend_cli_parse(int argc, char **argv, FrontendCliOptions *options, char *error, size_t error_size);
void frontend_cli_print(FILE *stream);
#endif
