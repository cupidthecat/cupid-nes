/*
 * cli_parse.c - Shared application option parser and desktop reference
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cli_options.h"
#include "settings.h"
#include "../cpu/cpu.h"
#include "../rom/mapper.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

bool frontend_cli_parse(int argc, char **argv, FrontendCliOptions *options, char *error, size_t error_size) {
    if (!options || argc < 1 || !argv || !error || !error_size) {
        return false;
    }
    error[0] = 0;
    const char *rom_path = NULL;
    const char *movie_path = NULL;
    bool open_tas_editor = false;
    bool movie_launch_failed = false;
    const char *barcode = NULL;
    const char *barcode_battler = NULL;
    const char *tape_play_path = NULL;
    const char *tape_record_path = NULL;
    const char *fds_bios_path = NULL;
    const char *studybox_bios_path = NULL;
    size_t fds_frontend_side = 0;
    bool fds_side_set = false;
    bool fds_start_ejected = false;
    bool fds_start_write_protected = false;
    bool fds_write_protect_cli = false;
    bool vs_dip_set = false;
    uint16_t vs_dips = 0;
    uint8_t input_overrides = 0;
    bool startup_phase_set = false;
    bool startup_seed_set = false;
    unsigned startup_cpu_offset = 0, startup_ppu_phase = 0;
    uint32_t startup_seed = 0;
    bool power_on_seed_set = false;
    uint32_t power_on_seed = 0;
    const char *epsm_adpcm_path = NULL;
    const char *fcns_kanji_path = NULL;
    const char *game_db_path = NULL;
    const char *data_dir_override = NULL;
    bool disable_game_db_overrides = false;
    bool ntsc_composite_requested = false;
    uint32_t frontend_cli_overrides = 0;
    bool help = false;
    for (int i = 1; i < argc; ++i) {
        FrontendCliId id = frontend_cli_find(argv[i]);
        if (id == FRONTEND_CLI_HELP) {
            help = true;
            continue;
        }
        if (id != FRONTEND_CLI_UNKNOWN) {
            const FrontendCliOption *definition = frontend_cli_at((size_t)id);
            if (!frontend_cli_accepts(definition, definition->takes_value && i + 1 < argc ? argv[i + 1] : NULL)) {
                if (error && error_size) {
                    if (id == FRONTEND_CLI_REGION)
                        snprintf(error, error_size, "Region must be auto, ntsc, pal, or dendy\n");
                    else if (id == FRONTEND_CLI_CONSOLE)
                        snprintf(error, error_size, "Console must be nes-001, nes-101, famicom, or av-famicom\n");
                    else
                        snprintf(error, error_size, "%s expects %s", definition->name, definition->values);
                }
                return false;
            }
        }
        if (id == FRONTEND_CLI_MOVIE || id == FRONTEND_CLI_TAS) {
            open_tas_editor = id == FRONTEND_CLI_TAS;
            if (++i == argc || movie_path || !*argv[i]) {
                snprintf(error, error_size, "Choose one movie file with --movie or --tas\n");
                return false;
            }
            movie_path = argv[i];
        } else if (id == FRONTEND_CLI_REGION) {
            if (++i == argc || !nes_set_region_mode_name(argv[i])) {
                snprintf(error, error_size, "Region must be auto, ntsc, pal, or dendy\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_REGION;
        } else if (id == FRONTEND_CLI_CONSOLE) {
            if (++i == argc || !nes_set_console_model_name(argv[i])) {
                snprintf(error, error_size, "Console must be nes-001, nes-101, famicom, or av-famicom\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_CONSOLE;
        } else if (id == FRONTEND_CLI_CPU_REVISION) {
            if (++i == argc) {
                snprintf(error, error_size, "CPU revision must be early-2a03 or late-2a03\n");
                return false;
            }
            if (strcmp(argv[i], "early-2a03") == 0) {
                apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03);
            } else if (strcmp(argv[i], "late-2a03") == 0) {
                apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03);
            } else {
                snprintf(error, error_size, "CPU revision must be early-2a03 or late-2a03\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_CPU_REVISION;
        } else if (id == FRONTEND_CLI_CPU_TEST_MODE) {
            cpu_set_test_mode(true);
        } else if (id == FRONTEND_CLI_APU_DISABLE_NOISE_MODE) {
            apu_set_disable_noise_mode(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_APU_NOISE_MODE;
        } else if (id == FRONTEND_CLI_APU_SWAP_DUTY_CYCLES) {
            apu_set_swap_duty_cycles(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_APU_DUTY;
        } else if (id == FRONTEND_CLI_EPSM_ADPCM) {
            if (++i == argc) {
                snprintf(error, error_size, "--epsm-adpcm requires an 8 KiB YMF288 ADPCM ROM file\n");
                return false;
            }
            epsm_adpcm_path = argv[i];
        } else if (id == FRONTEND_CLI_FCNS_KANJI) {
            if (++i == argc) {
                snprintf(error, error_size, "--fcns-kanji requires a 256 KiB Kanji ROM file\n");
                return false;
            }
            fcns_kanji_path = argv[i];
        } else if (id == FRONTEND_CLI_GAME_DB) {
            if (++i == argc) {
                snprintf(error, error_size, "--game-db requires a database file\n");
                return false;
            }
            game_db_path = argv[i];
            frontend_cli_overrides |= FRONTEND_OVERRIDE_DATABASE;
        } else if (id == FRONTEND_CLI_DATA_DIR) {
            if (++i == argc || !argv[i][0]) {
                snprintf(error, error_size, "--data-dir requires a directory\n");
                return false;
            }
            data_dir_override = argv[i];
        } else if (id == FRONTEND_CLI_NO_GAME_DB_OVERRIDES) {
            disable_game_db_overrides = true;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_DATABASE_CORRECTIONS;
        } else if (id == FRONTEND_CLI_STARTUP_PHASE) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                snprintf(error, error_size, "Choose one startup phase CPU:PPU or startup seed\n");
                return false;
            }
            char *end;
            errno = 0;
            unsigned long cpu_offset = strtoul(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end != ':' || cpu_offset > 15) {
                snprintf(error, error_size, "Startup phase must be CPU:PPU in regional master clocks\n");
                return false;
            }
            const char *ppu_text = end + 1;
            errno = 0;
            unsigned long ppu_phase = strtoul(ppu_text, &end, 10);
            if (errno || ppu_text[0] < '0' || ppu_text[0] > '9' || *end || ppu_phase > 4) {
                snprintf(error, error_size, "Startup phase must be CPU:PPU in regional master clocks\n");
                return false;
            }
            startup_cpu_offset = (unsigned)cpu_offset;
            startup_ppu_phase = (unsigned)ppu_phase;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_STARTUP;
            startup_phase_set = true;
        } else if (id == FRONTEND_CLI_STARTUP_SEED) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                snprintf(error, error_size, "Choose one startup phase CPU:PPU or startup seed\n");
                return false;
            }
            char *end;
            errno = 0;
            unsigned long long seed = strtoull(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end || seed > UINT32_MAX) {
                snprintf(error, error_size, "Startup seed must be an integer from 0 to 4294967295\n");
                return false;
            }
            startup_seed = (uint32_t)seed;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_STARTUP;
            startup_seed_set = true;
        } else if (id == FRONTEND_CLI_RAM_POWER_ON) {
            if (++i == argc || !nes_set_ram_power_on_state_name(argv[i])) {
                snprintf(error, error_size, "RAM power-on state must be default, zero, ones, or random\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_RAM_POWER;
        } else if (id == FRONTEND_CLI_POWER_ON_SEED) {
            if (++i == argc || power_on_seed_set) {
                snprintf(error, error_size, "Power-on seed must be an integer from 0 to 4294967295\n");
                return false;
            }
            char *end;
            errno = 0;
            unsigned long long seed = strtoull(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end || seed > UINT32_MAX) {
                snprintf(error, error_size, "Power-on seed must be an integer from 0 to 4294967295\n");
                return false;
            }
            power_on_seed = (uint32_t)seed;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_POWER_SEED;
            power_on_seed_set = true;
        } else if (id == FRONTEND_CLI_RANDOM_VBLANK) {
            nes_set_randomize_vblank(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_RANDOM_VBLANK;
        } else if (id == FRONTEND_CLI_PPU_REVISION) {
            if (++i == argc || !ppu_set_revision_name(argv[i])) {
                snprintf(error, error_size, "PPU revision must be 2c02-pre-e or 2c02e-plus\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_REVISION;
        } else if (id == FRONTEND_CLI_PPU_OAM_ROW_CORRUPTION) {
            ppu_set_oam_row_corruption_worst_case(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_OAM_ROW;
        } else if (id == FRONTEND_CLI_PPU_STARTUP_RESTRICTION) {
            ppu_set_startup_write_restriction(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_STARTUP;
        } else if (id == FRONTEND_CLI_PPU_OAM_DECAY) {
            ppu_set_oam_decay(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_OAM_DECAY;
        } else if (id == FRONTEND_CLI_PPU_SPRITE_EVAL_WRAP_BUG) {
            ppu_set_sprite_eval_wrap_bug(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_SPRITE_WRAP;
        } else if (id == FRONTEND_CLI_PPU_DISABLE_OAMDATA_READ) {
            ppu_set_oamdata_read_disabled(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_OAMDATA;
        } else if (id == FRONTEND_CLI_PPU_DISABLE_PALETTE_READBACK) {
            ppu_set_palette_readback_disabled(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_PALETTE;
        } else if (id == FRONTEND_CLI_PPU_RESET_SUPPRESSION) {
            ppu_set_reset_suppression(true);
            frontend_cli_overrides |= FRONTEND_OVERRIDE_PPU_RESET;
        } else if (id == FRONTEND_CLI_VIDEO_FILTER) {
            if (++i == argc) {
                snprintf(error, error_size, "Video filter must be direct or ntsc-composite\n");
                return false;
            }
            if (strcmp(argv[i], "direct") == 0) {
                ntsc_composite_requested = false;
            } else if (strcmp(argv[i], "ntsc-composite") == 0) {
                ntsc_composite_requested = true;
            } else {
                snprintf(error, error_size, "Video filter must be direct or ntsc-composite\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_VIDEO_FILTER;
        } else if (id == FRONTEND_CLI_MMC3_REVISION) {
            if (++i == argc || !cart_set_mmc3_revision_name(argv[i])) {
                snprintf(error, error_size, "MMC3 revision must be standard or a\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_MMC3_REVISION;
        } else if (id == FRONTEND_CLI_CART_DIP) {
            if (++i == argc) {
                snprintf(error, error_size, "Cartridge DIP value must be an integer from 0 to 255\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_CART_DIPS;
            char *end = NULL;
            errno = 0;
            unsigned long value = strtoul(argv[i], &end, 0);
            if (errno || end == argv[i] || *end || argv[i][0] == '-' || value > 0xFFu ||
                !cart_set_dip_switches((unsigned)value)) {
                snprintf(error, error_size, "Cartridge DIP value must be an integer from 0 to 255\n");
                return false;
            }
        } else if (id == FRONTEND_CLI_ADAPTER) {
            if (++i == argc || !joypad_set_adapter_name(argv[i])) {
                snprintf(error, error_size, "Adapter must be none, four-score, famicom-2, or famicom-4\n");
                return false;
            }
            input_overrides |= NES_INPUT_OVERRIDE_ADAPTER;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_ADAPTER;
        } else if (id == FRONTEND_CLI_PORT1 || id == FRONTEND_CLI_PORT2) {
            unsigned port = id == FRONTEND_CLI_PORT2 ? 1 : 0;
            if (++i == argc || !joypad_set_port_device_name(port, argv[i])) {
                snprintf(error, error_size,
                         "Port device must be pad, none, arkanoid, power-pad-a, power-pad-b, zapper, subor-mouse (port "
                         "2 only), snes-pad, snes-mouse, ntt-keypad, or virtual-boy\n");
                return false;
            }
            input_overrides |= port ? NES_INPUT_OVERRIDE_PORT2 : NES_INPUT_OVERRIDE_PORT1;
            frontend_cli_overrides |= port ? FRONTEND_OVERRIDE_PORT2 : FRONTEND_OVERRIDE_PORT1;
        } else if (id == FRONTEND_CLI_EXPANSION) {
            if (++i == argc || !joypad_set_expansion_device_name(argv[i])) {
                snprintf(error, error_size,
                         "Expansion device must be none, arkanoid, family-trainer-a, family-trainer-b, zapper, "
                         "family-basic, turbo-file, battle-box, subor-keyboard, hori-track, konami-hyper-shot, "
                         "bandai-hyper-shot, party-tap, pachinko, exciting-boxing, jissen-mahjong, barcode-battler, "
                         "oeka-kids-tablet, or fcns\n");
                return false;
            }
            input_overrides |= NES_INPUT_OVERRIDE_EXPANSION;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_EXPANSION;
        } else if (id == FRONTEND_CLI_ZAPPER_RADIUS) {
            if (++i == argc) {
                snprintf(error, error_size, "Zapper radius must be an integer from 0 to 255\n");
                return false;
            }
            frontend_cli_overrides |= FRONTEND_OVERRIDE_ZAPPER_RADIUS;
            char *end;
            unsigned long radius = strtoul(argv[i], &end, 10);
            if (end == argv[i] || *end || radius > NES_ZAPPER_MAX_RADIUS ||
                !joypad_set_zapper_radius((unsigned)radius)) {
                snprintf(error, error_size, "Zapper radius must be an integer from 0 to 255\n");
                return false;
            }
        } else if (id == FRONTEND_CLI_VS_DIP) {
            if (++i == argc) {
                snprintf(error, error_size, "VS DIP value must be an integer from 0 to 65535\n");
                return false;
            }
            char *end;
            unsigned long value = strtoul(argv[i], &end, 0);
            if (end == argv[i] || *end || value > 0xFFFFu) {
                snprintf(error, error_size, "VS DIP value must be an integer from 0 to 65535\n");
                return false;
            }
            vs_dips = (uint16_t)value;
            frontend_cli_overrides |= FRONTEND_OVERRIDE_VS_DIPS;
            vs_dip_set = true;
        } else if (id == FRONTEND_CLI_BARCODE) {
            if (++i == argc) {
                snprintf(error, error_size, "Barcode requires 8 or 13 decimal digits\n");
                return false;
            }
            barcode = argv[i];
        } else if (id == FRONTEND_CLI_BARCODE_BATTLER) {
            if (++i == argc) {
                snprintf(error, error_size, "Barcode Battler scan requires 8 or 13 decimal digits\n");
                return false;
            }
            barcode_battler = argv[i];
        } else if (id == FRONTEND_CLI_TAPE_PLAY || id == FRONTEND_CLI_TAPE_RECORD) {
            bool record = id == FRONTEND_CLI_TAPE_RECORD;
            if (++i == argc || tape_play_path || tape_record_path) {
                snprintf(error, error_size, "Choose one tape file with --tape-play or --tape-record\n");
                return false;
            }
            if (record) {
                tape_record_path = argv[i];
            } else {
                tape_play_path = argv[i];
            }
        } else if (id == FRONTEND_CLI_FDS_BIOS) {
            if (++i == argc) {
                snprintf(error, error_size, "--fds-bios requires an 8KB BIOS file\n");
                return false;
            }
            fds_bios_path = argv[i];
        } else if (id == FRONTEND_CLI_STUDYBOX_BIOS) {
            if (++i == argc) {
                snprintf(error, error_size, "--studybox-bios requires a 256 KiB BIOS file\n");
                return false;
            }
            studybox_bios_path = argv[i];
        } else if (id == FRONTEND_CLI_FDS_SIDE) {
            if (++i == argc) {
                snprintf(error, error_size, "--fds-side requires a side number starting at 1\n");
                return false;
            }
            char *end = NULL;
            unsigned long side = strtoul(argv[i], &end, 10);
            if (!side || *end) {
                snprintf(error, error_size, "FDS side must be a positive number\n");
                return false;
            }
            fds_frontend_side = (size_t)(side - 1);
            fds_side_set = true;
        } else if (id == FRONTEND_CLI_FDS_EJECT) {
            fds_start_ejected = true;
        } else if (id == FRONTEND_CLI_FDS_WRITE_PROTECT) {
            fds_start_write_protected = true;
            fds_write_protect_cli = true;
        } else if (argv[i][0] == '-' || rom_path) {
            snprintf(error, error_size, "Unexpected argument: %s\n", argv[i]);
            return false;
        } else {
            rom_path = argv[i];
        }
    }
    options->rom_path = rom_path;
    options->movie_path = movie_path;
    options->open_tas_editor = open_tas_editor;
    options->movie_launch_failed = movie_launch_failed;
    options->barcode = barcode;
    options->barcode_battler = barcode_battler;
    options->tape_play_path = tape_play_path;
    options->tape_record_path = tape_record_path;
    options->fds_bios_path = fds_bios_path;
    options->studybox_bios_path = studybox_bios_path;
    options->fds_frontend_side = fds_frontend_side;
    options->fds_side_set = fds_side_set;
    options->fds_start_ejected = fds_start_ejected;
    options->fds_start_write_protected = fds_start_write_protected;
    options->fds_write_protect_cli = fds_write_protect_cli;
    options->vs_dip_set = vs_dip_set;
    options->vs_dips = vs_dips;
    options->input_overrides = input_overrides;
    options->startup_phase_set = startup_phase_set;
    options->startup_seed_set = startup_seed_set;
    options->startup_cpu_offset = startup_cpu_offset;
    options->startup_ppu_phase = startup_ppu_phase;
    options->startup_seed = startup_seed;
    options->power_on_seed_set = power_on_seed_set;
    options->power_on_seed = power_on_seed;
    options->epsm_adpcm_path = epsm_adpcm_path;
    options->fcns_kanji_path = fcns_kanji_path;
    options->game_db_path = game_db_path;
    options->data_dir_override = data_dir_override;
    options->disable_game_db_overrides = disable_game_db_overrides;
    options->ntsc_composite_requested = ntsc_composite_requested;
    options->frontend_cli_overrides = frontend_cli_overrides;
    options->help = help;
    return true;
}
