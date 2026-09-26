/*
 * cli_options.c - Shared application option parser and desktop reference
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cli_options.h"
#include <string.h>
static const FrontendCliOption options[] = {
    {FRONTEND_CLI_MOVIE, "--movie", NULL, "path", "unset", "Play a movie with the supplied image.", true, false},
    {FRONTEND_CLI_TAS, "--tas", NULL, "path", "unset", "Open a movie in the TAS editor with the supplied image.", true,
     false},
    {FRONTEND_CLI_REGION, "--region", NULL, "auto|ntsc|pal|dendy", "auto (global/game settings may replace)",
     "Select regional machine timing.", true, true},
    {FRONTEND_CLI_CONSOLE, "--console", NULL, "nes-001|nes-101|famicom|av-famicom",
     "nes-001 (global/game settings may replace)", "Select the console hardware model.", true, true},
    {FRONTEND_CLI_CPU_REVISION, "--cpu-revision", NULL, "early-2a03|late-2a03",
     "late-2a03 (global/game settings may replace)", "Select CPU/APU silicon revision.", true, true},
    {FRONTEND_CLI_CPU_TEST_MODE, "--cpu-test-mode", NULL, "flag", "off", "Enable the diagnostic CPU test registers.",
     false, false},
    {FRONTEND_CLI_APU_DISABLE_NOISE_MODE, "--apu-disable-noise-mode", NULL, "flag",
     "off (global/game settings may replace)", "Force the long noise sequence.", false, false},
    {FRONTEND_CLI_APU_SWAP_DUTY_CYCLES, "--apu-swap-duty-cycles", NULL, "flag",
     "off (global/game settings may replace)", "Swap APU pulse duty patterns.", false, false},
    {FRONTEND_CLI_EPSM_ADPCM, "--epsm-adpcm", NULL, "path", "unset (global/game settings may replace)",
     "Use an 8 KiB percussion ROM.", true, false},
    {FRONTEND_CLI_FCNS_KANJI, "--fcns-kanji", NULL, "path", "unset (global/game settings may replace)",
     "Use a 256 KiB character ROM.", true, false},
    {FRONTEND_CLI_GAME_DB, "--game-db", NULL, "path", "unset (global/game settings may replace)",
     "Choose the game metadata database.", true, false},
    {FRONTEND_CLI_DATA_DIR, "--data-dir", NULL, "path", "platform application-data directory",
     "Choose the application settings and saves directory.", true, false},
    {FRONTEND_CLI_NO_GAME_DB_OVERRIDES, "--no-game-db-overrides", NULL, "flag",
     "off (global/game settings may replace)", "Disable database hardware corrections.", false, false},
    {FRONTEND_CLI_STARTUP_PHASE, "--startup-phase", NULL, "CPU:PPU (CPU 0-15, PPU 0-4; regional limits apply)",
     "unset (global/game settings may replace)", "Set CPU and PPU startup alignment; excludes startup-seed.", true,
     false},
    {FRONTEND_CLI_STARTUP_SEED, "--startup-seed", NULL, "0-4294967295 (decimal)",
     "unset (global/game settings may replace)", "Choose reproducible startup alignment; excludes startup-phase.", true,
     false},
    {FRONTEND_CLI_RAM_POWER_ON, "--ram-power-on", NULL, "default|zero|ones|random",
     "default (global/game settings may replace)", "Choose initial RAM contents.", true, true},
    {FRONTEND_CLI_POWER_ON_SEED, "--power-on-seed", NULL, "0-4294967295 (decimal)",
     "unset (global/game settings may replace)", "Seed randomized power-on state.", true, false},
    {FRONTEND_CLI_RANDOM_VBLANK, "--random-vblank", NULL, "flag", "off (global/game settings may replace)",
     "Randomize startup vblank alignment.", false, false},
    {FRONTEND_CLI_PPU_REVISION, "--ppu-revision", NULL, "2c02-pre-e|2c02e-plus",
     "2c02e-plus (global/game settings may replace)", "Select PPU silicon revision.", true, true},
    {FRONTEND_CLI_PPU_OAM_ROW_CORRUPTION, "--ppu-oam-row-corruption", NULL, "flag",
     "off (global/game settings may replace)", "Enable worst-case OAM row corruption.", false, false},
    {FRONTEND_CLI_PPU_STARTUP_RESTRICTION, "--ppu-startup-restriction", NULL, "flag",
     "off (global/game settings may replace)", "Restrict PPU startup writes.", false, false},
    {FRONTEND_CLI_PPU_OAM_DECAY, "--ppu-oam-decay", NULL, "flag", "off (global/game settings may replace)",
     "Enable OAM decay.", false, false},
    {FRONTEND_CLI_PPU_SPRITE_EVAL_WRAP_BUG, "--ppu-sprite-eval-wrap-bug", NULL, "flag",
     "off (global/game settings may replace)", "Enable sprite evaluation wrap behavior.", false, false},
    {FRONTEND_CLI_PPU_DISABLE_OAMDATA_READ, "--ppu-disable-oamdata-read", NULL, "flag",
     "off (global/game settings may replace)", "Disable OAMDATA readback.", false, false},
    {FRONTEND_CLI_PPU_DISABLE_PALETTE_READBACK, "--ppu-disable-palette-readback", NULL, "flag",
     "off (global/game settings may replace)", "Disable palette readback.", false, false},
    {FRONTEND_CLI_PPU_RESET_SUPPRESSION, "--ppu-reset-suppression", NULL, "flag",
     "off (global/game settings may replace)", "Enable PPU reset suppression.", false, false},
    {FRONTEND_CLI_VIDEO_FILTER, "--video-filter", NULL, "direct|ntsc-composite",
     "direct (global/game settings may replace)", "Select direct or composite video.", true, true},
    {FRONTEND_CLI_MMC3_REVISION, "--mmc3-revision", NULL, "standard|a", "standard (global/game settings may replace)",
     "Select MMC3 interrupt revision.", true, true},
    {FRONTEND_CLI_CART_DIP, "--cart-dip", NULL, "0-255 (decimal or 0x hex)", "0 (global/game settings may replace)",
     "Set cartridge DIP switches.", true, false},
    {FRONTEND_CLI_ADAPTER, "--adapter", NULL, "none|four-score|famicom-2|famicom-4",
     "none (global/game settings may replace)", "Select multiplayer adapter.", true, true},
    {FRONTEND_CLI_PORT1, "--port1", NULL,
     "pad|none|arkanoid|power-pad-a|power-pad-b|zapper|snes-pad|snes-mouse|ntt-keypad|virtual-boy",
     "pad (global/game settings may replace)", "Select player port 1 device.", true, true},
    {FRONTEND_CLI_PORT2, "--port2", NULL,
     "pad|none|arkanoid|power-pad-a|power-pad-b|zapper|subor-mouse|snes-pad|snes-mouse|ntt-keypad|virtual-boy",
     "pad (global/game settings may replace)", "Select player port 2 device.", true, true},
    {FRONTEND_CLI_EXPANSION, "--expansion", NULL,
     "none|arkanoid|family-trainer-a|family-trainer-b|zapper|family-basic|turbo-file|battle-box|subor-keyboard|hori-"
     "track|konami-hyper-shot|bandai-hyper-shot|party-tap|pachinko|exciting-boxing|jissen-mahjong|barcode-battler|oeka-"
     "kids-tablet|fcns",
     "none (global/game settings may replace)", "Select expansion port device.", true, true},
    {FRONTEND_CLI_ZAPPER_RADIUS, "--zapper-radius", NULL, "0-255", "0 (global/game settings may replace)",
     "Set the light sensor radius.", true, false},
    {FRONTEND_CLI_VS_DIP, "--vs-dip", NULL, "0-65535 (decimal or 0x hex)", "0 (global/game settings may replace)",
     "Set VS DIP switches.", true, false},
    {FRONTEND_CLI_BARCODE, "--barcode", NULL, "8 or 13 decimal digits", "unset", "Scan a cartridge barcode.", true,
     false},
    {FRONTEND_CLI_BARCODE_BATTLER, "--barcode-battler", NULL, "8 or 13 decimal digits", "unset",
     "Scan an expansion barcode.", true, false},
    {FRONTEND_CLI_TAPE_PLAY, "--tape-play", NULL, "path", "unset (global/game settings may replace)",
     "Play a tape file; excludes tape-record.", true, false},
    {FRONTEND_CLI_TAPE_RECORD, "--tape-record", NULL, "path", "unset (global/game settings may replace)",
     "Record a tape file; excludes tape-play.", true, false},
    {FRONTEND_CLI_FDS_BIOS, "--fds-bios", NULL, "path", "unset (global/game settings may replace)",
     "Use an 8 KiB disk-system BIOS.", true, false},
    {FRONTEND_CLI_STUDYBOX_BIOS, "--studybox-bios", NULL, "path", "unset (global/game settings may replace)",
     "Use a 256 KiB firmware image.", true, false},
    {FRONTEND_CLI_FDS_SIDE, "--fds-side", NULL, "positive side number starting at 1", "1",
     "Insert this disk side on startup.", true, false},
    {FRONTEND_CLI_FDS_EJECT, "--fds-eject", NULL, "flag", "off", "Start with no disk inserted.", false, false},
    {FRONTEND_CLI_FDS_WRITE_PROTECT, "--fds-write-protect", NULL, "flag", "off (global/game settings may replace)",
     "Protect disk writes.", false, false},
    {FRONTEND_CLI_HELP, "--help", "-h", "flag", "off", "Print application options and exit.", false, false}};
_Static_assert(sizeof(options) / sizeof(options[0]) == FRONTEND_CLI_COUNT, "option table coverage");

size_t frontend_cli_count(void) {
    return sizeof(options) / sizeof(options[0]);
}

const FrontendCliOption *frontend_cli_at(size_t index) {
    return index < frontend_cli_count() ? &options[index] : NULL;
}

FrontendCliId frontend_cli_find(const char *text) {
    for (size_t i = 0; text && i < frontend_cli_count(); ++i) {
        if (!strcmp(text, options[i].name) || (options[i].alias && !strcmp(text, options[i].alias))) {
            return options[i].id;
        }
    }
    return FRONTEND_CLI_UNKNOWN;
}

bool frontend_cli_accepts(const FrontendCliOption *option, const char *value) {
    if (!option) {
        return false;
    }
    if (!option->takes_value) {
        return value == NULL;
    }
    if (!value || !*value) {
        return false;
    }
    if (!option->choices) {
        return true;
    }
    const char *cursor = option->values;
    while (*cursor) {
        const char *end = strchr(cursor, '|');
        size_t length = end ? (size_t)(end - cursor) : strlen(cursor);
        if (strlen(value) == length && !memcmp(value, cursor, length)) {
            return true;
        }
        if (!end) {
            break;
        }
        cursor = end + 1;
    }
    return false;
}

void frontend_cli_print(FILE *stream) {
    if (!stream) {
        return;
    }
    fprintf(stream, "Cupid [options] [image]\nWithout an image, open the desktop startup view.\n");
    for (size_t i = 0; i < frontend_cli_count(); ++i) {
        const FrontendCliOption *o = &options[i];
        fprintf(stream, "%s%s%s %s\n  %s\n  Default: %s\n", o->name, o->alias ? ", " : "", o->alias ? o->alias : "",
                o->takes_value ? o->values : "", o->description, o->default_value);
    }
}
