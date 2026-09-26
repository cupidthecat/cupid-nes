/*
 * main.c - Main entry point for Cupid NES Emulator
 *
 * Author: @frankischilling
 *
 * This file initializes the loaded cartridge, CPU, PPU, APU, SDL video and audio, input,
 * palette tools, frame pacing, and the main emulation loop. It also handles controller
 * input, reset events, palette file loading, and final emulator shutdown.
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

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "rom/rom.h"
#include "rom/fds.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "joypad/joypad.h"
#include "joypad/family_basic.h"
#include "../include/globals.h"
#include "apu/apu.h"
#include "apu/epsm.h"
#include <time.h>
#include "rom/mapper.h"
#include "debugger/debugger.h"
#include <math.h>
#include <limits.h>
#include "ui/palette_tool.h"
#include "ui/nsf_player_runtime.h"
#include "ui/capture_runtime.h"
#include "ui/audio_runtime.h"
#include "ui/frontend_execution.h"
#include "ui/tas_frontend.h"
#include "ui/netplay_frontend.h"
#include "ui/app_paths.h"
#include "ui/game_database.h"
#include "ui/platform_frontend.h"
#include "ui/settings.h"
#include "ui/cli_options.h"
#include "ui/cli_help.h"
#include "ui/game_config.h"
#include "ui/lifecycle_frontend.h"
#include "ui/update_checker.h"
#include "ui/presentation_host.h"
#include "ui/header_editor_frontend.h"
#include "ui/overclock_frontend.h"
#include "ui/frontend_session.h"
#include "ui/idle_frontend.h"
#include "ui/image_open.h"
#include "ui/session_actions.h"
#include "ui/host_input.h"
#include "ui/peripheral_input.h"
#include "ui/desktop_ui.h"
#include "ui/device_frontend.h"
#include "ui/storage_frontend.h"
#include "ui/state_runtime.h"
#include "ui/debug_frontend.h"
#include "ui/cheat_frontend.h"
#include "cheats/cheats.h"
#include "ui/memory_search_frontend.h"
#include "ui/video_runtime.h"
#include "ui/frontend_commands.h"
#include "ui/frontend_panels.h"
#include "system/timing.h"
#include "system/hardware.h"
#include "system/vs_system.h"
#include "video/ntsc_composite.h"

#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_BUFFER_SAMPLES 1024

// SDL presents the framebuffer that the PPU fills.
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

Joypad pad1 = {0}, pad2 = {0};

typedef struct {
    NsfPlayer *music;
    DebugFrontend *debug;
    CheatFrontend *cheats;
    MemoryToolsFrontend *memory;
    NesCaptureRuntime *capture;
    FrontendDeviceRuntime *devices;
    FrontendVideoRuntime *video;
    LifecycleFrontend *lifecycle;
    FrontendImageEnvironment *image_environment;
    FrontendPresentationHost *presentation;
    FrontendDesktopUi *desktop;
    GameConfigFrontend *configuration;
    bool (*finish_capture)(void *, char *, size_t);
    void *capture_context;
    bool image_changed;
} LiveFrontend;

static void live_image_changed(void *context) {
    LiveFrontend *live = context;
    if (!live) return;
    if (live->devices) frontend_devices_session_changed(live->devices);
    if (live->music) nsf_player_image_changed(live->music);
    if (live->debug) debug_frontend_image_changed(live->debug);
    if (live->cheats) cheat_frontend_image_changed(live->cheats, rom_file_crc32());
    if (live->image_environment && live->image_environment->cheat_override) {
        if (live->image_environment->cheat_path[0]) (void)cheats_load_file(live->image_environment->cheat_path);
        else (void)cheats_clear();
    }
    if (live->memory) memory_tools_image_changed(live->memory);
    if (live->capture) nes_capture_frontend_refresh(&live->capture->frontend);
    if (live->lifecycle) lifecycle_image_changed(live->lifecycle);
    live->image_changed = true;
}

static void live_state_restored(void *context) {
    LiveFrontend *live = context;
    if (!live) return;
    if (live->lifecycle) state_recorder_restart_clock(&live->lifecycle->recorder);
    if (live->video) nes_hd_runtime_reset_audio(live->video->hd);
    if (live->devices) frontend_devices_session_changed(live->devices);
    if (live->music) nsf_player_image_changed(live->music);
    if (live->capture) nes_capture_frontend_refresh(&live->capture->frontend);
}

static void live_configuration_changed(void *context) {
    LiveFrontend *live = context;
    if (live->presentation) frontend_presentation_host_reset(live->presentation);
    if (live->lifecycle) state_recorder_restart_clock(&live->lifecycle->recorder);
}

static bool live_before_machine_change(void *context, char *error, size_t size) {
    LiveFrontend *live = context;
    if (live->finish_capture && !live->finish_capture(live->capture_context, error, size)) return false;
    live_configuration_changed(live);
    return true;
}

static void live_presentation_changed(void *context) {
    LiveFrontend *live = context;
    FrontendSettingsReport report;
    (void)frontend_presentation_capture_settings(live->video, live->presentation->audio, live->video->settings);
    if (!game_config_save_globals(live->configuration, frontend_paths_config_file(),
                                  live->video->settings, &report))
        frontend_desktop_set_status(live->desktop, report.message);
}

static void live_overclock_changed(void *context, const NesOverclockConfig *config) {
    LiveFrontend *live = context;
    FrontendSettingsReport report;
    live->video->settings->overclock = *config;
    live->desktop->staged.overclock = *config;
    if (!game_config_save_globals(live->configuration, frontend_paths_config_file(),
                                  live->video->settings, &report))
        frontend_desktop_set_status(live->desktop, report.message);
}

static int application_main(int argc, char *argv[]) {
    SDL_AudioSpec have;
    SDL_AudioDeviceID audio_dev = 0; SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    FrontendCliOptions parsed_options;
    char cli_error[512];
    if (!frontend_cli_parse(argc, argv, &parsed_options, cli_error, sizeof(cli_error))) {
        fprintf(stderr, "%s\n", cli_error);
        return 1;
    }
    if (parsed_options.help) { frontend_cli_print(stdout); return 0; }
    const char * rom_path = parsed_options.rom_path;
    const char * movie_path = parsed_options.movie_path;
    bool open_tas_editor = parsed_options.open_tas_editor;
    bool movie_launch_failed = parsed_options.movie_launch_failed;
    const char * barcode = parsed_options.barcode;
    const char * barcode_battler = parsed_options.barcode_battler;
    const char * tape_play_path = parsed_options.tape_play_path;
    const char * tape_record_path = parsed_options.tape_record_path;
    const char * fds_bios_path = parsed_options.fds_bios_path;
    const char * studybox_bios_path = parsed_options.studybox_bios_path;
    size_t fds_frontend_side = parsed_options.fds_frontend_side;
    bool fds_side_set = parsed_options.fds_side_set;
    bool fds_start_ejected = parsed_options.fds_start_ejected;
    bool fds_start_write_protected = parsed_options.fds_start_write_protected;
    bool fds_write_protect_cli = parsed_options.fds_write_protect_cli;
    bool vs_dip_set = parsed_options.vs_dip_set;
    uint16_t vs_dips = parsed_options.vs_dips;
    uint8_t input_overrides = parsed_options.input_overrides;
    bool startup_phase_set = parsed_options.startup_phase_set;
    bool startup_seed_set = parsed_options.startup_seed_set;
    unsigned startup_cpu_offset = parsed_options.startup_cpu_offset;
    unsigned startup_ppu_phase = parsed_options.startup_ppu_phase;
    uint32_t startup_seed = parsed_options.startup_seed;
    bool power_on_seed_set = parsed_options.power_on_seed_set;
    uint32_t power_on_seed = parsed_options.power_on_seed;
    const char * epsm_adpcm_path = parsed_options.epsm_adpcm_path;
    const char * fcns_kanji_path = parsed_options.fcns_kanji_path;
    const char * game_db_path = parsed_options.game_db_path;
    const char * data_dir_override = parsed_options.data_dir_override;
    bool disable_game_db_overrides = parsed_options.disable_game_db_overrides;
    bool ntsc_composite_requested = parsed_options.ntsc_composite_requested;
    uint32_t frontend_cli_overrides = parsed_options.frontend_cli_overrides;
    joypad_set_configuration_overrides(input_overrides);
    if (movie_path && !rom_path) {
        fprintf(stderr, "A movie launch also requires a game image path\n");
        return 1;
    }

    char path_error[160];
    if (!frontend_paths_init(data_dir_override, path_error, sizeof(path_error))) {
        fprintf(stderr, "%s\n", path_error);
        return 1;
    }

    FrontendSettings frontend_settings;
    FrontendSettingsReport settings_report;
    frontend_settings_defaults(&frontend_settings);
    const char *settings_path = frontend_paths_config_file();
    if (!frontend_settings_load(settings_path, &frontend_settings, &settings_report)) {
        fprintf(stderr, "%s; using default application settings\n",
                settings_report.message[0] ? settings_report.message : "Could not load settings");
        frontend_settings_defaults(&frontend_settings);
    } else if (settings_report.migrated || settings_report.unknown_settings) {
        fprintf(stderr, "%s\n", settings_report.message);
    }
    frontend_settings.cli_overrides = frontend_cli_overrides;
    if (!(frontend_cli_overrides & FRONTEND_OVERRIDE_VIDEO_FILTER))
        ntsc_composite_requested = frontend_settings.ntsc_composite;
    frontend_settings.audio_mix.muted = frontend_settings.muted;
    if (!epsm_adpcm_path && frontend_settings.epsm_adpcm_path[0])
        epsm_adpcm_path = frontend_settings.epsm_adpcm_path;
    if (!fcns_kanji_path && frontend_settings.fcns_kanji_path[0])
        fcns_kanji_path = frontend_settings.fcns_kanji_path;
    if (!tape_play_path && !tape_record_path) {
        if (frontend_settings.tape_play_path[0]) tape_play_path = frontend_settings.tape_play_path;
        else if (frontend_settings.tape_record_path[0]) tape_record_path = frontend_settings.tape_record_path;
    }
    if (!startup_phase_set && !startup_seed_set) {
        startup_phase_set = frontend_settings.startup_phase_set;
        startup_seed_set = frontend_settings.startup_seed_set;
        startup_cpu_offset = frontend_settings.startup_cpu_offset;
        startup_ppu_phase = frontend_settings.startup_ppu_phase;
        startup_seed = frontend_settings.startup_seed;
    }
    if (!power_on_seed_set && frontend_settings.power_on_seed_set) {
        power_on_seed_set = true;
        power_on_seed = frontend_settings.power_on_seed;
    }
    if (!frontend_settings_validate(&frontend_settings, path_error, sizeof(path_error))) {
        fprintf(stderr, "%s\n", path_error[0] ? path_error : "Saved settings are invalid");
        frontend_paths_shutdown();
        return 1;
    }
    if (!frontend_settings_apply_core(&frontend_settings, path_error, sizeof(path_error))) {
        fprintf(stderr, "%s\n", path_error[0] ? path_error : "Saved settings are invalid");
        frontend_paths_shutdown();
        return 1;
    }

    FrontendSettings global_settings = frontend_settings;
    GameConfigFrontend game_config;
    if (!game_config_init(&game_config, &global_settings, frontend_paths_data_dir())) return 1;
    game_config.launch.ntsc_composite = ntsc_composite_requested;
    if (frontend_cli_overrides & FRONTEND_OVERRIDE_VIDEO_FILTER)
        game_config.cli_fields |= UINT64_C(1) << game_config_find_field("composite");
    const char *launch_paths[] = {fds_bios_path, studybox_bios_path, epsm_adpcm_path, fcns_kanji_path};
    const char *field_names[] = {"fds_bios", "studybox_bios", "epsm_adpcm", "fcns_kanji"};
    char *launch_targets[] = {game_config.launch.fds_bios_path, game_config.launch.studybox_bios_path,
                             game_config.launch.epsm_adpcm_path, game_config.launch.fcns_kanji_path};
    for (unsigned i = 0; i < 4; ++i) {
        if (launch_paths[i]) {
            if (strlen(launch_paths[i]) >= FRONTEND_SETTINGS_PATH_TEXT) return 1;
            strcpy(launch_targets[i], launch_paths[i]);
        }
    }
    const char *explicit_paths[] = {parsed_options.fds_bios_path, parsed_options.studybox_bios_path,
                                   parsed_options.epsm_adpcm_path, parsed_options.fcns_kanji_path};
    for (unsigned i = 0; i < 4; ++i) if (explicit_paths[i])
        game_config.cli_fields |= UINT64_C(1) << game_config_find_field(field_names[i]);
    if (fds_write_protect_cli) {
        game_config.cli_fields |= UINT64_C(1) << game_config_find_field("disk_write_protect");
        game_config.launch.fds_write_protected = true;
    }
    FrontendImageEnvironment image_environment = {.configuration = &game_config, .effective = &frontend_settings};
    FrontendSession frontend_session;
    frontend_session_init(&frontend_session, frontend_image_open, &image_environment);
    const char *recent_path = frontend_paths_recent_file();
    if (!frontend_session_load_recent(&frontend_session, recent_path,
                                      path_error, sizeof(path_error))) {
        fprintf(stderr, "%s; recent images will start empty\n",
                path_error[0] ? path_error : "Could not load recent images");
    }
    frontend_session_trim_recent(&frontend_session, frontend_settings.recent_file_limit);

    FrontendImageRequest startup_request;
    bool startup_request_ready = false;
    if (rom_path) {
        if (!frontend_image_request_init(&startup_request, rom_path)) {
            fprintf(stderr, "Image path is too long\n");
            frontend_paths_shutdown();
            return 1;
        }
        startup_request_ready = true;
    }
    if (startup_request_ready) {
        if (!startup_request.save_identity[0])
            frontend_image_request_apply_settings(&startup_request, &frontend_settings);
        if (fds_bios_path
            && !frontend_image_request_set_fds_bios(&startup_request, fds_bios_path)) {
            fprintf(stderr, "FDS BIOS path is too long\n");
            frontend_paths_shutdown();
            return 1;
        }
        if (studybox_bios_path
            && !frontend_image_request_set_studybox_bios(&startup_request, studybox_bios_path)) {
            fprintf(stderr, "StudyBox BIOS path is too long\n");
            frontend_paths_shutdown();
            return 1;
        }
        if (fds_write_protect_cli) startup_request.fds_write_protected = true;
    }

    if (!fds_bios_path && (fds_side_set || fds_start_ejected || fds_start_write_protected)) {
        fprintf(stderr, "FDS media options require --fds-bios\n");
        frontend_paths_shutdown();
        return 1;
    }
    if (fds_bios_path && studybox_bios_path) {
        fprintf(stderr, "Choose either FDS or StudyBox firmware for the image\n");
        frontend_paths_shutdown();
        return 1;
    }
    if (!joypad_configuration_valid()) {
        fprintf(stderr, "An adapter and another device cannot share the same connector\n");
        frontend_paths_shutdown();
        return 1;
    }
    if ((tape_play_path || tape_record_path)
        && joypad_expansion_device() != NES_EXPANSION_FAMILY_BASIC) {
        fprintf(stderr, "Tape input requires --expansion family-basic\n");
        frontend_paths_shutdown();
        return 1;
    }

    printf("Console: %s\n", nes_console_model_name());
    printf("CPU revision: %s\n", apu_get_cpu_revision() == APU_CPU_REVISION_EARLY_2A03 ? "early-2a03" : "late-2a03");
    printf("APU noise short mode: %s\n", apu_noise_mode_disabled() ? "disabled" : "standard");
    printf("APU pulse duty mapping: %s\n", apu_swap_duty_cycles_enabled() ? "swapped" : "standard");
    printf("RAM power-on state: %s\n", nes_ram_power_on_state_name());
    printf("Random power-on VBL flag: %s\n", nes_randomize_vblank_enabled() ? "enabled" : "disabled");
    if (power_on_seed_set) {
        printf("Power-on seed: %llu\n", (unsigned long long)power_on_seed);
    }
    printf("PPU revision: %s\n", ppu_revision_name());
    printf("CPU test-register reads: %s\n", cpu_test_mode_enabled() ? "enabled" : "disabled");
    printf("PPU OAM row corruption: %s\n", ppu_oam_row_corruption_worst_case() ? "worst-case" : "compatibility");
    printf("PPU startup write restriction: %s\n",
           ppu_startup_write_restriction_enabled() ? "enabled" : "compatibility");
    printf("PPU OAM decay: %s\n", ppu_oam_decay_enabled() ? "enabled" : "compatibility");
    printf("PPU sprite-evaluation wrap bug: %s\n", ppu_sprite_eval_wrap_bug_enabled() ? "enabled" : "disabled");
    printf("PPU OAMDATA reads: %s\n",
           ppu_oamdata_read_disabled() ? "open-bus only" : "enabled");
    printf("PPU palette readback: %s\n",
           ppu_palette_readback_disabled() ? "buffered" : "enabled");
    printf("PPU soft-reset suppression: %s\n",
           ppu_reset_suppression_enabled() ? "enabled" : "disabled");
    printf("MMC3 revision: %s\n", cart_mmc3_revision_name());
    printf("Input adapter: %s\n", joypad_adapter_name());
    if (epsm_adpcm_path && !epsm_load_adpcm_file(epsm_adpcm_path)) {
        fprintf(stderr, "Could not load the 8 KiB YMF288 ADPCM ROM: %s\n", epsm_adpcm_path);
        return 1;
    }
    if (fcns_kanji_path && !rom_set_fcns_kanji_firmware(fcns_kanji_path)) {
        fprintf(stderr, "Could not load the 256 KiB FCNS Kanji ROM: %s\n", fcns_kanji_path);
        return 1;
    }
    if (!game_db_path && frontend_settings.game_database_path[0])game_db_path=frontend_settings.game_database_path;
    if (!(frontend_cli_overrides&FRONTEND_OVERRIDE_DATABASE_CORRECTIONS))disable_game_db_overrides=frontend_settings.disable_database_corrections;
    rom_database_set_overrides(!disable_game_db_overrides);
    FrontendDatabaseStatus database_status;
    if (!frontend_database_load(game_db_path, frontend_paths_data_dir(), &database_status)) {
        fprintf(stderr, "Game database: %s: %s\n", database_status.path, database_status.message);
        frontend_paths_shutdown();
        return 1;
    }
    printf("Game database: %s: %s\n", database_status.path, database_status.message);
    if (power_on_seed_set) nes_seed_power_on_random(power_on_seed);
    bool ntsc_composite_active = ntsc_composite_requested
        && ntsc_composite_supported(nes_timing()->region, vs_enabled());
    const char *storage_identity = NULL;
    char startup_error[256] = {0};
    if (startup_request_ready && !frontend_session_open(&frontend_session, &startup_request,
                                                       startup_error, sizeof(startup_error))) {
        fprintf(stderr, "Failed to open image: %s\n", startup_error);
        if (image_environment.machine_changed && !unload_rom()) return 1;
    }
    if (frontend_session.active) {
    rom_path = frontend_session.current.path;
    printf("Loading ROM: %s\n", rom_path);
    fds_bios_path = frontend_session.current.fds_bios_path[0]
        ? frontend_session.current.fds_bios_path : NULL;
    studybox_bios_path = frontend_session.current.studybox_bios_path[0]
        ? frontend_session.current.studybox_bios_path : NULL;
    printf("Metadata source: %s\n", rom_metadata_source_name());
    printf("Timing region: %s (selection: %s)\n",
           nes_region_name(nes_timing()->region), nes_region_mode_name());
    if (!rom_is_fds() && !rom_is_studybox()) {
        printf("File CRC32: %08X\n", (unsigned)rom_file_crc32());
        printf("PRG CRC32: %08X\n", (unsigned)rom_prg_crc32());
        printf("PRG+CHR CRC32: %08X\n", (unsigned)rom_prg_chr_crc32());
    }
    if (epsm_enabled()) {
        printf("EPSM: 8 MHz YMF288, stereo output\n");
        if (!epsm_has_adpcm_rom())
            fprintf(stderr, "EPSM percussion uses zero-filled data without --epsm-adpcm FILE\n");
    }
    ntsc_composite_active = frontend_settings.ntsc_composite
        && ntsc_composite_supported(nes_timing()->region, vs_enabled());
    if (ntsc_composite_requested && !ntsc_composite_active)
        printf("Video filter: direct (NTSC composite is unavailable for this hardware)\n");
    else
        printf("Video filter: %s\n", ntsc_composite_active ? "ntsc-composite" : "direct");
    if (startup_phase_set && !cpu_set_startup_alignment(startup_cpu_offset, startup_ppu_phase)) {
        fprintf(stderr, "Startup phase must be CPU 0..%u and PPU 0..%u for this image\n",
                (unsigned)nes_timing()->cpu_divider - 1, (unsigned)nes_timing()->ppu_divider - 1);
        unload_rom();
        return 1;
    }
    if (startup_seed_set) cpu_seed_startup_alignment(startup_seed);
    storage_identity = frontend_session.current_result.save_identity[0]
        ? frontend_session.current_result.save_identity : rom_path;
    if (!joypad_persistent_configure(storage_identity)) {
        fprintf(stderr, "Failed to load expansion-device storage\n");
        unload_rom();
        return 1;
    }
    if (!vs_dip_set && vs_enabled()) (void)vs_set_dip_switches(frontend_settings.vs_dips);
    if (vs_dip_set && !vs_set_dip_switches(vs_dips)) {
        fprintf(stderr, "--vs-dip requires a VS System image\n");
        unload_rom();
        return 1;
    }
    if (barcode && !cart_set_barcode(barcode)) {
        fprintf(stderr, "Barcode input requires a Datach cartridge and 8 or 13 decimal digits\n");
        unload_rom();
        return 1;
    }
    if (barcode) printf("Press F8 to scan the configured barcode\n");
    if (rom_is_fds()) {
        if (fds_side_set && !fds_insert_disk(fds_frontend_side)) {
            fprintf(stderr, "FDS side is outside the loaded disk image\n");
            unload_rom();
            return 1;
        }
        if (fds_start_ejected) fds_eject_disk();
    }
    cpu_total_cycles = 0;
    if (barcode_battler && !joypad_scan_barcode_battler(barcode_battler)) {
        fprintf(stderr, "Barcode Battler input requires --expansion barcode-battler and 8 or 13 decimal digits\n");
        unload_rom();
        return 1;
    }
    if (barcode_battler) printf("Press F8 to scan the configured Barcode Battler code\n");
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    // Print ROM metadata at startup so mapper selection can be checked from the log.
    if (!rom_is_fds() && !rom_is_studybox() && !rom_is_nsf()) {
        printf("=== ROM Header Info ===\n");
        printf("Signature: %c%c%c 0x%02X\n",
               ines_header.signature[0],
               ines_header.signature[1],
               ines_header.signature[2],
               ines_header.signature[3]);
        printf("PRG-ROM Chunks: %d\n", ines_header.prg_rom_chunks);
        printf("CHR-ROM Chunks: %d\n", ines_header.chr_rom_chunks);
        printf("Flags6: 0x%02X\n", ines_header.flags6);
        printf("Flags7: 0x%02X\n", ines_header.flags7);
        printf("Mirroring: %s\n", (mirroring_mode == 0 ? "Horizontal" : "Vertical"));
        printf("=======================\n");
    }
    
    if (!rom_is_fds() && !rom_is_studybox() && !rom_is_nsf()
        && (ines_header.prg_rom_chunks > 1 || (ines_header.flags6 & 0xF0))) {
        printf("WARNING: This ROM likely uses a mapper (mapper number: %d).\n",
            (ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4));
    }
    
    if (!rom_is_fds() && !rom_is_studybox() && !rom_is_nsf())
        printf("Mapper detected: %d\n", ((ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4)));
    if (rom_is_nsf()) {
        const NsfMetadata *music = rom_nsf_metadata();
        printf("Music track: %u/%u", rom_nsf_current_track() + 1u,
               music ? (unsigned)music->total_songs : 0u);
        if (music && music->track_names[rom_nsf_current_track()][0])
            printf(" - %s", music->track_names[rom_nsf_current_track()]);
        printf("\nPage Up/Page Down changes tracks.\n");
    }


    printf("Resetting CPU...\n");
    if (!cpu_power_on(&cpu)) {
        fprintf(stderr, "Invalid CPU startup alignment\n");
        unload_rom();
        return 1;
    }
    CpuStartupAlignment alignment = cpu_get_startup_alignment();
    printf("Startup alignment: CPU %u, PPU %u%s\n", (unsigned)alignment.cpu_offset,
           (unsigned)alignment.ppu_phase, startup_seed_set ? " (seeded)" : "");
    if (startup_seed_set) printf("Startup seed: %llu\n", (unsigned long long)startup_seed);
    vs_power_on_secondary();
    if (vs_enabled()) {
        printf("VS System: %s, PPU model %u, DIP $%04X\n",
               vs_dual_system() ? "dual" : "single", (unsigned)vs_ppu_model(),
               (unsigned)vs_dip_switches());
        printf("VS controls: 5-8 coin slots, F1/F2 service buttons\n");
    }
    if (tape_play_path && !family_basic_tape_load_file(tape_play_path)) {
        fprintf(stderr, "Could not load tape: %s\n", tape_play_path);
        unload_rom();
        return 1;
    }
    if (tape_play_path || tape_record_path)
        printf("Family BASIC tape: F10 starts the tape; F11 stops and saves a recording\n");
    if (!frontend_session_save_recent(&frontend_session, recent_path,
                                      path_error, sizeof(path_error))) {
        fprintf(stderr, "%s\n", path_error);
    }
    printf("CPU state after reset:\n");
    printf("  PC: 0x%04X\n", cpu.pc);
    printf("  SP: 0x%02X\n", cpu.sp);
    printf("  A: 0x%02X\n", cpu.a);
    printf("  X: 0x%02X\n", cpu.x);
    printf("  Y: 0x%02X\n", cpu.y);
    printf("  Status: 0x%02X\n", cpu.status);

    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    if (!nes_audio_mix_set(&frontend_settings.audio_mix, path_error, sizeof(path_error))) {
        fprintf(stderr, "%s\n", path_error[0] ? path_error : "Saved audio settings are invalid");
        frontend_paths_shutdown();
        return 1;
    }
    frontend_host_input_open_controllers(&frontend_settings);
    FrontendAudioRuntime audio_runtime;
    frontend_audio_runtime_bind(&audio_runtime, &audio_dev, &have);
    char audio_error[320] = {0};
    if (!frontend_audio_runtime_apply(&audio_runtime, &frontend_settings, true,
                                      audio_error, sizeof(audio_error))) {
        fprintf(stderr, "Warning: audio disabled (%s)\n",
                audio_error[0] ? audio_error : SDL_GetError());
    }
    if (audio_dev) {
        printf("=== Audio Info ===\n");
        printf("Requested: %u Hz, Got: %d Hz\n", frontend_settings.audio_sample_rate, have.freq);
        printf("Requested: %u samples buffer, Got: %d samples\n",
               frontend_settings.audio_buffer_samples, have.samples);
        printf("Cycles per sample: %.6f\n", nes_timing()->cpu_hz / have.freq);
        printf("==================\n");
    }
    if (!window) window = SDL_CreateWindow("Cupid NES Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        (int)frontend_settings.window_width, (int)frontend_settings.window_height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) { fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError()); return 1; }
    if (!renderer) renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    if (!renderer) { fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError()); return 1; }
    if (frontend_settings.fullscreen)
        (void)SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    FrontendVideoRuntime video_runtime;
    if (!frontend_video_runtime_init(&video_runtime, renderer, &frontend_settings,
                                     ntsc_composite_requested,
                                     path_error, sizeof(path_error))) {
        fprintf(stderr, "Video output: %s\n", path_error[0] ? path_error : SDL_GetError());
        return 1;
    }
    unsigned display_width = 0, display_height = 0;
    frontend_video_runtime_display_size(&video_runtime, &display_width, &display_height);
    int video_width = (int)display_width;
    int video_height = (int)display_height;
    bool running = true;
    SDL_Event e;
    FrontendExecutionRuntime execution_runtime;
    frontend_execution_init(&execution_runtime, &audio_dev,
                            audio_dev ? have.freq : (int)frontend_settings.audio_sample_rate,
                            rom_path, fds_bios_path, studybox_bios_path,
                            &fds_frontend_side);
    execution_runtime.settings = &frontend_settings;
    if(frontend_settings.movie_file_path[0])
        (void)frontend_execution_movie_set_path(&execution_runtime,frontend_settings.movie_file_path,path_error,sizeof(path_error));
    frontend_audio_runtime_set_execution(&audio_runtime, &execution_runtime);
    if (!frontend_execution_set_speeds(&execution_runtime, frontend_settings.speed,
                                       frontend_settings.fast_forward_speed)) {
        fprintf(stderr, "Saved emulation speed settings are invalid\n");
        running = false;
    }
    frontend_execution_set_muted(&execution_runtime, frontend_settings.muted);
    frontend_panels_reset();
    if (!frontend_execution_register_commands(&execution_runtime)) {
        fprintf(stderr, "Could not initialize frontend commands\n");
        running = false;
    }
    if (!header_editor_frontend_register(&execution_runtime)) running = false;
    FrontendSessionActions session_actions;
    frontend_session_actions_init(&session_actions, &frontend_session,
                                  &frontend_settings, recent_path);
    session_actions.game_config = &game_config;
    frontend_session_actions_set_execution(&session_actions, &execution_runtime);
    frontend_execution_set_open_handler(&execution_runtime, frontend_session_action_open,
                                        &session_actions);
    frontend_execution_set_reload_handler(&execution_runtime, frontend_session_action_reload,
                                          &session_actions);
    FrontendDesktopUi desktop_ui;
    frontend_desktop_init(&desktop_ui, window, renderer, &frontend_settings,
                          &execution_runtime, &session_actions, settings_path);
    desktop_ui.native_windows = true;
    desktop_ui.idle_session = &frontend_session;
    if (startup_error[0]) frontend_desktop_set_status(&desktop_ui, startup_error);
    frontend_desktop_set_runtime(&desktop_ui, &audio_runtime, &video_runtime);
    if (running && !frontend_desktop_register_commands(&desktop_ui)) {
        fprintf(stderr, "Could not initialize desktop commands\n");
        running = false;
    }
    FrontendDeviceRuntime device_runtime;
    frontend_devices_init(&device_runtime, &execution_runtime, &frontend_settings, &fds_frontend_side);
    if (!frontend_devices_set_tape_paths(&device_runtime,
        tape_play_path ? tape_play_path : frontend_settings.tape_play_path,
        tape_record_path ? tape_record_path : frontend_settings.tape_record_path,
                                          path_error, sizeof(path_error))
        || !frontend_devices_register(&device_runtime)) {
        frontend_desktop_set_status(&desktop_ui, path_error);
        running = false;
    }
    FrontendStorage storage_runtime;
    if(!frontend_storage_register(&storage_runtime,&session_actions,&execution_runtime,game_db_path))running=false;
    StateRuntime state_runtime;
    state_runtime_init(&state_runtime, &frontend_settings, frontend_paths_data_dir(),
                       &execution_runtime);
    if (running && !state_runtime_register_ui(&state_runtime)) {
        fprintf(stderr, "Could not initialize save-state controls\n");
        running = false;
    }
    LifecycleFrontend lifecycle;
    if (!lifecycle_init(&lifecycle, &session_actions, &state_runtime, frontend_paths_data_dir(), path_error, sizeof(path_error)))
        frontend_desktop_set_status(&desktop_ui, path_error);
    FrontendCliHelp cli_help;
    UpdateChecker update_checker;
    char update_preferences[2048];
    frontend_paths_join(update_preferences, sizeof(update_preferences), "updates.ini");
    if (!update_checker_init(&update_checker, CUPID_VERSION, update_preferences, path_error, sizeof(path_error)))
        frontend_desktop_set_status(&desktop_ui, path_error);
    if (!lifecycle_register_ui(&lifecycle) || !frontend_cli_help_register(&cli_help) ||
        !game_config_register_ui(&game_config, &global_settings, frontend_paths_data_dir()) ||
        !update_checker_register_ui(&update_checker)) running = false;
    DebugFrontend *debug_frontend = debug_frontend_create(&execution_runtime);
    if (running && (!debug_frontend || !debug_frontend_register_ui(debug_frontend))) {
        fprintf(stderr, "Could not initialize debugger controls\n");
        running = false;
    }
    CheatFrontend *cheat_frontend = cheat_frontend_create(frontend_paths_data_dir());
    if (running && (!cheat_frontend || !cheat_frontend_register_ui(cheat_frontend))) {
        fprintf(stderr, "Could not initialize cheat controls\n"); running = false;
    }
    if (cheat_frontend) cheat_frontend_image_changed(cheat_frontend, rom_file_crc32());
    if (image_environment.cheat_override) {
        if (image_environment.cheat_path[0]) (void)cheats_load_file(image_environment.cheat_path);
        else (void)cheats_clear();
    }
    MemoryToolsFrontend *memory_frontend = memory_tools_create(cheat_frontend);
    if (running && (!memory_frontend || !memory_tools_register(memory_frontend))) {
        fprintf(stderr, "Could not initialize memory search controls\n");
        running = false;
    }
    NsfPlayer music_player = {0};
    if (running && !nsf_player_bind_frontend(&music_player, &execution_runtime, SDL_GetTicks())) {
        fprintf(stderr, "Could not initialize the music player\n");
        running = false;
    }
    if (running && !nsf_player_set_options(&music_player, &frontend_settings.nsf_player)) {
        fprintf(stderr, "Saved music-player settings are invalid\n");
        running = false;
    }
    unsigned last_music_track = UINT_MAX;
    unsigned last_music_second = UINT_MAX;
    const char *capture_protected_paths[] = {
        frontend_settings.epsm_adpcm_path, frontend_settings.fcns_kanji_path,
        device_runtime.tape_input, device_runtime.tape_output, game_db_path
    };
    state_runtime_set_protected_paths(&state_runtime, capture_protected_paths,
                                      sizeof(capture_protected_paths) / sizeof(capture_protected_paths[0]));
    NesCaptureRuntime capture_runtime = {0};
    if (running && !nes_capture_runtime_init(&capture_runtime, &execution_runtime,
        NULL, NULL, capture_protected_paths,
        sizeof(capture_protected_paths) / sizeof(capture_protected_paths[0]))) {
        fprintf(stderr, "Could not initialize capture controls\n");
        running = false;
    }
    capture_runtime.devices = &device_runtime;
    storage_runtime.capture = &capture_runtime.frontend;
    desktop_ui.capture = &capture_runtime.frontend;
    desktop_ui.devices = &device_runtime;
    desktop_ui.music = &music_player;
    const char *device_protected_paths[] = {frontend_settings.epsm_adpcm_path, frontend_settings.fcns_kanji_path, game_db_path,
        frontend_settings.state_file_path, frontend_settings.capture_paths[0],
        frontend_settings.capture_paths[1], frontend_settings.capture_paths[2]};
    frontend_devices_set_protected_paths(&device_runtime, device_protected_paths,
        sizeof(device_protected_paths) / sizeof(device_protected_paths[0]));
    nes_capture_runtime_set_video(&capture_runtime, &video_runtime);
    if (!nes_capture_runtime_preferences(&capture_runtime, &frontend_settings.movie_preferences,
                                         path_error, sizeof(path_error)))
        frontend_desktop_set_status(&desktop_ui, path_error);
    nes_hd_builder_set_protected_paths(video_runtime.builder, capture_protected_paths,
        sizeof(capture_protected_paths) / sizeof(capture_protected_paths[0]));
    if (running) {
        capture_runtime.frontend.options = frontend_settings.capture;
        for (unsigned i = 0; i < 3; ++i) {
            char capture_path_error[160] = {0};
            if (frontend_settings.capture_paths[i][0]
                && !nes_capture_frontend_set_path(&capture_runtime.frontend,
                    (FrontendSaveFileType)i, frontend_settings.capture_paths[i],
                    capture_path_error, sizeof(capture_path_error))) {
                fprintf(stderr, "%s\n", capture_path_error);
            }
        }
    }
    if (running && frontend_session.active && !frontend_video_runtime_attach_hd(&video_runtime, &frontend_session,
                                                     path_error, sizeof(path_error))) {
        frontend_desktop_set_status(&desktop_ui, path_error);
    }
    nes_hd_frontend_bind_execution(video_runtime.hd_frontend, &execution_runtime);
    FrontendPresentationHost presentation_host = {0};
    LiveFrontend live = {
        .music = &music_player, .debug = debug_frontend, .cheats = cheat_frontend, .memory = memory_frontend,
        .capture = &capture_runtime, .devices = &device_runtime, .video = &video_runtime, .lifecycle = &lifecycle, .image_environment = &image_environment,
        .presentation = &presentation_host, .desktop = &desktop_ui, .configuration = &game_config,
        .finish_capture = execution_runtime.before_machine_change, .capture_context = execution_runtime.machine_change_context
    };
    if (!frontend_presentation_host_init(&presentation_host, &video_runtime, &audio_runtime,
                                         &execution_runtime, live_presentation_changed, &live)) running = false;
    FrontendOverclock overclock_frontend = {.changed = live_overclock_changed, .userdata = &live};
    if (!frontend_overclock_register(&overclock_frontend)) running = false;
    execution_runtime.before_machine_change = live_before_machine_change;
    execution_runtime.machine_change_context = &live;
    if (!frontend_presentation_restore_settings(&video_runtime, &audio_runtime, &frontend_settings,
                                                path_error, sizeof(path_error)))
        frontend_desktop_set_status(&desktop_ui, path_error);
    session_actions.before_configuration = live_configuration_changed;
    session_actions.configuration_context = &live;
    frontend_session_actions_set_image_changed(&session_actions, live_image_changed, &live);
    state_runtime_set_restored(&state_runtime, live_state_restored, &live);
    frontend_execution_set_restore_handler(&execution_runtime, live_state_restored, &live);
    frontend_command_set_session_active(frontend_session.active);
    frontend_panel_set_session_active(frontend_session.active);
    lifecycle_image_changed(&lifecycle);
    if (running && !lifecycle_resume_startup(&lifecycle, parsed_options.rom_path != NULL || movie_path != NULL,
                                            path_error, sizeof(path_error)))
        frontend_desktop_set_status(&desktop_ui, path_error);
    if (running && !frontend_session.active && !parsed_options.rom_path && !lifecycle.resume_on_start &&
        frontend_settings.reopen_last_image && frontend_session.recent_count) {
        if (!frontend_session_action_open_recent(&session_actions, 0, path_error, sizeof(path_error)))
            frontend_desktop_set_status(&desktop_ui, path_error);
    }
    (void)update_checker_start(&update_checker, true, path_error, sizeof(path_error));
    if (!frontend_session.active && audio_dev) SDL_PauseAudioDevice(audio_dev, 1);
    if (running && movie_path) {
        bool opened = open_tas_editor
            ? tas_frontend_open(&execution_runtime, movie_path, false, path_error, sizeof(path_error))
            : frontend_execution_movie_set_path(&execution_runtime, movie_path, path_error, sizeof(path_error)) &&
              frontend_execution_movie_play(&execution_runtime, path_error, sizeof(path_error));
        if (!opened) {
            fprintf(stderr, "Movie: %s\n", path_error);
            movie_launch_failed = true;
            running = false;
        } else if (open_tas_editor) {
            (void)frontend_desktop_open_panel(&desktop_ui, TAS_PANEL);
        }
    }
    char last_capture_error[256] = {0};
    palette_tool_init();
    const double performance_frequency = (double)SDL_GetPerformanceFrequency();
    double frame_deadline = (double)SDL_GetPerformanceCounter();
    double fps_started = frame_deadline;
    unsigned fps_frames = 0;
    uint64_t timing_revision = UINT64_MAX;
    double paced_speed = 0;
    int active_vsync = -1;
    double recorder_fraction_ms = 0;
    while (running) {
        frontend_presentation_host_tick(&presentation_host);
        frontend_settings.movie_preferences = capture_runtime.frontend.preferences;
        update_checker_poll(&update_checker);
        if (update_checker.notify) {
            frontend_desktop_set_status(&desktop_ui, update_checker.message);
            (void)frontend_desktop_open_panel(&desktop_ui, UPDATE_PANEL);
            update_checker.notify = false;
        }
        while (SDL_PollEvent(&e)) {
            path_error[0] = 0;
            if (frontend_presentation_host_event(&presentation_host, &e, path_error, sizeof(path_error))) {
                if (path_error[0]) frontend_desktop_set_status(&desktop_ui, path_error);
                continue;
            }
            if (!frontend_desktop_input_captured(&desktop_ui) ||
                e.type == SDL_CONTROLLERDEVICEADDED || e.type == SDL_CONTROLLERDEVICEREMOVED)
                frontend_host_input_event(&e, &frontend_settings, &execution_runtime);
            if (e.type == SDL_WINDOWEVENT
                && e.window.windowID == SDL_GetWindowID(window)
                && e.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
                frontend_host_input_release_all();
                frontend_execution_release_host_input(&execution_runtime);
            }
            bool was_captured = frontend_desktop_input_captured(&desktop_ui);
            if (e.type != SDL_QUIT && frontend_desktop_handle_event(&desktop_ui, &e)) {
                if (desktop_ui.idle_recent_index >= 0) {
                    int recent = desktop_ui.idle_recent_index;
                    desktop_ui.idle_recent_index = -1;
                    if (!frontend_session_action_open_recent(&session_actions, (size_t)recent, path_error, sizeof(path_error)))
                        frontend_desktop_set_status(&desktop_ui, path_error);
                }
                if (!was_captured && frontend_desktop_input_captured(&desktop_ui)) {
                    frontend_host_input_release_all();
                    frontend_execution_release_host_input(&execution_runtime);
                }
                continue;
            }
            bool main_mouse_event = e.type == SDL_MOUSEMOTION
                ? e.motion.windowID == SDL_GetWindowID(window)
                : (e.type == SDL_MOUSEBUTTONDOWN || e.type == SDL_MOUSEBUTTONUP)
                    && e.button.windowID == SDL_GetWindowID(window);
            if (main_mouse_event) {
                int mouse_x, mouse_y, window_width, window_height;
                uint32_t buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
                SDL_GetWindowSize(window, &window_width, &window_height);
                SDL_Rect game_rect;
                frontend_desktop_game_rect(&desktop_ui, window_width, window_height,
                    video_width, video_height, frontend_settings.integer_scaling, &game_rect);
                bool pointer_on_screen = mouse_x >= game_rect.x && mouse_y >= game_rect.y
                    && mouse_x < game_rect.x + game_rect.w && mouse_y < game_rect.y + game_rect.h;
                int local_x = pointer_on_screen ? mouse_x - game_rect.x : 0;
                int local_y = pointer_on_screen ? mouse_y - game_rect.y : 0;
                int presented_x = pointer_on_screen && game_rect.w > 0
                    ? (int)video_runtime.frame.width * local_x / game_rect.w : -1;
                int presented_y = pointer_on_screen && game_rect.h > 0
                    ? (int)video_runtime.frame.height * local_y / game_rect.h : -1;
                unsigned pointer_side = 0;
                int pointer_x = -1, pointer_y = -1;
                if (pointer_on_screen && !frontend_video_runtime_aim(&video_runtime,
                        presented_x, presented_y, &pointer_side, &pointer_x, &pointer_y)) {
                    pointer_on_screen = false;
                    pointer_x = pointer_y = -1;
                }
                (void)pointer_side;
                int position = pointer_x >= 0 ? 0x54 + 160 * pointer_x / SCREEN_WIDTH : 0x54;
                bool zapper_on_screen = pointer_on_screen && !(buttons & SDL_BUTTON_RMASK);
                int aim_x = zapper_on_screen ? pointer_x : -1;
                int aim_y = zapper_on_screen ? pointer_y : -1;
                bool trigger = (buttons & (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) != 0;
                for (unsigned slot = 0; slot < 3; ++slot) {
                    joypad_set_paddle(slot, position, (buttons & SDL_BUTTON_LMASK) != 0);
                    joypad_set_zapper(slot, aim_x, aim_y, trigger);
                }
                if (joypad_port_device(1) == NES_PORT_SUBOR_MOUSE) {
                    if (e.type == SDL_MOUSEMOTION)
                        joypad_add_subor_mouse_motion(e.motion.xrel, e.motion.yrel);
                    joypad_set_subor_mouse_buttons((buttons & SDL_BUTTON_LMASK) != 0,
                                                   (buttons & SDL_BUTTON_RMASK) != 0);
                }
                for (unsigned port = 0; port < 2; ++port) {
                    if (joypad_port_device(port) != NES_PORT_SNES_MOUSE) continue;
                    if (e.type == SDL_MOUSEMOTION)
                        joypad_add_snes_mouse_motion(port, e.motion.xrel, e.motion.yrel);
                    joypad_set_snes_mouse_buttons(port, (buttons & SDL_BUTTON_LMASK) != 0,
                                                  (buttons & SDL_BUTTON_RMASK) != 0);
                }
                if (e.type == SDL_MOUSEMOTION
                    && joypad_expansion_device() == NES_EXPANSION_HORI_TRACK)
                    joypad_add_hori_track_motion(e.motion.xrel, e.motion.yrel);
                if (joypad_expansion_device() == NES_EXPANSION_PACHINKO)
                    joypad_set_pachinko_controls((buttons & SDL_BUTTON_LMASK) != 0,
                                                 (buttons & SDL_BUTTON_RMASK) != 0);
                oeka_kids_pointer_event(pointer_x, pointer_y, pointer_on_screen, buttons);
            }
            if (e.type == SDL_QUIT) {
                NesFileResult capture_result = nes_capture_session_stop(&capture_runtime.frontend.session);
                nes_capture_frontend_refresh(&capture_runtime.frontend);
                if (capture_result != NES_FILE_OK) {
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Capture Save Error",
                        capture_runtime.frontend.session.error, window);
                } else if (nes_movie_mode(execution_runtime.movie) != NES_MOVIE_IDLE &&
                           !frontend_execution_movie_stop(&execution_runtime, path_error, sizeof(path_error))) {
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Movie Save Error", path_error, window);
                } else if (!frontend_devices_finish(&device_runtime, path_error, sizeof(path_error))) {
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tape Save Error",
                        "The tape recording could not be saved. The emulator will remain open.", window);
                } else if (!joypad_persistent_flush() || !rom_flush_persistent()) {
                    fprintf(stderr, "Failed to save persistent data; keeping the emulator open\n");
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save Error",
                        "Cartridge, disk, or peripheral data could not be saved. The emulator will remain open so the changes are not discarded.", window);
                } else if (!lifecycle_save_last(&lifecycle, path_error, sizeof(path_error))) {
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Session Save Error", path_error, window);
                } else {
                    running = false;
                }
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && frontend_devices_handle_tape_key(&device_runtime, &e.key, path_error, sizeof(path_error))) {
                if (path_error[0]) frontend_desktop_set_status(&desktop_ui, path_error);
                continue;
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && family_basic_key_event(&e.key, NULL, NULL,
                                          cpu_total_cycles)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && subor_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && party_tap_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && boxing_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && jissen_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && extended_port_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && mat_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)) {
                char capture_error[256] = {0};
                if (nes_capture_frontend_handle_shortcut(&capture_runtime.frontend, &e.key,
                                                         capture_error, sizeof(capture_error))) {
                    if (capture_error[0]) fprintf(stderr, "%s\n", capture_error);
                    continue;
                }
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)) {
                char music_error[160] = {0};
                if (nsf_player_handle_shortcut(&music_player, &e.key,
                                               music_error, sizeof(music_error))) {
                    if (music_error[0]) fprintf(stderr, "%s\n", music_error);
                    continue;
                }
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) &&
                e.key.windowID == SDL_GetWindowID(window) && frontend_host_input_bound_player_key(
                    frontend_settings_active_profile_const(&frontend_settings), &e.key)) continue;
            
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                int down = (e.type == SDL_KEYDOWN);
    
                switch (e.key.keysym.sym) {
                    case SDLK_F10:
                        if (down && rom_is_fds()) fds_set_write_protected(!fds_write_protected());
                        break;
                    case SDLK_F9:
                        if (down && rom_is_fds() && fds_side_count()) {
                            fds_frontend_side = (fds_frontend_side + 1) % fds_side_count();
                            (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F8:
                        if (down && !e.key.repeat && barcode) cart_set_barcode(barcode);
                        if (down && !e.key.repeat && barcode_battler)
                            joypad_scan_barcode_battler(barcode_battler);
                        if (down && !e.key.repeat && rom_is_fds()) {
                            if (fds_disk_inserted()) fds_eject_disk();
                            else (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F7:
                        if (down) { palette_tool_toggle_overlay(); }
                        break;
                    case SDLK_F6:
                        if (down) { ppu_palette_reset_default(); frontend_desktop_set_status(&desktop_ui,"Palette reset"); }
                        break;
                    case SDLK_5: if (vs_enabled()) vs_set_coin(0, down != 0); break;
                    case SDLK_6: if (vs_enabled()) vs_set_coin(1, down != 0); break;
                    case SDLK_7: if (vs_dual_system()) vs_set_coin(2, down != 0); break;
                    case SDLK_8: if (vs_dual_system()) vs_set_coin(3, down != 0); break;
                    case SDLK_F1: if (vs_enabled()) vs_set_service(0, down != 0); break;
                    case SDLK_F2: if (vs_dual_system()) vs_set_service(1, down != 0); break;
                    case SDLK_m:
                        joypad_set_microphone(down != 0);
                        (void)cart_set_karaoke_input(CART_KARAOKE_MICROPHONE, down);
                        break;
                    default: break;
                }

                // Ctrl+V accepts the palette text formats handled by the parser.
                if (down && (e.key.keysym.sym == SDLK_v)) {
                    const SDL_Keymod mods = SDL_GetModState();
                    if ((mods & KMOD_CTRL) != 0) {
                        if (SDL_HasClipboardText()) {
                            char *txt = SDL_GetClipboardText();
                            if (txt) {
                                int rc = ppu_palette_load_hex_string(txt);
                                frontend_desktop_set_status(&desktop_ui,rc==0?"Palette loaded":"Invalid palette");
                                SDL_free(txt);
                                if (rc != 0) {
                                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_WARNING, "Palette Paste Error",
                                        "Clipboard text did not contain a valid palette.\n\nAccepts: 64 x RRGGBB tokens, or raw 192/1536 hex bytes.", NULL);
                                }
                            }
                        }
                    }
                }
            }

            if (e.type == SDL_DROPFILE) {
                char *dropped_f = e.drop.file;
                if (dropped_f) {
                    const char *dot = strrchr(dropped_f, '.');
                    if (dot && SDL_strcasecmp(dot, ".pal") == 0) {
                        int rc = ppu_palette_load_pal_file(dropped_f);
                        frontend_desktop_set_status(&desktop_ui,rc==0?"Palette loaded":"Invalid palette");
                        if (rc != 0)
                            SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Palette Load Error",
                                "Failed to load .pal file. Expected 192 or 1536 bytes.", window);
                    } else if (dot && (!SDL_strcasecmp(dot, ".fm2") || !SDL_strcasecmp(dot, ".fm3") ||
                                       !SDL_strcasecmp(dot, ".ctas") || !SDL_strcasecmp(dot, ".cmv") ||
                                       !SDL_strcasecmp(dot, ".movie"))) {
                        char movie_error[256] = {0};
                        if (!tas_frontend_open(&execution_runtime, dropped_f, false, movie_error, sizeof(movie_error)))
                            frontend_desktop_set_status(&desktop_ui, movie_error);
                        else {
                            (void)frontend_desktop_open_panel(&desktop_ui, TAS_PANEL);
                        }
                    } else {
                        char drop_error[256] = {0};
                        if (!frontend_session_action_open_path(
                                &session_actions, dropped_f, drop_error, sizeof(drop_error))) {
                            frontend_desktop_set_status(&desktop_ui, drop_error);
                            SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Open Image Error",
                                drop_error[0] ? drop_error : "The dropped file could not be opened.",
                                window);
                        }
                    }
                    SDL_free(dropped_f);
                }
            }
        }

        if (frontend_desktop_quit_requested(&desktop_ui)) {
            desktop_ui.quit_requested = false;
            SDL_Event quit = {0}; quit.type = SDL_QUIT; SDL_PushEvent(&quit);
        }
        // A successful quit flush must be the last chance for emulation to mutate
        // writable disk media. Do not run another frame after accepting SDL_QUIT.
        if (!running) break;

        if (live.image_changed) {
            live.image_changed = false;
            recorder_fraction_ms = 0;
            if (!frontend_session.active) {
                frontend_presentation_host_reset(&presentation_host);
                frontend_video_runtime_unload_image(&video_runtime);
                frontend_settings = global_settings;
                game_config.key[0] = 0;
                game_config.config.present = 0;
            }
            if (!frontend_presentation_restore_settings(&video_runtime, &audio_runtime, &frontend_settings,
                                                        path_error, sizeof(path_error)))
                frontend_desktop_set_status(&desktop_ui, path_error);
            if (SDL_SetWindowFullscreen(window, frontend_settings.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0) != 0)
                frontend_desktop_set_status(&desktop_ui, SDL_GetError());
            (void)frontend_command_set_checked(FRONTEND_COMMAND_FULLSCREEN, frontend_settings.fullscreen);
            frontend_settings.audio_mix.muted = frontend_settings.muted;
            frontend_video_runtime_set_composite(&video_runtime, frontend_settings.ntsc_composite);
            if (!nes_video_presentation_set(&frontend_settings.presentation, path_error, sizeof(path_error)) ||
                !nes_audio_mix_set(&frontend_settings.audio_mix, path_error, sizeof(path_error)))
                frontend_desktop_set_status(&desktop_ui, path_error);
            if (frontend_session.active && !frontend_video_runtime_attach_hd(&video_runtime, &frontend_session,
                                                  path_error, sizeof(path_error)))
                frontend_desktop_set_status(&desktop_ui, path_error);
            if (frontend_session.active && !frontend_audio_runtime_apply(&audio_runtime, &frontend_settings, true,
                                              path_error, sizeof(path_error))) {
                frontend_desktop_set_status(&desktop_ui, path_error);
            }
            if (frontend_session.active && !frontend_video_runtime_refresh(&video_runtime, path_error, sizeof(path_error))) {
                fprintf(stderr, "Video output: %s\n", path_error[0] ? path_error : "frame unavailable");
                running = false;
                break;
            }
            frontend_video_runtime_display_size(&video_runtime, &display_width, &display_height);
            video_width = (int)display_width;
            video_height = (int)display_height;
            (void)frontend_execution_set_speeds(&execution_runtime, frontend_settings.speed,
                                                frontend_settings.fast_forward_speed);
            frontend_execution_set_muted(&execution_runtime, frontend_settings.muted);
            SDL_SetWindowTitle(window, frontend_session.current_result.title[0]
                ? frontend_session.current_result.title : "Cupid NES Emulator");
        }
    
        frontend_desktop_update_activity(&desktop_ui);
        if (!frontend_session.active) {
            presentation_host.previous_end = 0;
            nes_frame_timing_reset(&presentation_host.timing.timing);
            if (audio_dev) SDL_PauseAudioDevice(audio_dev, 1);
            SDL_SetRenderDrawColor(renderer, 14, 18, 27, 255);
            SDL_RenderClear(renderer);
            frontend_desktop_render(&desktop_ui, 256, 240, NULL, NULL, "Idle");
            SDL_RenderPresent(renderer);
            SDL_Delay(8);
            continue;
        }
        double loop_speed = frontend_execution_speed(&execution_runtime);
        double loop_now = (double)SDL_GetPerformanceCounter();
        if (timing_revision != execution_runtime.timing_revision || paced_speed != loop_speed ||
            loop_now - frame_deadline > performance_frequency * 0.050) {
            frame_deadline = loop_now;
            timing_revision = execution_runtime.timing_revision;
            paced_speed = loop_speed;
        }
        int wanted_vsync = frontend_settings.vsync && loop_speed == 1.0;
        if (active_vsync != wanted_vsync) {
            (void)SDL_RenderSetVSync(renderer, wanted_vsync);
            active_vsync = wanted_vsync;
        }
        if (!execution_runtime.suspend_reasons && !execution_runtime.rewind_held &&
            execution_control_should_run_frame(&execution_runtime.execution))
            nes_capture_frontend_begin_frame(&capture_runtime.frontend);
        uint64_t host_emulation_start = SDL_GetPerformanceCounter();
        uint64_t frame_start_cycles = cpu_total_cycles;
        bool ran_frame = frontend_execution_run_frame(&execution_runtime);
        if (ran_frame) frontend_devices_frame_complete(&device_runtime);
        if (ran_frame && !execution_runtime.rewind_held) {
            recorder_fraction_ms += 1000.0 / nes_timing()->fps;
            uint64_t elapsed_ms = (uint64_t)recorder_fraction_ms;
            recorder_fraction_ms -= (double)elapsed_ms;
            if (!state_recorder_tick(&lifecycle.recorder, elapsed_ms, path_error, sizeof(path_error)))
                frontend_desktop_set_status(&desktop_ui, path_error);
        }
        uint64_t frame_elapsed_cycles = ran_frame && cpu_total_cycles >= frame_start_cycles
            ? cpu_total_cycles - frame_start_cycles : 0;
        uint64_t host_render_start = SDL_GetPerformanceCounter();
        char video_error[192] = {0};
        bool video_ready = frontend_video_runtime_refresh(&video_runtime,
                                                          video_error, sizeof(video_error));
        nes_capture_frontend_end_frame(&capture_runtime.frontend, ran_frame && video_ready);
        if (!video_ready) {
            fprintf(stderr, "Video output: %s\n",
                    video_error[0] ? video_error : "frame unavailable");
            running = false;
            break;
        }
        if (strcmp(last_capture_error, capture_runtime.frontend.session.error)) {
            snprintf(last_capture_error, sizeof(last_capture_error), "%s", capture_runtime.frontend.session.error);
            if (last_capture_error[0]) fprintf(stderr, "%s\n", last_capture_error);
        }
        char music_error[160] = {0};
        (void)nsf_player_poll(&music_player, music_error, sizeof(music_error));
        if (music_error[0]) fprintf(stderr, "%s\n", music_error);
        NsfPlayerInfo music_info;
        if (nsf_player_info(&music_player, &music_info)) {
            unsigned seconds = music_info.position_seconds > UINT_MAX
                ? UINT_MAX : (unsigned)music_info.position_seconds;
            if (music_info.track != last_music_track || seconds != last_music_second) {
                char title[512];
                snprintf(title, sizeof(title), "Cupid NES | %s | Track %u/%u | %u:%02u",
                         music_info.metadata->title, music_info.track + 1u,
                         (unsigned)music_info.metadata->total_songs, seconds / 60u, seconds % 60u);
                SDL_SetWindowTitle(window, title);
                last_music_track = music_info.track;
                last_music_second = seconds;
            }
        }

        frontend_video_runtime_display_size(&video_runtime, &display_width, &display_height);
        video_width = (int)display_width;
        video_height = (int)display_height;
        SDL_RenderClear(renderer);
        int ww = 0, hh = 0;
        SDL_GetWindowSize(window, &ww, &hh);
        SDL_Rect game_rect;
        frontend_desktop_game_rect(&desktop_ui, ww, hh, video_width, video_height,
                                           frontend_settings.integer_scaling, &game_rect);
        SDL_RenderCopy(renderer, frontend_video_runtime_texture(&video_runtime), NULL, &game_rect);
        frontend_desktop_render(&desktop_ui, video_width, video_height,
            frontend_session.current_result.title,
            nes_region_name(nes_timing()->region),
            nes_netplay_mode(execution_runtime.netplay) == NES_NETPLAY_CONNECTED
                || nes_netplay_mode(execution_runtime.netplay) == NES_NETPLAY_LISTENING
                ? frontend_netplay_status(execution_runtime.network)
                : execution_runtime.rewind_held ? "Rewinding"
                : frontend_execution_paused(&execution_runtime) ? "Paused" : "Running");
        uint64_t host_present_start = SDL_GetPerformanceCounter();
        SDL_RenderPresent(renderer);
        uint64_t host_present_end = SDL_GetPerformanceCounter();
    
        double fps_now = (double)SDL_GetPerformanceCounter();
        if (ran_frame) ++fps_frames;
        if (fps_now - fps_started >= performance_frequency * 0.5) {
            frontend_desktop_set_fps(&desktop_ui, fps_frames * performance_frequency / (fps_now - fps_started));
            fps_frames = 0; fps_started = fps_now;
        }
        double speed = frontend_execution_speed(&execution_runtime);
        if (ran_frame) {
            frame_deadline += (double)frame_elapsed_cycles
                            * performance_frequency / (nes_timing()->cpu_hz * speed);
        }
        double current_ticks = (double)SDL_GetPerformanceCounter();
        if (!ran_frame) {
            frame_deadline = current_ticks;
            double rewind_wait = performance_frequency / nes_timing()->fps - (current_ticks - loop_now);
            if (execution_runtime.rewind_held) {
                if (rewind_wait > 0) SDL_Delay((Uint32)(rewind_wait * 1000.0 / performance_frequency));
            } else SDL_Delay(8);
        } else if (frame_deadline > current_ticks) {
            // Carry fractional milliseconds into the next deadline instead of
            // running every frame early after truncating SDL's delay argument.
            SDL_Delay((Uint32)((frame_deadline - current_ticks) * 1000.0 / performance_frequency));
        } else if (current_ticks - frame_deadline > performance_frequency * 0.050) {
            frame_deadline = current_ticks;
        }
        if (ran_frame) (void)frontend_presentation_host_record(&presentation_host, host_emulation_start,
            host_render_start, host_present_start, host_present_end, SDL_GetPerformanceCounter());
        else {
            presentation_host.previous_end = 0;
            nes_frame_timing_reset(&presentation_host.timing.timing);
        }
    }
    (void)frontend_presentation_capture_settings(&video_runtime, &audio_runtime, &frontend_settings);
    frontend_desktop_update_window_settings(&desktop_ui);
    frontend_presentation_host_shutdown(&presentation_host);
    execution_runtime.before_machine_change = live.finish_capture;
    execution_runtime.machine_change_context = live.capture_context;
    bool capture_saved = nes_capture_runtime_shutdown(&capture_runtime) == NES_FILE_OK;
    if (!capture_saved) fprintf(stderr, "%s\n", capture_runtime.frontend.session.error);
    frontend_settings.nsf_player = music_player.options;
    frontend_settings.capture = capture_runtime.frontend.options;
    frontend_settings.movie_preferences = capture_runtime.frontend.preferences;
    memcpy(frontend_settings.capture_paths,capture_runtime.frontend.paths,sizeof(frontend_settings.capture_paths));
    snprintf(frontend_settings.tape_play_path,sizeof(frontend_settings.tape_play_path),"%s",device_runtime.tape_input);
    snprintf(frontend_settings.tape_record_path,sizeof(frontend_settings.tape_record_path),"%s",device_runtime.tape_output);
    snprintf(frontend_settings.movie_file_path,sizeof(frontend_settings.movie_file_path),"%s",execution_runtime.movie_path);
    bool tape_saved = frontend_devices_finish(&device_runtime, path_error, sizeof(path_error));
    frontend_devices_unregister();
    frontend_storage_unregister();
    update_checker_shutdown(&update_checker);
    lifecycle_unregister_ui();
    frontend_cli_help_unregister();
    game_config_unregister_ui();
    frontend_overclock_unregister();
    header_editor_frontend_unregister();
    state_runtime_shutdown(&state_runtime);
    frontend_execution_shutdown(&execution_runtime);
    memory_tools_destroy(memory_frontend);
    cheat_frontend_destroy(cheat_frontend);
    debug_frontend_destroy(debug_frontend);
    nsf_player_shutdown(&music_player);
    frontend_desktop_shutdown(&desktop_ui);
    debugger_shutdown();
    frontend_video_runtime_shutdown(&video_runtime);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    frontend_audio_runtime_shutdown(&audio_runtime);
    apu_audio_shutdown_state(&apu);
    frontend_host_input_shutdown();
    bool tape_failed = family_basic_tape_failed();
    bool peripheral_saved = joypad_persistent_shutdown();
    if (tape_failed) fprintf(stderr, "Tape recording stopped because the capture buffer could not grow\n");
    family_basic_shutdown();
    if (!unload_rom()) {
        fprintf(stderr, "Failed to unload modified FDS media\n");
        SDL_Quit();
        return 1;
    }
    SDL_Quit();
    bool settings_saved = game_config_save_globals(&game_config, settings_path, &frontend_settings,
                                                 &settings_report);
    if (!settings_saved) fprintf(stderr, "%s\n", settings_report.message);
    frontend_paths_shutdown();
    return !movie_launch_failed && tape_saved && !tape_failed && peripheral_saved && capture_saved && settings_saved ? 0 : 1;
}

int main(int argc, char *argv[]) {
    return frontend_application_entry_utf8(argc, argv, application_main);
}
