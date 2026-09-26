/*
 * settings_core.c - Settings validation and hardware transactions
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "settings.h"
#include "frontend_session.h"
#include "../rom/mapper.h"
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

bool frontend_settings_presentation_valid(const FrontendSettings *settings) {
    if (!settings || !memchr(settings->audio_backend, 0, sizeof(settings->audio_backend)) ||
        (strcmp(settings->audio_backend, "default") && strcmp(settings->audio_backend, "wasapi") &&
         strcmp(settings->audio_backend, "directsound")) ||
        !memchr(settings->shader_path, 0, sizeof(settings->shader_path)) || strchr(settings->shader_path, '\n') ||
        strchr(settings->shader_path, '\r') || settings->shader_parameter_count > 64) {
        return false;
    }
    for (size_t i = 0; i < settings->shader_parameter_count; ++i) {
        const char *name = settings->shader_parameters[i].name;
        if (!name[0] || !memchr(name, 0, sizeof(settings->shader_parameters[i].name)) ||
            !isfinite(settings->shader_parameters[i].value)) {
            return false;
        }
        for (const char *p = name; *p; ++p) {
            if (!isalnum((unsigned char)*p) && *p != '_') {
                return false;
            }
        }
        for (size_t j = 0; j < i; ++j) {
            if (!strcmp(settings->shader_parameters[j].name, name)) {
                return false;
            }
        }
    }
    return true;
}

bool frontend_settings_validate(const FrontendSettings *settings, char *error, size_t error_size) {
    if (!settings) {
        return false;
    }
    if (!frontend_settings_presentation_valid(settings)) {
        if (error && error_size) {
            snprintf(error, error_size, "Audio backend or shader settings are invalid");
        }
        return false;
    }
#define SETTINGS_FAIL(message)                                                                                         \
    do {                                                                                                               \
        if (error && error_size)                                                                                       \
            snprintf(error, error_size, "%s", (message));                                                              \
        return false;                                                                                                  \
    } while (0)
    if ((unsigned)settings->region_mode > NES_REGION_MODE_DENDY ||
        (unsigned)settings->console_model > NES_CONSOLE_HVC101) {
        SETTINGS_FAIL("Timing or console selection is invalid");
    }
    if (!isfinite(settings->speed) || settings->speed < 0.1 || settings->speed > 16.0 ||
        !isfinite(settings->fast_forward_speed) || settings->fast_forward_speed < 0.1 ||
        settings->fast_forward_speed > 16.0) {
        SETTINGS_FAIL("Emulation speed must be from 0.1x to 16x");
    }
    if (!settings->rewind_step_frames || settings->rewind_step_frames > 30) {
        SETTINGS_FAIL("Rewind speed must be from 1 to 30 frames per activation");
    }
    if (settings->rewind_seconds > 60 || settings->run_ahead_frames > 4) {
        SETTINGS_FAIL("Rewind must be at most 60 seconds and run-ahead at most 4 frames");
    }
    if (settings->window_width < 320 || settings->window_width > 16384 || settings->window_height < 240 ||
        settings->window_height > 16384) {
        SETTINGS_FAIL("Window size is outside the supported range");
    }
    if (settings->recent_file_limit > FRONTEND_RECENT_MAX) {
        SETTINGS_FAIL("Recent-file history length is invalid");
    }
    if ((unsigned)settings->aspect_mode > FRONTEND_ASPECT_4_3) {
        SETTINGS_FAIL("Video aspect ratio is invalid");
    }
    if (!nes_pixel_filter_validate(&settings->pixel_filter, error, error_size)) {
        return false;
    }
    if (!ntsc_composite_validate(&settings->ntsc_picture, error, error_size)) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!nes_video_overscan_valid(settings->presentation.overscan[i])) {
            SETTINGS_FAIL("Video overscan must leave visible pixels on every side");
        }
    }
    if (settings->audio_sample_rate < 8000 || settings->audio_sample_rate > 192000 ||
        settings->audio_buffer_samples < 64 || settings->audio_buffer_samples > 8192 ||
        settings->audio_mix.master_volume > 100) {
        SETTINGS_FAIL("Audio output settings are outside the supported range");
    }
    for (unsigned i = 0; i < NES_AUDIO_CHANNEL_COUNT; ++i) {
        if (settings->audio_mix.volume[i] > 200 || settings->audio_mix.pan[i] < -100 ||
            settings->audio_mix.pan[i] > 100) {
            SETTINGS_FAIL("Audio channel volume or panning is outside the supported range");
        }
    }
    if (settings->state_slot >= NES_STATE_SLOT_COUNT || settings->zapper_radius > NES_ZAPPER_MAX_RADIUS) {
        SETTINGS_FAIL("State slot or light-gun radius is invalid");
    }
    if ((unsigned)settings->input.adapter > NES_ADAPTER_FAMICOM_FOUR ||
        (unsigned)settings->input.ports[0] > NES_PORT_VIRTUAL_BOY ||
        (unsigned)settings->input.ports[1] > NES_PORT_VIRTUAL_BOY ||
        (unsigned)settings->input.expansion > NES_EXPANSION_FCNS_CONTROLLER) {
        SETTINGS_FAIL("Controller connector selection is invalid");
    }
    if (!settings->profile_count || settings->profile_count > FRONTEND_SETTINGS_MAX_PROFILES ||
        !frontend_settings_active_profile_const(settings)) {
        SETTINGS_FAIL("The active controller binding profile does not exist");
    }
    if (settings->startup_phase_set && settings->startup_seed_set) {
        SETTINGS_FAIL("Startup phase and seed are mutually exclusive");
    }
    if (settings->startup_cpu_offset > 15 || settings->startup_ppu_phase > 4 || settings->cart_dips > 255) {
        SETTINGS_FAIL("Advanced hardware value is outside the supported range");
    }
    if ((unsigned)settings->cpu_revision > APU_CPU_REVISION_LATE_2A03 ||
        (unsigned)settings->ppu_revision > PPU_REVISION_2C02_E_PLUS ||
        (unsigned)settings->ram_power_state > NES_RAM_POWER_RANDOM) {
        SETTINGS_FAIL("Advanced hardware selection is invalid");
    }
    if (settings->nsf_player.silence_ms < 10 || settings->nsf_player.silence_ms > 600000 ||
        !isfinite(settings->nsf_player.silence_threshold) || settings->nsf_player.silence_threshold < 0.0f ||
        settings->nsf_player.silence_threshold > 0.1f) {
        SETTINGS_FAIL("Music silence detection settings are invalid");
    }
    if (settings->capture.sample_rate < 8000 || settings->capture.sample_rate > 192000 ||
        settings->capture.byte_limit < 1024 || settings->capture.byte_limit > UINT32_MAX ||
        (unsigned)settings->capture.codec > NES_CAPTURE_CODEC_ZMBV ||
        (unsigned)settings->capture.format > NES_CAPTURE_FORMAT_GIF || settings->capture.compression_level > 9 ||
        settings->capture.gif_scale < 1 || settings->capture.gif_scale > 4 ||
        !nes_movie_preferences_valid(&settings->movie_preferences)) {
        SETTINGS_FAIL("Capture settings are outside the supported range");
    }
    if (settings->overclock.postrender_scanlines > NES_OVERCLOCK_MAX_SCANLINES ||
        settings->overclock.vblank_scanlines > NES_OVERCLOCK_MAX_SCANLINES) {
        SETTINGS_FAIL("CPU overclock scanline count is outside the supported range");
    }
    if (error && error_size) {
        error[0] = '\0';
    }
    return true;
#undef SETTINGS_FAIL
}

bool frontend_settings_apply_core(const FrontendSettings *settings, char *error, size_t error_size) {
    if (!settings) {
        return false;
    }
    NesInputConfiguration previous = {.adapter = joypad_adapter(),
                                      .ports = {joypad_port_device(0), joypad_port_device(1)},
                                      .expansion = joypad_expansion_device()};
    uint8_t previous_overrides = joypad_configuration_overrides();
    unsigned previous_radius = joypad_zapper_radius();
    NesRegionMode previous_region = nes_region_mode();
    NesConsoleModel previous_console = nes_console_model();
    ApuCpuRevision previous_cpu_revision = apu_get_cpu_revision();
    bool previous_noise_mode = apu_noise_mode_disabled();
    bool previous_duty_swap = apu_swap_duty_cycles_enabled();
    NesRamPowerOnState previous_ram_power = nes_ram_power_on_state();
    bool previous_random_vblank = nes_randomize_vblank_enabled();
    PpuRevision previous_ppu_revision = ppu_revision();
    bool previous_oam_row = ppu_oam_row_corruption_worst_case();
    bool previous_ppu_startup = ppu_startup_write_restriction_enabled();
    bool previous_oam_decay = ppu_oam_decay_enabled();
    bool previous_sprite_wrap = ppu_sprite_eval_wrap_bug_enabled();
    bool previous_oamdata = ppu_oamdata_read_disabled();
    bool previous_palette = ppu_palette_readback_disabled();
    bool previous_ppu_reset = ppu_reset_suppression_enabled();
    bool previous_mmc3_a = strcmp(cart_mmc3_revision_name(), "a") == 0;
    unsigned previous_cart_dips = cart_dip_switches();
    NesOverclockConfig previous_overclock = nes_overclock_config();

    NesInputConfiguration selected = settings->input;
    if (settings->cli_overrides & FRONTEND_OVERRIDE_ADAPTER) {
        selected.adapter = previous.adapter;
    }
    if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT1) {
        selected.ports[0] = previous.ports[0];
    }
    if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT2) {
        selected.ports[1] = previous.ports[1];
    }
    if (settings->cli_overrides & FRONTEND_OVERRIDE_EXPANSION) {
        selected.expansion = previous.expansion;
    }

    bool valid =
        ((settings->cli_overrides & FRONTEND_OVERRIDE_REGION) || nes_set_region_mode(settings->region_mode)) &&
        ((settings->cli_overrides & FRONTEND_OVERRIDE_CONSOLE) || nes_set_console_model(settings->console_model)) &&
        joypad_apply_configuration(&selected) &&
        ((settings->cli_overrides & FRONTEND_OVERRIDE_ZAPPER_RADIUS) ||
         joypad_set_zapper_radius(settings->zapper_radius)) &&
        joypad_configuration_valid();
    valid = valid && ((settings->cli_overrides & FRONTEND_OVERRIDE_CPU_REVISION) ||
                      apu_set_cpu_revision(settings->cpu_revision));
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_APU_NOISE_MODE)) {
        apu_set_disable_noise_mode(settings->apu_disable_noise_mode);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_APU_DUTY)) {
        apu_set_swap_duty_cycles(settings->apu_swap_duty_cycles);
    }
    valid = valid && ((settings->cli_overrides & FRONTEND_OVERRIDE_RAM_POWER) ||
                      nes_set_ram_power_on_state(settings->ram_power_state));
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_RANDOM_VBLANK)) {
        nes_set_randomize_vblank(settings->randomize_vblank);
    }
    valid = valid &&
            ((settings->cli_overrides & FRONTEND_OVERRIDE_PPU_REVISION) || ppu_set_revision(settings->ppu_revision));
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_OAM_ROW)) {
        ppu_set_oam_row_corruption_worst_case(settings->ppu_oam_row_corruption);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_STARTUP)) {
        ppu_set_startup_write_restriction(settings->ppu_startup_restriction);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_OAM_DECAY)) {
        ppu_set_oam_decay(settings->ppu_oam_decay);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_SPRITE_WRAP)) {
        ppu_set_sprite_eval_wrap_bug(settings->ppu_sprite_eval_wrap_bug);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_OAMDATA)) {
        ppu_set_oamdata_read_disabled(settings->ppu_oamdata_read_disabled);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_PALETTE)) {
        ppu_set_palette_readback_disabled(settings->ppu_palette_readback_disabled);
    }
    if (!(settings->cli_overrides & FRONTEND_OVERRIDE_PPU_RESET)) {
        ppu_set_reset_suppression(settings->ppu_reset_suppression);
    }
    valid = valid && ((settings->cli_overrides & FRONTEND_OVERRIDE_MMC3_REVISION) ||
                      cart_set_mmc3_revision_name(settings->mmc3_revision_a ? "a" : "standard"));
    valid = valid &&
            ((settings->cli_overrides & FRONTEND_OVERRIDE_CART_DIPS) || cart_set_dip_switches(settings->cart_dips));
    valid = valid && nes_set_overclock_config(&settings->overclock);
    if (valid) {
        uint8_t overrides = settings->saved_input_overrides;
        if (settings->cli_overrides & FRONTEND_OVERRIDE_ADAPTER) {
            overrides |= NES_INPUT_OVERRIDE_ADAPTER;
        }
        if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT1) {
            overrides |= NES_INPUT_OVERRIDE_PORT1;
        }
        if (settings->cli_overrides & FRONTEND_OVERRIDE_PORT2) {
            overrides |= NES_INPUT_OVERRIDE_PORT2;
        }
        if (settings->cli_overrides & FRONTEND_OVERRIDE_EXPANSION) {
            overrides |= NES_INPUT_OVERRIDE_EXPANSION;
        }
        joypad_set_configuration_overrides(overrides);
        return true;
    }
    (void)nes_set_region_mode(previous_region);
    (void)nes_set_console_model(previous_console);
    (void)joypad_apply_configuration(&previous);
    (void)joypad_set_zapper_radius(previous_radius);
    joypad_set_configuration_overrides(previous_overrides);
    (void)apu_set_cpu_revision(previous_cpu_revision);
    apu_set_disable_noise_mode(previous_noise_mode);
    apu_set_swap_duty_cycles(previous_duty_swap);
    (void)nes_set_ram_power_on_state(previous_ram_power);
    nes_set_randomize_vblank(previous_random_vblank);
    (void)ppu_set_revision(previous_ppu_revision);
    ppu_set_oam_row_corruption_worst_case(previous_oam_row);
    ppu_set_startup_write_restriction(previous_ppu_startup);
    ppu_set_oam_decay(previous_oam_decay);
    ppu_set_sprite_eval_wrap_bug(previous_sprite_wrap);
    ppu_set_oamdata_read_disabled(previous_oamdata);
    ppu_set_palette_readback_disabled(previous_palette);
    ppu_set_reset_suppression(previous_ppu_reset);
    (void)cart_set_mmc3_revision_name(previous_mmc3_a ? "a" : "standard");
    (void)cart_set_dip_switches(previous_cart_dips);
    (void)nes_set_overclock_config(&previous_overclock);
    if (error && error_size) {
        snprintf(error, error_size, "Saved hardware or controller settings conflict");
    }
    return false;
}
