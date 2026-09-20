/*
 * desktop_ui.c - SDL desktop menus, toolbar, status, and settings interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "desktop_ui.h"
#include "app_paths.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "ui_font.h"
#include "../joypad/joypad.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include <stdio.h>
#include <string.h>

enum { MENU_H = 28, TOOLBAR_H = 36, STATUS_H = 24, MODAL_W = 620, MODAL_H = 430 };
static const char *const menu_names[] = {"File", "Emulation", "View", "Tools", "Help"};
static const char *const setting_categories[] = {
    "General", "Emulation", "Video", "Audio", "Controllers",
    "Media and firmware", "Files and storage", "Advanced hardware"
};

static void fill(SDL_Renderer *r, SDL_Rect rect, Uint8 red, Uint8 green, Uint8 blue, Uint8 alpha) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, red, green, blue, alpha);
    SDL_RenderFillRect(r, &rect);
}

static bool inside(int x, int y, SDL_Rect rect) {
    return x >= rect.x && y >= rect.y && x < rect.x + rect.w && y < rect.y + rect.h;
}

static void copy_status(FrontendDesktopUi *ui, const char *text) {
    if (!ui) return;
    snprintf(ui->status, sizeof(ui->status), "%s", text ? text : "");
    ui->status_until = SDL_GetTicks() + 6000;
}

static void visible_text(char *target, size_t capacity, const char *text) {
    if (!target || !capacity) return;
    if (!text) text = "";
    size_t size = strlen(text);
    if (size < capacity) {
        memcpy(target, text, size + 1);
        return;
    }
    if (capacity < 4) {
        target[0] = '\0';
        return;
    }
    size_t prefix = capacity - 4;
    while (prefix && ((unsigned char)text[prefix] & 0xC0u) == 0x80u) --prefix;
    memcpy(target, text, prefix);
    memcpy(target + prefix, "...", 4);
}

void frontend_desktop_set_status(FrontendDesktopUi *ui, const char *message) {
    copy_status(ui, message);
}

void frontend_desktop_set_fps(FrontendDesktopUi *ui, double fps) {
    if (ui) ui->fps = fps;
}

static void sync_scale(FrontendDesktopUi *ui) {
    if (!ui || !ui->window || !ui->renderer) return;
    int w = 0, h = 0;
    SDL_GetWindowSize(ui->window, &w, &h);
    if (w > 0 && h > 0) SDL_RenderSetLogicalSize(ui->renderer, w, h);
    float ddpi = 96.0f;
    int display = SDL_GetWindowDisplayIndex(ui->window);
    if (display >= 0) (void)SDL_GetDisplayDPI(display, &ddpi, NULL, NULL);
    ui->ui_scale = ddpi >= 144.0f ? 2 : 1;
}

void frontend_desktop_compute_game_rect(int ww, int wh, int vw, int vh,
                                        bool integer_scaling, SDL_Rect *rect) {
    if (!rect) return;
    int top = MENU_H + TOOLBAR_H;
    int usable_h = wh - top - STATUS_H;
    if (ww <= 0 || usable_h <= 0 || vw <= 0 || vh <= 0) {
        *rect = (SDL_Rect){0, top, 0, 0};
        return;
    }
    double scale = (double)ww / vw;
    double vertical = (double)usable_h / vh;
    if (vertical < scale) scale = vertical;
    if (integer_scaling && scale >= 1.0) scale = (int)scale;
    int w = (int)(vw * scale);
    int h = (int)(vh * scale);
    rect->x = (ww - w) / 2;
    rect->y = top + (usable_h - h) / 2;
    rect->w = w;
    rect->h = h;
}

static bool invoke_command(FrontendDesktopUi *ui, unsigned id) {
    char error[256] = {0};
    bool ok = frontend_command_invoke(id, error, sizeof(error));
    if (error[0]) copy_status(ui, error);
    return ok;
}

static void set_settings_open(FrontendDesktopUi *ui, bool open) {
    if (!ui || ui->settings_open == open) return;
    if (open) {
        ui->staged = *ui->settings;
        ui->settings_category = 0;
        ui->settings_row = 0;
        ui->settings_scroll = 0;
        ui->settings_open = true;
        if (ui->execution && ui->settings->pause_on_ui && !frontend_execution_paused(ui->execution)) {
            ui->paused_for_ui = invoke_command(ui, FRONTEND_COMMAND_PAUSE);
        }
    } else {
        ui->settings_open = false;
        SDL_StopTextInput();
        ui->edit_text_active = false;
        ui->capture_binding = false;
        if (ui->execution && ui->paused_for_ui && frontend_execution_paused(ui->execution))
            (void)invoke_command(ui, FRONTEND_COMMAND_PAUSE);
        ui->paused_for_ui = false;
    }
}

static bool settings_layers_enabled(const FrontendSettings *settings) {
    return settings && (!settings->presentation.show_background
        || !settings->presentation.show_sprites);
}

static void restore_runtime_settings(FrontendDesktopUi *ui,
                                     const FrontendSettings *previous) {
    if (!ui || !previous) return;
    char ignored[160] = {0};
    (void)frontend_settings_apply_core(previous, ignored, sizeof(ignored));
    (void)nes_video_presentation_set(&previous->presentation, ignored, sizeof(ignored));
    (void)nes_audio_mix_set(&previous->audio_mix, ignored, sizeof(ignored));
    if (ui->execution) {
        (void)frontend_execution_set_speeds(ui->execution, previous->speed,
                                            previous->fast_forward_speed);
        (void)frontend_execution_set_rewind_seconds(ui->execution, previous->rewind_seconds);
        (void)frontend_execution_set_run_ahead(ui->execution, previous->run_ahead_frames);
        frontend_execution_set_muted(ui->execution, previous->muted);
    }
    if (ui->window)
        (void)SDL_SetWindowFullscreen(ui->window,
            previous->fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    if (ui->renderer) (void)SDL_RenderSetVSync(ui->renderer, previous->vsync ? 1 : 0);
    if (ui->video) {
        if (!frontend_settings_cli_overridden(previous, FRONTEND_OVERRIDE_VIDEO_FILTER))
            frontend_video_runtime_set_composite(ui->video, previous->ntsc_composite);
        (void)frontend_video_runtime_refresh(ui->video, ignored, sizeof(ignored));
    }
}

static bool save_settings(FrontendDesktopUi *ui) {
    if (!ui || !ui->settings) return false;
    ui->staged.audio_mix.muted = ui->staged.muted;
    char error[256] = {0};
    if (!frontend_settings_validate(&ui->staged, error, sizeof(error))) {
        copy_status(ui, error[0] ? error : "Settings are invalid");
        return false;
    }

    FrontendSettings previous = *ui->settings;
    FrontendAudioPrepared prepared_audio = {0};
    if (ui->audio
        && !frontend_audio_runtime_prepare(ui->audio, &ui->staged, &prepared_audio,
                                           error, sizeof(error))) {
        copy_status(ui, error);
        return false;
    }

    bool previous_layers = settings_layers_enabled(&previous);
    bool staged_layers = settings_layers_enabled(&ui->staged);
    bool trace_prepared = false;
    if (staged_layers && !previous_layers) {
        if (!nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, true)) {
            frontend_audio_runtime_cancel(&prepared_audio);
            copy_status(ui, "Could not allocate video layer storage");
            return false;
        }
        trace_prepared = true;
    }

    bool fullscreen_changed = ui->window && previous.fullscreen != ui->staged.fullscreen;
    if (fullscreen_changed
        && SDL_SetWindowFullscreen(ui->window,
            ui->staged.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0) != 0) {
        if (trace_prepared) (void)nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false);
        frontend_audio_runtime_cancel(&prepared_audio);
        copy_status(ui, SDL_GetError());
        return false;
    }
    bool vsync_changed = ui->renderer && previous.vsync != ui->staged.vsync;
    if (vsync_changed && SDL_RenderSetVSync(ui->renderer, ui->staged.vsync ? 1 : 0) != 0) {
        if (fullscreen_changed)
            (void)SDL_SetWindowFullscreen(ui->window,
                previous.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
        if (trace_prepared) (void)nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false);
        frontend_audio_runtime_cancel(&prepared_audio);
        copy_status(ui, SDL_GetError());
        return false;
    }

    if (!frontend_settings_apply_core(&ui->staged, error, sizeof(error))) goto rollback;
    if (ui->execution) {
        if (!frontend_execution_set_speeds(ui->execution, ui->staged.speed,
                                           ui->staged.fast_forward_speed)
            || !frontend_execution_set_rewind_seconds(ui->execution, ui->staged.rewind_seconds)
            || !frontend_execution_set_run_ahead(ui->execution, ui->staged.run_ahead_frames)) {
            snprintf(error, sizeof(error), "Replay or speed settings could not be applied");
            goto rollback;
        }
        frontend_execution_set_muted(ui->execution, ui->staged.muted);
    }
    if (!nes_video_presentation_set(&ui->staged.presentation, error, sizeof(error))) goto rollback;
    if (!nes_audio_mix_set(&ui->staged.audio_mix, error, sizeof(error))) goto rollback;
    if (ui->video) {
        if (!frontend_settings_cli_overridden(&ui->staged, FRONTEND_OVERRIDE_VIDEO_FILTER))
            frontend_video_runtime_set_composite(ui->video, ui->staged.ntsc_composite);
        if (!frontend_video_runtime_refresh(ui->video, error, sizeof(error))) goto rollback;
    }

    if (ui->settings_path) {
        FrontendSettingsReport report;
        if (!frontend_settings_save(ui->settings_path, &ui->staged, &report)) {
            snprintf(error, sizeof(error), "%s", report.message);
            goto rollback;
        }
    }

    frontend_audio_runtime_commit(ui->audio, &prepared_audio);
    *ui->settings = ui->staged;
    if (ui->video) ui->video->settings = ui->settings;
    (void)frontend_command_set_checked(FRONTEND_COMMAND_FULLSCREEN, ui->settings->fullscreen);
    (void)frontend_command_set_checked(FRONTEND_COMMAND_MUTE, ui->settings->muted);
    copy_status(ui, "Settings applied");
    return true;

rollback:
    frontend_audio_runtime_cancel(&prepared_audio);
    restore_runtime_settings(ui, &previous);
    if (trace_prepared && !previous_layers)
        (void)nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false);
    copy_status(ui, error[0] ? error : "Settings could not be applied");
    return false;
}

static bool command_settings(void *context, char *error, size_t error_size) {
    (void)error; (void)error_size;
    set_settings_open(context, true);
    return true;
}

static bool command_fullscreen(void *context, char *error, size_t error_size) {
    FrontendDesktopUi *ui = context;
    ui->settings->fullscreen = !ui->settings->fullscreen;
    if (SDL_SetWindowFullscreen(ui->window,
        ui->settings->fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0) != 0) {
        if (error && error_size) snprintf(error, error_size, "%s", SDL_GetError());
        ui->settings->fullscreen = !ui->settings->fullscreen;
        return false;
    }
    (void)frontend_command_set_checked(FRONTEND_COMMAND_FULLSCREEN, ui->settings->fullscreen);
    return true;
}

static bool command_mute(void *context, char *error, size_t error_size) {
    (void)error; (void)error_size;
    FrontendDesktopUi *ui = context;
    ui->settings->muted = !ui->settings->muted;
    if (ui->execution) frontend_execution_set_muted(ui->execution, ui->settings->muted);
    (void)frontend_command_set_checked(FRONTEND_COMMAND_MUTE, ui->settings->muted);
    return true;
}

void frontend_desktop_init(FrontendDesktopUi *ui, SDL_Window *window,
                           SDL_Renderer *renderer, FrontendSettings *settings,
                           FrontendExecutionRuntime *execution,
                           FrontendSessionActions *sessions,
                           const char *settings_path) {
    if (!ui) return;
    memset(ui, 0, sizeof(*ui));
    ui->window = window;
    ui->renderer = renderer;
    ui->settings = settings;
    ui->execution = execution;
    ui->sessions = sessions;
    ui->settings_path = settings_path;
    ui->settings_player = 0;
    sync_scale(ui);
}

void frontend_desktop_set_runtime(FrontendDesktopUi *ui,
                                  FrontendAudioRuntime *audio,
                                  FrontendVideoRuntime *video) {
    if (!ui) return;
    ui->audio = audio;
    ui->video = video;
}

bool frontend_desktop_register_commands(FrontendDesktopUi *ui) {
    if (!ui || !ui->settings) return false;
    const FrontendCommandSpec specs[] = {
        {FRONTEND_COMMAND_SETTINGS, "Settings...", "File", "Ctrl+,", 0, command_settings, ui},
        {FRONTEND_COMMAND_FULLSCREEN, "Fullscreen", "View", "Alt+Enter",
         FRONTEND_COMMAND_CHECKABLE, command_fullscreen, ui},
        {FRONTEND_COMMAND_MUTE, "Mute", "Audio", "Ctrl+M",
         FRONTEND_COMMAND_CHECKABLE, command_mute, ui}
    };
    for (size_t i = 0; i < sizeof(specs) / sizeof(specs[0]); ++i)
        if (!frontend_command_register(&specs[i])) return false;
    (void)frontend_command_set_checked(FRONTEND_COMMAND_FULLSCREEN, ui->settings->fullscreen);
    (void)frontend_command_set_checked(FRONTEND_COMMAND_MUTE, ui->settings->muted);
    return true;
}

static const char *region_value(NesRegionMode mode) {
    static const char *const names[] = {"Auto", "NTSC", "PAL", "Dendy"};
    return (unsigned)mode < 4 ? names[mode] : "?";
}

static const char *console_value(NesConsoleModel model) {
    static const char *const names[] = {"NES-001", "NES-101", "Famicom", "AV Famicom"};
    return (unsigned)model < 4 ? names[model] : "?";
}

static const char *adapter_value(NesInputAdapter adapter) {
    static const char *const names[] = {"None", "Four Score", "Famicom 2-player", "Famicom 4-player"};
    return (unsigned)adapter < 4 ? names[adapter] : "?";
}

static const char *port_value(NesPortDevice device) {
    static const char *const names[] = {"Gamepad", "None", "Arkanoid", "Power Pad A",
        "Power Pad B", "Zapper", "Subor mouse", "SNES pad", "SNES mouse",
        "NTT keypad", "Virtual Boy"};
    return (unsigned)device < 11 ? names[device] : "?";
}

static const char *expansion_value(NesExpansionDevice device) {
    static const char *const names[] = {"None", "Arkanoid", "Family Trainer A",
        "Family Trainer B", "Zapper", "Family BASIC", "Turbo File", "Battle Box",
        "Subor keyboard", "Hori Track", "Konami Hyper Shot", "Bandai Hyper Shot",
        "Party Tap", "Pachinko", "Exciting Boxing", "Jissen Mahjong",
        "Barcode Battler", "Oeka Kids tablet", "FCNS controller"};
    return (unsigned)device < 19 ? names[device] : "?";
}

static const char *gamepad_value(SDL_GameControllerButton button) {
    return button == SDL_CONTROLLER_BUTTON_INVALID ? "Unbound"
        : frontend_gamepad_button_name(button);
}

static const char *on_off(bool enabled) { return enabled ? "On" : "Off"; }

static int setting_rows(const FrontendDesktopUi *ui) {
    switch (ui->settings_category) {
        case 0: return 8;
        case 1: return 5;
        case 2: return 19;
        case 3: return 5 + NES_AUDIO_CHANNEL_COUNT * 2;
        case 4: return 24 + FRONTEND_SHORTCUT_COUNT * 2;
        case 5: return 17;
        case 6: return 8;
        case 7: return 25;
        default: return 0;
    }
}

static void setting_text(FrontendDesktopUi *ui, int row, char *label, size_t lc,
                         char *value, size_t vc) {
    FrontendSettings *s = &ui->staged;
    label[0] = value[0] = '\0';
    if (ui->settings_category == 0) {
        const char *labels[] = {"Reopen last image", "Remember window size", "Recent image count",
            "Pause on focus loss", "Pause while UI is open", "Show frame rate",
            "Window width", "Window height"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", on_off(s->reopen_last_image));
        else if (row == 1) snprintf(value, vc, "%s", on_off(s->remember_window_size));
        else if (row == 2) snprintf(value, vc, "%u", s->recent_file_limit);
        else if (row == 3) snprintf(value, vc, "%s", on_off(s->pause_on_focus_loss));
        else if (row == 4) snprintf(value, vc, "%s", on_off(s->pause_on_ui));
        else if (row == 5) snprintf(value, vc, "%s", on_off(s->show_fps));
        else snprintf(value, vc, "%u", row == 6 ? s->window_width : s->window_height);
    } else if (ui->settings_category == 1) {
        const char *labels[] = {"Timing selection", "Emulation speed", "Fast-forward speed",
            "Rewind history", "Run-ahead frames"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", region_value(s->region_mode));
        else if (row == 1 || row == 2)
            snprintf(value, vc, "%.2gx", row == 1 ? s->speed : s->fast_forward_speed);
        else if (row == 3) snprintf(value, vc, "%u seconds", s->rewind_seconds);
        else snprintf(value, vc, "%u", s->run_ahead_frames);
    } else if (ui->settings_category == 2) {
        if (row < 7) {
            const char *labels[] = {"Fullscreen", "Integer scaling", "VSync", "Aspect ratio",
                "Composite output", "Background layer", "Sprite layer"};
            snprintf(label, lc, "%s", labels[row]);
            if (row == 0) snprintf(value, vc, "%s", on_off(s->fullscreen));
            else if (row == 1) snprintf(value, vc, "%s", on_off(s->integer_scaling));
            else if (row == 2) snprintf(value, vc, "%s", on_off(s->vsync));
            else if (row == 3) snprintf(value, vc, "%s", s->aspect_mode == FRONTEND_ASPECT_4_3 ? "4:3" : "Source");
            else if (row == 4) snprintf(value, vc, "%s", on_off(s->ntsc_composite));
            else if (row == 5) snprintf(value, vc, "%s", on_off(s->presentation.show_background));
            else snprintf(value, vc, "%s", on_off(s->presentation.show_sprites));
        } else {
            static const char *const regions[] = {"NTSC", "PAL", "Dendy"};
            static const char *const edges[] = {"left", "right", "top", "bottom"};
            unsigned index = (unsigned)(row - 7), region = index / 4, edge = index % 4;
            const NesVideoOverscan *o = &s->presentation.overscan[region];
            const unsigned values[] = {o->left, o->right, o->top, o->bottom};
            snprintf(label, lc, "%s overscan %s", regions[region], edges[edge]);
            snprintf(value, vc, "%u px", values[edge]);
        }
    } else if (ui->settings_category == 3) {
        if (row == 0) { snprintf(label, lc, "Mute audio"); snprintf(value, vc, "%s", on_off(s->muted)); }
        else if (row == 1) { snprintf(label, lc, "Master volume"); snprintf(value, vc, "%u%%", s->audio_mix.master_volume); }
        else if (row == 2) { snprintf(label, lc, "Output device"); visible_text(value, vc, s->audio_device[0] ? s->audio_device : "Default"); }
        else if (row == 3) { snprintf(label, lc, "Sample rate"); snprintf(value, vc, "%u Hz", s->audio_sample_rate); }
        else if (row == 4) { snprintf(label, lc, "Buffer size"); snprintf(value, vc, "%u samples", s->audio_buffer_samples); }
        else {
            unsigned index = (unsigned)(row - 5), channel = index / 2;
            bool pan = (index & 1u) != 0;
            snprintf(label, lc, "%s %s", nes_audio_channel_name(channel), pan ? "pan" : "volume");
            if (pan) snprintf(value, vc, "%d", s->audio_mix.pan[channel]);
            else snprintf(value, vc, "%u%%", s->audio_mix.volume[channel]);
        }
    } else if (ui->settings_category == 4) {
        const FrontendBindingProfile *profile = frontend_settings_active_profile_const(s);
        if (row == 0) { snprintf(label, lc, "Binding profile"); snprintf(value, vc, "%s", s->active_profile); }
        else if (row == 1) { snprintf(label, lc, "Input adapter"); snprintf(value, vc, "%s", adapter_value(s->input.adapter)); }
        else if (row == 2 || row == 3) { snprintf(label, lc, "Port %d device", row - 1); snprintf(value, vc, "%s", port_value(s->input.ports[row - 2])); }
        else if (row == 4) { snprintf(label, lc, "Expansion device"); snprintf(value, vc, "%s", expansion_value(s->input.expansion)); }
        else if (row == 5) { snprintf(label, lc, "Light radius"); snprintf(value, vc, "%u", s->zapper_radius); }
        else if (row == 6) { snprintf(label, lc, "Editing player"); snprintf(value, vc, "%u", ui->settings_player + 1u); }
        else if (row == 7) { snprintf(label, lc, "Host controller GUID"); visible_text(value, vc, s->device_guid[ui->settings_player][0] ? s->device_guid[ui->settings_player] : "Auto"); }
        else if (row < 16) {
            unsigned button = (unsigned)(row - 8);
            const FrontendHostBinding *b = profile ? &profile->players[ui->settings_player][button] : NULL;
            snprintf(label, lc, "Player key: %s", frontend_player_button_name(button));
            snprintf(value, vc, "%s", b && b->key != SDL_SCANCODE_UNKNOWN ? SDL_GetScancodeName(b->key) : "Unbound");
        } else if (row < 24) {
            unsigned button = (unsigned)(row - 16);
            const FrontendHostBinding *b = profile ? &profile->players[ui->settings_player][button] : NULL;
            snprintf(label, lc, "Player pad: %s", frontend_player_button_name(button));
            snprintf(value, vc, "%s", b ? gamepad_value(b->gamepad_button) : "Unbound");
        } else if (row < 24 + FRONTEND_SHORTCUT_COUNT) {
            unsigned shortcut = (unsigned)(row - 24);
            const FrontendHostBinding *b = profile ? &profile->shortcuts[shortcut] : NULL;
            snprintf(label, lc, "Shortcut key: %s", frontend_shortcut_name((FrontendShortcut)shortcut));
            snprintf(value, vc, "%s", b && b->key != SDL_SCANCODE_UNKNOWN ? SDL_GetScancodeName(b->key) : "Unbound");
        } else {
            unsigned shortcut = (unsigned)(row - 24 - FRONTEND_SHORTCUT_COUNT);
            const FrontendHostBinding *b = profile ? &profile->shortcuts[shortcut] : NULL;
            snprintf(label, lc, "Shortcut pad: %s", frontend_shortcut_name((FrontendShortcut)shortcut));
            snprintf(value, vc, "%s", b ? gamepad_value(b->gamepad_button) : "Unbound");
        }
    } else if (ui->settings_category == 5) {
        const char *labels[] = {"FDS BIOS path", "StudyBox BIOS path", "EPSM ADPCM ROM", "FCNS Kanji ROM",
            "Disk overlay path", "Disk save mode", "Disk write protection", "Automatic disk insertion",
            "Fast-forward disk loading", "Tape playback path", "Tape recording path", "Music auto advance",
            "Music repeat", "Music shuffle", "Music silence detection", "Music silence time", "Music silence threshold"};
        snprintf(label, lc, "%s", labels[row]);
        if (row <= 4 || row == 9 || row == 10) {
            const char *paths[] = {s->fds_bios_path, s->studybox_bios_path, s->epsm_adpcm_path,
                s->fcns_kanji_path, s->disk_overlay_path, s->tape_play_path, s->tape_record_path};
            unsigned path = row <= 4 ? (unsigned)row : (unsigned)(row - 4);
            visible_text(value, vc, paths[path]);
        } else if (row == 5) snprintf(value, vc, "%s", s->disk_save_mode == FDS_SAVE_OVERLAY ? "Overlay" : "In place");
        else if (row == 6) snprintf(value, vc, "%s", on_off(s->fds_write_protected));
        else if (row == 7) snprintf(value, vc, "%s", on_off(s->fds_auto_insert));
        else if (row == 8) snprintf(value, vc, "%s", on_off(s->fds_loading_fast_forward));
        else if (row == 11) snprintf(value, vc, "%s", on_off(s->nsf_player.automatic));
        else if (row == 12) snprintf(value, vc, "%s", on_off(s->nsf_player.repeat));
        else if (row == 13) snprintf(value, vc, "%s", on_off(s->nsf_player.shuffle));
        else if (row == 14) snprintf(value, vc, "%s", on_off(s->nsf_player.detect_silence));
        else if (row == 15) snprintf(value, vc, "%u ms", s->nsf_player.silence_ms);
        else snprintf(value, vc, "%.4g", (double)s->nsf_player.silence_threshold);
    } else if (ui->settings_category == 6) {
        const char *labels[] = {"State slot", "State file path", "Screenshot path", "Audio capture path",
            "Video capture path", "Capture displayed output", "Capture sample rate", "Capture byte limit"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "Slot %u", s->state_slot + 1u);
        else if (row == 1) visible_text(value, vc, s->state_file_path);
        else if (row <= 4) visible_text(value, vc, s->capture_paths[row - 2]);
        else if (row == 5) snprintf(value, vc, "%s", on_off(s->capture.displayed_output));
        else if (row == 6) snprintf(value, vc, "%u Hz", s->capture.sample_rate);
        else snprintf(value, vc, "%llu bytes", (unsigned long long)s->capture.byte_limit);
    } else if (ui->settings_category == 7) {
        const char *labels[] = {"Console wiring", "Effective timing", "CPU revision", "PPU revision",
            "RAM power-on state", "Random power-on VBL", "Disable APU noise short mode", "Swap APU pulse duties",
            "PPU OAM row corruption", "PPU startup write restriction", "PPU OAM decay", "PPU sprite evaluation wrap",
            "Disable PPU OAMDATA reads", "Disable PPU palette readback", "PPU reset suppression", "MMC3 revision",
            "Cartridge DIP switches", "VS DIP switches", "Use fixed startup phase", "Startup CPU offset",
            "Startup PPU phase", "Use startup seed", "Startup seed", "Use power-on seed", "Power-on seed"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", console_value(s->console_model));
        else if (row == 1) snprintf(value, vc, "%s", nes_region_name(nes_timing()->region));
        else if (row == 2) snprintf(value, vc, "%s", s->cpu_revision == APU_CPU_REVISION_EARLY_2A03 ? "Early 2A03" : "Late 2A03");
        else if (row == 3) snprintf(value, vc, "%s", s->ppu_revision == PPU_REVISION_2C02_PRE_E ? "2C02 pre-E" : "2C02E+");
        else if (row == 4) { const char *names[] = {"Default", "Zero", "Ones", "Random"}; snprintf(value, vc, "%s", names[s->ram_power_state]); }
        else if (row >= 5 && row <= 14) {
            const bool flags[] = {s->randomize_vblank, s->apu_disable_noise_mode, s->apu_swap_duty_cycles,
                s->ppu_oam_row_corruption, s->ppu_startup_restriction, s->ppu_oam_decay,
                s->ppu_sprite_eval_wrap_bug, s->ppu_oamdata_read_disabled,
                s->ppu_palette_readback_disabled, s->ppu_reset_suppression};
            snprintf(value, vc, "%s", on_off(flags[row - 5]));
        } else if (row == 15) snprintf(value, vc, "%s", s->mmc3_revision_a ? "A" : "Standard");
        else if (row == 16) snprintf(value, vc, "0x%02X", s->cart_dips);
        else if (row == 17) snprintf(value, vc, "0x%04X", s->vs_dips);
        else if (row == 18) snprintf(value, vc, "%s", on_off(s->startup_phase_set));
        else if (row == 19) snprintf(value, vc, "%u", s->startup_cpu_offset);
        else if (row == 20) snprintf(value, vc, "%u", s->startup_ppu_phase);
        else if (row == 21) snprintf(value, vc, "%s", on_off(s->startup_seed_set));
        else if (row == 22) snprintf(value, vc, "%u", s->startup_seed);
        else if (row == 23) snprintf(value, vc, "%s", on_off(s->power_on_seed_set));
        else snprintf(value, vc, "%u", s->power_on_seed);
    }
}

static void start_text_edit(FrontendDesktopUi *ui, unsigned control, const char *text) {
    if (!text) text = "";
    size_t size = strlen(text);
    if (size >= sizeof(ui->edit_text)) {
        copy_status(ui, "This value is too long to edit here");
        return;
    }
    ui->edit_control = control;
    memcpy(ui->edit_text, text, size + 1);
    ui->edit_text_active = true;
    SDL_StartTextInput();
}

static void begin_edit(FrontendDesktopUi *ui, unsigned row, const char *text) {
    start_text_edit(ui, 0x80000000u | row, text);
}

static void adjust_setting(FrontendDesktopUi *ui, int row, int direction) {
    FrontendSettings *s = &ui->staged;
    if (ui->settings_category == 0) {
        if (row == 0) s->reopen_last_image = !s->reopen_last_image;
        else if (row == 1) s->remember_window_size = !s->remember_window_size;
        else if (row == 2) { int v = (int)s->recent_file_limit + direction; s->recent_file_limit = (unsigned)(v < 0 ? 0 : v > FRONTEND_RECENT_MAX ? FRONTEND_RECENT_MAX : v); }
        else if (row == 3) s->pause_on_focus_loss = !s->pause_on_focus_loss;
        else if (row == 4) s->pause_on_ui = !s->pause_on_ui;
        else if (row == 5) s->show_fps = !s->show_fps;
        else if (row == 6) { int v = (int)s->window_width + direction * 32; s->window_width = (unsigned)(v < 320 ? 320 : v > 16384 ? 16384 : v); }
        else { int v = (int)s->window_height + direction * 24; s->window_height = (unsigned)(v < 240 ? 240 : v > 16384 ? 16384 : v); }
    } else if (ui->settings_category == 1) {
        if (row == 0) s->region_mode = (NesRegionMode)(((int)s->region_mode + direction + 4) % 4);
        else if (row == 1) { s->speed += direction * 0.25; if (s->speed < .1) s->speed = .1; if (s->speed > 16) s->speed = 16; }
        else if (row == 2) { s->fast_forward_speed += direction; if (s->fast_forward_speed < .1) s->fast_forward_speed = .1; if (s->fast_forward_speed > 16) s->fast_forward_speed = 16; }
        else if (row == 3) { int v = (int)s->rewind_seconds + direction; s->rewind_seconds = (unsigned)(v < 0 ? 0 : v > 60 ? 60 : v); }
        else { int v = (int)s->run_ahead_frames + direction; s->run_ahead_frames = (unsigned)(v < 0 ? 0 : v > 4 ? 4 : v); }
    } else if (ui->settings_category == 2) {
        if (row == 0) s->fullscreen = !s->fullscreen;
        else if (row == 1) s->integer_scaling = !s->integer_scaling;
        else if (row == 2) s->vsync = !s->vsync;
        else if (row == 3) s->aspect_mode = s->aspect_mode == FRONTEND_ASPECT_SOURCE ? FRONTEND_ASPECT_4_3 : FRONTEND_ASPECT_SOURCE;
        else if (row == 4) s->ntsc_composite = !s->ntsc_composite;
        else if (row == 5) s->presentation.show_background = !s->presentation.show_background;
        else if (row == 6) s->presentation.show_sprites = !s->presentation.show_sprites;
        else {
            unsigned index = (unsigned)(row - 7), region = index / 4, edge = index % 4;
            NesVideoOverscan *o = &s->presentation.overscan[region];
            unsigned *values[] = {&o->left, &o->right, &o->top, &o->bottom};
            int v = (int)*values[edge] + direction;
            *values[edge] = (unsigned)(v < 0 ? 0 : v > 239 ? 239 : v);
        }
    } else if (ui->settings_category == 3) {
        if (row == 0) s->muted = !s->muted;
        else if (row == 1) { int v = (int)s->audio_mix.master_volume + direction * 5; s->audio_mix.master_volume = (unsigned)(v < 0 ? 0 : v > 100 ? 100 : v); }
        else if (row == 2) begin_edit(ui, (unsigned)row, s->audio_device);
        else if (row == 3) { int v = (int)s->audio_sample_rate + direction * 1000; s->audio_sample_rate = (unsigned)(v < 8000 ? 8000 : v > 192000 ? 192000 : v); }
        else if (row == 4) { int v = (int)s->audio_buffer_samples + direction * 64; s->audio_buffer_samples = (unsigned)(v < 64 ? 64 : v > 8192 ? 8192 : v); }
        else {
            unsigned index = (unsigned)(row - 5), channel = index / 2;
            if (index & 1u) { int v = s->audio_mix.pan[channel] + direction * 5; s->audio_mix.pan[channel] = v < -100 ? -100 : v > 100 ? 100 : v; }
            else { int v = (int)s->audio_mix.volume[channel] + direction * 5; s->audio_mix.volume[channel] = (unsigned)(v < 0 ? 0 : v > 200 ? 200 : v); }
        }
    }
    else if (ui->settings_category == 4) {
        if (row == 0 && s->profile_count) {
            size_t index = 0; for (; index < s->profile_count; ++index) if (!strcmp(s->profiles[index].name, s->active_profile)) break;
            index = (index + s->profile_count + direction) % s->profile_count; strcpy(s->active_profile, s->profiles[index].name);
        } else if (row == 1) s->input.adapter = (NesInputAdapter)(((int)s->input.adapter + direction + 4) % 4);
        else if (row == 2 || row == 3) s->input.ports[row - 2] = (NesPortDevice)(((int)s->input.ports[row - 2] + direction + 11) % 11);
        else if (row == 4) s->input.expansion = (NesExpansionDevice)(((int)s->input.expansion + direction + 19) % 19);
        else if (row == 5) { int v = (int)s->zapper_radius + direction; s->zapper_radius = (unsigned)(v < 0 ? 0 : v > 255 ? 255 : v); }
        else if (row == 6) ui->settings_player = (ui->settings_player + NES_INPUT_PLAYERS + direction) % NES_INPUT_PLAYERS;
        else if (row == 7) begin_edit(ui, (unsigned)row, s->device_guid[ui->settings_player]);
        else {
            ui->capture_binding = true;
            ui->capture_gamepad = row >= 16 && row < 24
                ? true : row >= 24 + FRONTEND_SHORTCUT_COUNT;
            ui->capture_shortcut = row >= 24;
            ui->capture_index = row < 16 ? (unsigned)(row - 8)
                : row < 24 ? (unsigned)(row - 16)
                : row < 24 + FRONTEND_SHORTCUT_COUNT ? (unsigned)(row - 24)
                : (unsigned)(row - 24 - FRONTEND_SHORTCUT_COUNT);
            copy_status(ui, ui->capture_gamepad
                ? "Press a controller button for this binding (Backspace clears)"
                : "Press a key for this binding (Backspace clears)");
        }
    } else if (ui->settings_category == 5) {
        if (row <= 4 || row == 9 || row == 10) {
            const char *paths[] = {s->fds_bios_path, s->studybox_bios_path, s->epsm_adpcm_path,
                s->fcns_kanji_path, s->disk_overlay_path, s->tape_play_path, s->tape_record_path};
            begin_edit(ui, (unsigned)row, paths[row <= 4 ? row : row - 4]);
        } else if (row == 5) s->disk_save_mode = s->disk_save_mode == FDS_SAVE_OVERLAY ? FDS_SAVE_IN_PLACE : FDS_SAVE_OVERLAY;
        else if (row == 6) s->fds_write_protected = !s->fds_write_protected;
        else if (row == 7) s->fds_auto_insert = !s->fds_auto_insert;
        else if (row == 8) s->fds_loading_fast_forward = !s->fds_loading_fast_forward;
        else if (row == 11) s->nsf_player.automatic = !s->nsf_player.automatic;
        else if (row == 12) s->nsf_player.repeat = !s->nsf_player.repeat;
        else if (row == 13) s->nsf_player.shuffle = !s->nsf_player.shuffle;
        else if (row == 14) s->nsf_player.detect_silence = !s->nsf_player.detect_silence;
        else if (row == 15) { int v = (int)s->nsf_player.silence_ms + direction * 100; s->nsf_player.silence_ms = (unsigned)(v < 10 ? 10 : v > 600000 ? 600000 : v); }
        else { float v = s->nsf_player.silence_threshold + (float)direction * 0.0005f; s->nsf_player.silence_threshold = v < 0 ? 0 : v > 0.1f ? 0.1f : v; }
    } else if (ui->settings_category == 6) {
        if (row == 0) s->state_slot = (s->state_slot + NES_STATE_SLOT_COUNT + direction) % NES_STATE_SLOT_COUNT;
        else if (row <= 4) { const char *text = row == 1 ? s->state_file_path : s->capture_paths[row - 2]; begin_edit(ui, (unsigned)row, text); }
        else if (row == 5) s->capture.displayed_output = !s->capture.displayed_output;
        else if (row == 6) { int v = (int)s->capture.sample_rate + direction * 1000; s->capture.sample_rate = (unsigned)(v < 8000 ? 8000 : v > 192000 ? 192000 : v); }
        else { int64_t v = (int64_t)s->capture.byte_limit + (int64_t)direction * 1024 * 1024; s->capture.byte_limit = (uint64_t)(v < 1024 ? 1024 : v > UINT32_MAX ? UINT32_MAX : v); }
    } else if (ui->settings_category == 7) {
        if (row == 0) s->console_model = (NesConsoleModel)(((int)s->console_model + direction + 4) % 4);
        else if (row == 2) s->cpu_revision = s->cpu_revision == APU_CPU_REVISION_EARLY_2A03 ? APU_CPU_REVISION_LATE_2A03 : APU_CPU_REVISION_EARLY_2A03;
        else if (row == 3) s->ppu_revision = s->ppu_revision == PPU_REVISION_2C02_PRE_E ? PPU_REVISION_2C02_E_PLUS : PPU_REVISION_2C02_PRE_E;
        else if (row == 4) s->ram_power_state = (NesRamPowerOnState)(((int)s->ram_power_state + direction + 4) % 4);
        else if (row >= 5 && row <= 14) {
            bool *flags[] = {&s->randomize_vblank, &s->apu_disable_noise_mode, &s->apu_swap_duty_cycles,
                &s->ppu_oam_row_corruption, &s->ppu_startup_restriction, &s->ppu_oam_decay,
                &s->ppu_sprite_eval_wrap_bug, &s->ppu_oamdata_read_disabled,
                &s->ppu_palette_readback_disabled, &s->ppu_reset_suppression};
            *flags[row - 5] = !*flags[row - 5];
        } else if (row == 15) s->mmc3_revision_a = !s->mmc3_revision_a;
        else if (row == 16) { int v = (int)s->cart_dips + direction; s->cart_dips = (unsigned)(v < 0 ? 0 : v > 255 ? 255 : v); }
        else if (row == 17) { int v = (int)s->vs_dips + direction; s->vs_dips = (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v); }
        else if (row == 18) { s->startup_phase_set = !s->startup_phase_set; if (s->startup_phase_set) s->startup_seed_set = false; }
        else if (row == 19) { int v = (int)s->startup_cpu_offset + direction; s->startup_cpu_offset = (unsigned)(v < 0 ? 0 : v > 15 ? 15 : v); }
        else if (row == 20) { int v = (int)s->startup_ppu_phase + direction; s->startup_ppu_phase = (unsigned)(v < 0 ? 0 : v > 4 ? 4 : v); }
        else if (row == 21) { s->startup_seed_set = !s->startup_seed_set; if (s->startup_seed_set) s->startup_phase_set = false; }
        else if (row == 22) s->startup_seed = (uint32_t)(s->startup_seed + direction);
        else if (row == 23) s->power_on_seed_set = !s->power_on_seed_set;
        else s->power_on_seed = (uint32_t)(s->power_on_seed + direction);
    }
}

static void commit_edit(FrontendDesktopUi *ui) {
    if (!ui || !ui->edit_text_active) return;
    unsigned row = ui->edit_control & 0x7fffffffu;
    FrontendSettings *s = &ui->staged;
    if (ui->settings_open) {
        if (ui->settings_category == 3 && row == 2) {
            if (strlen(ui->edit_text) >= sizeof(s->audio_device)) {
                copy_status(ui, "Audio device name is too long");
                return;
            }
            snprintf(s->audio_device, sizeof(s->audio_device), "%s", ui->edit_text);
        } else if (ui->settings_category == 4 && row == 7) {
            size_t size = strlen(ui->edit_text);
            if (size >= FRONTEND_SETTINGS_GUID_TEXT) {
                copy_status(ui, "Controller identifier is too long");
                return;
            }
            memcpy(s->device_guid[ui->settings_player], ui->edit_text, size + 1);
        }
        else if (ui->settings_category == 5 && (row <= 4 || row == 9 || row == 10)) {
            char *targets[] = {s->fds_bios_path, s->studybox_bios_path, s->epsm_adpcm_path,
                s->fcns_kanji_path, s->disk_overlay_path, s->tape_play_path, s->tape_record_path};
            unsigned target = row <= 4 ? row : row - 4;
            snprintf(targets[target], FRONTEND_SETTINGS_PATH_TEXT, "%s", ui->edit_text);
        } else if (ui->settings_category == 6) {
            if (row == 1) snprintf(s->state_file_path, sizeof(s->state_file_path), "%s", ui->edit_text);
            else if (row >= 2 && row <= 4) snprintf(s->capture_paths[row - 2], FRONTEND_SETTINGS_PATH_TEXT, "%s", ui->edit_text);
        }
    } else if (ui->panel_open) {
        char error[256] = {0};
        if (!frontend_panel_action(ui->panel_id, ui->edit_control, ui->edit_text, -1, error, sizeof(error))) copy_status(ui, error);
    }
    ui->edit_text_active = false; SDL_StopTextInput();
}

static void handle_binding_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *key) {
    FrontendBindingProfile *profile = frontend_settings_active_profile(&ui->staged);
    if (!profile) return;
    FrontendHostBinding *binding = ui->capture_shortcut ? &profile->shortcuts[ui->capture_index] : &profile->players[ui->settings_player][ui->capture_index];
    if (key->keysym.scancode == SDL_SCANCODE_BACKSPACE) { binding->key = SDL_SCANCODE_UNKNOWN; binding->modifiers = KMOD_NONE; }
    else { binding->key = key->keysym.scancode; binding->modifiers = (SDL_Keymod)(key->keysym.mod & (KMOD_CTRL|KMOD_SHIFT|KMOD_ALT|KMOD_GUI)); }
    ui->capture_binding = false; copy_status(ui, "Binding updated");
}

static void handle_binding_gamepad(FrontendDesktopUi *ui, SDL_GameControllerButton button) {
    FrontendBindingProfile *profile = frontend_settings_active_profile(&ui->staged);
    if (!profile) return;
    FrontendHostBinding *binding = ui->capture_shortcut
        ? &profile->shortcuts[ui->capture_index]
        : &profile->players[ui->settings_player][ui->capture_index];
    binding->gamepad_button = button;
    ui->capture_binding = false;
    copy_status(ui, "Binding updated");
}

static void render_settings(FrontendDesktopUi *ui) {
    int ww = 0, wh = 0; SDL_GetWindowSize(ui->window, &ww, &wh);
    SDL_Rect box = {(ww - MODAL_W)/2, (wh - MODAL_H)/2, MODAL_W, MODAL_H};
    fill(ui->renderer, box, 30, 32, 38, 250); SDL_SetRenderDrawColor(ui->renderer, 180,180,190,255); SDL_RenderDrawRect(ui->renderer, &box);
    frontend_draw_text(ui->renderer, box.x+16, box.y+14, 2, "Settings", 245,245,245,255);
    int cy = box.y + 48;
    for (int i=0;i<8;i++) { SDL_Rect c={box.x+12,cy+i*30,150,26}; if(i==ui->settings_category) fill(ui->renderer,c,65,75,95,255); frontend_draw_text(ui->renderer,c.x+8,c.y+7,1,setting_categories[i],230,230,235,255); }
    int rows = setting_rows(ui), start = ui->settings_scroll; if (ui->settings_row < start) start = ui->settings_row; if (ui->settings_row >= start+13) start = ui->settings_row-12; ui->settings_scroll=start;
    for (int visible=0; visible<13 && start+visible<rows; ++visible) {
        int row=start+visible, y=box.y+50+visible*24; SDL_Rect rr={box.x+174,y,430,22}; if(row==ui->settings_row) fill(ui->renderer,rr,55,60,72,255);
        char label[96], value[160]; setting_text(ui,row,label,sizeof(label),value,sizeof(value)); frontend_draw_text(ui->renderer,rr.x+5,rr.y+7,1,label,220,220,225,255); int tw=frontend_text_width(value,1); frontend_draw_text(ui->renderer,rr.x+rr.w-tw-6,rr.y+7,1,value,170,205,255,255);
    }
    const char *buttons[] = {"Apply", "OK", "Cancel", "Defaults", "Reset all"};
    for(int i=0;i<5;i++){ SDL_Rect b={box.x+175+i*84,box.y+390,78,26}; fill(ui->renderer,b,65,70,80,255); frontend_draw_text(ui->renderer,b.x+8,b.y+8,1,buttons[i],235,235,235,255); }
    if (ui->capture_binding) frontend_draw_text(ui->renderer,box.x+180,box.y+370,1,
        ui->capture_gamepad ? "Press controller button; Backspace clears" : "Press key; Backspace clears",255,210,120,255);
    if (ui->edit_text_active) frontend_draw_text(ui->renderer,box.x+180,box.y+370,1,ui->edit_text,255,210,120,255);
}

static void render_panel(FrontendDesktopUi *ui) {
    FrontendPanelControl controls[64]; FrontendPanelModel model={.controls=controls,.capacity=64}; char error[256]={0};
    if (!frontend_panel_snapshot(ui->panel_id,&model,error,sizeof(error))) { copy_status(ui,error); ui->panel_open=false; return; }
    int ww=0,wh=0; SDL_GetWindowSize(ui->window,&ww,&wh); SDL_Rect box={(ww-600)/2,(wh-430)/2,600,430}; fill(ui->renderer,box,28,30,35,250); SDL_SetRenderDrawColor(ui->renderer,180,180,190,255); SDL_RenderDrawRect(ui->renderer,&box);
    FrontendPanelInfo info; frontend_panel_get(ui->panel_id,&info); frontend_draw_text(ui->renderer,box.x+16,box.y+14,2,info.title,245,245,245,255);
    for(size_t i=0;i<model.count && i<15;i++){ int y=box.y+52+(int)i*23; char value[120]=""; const FrontendPanelControl *c=&controls[i]; if(c->type==FRONTEND_PANEL_CHECKBOX) snprintf(value,sizeof(value),"[%c]",c->selected?'x':' '); else if(c->type==FRONTEND_PANEL_CHOICE && c->selected>=0 && (size_t)c->selected<c->item_count) snprintf(value,sizeof(value),"%s",c->items[c->selected]); else if(c->value) snprintf(value,sizeof(value),"%s",c->value); frontend_draw_text(ui->renderer,box.x+18,y,1,c->label,c->enabled?220:110,c->enabled?220:110,c->enabled?225:110,255); frontend_draw_text(ui->renderer,box.x+360,y,1,value,170,205,255,255); }
    if(model.status) frontend_draw_text(ui->renderer,box.x+18,box.y+400,1,model.status,200,200,205,255);
}

static void render_info(FrontendDesktopUi *ui) {
    int ww=0,wh=0; SDL_GetWindowSize(ui->window,&ww,&wh); SDL_Rect box={(ww-520)/2,(wh-300)/2,520,300}; fill(ui->renderer,box,28,30,35,250);
    frontend_draw_text(ui->renderer,box.x+18,box.y+18,2,"Game information",245,245,245,255);
    char line[256]; snprintf(line,sizeof(line),"Metadata: %s",rom_metadata_source_name()); frontend_draw_text(ui->renderer,box.x+18,box.y+64,1,line,220,220,225,255);
    snprintf(line,sizeof(line),"Region: %s",nes_region_name(nes_timing()->region)); frontend_draw_text(ui->renderer,box.x+18,box.y+88,1,line,220,220,225,255);
    snprintf(line,sizeof(line),"Console: %s",nes_console_model_name()); frontend_draw_text(ui->renderer,box.x+18,box.y+112,1,line,220,220,225,255);
    if (!rom_is_fds() && !rom_is_studybox() && !rom_is_nsf()) { snprintf(line,sizeof(line),"Mapper: %d",rom_mapper_number(&ines_header)); frontend_draw_text(ui->renderer,box.x+18,box.y+136,1,line,220,220,225,255); }
    if(ui->sessions && ui->sessions->session && ui->sessions->session->active){ snprintf(line,sizeof(line),"Image: %.70s",ui->sessions->session->current.path); frontend_draw_text(ui->renderer,box.x+18,box.y+170,1,line,170,205,255,255); }
}

static int menu_index_at(int x) { int offset=10; for(int i=0;i<5;i++){ int w=frontend_text_width(menu_names[i],1)+22; if(x>=offset&&x<offset+w)return i; offset+=w;} return -1; }

static void render_menu(FrontendDesktopUi *ui) {
    if (ui->open_menu < 0 || ui->open_menu > 4) return;
    int x=10; for(int i=0;i<ui->open_menu;i++) x += frontend_text_width(menu_names[i],1)+22;
    const char *menu=menu_names[ui->open_menu]; int count=0; FrontendCommandInfo ci;
    for(size_t i=0;i<frontend_command_count();i++) if(frontend_command_at(i,&ci) && !strcmp(ci.menu,menu)) count++;
    int panel_start=count; if(ui->open_menu==3){ FrontendPanelInfo pi; for(size_t i=0;i<frontend_panel_count();i++) if(frontend_panel_at(i,&pi)&&pi.enabled) count++; }
    if (ui->open_menu == 4) count += 2;
    if (!count) return;
    SDL_Rect box={x,MENU_H,260,count*24+8}; fill(ui->renderer,box,36,38,44,250);
    int row=0; for(size_t i=0;i<frontend_command_count();i++){ if(!frontend_command_at(i,&ci)||strcmp(ci.menu,menu))continue; char line[128]; snprintf(line,sizeof(line),"%s%s%s",ci.checked?"[x] ":"",ci.label,ci.enabled?"":" (disabled)"); frontend_draw_text(ui->renderer,x+8,MENU_H+8+row*24,1,line,ci.enabled?230:120,ci.enabled?230:120,ci.enabled?235:120,255); row++; }
    if(ui->open_menu==3){ FrontendPanelInfo pi; for(size_t i=0;i<frontend_panel_count();i++){ if(!frontend_panel_at(i,&pi)||!pi.enabled)continue; frontend_draw_text(ui->renderer,x+8,MENU_H+8+row*24,1,pi.title,220,220,225,255); row++; } }
    if(ui->open_menu==4){ frontend_draw_text(ui->renderer,x+8,MENU_H+8+row++*24,1,"Game information",220,220,225,255); frontend_draw_text(ui->renderer,x+8,MENU_H+8+row*24,1,"Documentation",220,220,225,255); }
    (void)panel_start;
}

void frontend_desktop_render(FrontendDesktopUi *ui, int video_width, int video_height,
                             const char *title, const char *region, const char *run_state) {
    (void)video_width; (void)video_height;
    if (!ui || !ui->renderer || !ui->window) return;
    int ww=0,wh=0; SDL_GetWindowSize(ui->window,&ww,&wh);
    fill(ui->renderer,(SDL_Rect){0,0,ww,MENU_H},28,30,34,255);
    int x=10; for(int i=0;i<5;i++){ frontend_draw_text(ui->renderer,x,10,1,menu_names[i],230,230,235,255); x+=frontend_text_width(menu_names[i],1)+22; }
    fill(ui->renderer,(SDL_Rect){0,MENU_H,ww,TOOLBAR_H},42,45,52,255);
    const struct {const char *label; unsigned id;} tb[]={{"Open",FRONTEND_COMMAND_OPEN},{"Pause",FRONTEND_COMMAND_PAUSE},{"Reset",FRONTEND_COMMAND_SOFT_RESET},{"Settings",FRONTEND_COMMAND_SETTINGS}};
    x=10; for(size_t i=0;i<4;i++){ FrontendCommandInfo info={0}; bool ok=frontend_command_get(tb[i].id,&info)&&info.enabled; SDL_Rect b={x,MENU_H+5,82,26}; fill(ui->renderer,b,ok?65:48,ok?70:48,ok?82:48,255); frontend_draw_text(ui->renderer,b.x+8,b.y+8,1,tb[i].label,ok?235:120,ok?235:120,ok?240:120,255); x+=88; }
    fill(ui->renderer,(SDL_Rect){0,wh-STATUS_H,ww,STATUS_H},28,30,34,255); char status[320]; snprintf(status,sizeof(status),"%s | %s | %s%s%s",title&&*title?title:"No image",region?region:"-",run_state?run_state:"Idle",ui->settings&&ui->settings->show_fps?" | ":"",ui->settings&&ui->settings->show_fps?"FPS":""); frontend_draw_text(ui->renderer,8,wh-STATUS_H+8,1,status,205,205,210,255); if(ui->settings&&ui->settings->show_fps){char f[24];snprintf(f,sizeof(f),"%.1f",ui->fps);frontend_draw_text(ui->renderer,ww-frontend_text_width(f,1)-10,wh-STATUS_H+8,1,f,170,205,255,255);} if(ui->status[0] && SDL_GetTicks()<ui->status_until) frontend_draw_text(ui->renderer,ww/2-frontend_text_width(ui->status,1)/2,wh-STATUS_H+8,1,ui->status,255,210,120,255);
    render_menu(ui); if(ui->settings_open)render_settings(ui); if(ui->panel_open)render_panel(ui); if(ui->info_open)render_info(ui);
}

static void activate_menu_row(FrontendDesktopUi *ui, int row) {
    const char *menu=menu_names[ui->open_menu]; FrontendCommandInfo ci;
    for(size_t i=0;i<frontend_command_count();i++){ if(!frontend_command_at(i,&ci)||strcmp(ci.menu,menu))continue; if(row--==0){ if(ci.enabled)(void)invoke_command(ui,ci.id); ui->open_menu=-1; return; } }
    if(ui->open_menu==3){ FrontendPanelInfo pi; for(size_t i=0;i<frontend_panel_count();i++){ if(!frontend_panel_at(i,&pi)||!pi.enabled)continue; if(row--==0){ui->panel_id=pi.id;ui->panel_open=true;ui->open_menu=-1;return;} } }
    if(ui->open_menu==4){ if(row==0)ui->info_open=true; else if(row==1)(void)SDL_OpenURL("https://github.com/cupidthecat/cupid-nes/tree/main/docs"); ui->open_menu=-1; }
}

static void handle_settings_mouse(FrontendDesktopUi *ui, int x, int y) {
    int ww=0,wh=0;SDL_GetWindowSize(ui->window,&ww,&wh); SDL_Rect box={(ww-MODAL_W)/2,(wh-MODAL_H)/2,MODAL_W,MODAL_H}; if(!inside(x,y,box))return;
    if(x<box.x+165 && y>=box.y+48 && y<box.y+48+8*30){ ui->settings_category=(y-(box.y+48))/30; ui->settings_row=0; ui->settings_scroll=0; return; }
    if(x>=box.x+174 && y>=box.y+50 && y<box.y+50+13*24){ int row=ui->settings_scroll+(y-(box.y+50))/24; if(row<setting_rows(ui)){ui->settings_row=row;adjust_setting(ui,row,x>box.x+430?1:-1);}return;}
    if(y>=box.y+390&&y<box.y+416){int b=(x-(box.x+175))/84;if(b==0)(void)save_settings(ui);else if(b==1){if(save_settings(ui))set_settings_open(ui,false);}else if(b==2)set_settings_open(ui,false);else if(b==3){FrontendSettings d;frontend_settings_defaults(&d);d.cli_overrides=ui->staged.cli_overrides;ui->staged=d;}else if(b==4){if(ui->confirm_restore_all){FrontendSettings d;frontend_settings_defaults(&d);d.cli_overrides=ui->staged.cli_overrides;ui->staged=d;ui->confirm_restore_all=false;}else{ui->confirm_restore_all=true;copy_status(ui,"Click Reset all again to confirm");}}}
}

static void handle_panel_mouse(FrontendDesktopUi *ui, int x, int y) {
    int ww = 0, wh = 0;
    SDL_GetWindowSize(ui->window, &ww, &wh);
    SDL_Rect box = {(ww - 600) / 2, (wh - 430) / 2, 600, 430};
    if (!inside(x, y, box)) return;
    int row = (y - (box.y + 52)) / 23;
    if (row < 0) return;
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    char error[256] = {0};
    if (!frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error))
        || (size_t)row >= model.count) return;
    FrontendPanelControl *control = &controls[row];
    if (!control->enabled) return;
    if (control->type == FRONTEND_PANEL_TEXT && !control->read_only) {
        start_text_edit(ui, control->id, control->value);
        return;
    }
    int selected = control->selected;
    if (control->type == FRONTEND_PANEL_CHOICE && control->item_count)
        selected = (selected + 1) % (int)control->item_count;
    if (!frontend_panel_action(ui->panel_id, control->id, NULL, selected, error, sizeof(error))
        && error[0]) copy_status(ui, error);
}

bool frontend_desktop_handle_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui || !event) return false;
    if(event->type==SDL_WINDOWEVENT&&ui->window&&event->window.windowID==SDL_GetWindowID(ui->window)){if(event->window.event==SDL_WINDOWEVENT_SIZE_CHANGED){ui->settings->window_width=(unsigned)event->window.data1;ui->settings->window_height=(unsigned)event->window.data2;sync_scale(ui);}if(event->window.event==SDL_WINDOWEVENT_FOCUS_LOST&&ui->settings->pause_on_focus_loss&&ui->execution&&!frontend_execution_paused(ui->execution)){ui->paused_for_focus=invoke_command(ui,FRONTEND_COMMAND_PAUSE);}if(event->window.event==SDL_WINDOWEVENT_FOCUS_GAINED&&ui->paused_for_focus&&ui->execution&&frontend_execution_paused(ui->execution)){(void)invoke_command(ui,FRONTEND_COMMAND_PAUSE);ui->paused_for_focus=false;}}
    if(event->type==SDL_TEXTINPUT&&ui->edit_text_active){size_t n=strlen(ui->edit_text),a=strlen(event->text.text);if(n+a<sizeof(ui->edit_text)){memcpy(ui->edit_text+n,event->text.text,a+1);}return true;}
    if(ui->capture_binding&&event->type==SDL_CONTROLLERBUTTONDOWN&&ui->capture_gamepad){handle_binding_gamepad(ui,(SDL_GameControllerButton)event->cbutton.button);return true;}
    if((event->type==SDL_KEYDOWN||event->type==SDL_KEYUP)&&ui->capture_binding){if(event->type==SDL_KEYDOWN&&!event->key.repeat){if(event->key.keysym.scancode==SDL_SCANCODE_BACKSPACE&&ui->capture_gamepad)handle_binding_gamepad(ui,SDL_CONTROLLER_BUTTON_INVALID);else if(!ui->capture_gamepad)handle_binding_key(ui,&event->key);}return true;}
    if(event->type==SDL_KEYDOWN&&!event->key.repeat){SDL_Scancode sc=event->key.keysym.scancode;if(ui->edit_text_active){if(sc==SDL_SCANCODE_RETURN)commit_edit(ui);else if(sc==SDL_SCANCODE_ESCAPE){ui->edit_text_active=false;SDL_StopTextInput();}else if(sc==SDL_SCANCODE_BACKSPACE){size_t n=strlen(ui->edit_text);if(n){--n;while(n&&((unsigned char)ui->edit_text[n]&0xC0u)==0x80u)--n;ui->edit_text[n]='\0';}}return true;}if(sc==SDL_SCANCODE_ESCAPE){if(ui->settings_open)set_settings_open(ui,false);else if(ui->panel_open)ui->panel_open=false;else if(ui->info_open)ui->info_open=false;else ui->open_menu=-1;return true;}if(ui->settings_open){int rows=setting_rows(ui);if(sc==SDL_SCANCODE_TAB||sc==SDL_SCANCODE_DOWN)ui->settings_row=(ui->settings_row+1)%rows;else if(sc==SDL_SCANCODE_UP)ui->settings_row=(ui->settings_row+rows-1)%rows;else if(sc==SDL_SCANCODE_LEFT)adjust_setting(ui,ui->settings_row,-1);else if(sc==SDL_SCANCODE_RIGHT||sc==SDL_SCANCODE_RETURN)adjust_setting(ui,ui->settings_row,1);return true;}if((event->key.keysym.mod&KMOD_ALT)&&sc==SDL_SCANCODE_RETURN){(void)invoke_command(ui,FRONTEND_COMMAND_FULLSCREEN);return true;}if((event->key.keysym.mod&KMOD_CTRL)&&sc==SDL_SCANCODE_COMMA){set_settings_open(ui,true);return true;}}
    if(event->type==SDL_MOUSEBUTTONDOWN&&event->button.button==SDL_BUTTON_LEFT){int x=event->button.x,y=event->button.y;if(ui->settings_open){handle_settings_mouse(ui,x,y);return true;}if(ui->panel_open){handle_panel_mouse(ui,x,y);return true;}if(ui->info_open){ui->info_open=false;return true;}if(y<MENU_H){ui->open_menu=menu_index_at(x);return true;}if(ui->open_menu>=0){if(y>=MENU_H){activate_menu_row(ui,(y-MENU_H-8)/24);return true;}}if(y>=MENU_H&&y<MENU_H+TOOLBAR_H){int index=(x-10)/88;if(index>=0&&index<4){unsigned ids[]={FRONTEND_COMMAND_OPEN,FRONTEND_COMMAND_PAUSE,FRONTEND_COMMAND_SOFT_RESET,FRONTEND_COMMAND_SETTINGS};(void)invoke_command(ui,ids[index]);return true;}}}
    return ui->settings_open||ui->panel_open||ui->info_open||ui->edit_text_active;
}

bool frontend_desktop_input_captured(const FrontendDesktopUi *ui){return ui&&(ui->settings_open||ui->panel_open||ui->info_open||ui->edit_text_active||ui->capture_binding);}
bool frontend_desktop_quit_requested(const FrontendDesktopUi *ui){return ui&&ui->quit_requested;}
void frontend_desktop_update_window_settings(FrontendDesktopUi *ui){if(!ui||!ui->window||!ui->settings)return;int w,h;SDL_GetWindowSize(ui->window,&w,&h);if(w>0&&h>0){ui->settings->window_width=(unsigned)w;ui->settings->window_height=(unsigned)h;}}
void frontend_desktop_shutdown(FrontendDesktopUi *ui){if(!ui)return;set_settings_open(ui,false);SDL_StopTextInput();}

FrontendIdleResult frontend_desktop_idle_open(FrontendSettings *settings,
                                              const FrontendSession *session,
                                              const char *settings_path,
                                              FrontendImageRequest *request,
                                              SDL_Window **window,
                                              SDL_Renderer **renderer,
                                              char *error, size_t error_size) {
    if(!settings||!request||!window||!renderer)return FRONTEND_IDLE_ERROR;
    if(settings->reopen_last_image&&frontend_session_recent_count(session)){*request=*frontend_session_recent(session,0);return FRONTEND_IDLE_OPEN;}
    if(!(SDL_WasInit(SDL_INIT_VIDEO)&SDL_INIT_VIDEO)&&SDL_InitSubSystem(SDL_INIT_VIDEO)!=0){if(error&&error_size)snprintf(error,error_size,"Could not start desktop interface: %s",SDL_GetError());return FRONTEND_IDLE_ERROR;}
    *window=SDL_CreateWindow("Cupid NES",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,(int)settings->window_width,(int)settings->window_height,SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE|SDL_WINDOW_ALLOW_HIGHDPI);if(!*window){if(error&&error_size)snprintf(error,error_size,"Could not create window: %s",SDL_GetError());return FRONTEND_IDLE_ERROR;}
    *renderer=SDL_CreateRenderer(*window,-1,SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);if(!*renderer)*renderer=SDL_CreateRenderer(*window,-1,SDL_RENDERER_SOFTWARE);if(!*renderer){if(error&&error_size)snprintf(error,error_size,"Could not create renderer: %s",SDL_GetError());SDL_DestroyWindow(*window);*window=NULL;return FRONTEND_IDLE_ERROR;}
    FrontendDesktopUi ui;frontend_desktop_init(&ui,*window,*renderer,settings,NULL,NULL,settings_path);bool running=true;while(running){SDL_Event e;while(SDL_PollEvent(&e)){if(e.type==SDL_QUIT){running=false;break;}if(frontend_desktop_handle_event(&ui,&e))continue;if(e.type==SDL_MOUSEBUTTONDOWN&&e.button.button==SDL_BUTTON_LEFT){int ww,wh;SDL_GetWindowSize(*window,&ww,&wh);SDL_Rect open={ww/2-110,wh/2-50,220,42},set={ww/2-110,wh/2+6,220,34};if(inside(e.button.x,e.button.y,open)){char path[FRONTEND_IMAGE_PATH_MAX];if(frontend_open_image_dialog(path,sizeof(path),error,error_size)&&frontend_image_request_init(request,path)){frontend_desktop_update_window_settings(&ui);return FRONTEND_IDLE_OPEN;}if(error&&error_size&&error[0])copy_status(&ui,error);}else if(inside(e.button.x,e.button.y,set))set_settings_open(&ui,true);else{size_t count=frontend_session_recent_count(session);for(size_t i=0;i<count&&i<6;i++){SDL_Rect rr={ww/2-220,wh/2+60+(int)i*28,440,24};if(inside(e.button.x,e.button.y,rr)){*request=*frontend_session_recent(session,i);frontend_desktop_update_window_settings(&ui);return FRONTEND_IDLE_OPEN;}}}}}int ww,wh;SDL_GetWindowSize(*window,&ww,&wh);SDL_SetRenderDrawColor(*renderer,18,20,24,255);SDL_RenderClear(*renderer);frontend_draw_text(*renderer,ww/2-frontend_text_width("Cupid NES",2)/2,wh/2-125,2,"Cupid NES",245,245,245,255);SDL_Rect open={ww/2-110,wh/2-50,220,42};fill(*renderer,open,70,80,100,255);frontend_draw_text(*renderer,open.x+58,open.y+15,1,"Open Game...",245,245,245,255);SDL_Rect set={ww/2-110,wh/2+6,220,34};fill(*renderer,set,55,60,70,255);frontend_draw_text(*renderer,set.x+70,set.y+12,1,"Settings",235,235,235,255);for(size_t i=0;i<frontend_session_recent_count(session)&&i<6;i++){const FrontendImageRequest*r=frontend_session_recent(session,i);char line[80];snprintf(line,sizeof(line),"Recent: %.58s",r->archive_member[0]?r->archive_member:r->path);frontend_draw_text(*renderer,ww/2-210,wh/2+68+(int)i*28,1,line,180,205,235,255);}if(ui.settings_open)render_settings(&ui);if(ui.status[0]&&SDL_GetTicks()<ui.status_until)frontend_draw_text(*renderer,12,wh-20,1,ui.status,255,210,120,255);SDL_RenderPresent(*renderer);SDL_Delay(10);}frontend_desktop_update_window_settings(&ui);frontend_desktop_shutdown(&ui);SDL_DestroyRenderer(*renderer);SDL_DestroyWindow(*window);*renderer=NULL;*window=NULL;if(error&&error_size)error[0]='\0';return FRONTEND_IDLE_QUIT;
}
