/*
 * desktop_ui.c - SDL desktop menus, toolbar, status, and settings interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "desktop_ui.h"
#include "host_input.h"
#include "../rom/fds.h"
#include "../system/execution_policy.h"
#include "app_paths.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "desktop_internal.h"
#include "palette_tool.h"
#include "../joypad/joypad.h"
#include "../rom/rom.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include <stdio.h>
#include <string.h>

enum { MENU_H = 32, TOOLBAR_H = 48, STATUS_H = 28 };
static const char *const menu_names[] = {"File", "Emulation", "View", "Audio", "Media", "Tools", "Help"};
void desktop_copy_status(FrontendDesktopUi *ui, const char *text) {
    if (!ui) return;
    snprintf(ui->status, sizeof(ui->status), "%s", text ? text : "");
    ui->status_until = SDL_GetTicks() + 6000;
    if(text && *text){
        if(ui->log_count==12){memmove(ui->log_lines,ui->log_lines+1,11*sizeof(ui->log_lines[0]));--ui->log_count;}
        snprintf(ui->log_lines[ui->log_count++],sizeof(ui->log_lines[0]),"%s",text);
    }
}

static void visible_text(char *target,size_t capacity,const char *text) {
    if(!target||!capacity)return;
    if(!text)text="";
    size_t length=strlen(text);
    if(length<capacity){memcpy(target,text,length+1);return;}
    if(capacity<4){target[0]='\0';return;}
    size_t prefix=capacity-4;
    while(prefix&&((unsigned char)text[prefix]&0xC0u)==0x80u)--prefix;
    memcpy(target,text,prefix);memcpy(target+prefix,"...",4);
}

void frontend_desktop_set_status(FrontendDesktopUi *ui, const char *message) {
    desktop_copy_status(ui, message);
}

void frontend_desktop_set_fps(FrontendDesktopUi *ui, double fps) {
    if (ui) ui->fps = fps;
}

void desktop_sync_scale(FrontendDesktopUi *ui) {
    if (!ui || !ui->window || !ui->renderer) return;
    int w = 0, h = 0;
    SDL_GetWindowSize(ui->window, &w, &h);
    if (w > 0 && h > 0) SDL_RenderSetLogicalSize(ui->renderer, w, h);
    float ddpi = 96.0f;
    int display = SDL_GetWindowDisplayIndex(ui->window);
    if (display >= 0) (void)SDL_GetDisplayDPI(display, &ddpi, NULL, NULL);
    ui->ui_scale = ddpi >= 180.0f && w >= 1280 && h >= 960 ? 2.0f
        : ddpi >= 132.0f && w >= 960 && h >= 720 ? 1.5f : 1.0f;
    SDL_SetWindowMinimumSize(ui->window, 640, 480);
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

void frontend_desktop_game_rect(const FrontendDesktopUi *ui, int ww, int wh,
                                 int vw, int vh, bool integer_scaling, SDL_Rect *rect) {
    float scale=ui && ui->ui_scale>0?ui->ui_scale:1;
    frontend_desktop_compute_game_rect(ww,wh-(scale-1)*(MENU_H+TOOLBAR_H+STATUS_H),vw,vh,integer_scaling,rect);
    rect->y+=(scale-1)*(MENU_H+TOOLBAR_H);
}

bool desktop_invoke_command(FrontendDesktopUi *ui, unsigned id) {
    char error[256] = {0};
    bool ok = frontend_command_invoke(id, error, sizeof(error));
    if (error[0]) desktop_copy_status(ui, error);
    if (ok && ui->execution && ui->settings &&
        (id == FRONTEND_COMMAND_SPEED_HALF || id == FRONTEND_COMMAND_SPEED_NORMAL || id == FRONTEND_COMMAND_SPEED_DOUBLE))
        ui->settings->speed = ui->execution->execution.speed;
    return ok;
}

void desktop_settings_open(FrontendDesktopUi *ui, bool open) {
    if (!ui) return;
    if (open && ui->native_windows && !ui->parent) {
        ui->open_menu = -1;
        (void)desktop_open_window(ui, 0, 0);
        return;
    }
    if (ui->settings_open == open) return;
    if (open) {
        palette_tool_hide_overlay();
        ui->panel_open=ui->info_open=false;
        ui->open_menu=-1;
        ui->settings_focus=ui->settings_button=0;
        if(ui->capture){ui->settings->capture=ui->capture->options;memcpy(ui->settings->capture_paths,ui->capture->paths,sizeof(ui->settings->capture_paths));}
        if(ui->devices){snprintf(ui->settings->tape_play_path,sizeof(ui->settings->tape_play_path),"%s",ui->devices->tape_input);snprintf(ui->settings->tape_record_path,sizeof(ui->settings->tape_record_path),"%s",ui->devices->tape_output);}
        if(ui->music)ui->settings->nsf_player=ui->music->options;
        ui->staged = *ui->settings;
        if (ui->execution) {
            ui->staged.speed = ui->execution->execution.speed;
            ui->staged.fast_forward_speed = ui->execution->execution.fast_forward_speed;
            ui->staged.muted = ui->execution->muted;
            ui->staged.rewind_seconds = ui->execution->rewind_seconds;
            ui->staged.run_ahead_frames = ui->execution->run_ahead_frames;
        }
        ui->settings_category = 0;
        ui->settings_row = 0;
        ui->settings_scroll = 0;
        ui->settings_open = true;

    } else {
        ui->settings_open = false;
        SDL_StopTextInput();
        ui->edit_text_active = false;
        ui->capture_binding = false;

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
    (void)frontend_desktop_apply_features(ui,previous,ignored,sizeof(ignored));
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
    if ((ui->parent ? ui->parent->window : ui->window))
        (void)SDL_SetWindowFullscreen((ui->parent ? ui->parent->window : ui->window),
            previous->fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    if (ui->video) {
        if (!frontend_settings_cli_overridden(previous, FRONTEND_OVERRIDE_VIDEO_FILTER))
            frontend_video_runtime_set_composite(ui->video, previous->ntsc_composite);
        (void)frontend_video_runtime_refresh(ui->video, ignored, sizeof(ignored));
    }
}

static bool save_settings(FrontendDesktopUi *ui) {
    if (nes_execution_policy() != NES_EXECUTION_LIVE) {
        desktop_copy_status(ui, "Stop the deterministic session before applying settings");
        return false;
    }
    if (!ui || !ui->settings) return false;
    ui->staged.audio_mix.muted = ui->staged.muted;
    char error[256] = {0};
    if (!frontend_settings_validate(&ui->staged, error, sizeof(error))
        || !frontend_settings_validate_firmware(&ui->staged,error,sizeof(error))) {
        desktop_copy_status(ui, error[0] ? error : "Settings are invalid");
        return false;
    }

    FrontendSettings previous = *ui->settings;
    FrontendAudioPrepared prepared_audio = {0};
    if (ui->audio
        && !frontend_audio_runtime_prepare(ui->audio, &ui->staged, &prepared_audio,
                                           error, sizeof(error))) {
        desktop_copy_status(ui, error);
        return false;
    }

    bool previous_layers = settings_layers_enabled(&previous);
    bool staged_layers = settings_layers_enabled(&ui->staged);
    bool trace_prepared = false;
    if (staged_layers && !previous_layers) {
        if (!nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, true)) {
            frontend_audio_runtime_cancel(&prepared_audio);
            desktop_copy_status(ui, "Could not allocate video layer storage");
            return false;
        }
        trace_prepared = true;
    }

    bool fullscreen_changed = (ui->parent ? ui->parent->window : ui->window) && previous.fullscreen != ui->staged.fullscreen;
    if (fullscreen_changed
        && SDL_SetWindowFullscreen((ui->parent ? ui->parent->window : ui->window),
            ui->staged.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0) != 0) {
        if (trace_prepared) (void)nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false);
        frontend_audio_runtime_cancel(&prepared_audio);
        desktop_copy_status(ui, SDL_GetError());
        return false;
    }

    frontend_execution_begin_machine_change(ui->execution);
    if (!frontend_desktop_apply_features(ui,&ui->staged,error,sizeof(error)))goto rollback;
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

    frontend_execution_end_machine_change_preserving_audio(ui->execution);
    frontend_audio_runtime_commit(ui->audio, &prepared_audio);
    if (memcmp(previous.device_guid, ui->staged.device_guid, sizeof(previous.device_guid))) {
        frontend_host_input_shutdown();
        frontend_host_input_open_controllers(&ui->staged);
    }
    *ui->settings = ui->staged;
    if (ui->video) ui->video->settings = ui->settings;
    (void)frontend_command_set_checked(FRONTEND_COMMAND_FULLSCREEN, ui->settings->fullscreen);
    (void)frontend_command_set_checked(FRONTEND_COMMAND_MUTE, ui->settings->muted);
    desktop_copy_status(ui, "Applied. Timing: reload. Startup: power cycle. Firmware: restart.");
    return true;

rollback:
    frontend_audio_runtime_cancel(&prepared_audio);
    restore_runtime_settings(ui, &previous);
    frontend_execution_end_machine_change_preserving_audio(ui->execution);
    if (trace_prepared && !previous_layers)
        (void)nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, false);
    desktop_copy_status(ui, error[0] ? error : "Settings could not be applied");
    return false;
}

static bool command_settings(void *context, char *error, size_t error_size) {
    (void)error; (void)error_size;
    desktop_settings_open(context, true);
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
    ui->settings->audio_mix.muted = ui->settings->muted;
    (void)nes_audio_mix_set(&ui->settings->audio_mix, NULL, 0);
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
    ui->clay = renderer ? desktop_clay_create(renderer) : NULL;
    ui->idle_recent_index = -1;
    ui->visible_rows = 6;
    ui->window = window;
    ui->renderer = renderer;
    ui->settings = settings;
    ui->execution = execution;
    ui->sessions = sessions;
    ui->settings_path = settings_path;
    ui->settings_player = 0;
    ui->open_menu = -1;
    ui->focused = true;
    desktop_sync_scale(ui);
}

void frontend_desktop_set_runtime(FrontendDesktopUi *ui,
                                  FrontendAudioRuntime *audio,
                                  FrontendVideoRuntime *video) {
    if (!ui) return;
    ui->audio = audio;
    ui->video = video;
}

static bool command_palette(void *context, char *error, size_t size) {
    (void)error; (void)size;
    FrontendDesktopUi *ui = context;
    if (ui->native_windows) return desktop_open_window(ui, 3, 0) != NULL;
    palette_tool_toggle_overlay();
    return true;
}
static bool command_palette_load(void *context, char *error, size_t size) {
    (void)context;char path[4096]={0};
    if(!frontend_open_file_dialog(FRONTEND_OPEN_PALETTE,path,sizeof(path),error,size))return false;
    if(ppu_palette_load_pal_file(path)==0)return true;
    if(error&&size)snprintf(error,size,"Palette must contain 64 or 512 RGB colors");
    return false;
}
static bool command_palette_reset(void *context, char *error, size_t size) {
    (void)context;(void)error;(void)size;ppu_palette_reset_default();return true;
}

bool frontend_desktop_register_commands(FrontendDesktopUi *ui) {
    if (!ui || !ui->settings) return false;
    const FrontendCommandSpec specs[] = {
        {0x1B00,"Palette editor","View","F7",0,command_palette,ui},
        {0x1B01,"Load palette...","View",NULL,0,command_palette_load,ui},
        {0x1B02,"Reset palette","View",NULL,0,command_palette_reset,ui},
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

int desktop_setting_rows(const FrontendDesktopUi *ui) {
    switch (ui->settings_category) {
        case 0: return 8;
        case 1: return 6;
        case 2: return 19;
        case 3: return 5 + NES_AUDIO_CHANNEL_COUNT * 2;
        case 4: return 24 + FRONTEND_SHORTCUT_COUNT * 2;
        case 5: return 17;
        case 6: return 8;
        case 7: return 25;
        default: return 0;
    }
}

static void key_binding_text(const FrontendHostBinding *binding, char *text, size_t size) {
    if(!binding || binding->key==SDL_SCANCODE_UNKNOWN){snprintf(text,size,"Unbound");return;}
    SDL_Keymod mod=binding->modifiers;
    snprintf(text,size,"%s%s%s%s%s",mod&KMOD_CTRL?"Ctrl+":"",mod&KMOD_SHIFT?"Shift+":"",
        mod&KMOD_ALT?"Alt+":"",mod&KMOD_GUI?"GUI+":"",SDL_GetScancodeName(binding->key));
}

void desktop_setting_text(FrontendDesktopUi *ui, int row, char *label, size_t lc,
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
            "Rewind history", "Run-ahead frames", "Rewind speed"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", region_value(s->region_mode));
        else if (row == 1 || row == 2)
            snprintf(value, vc, "%.2gx", row == 1 ? s->speed : s->fast_forward_speed);
        else if (row == 3) snprintf(value, vc, "%u seconds", s->rewind_seconds);
        else if(row==4) snprintf(value, vc, "%u", s->run_ahead_frames);
        else snprintf(value,vc,"%u frames/activation",s->rewind_step_frames);
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
            key_binding_text(b,value,vc);
        } else if (row < 24) {
            unsigned button = (unsigned)(row - 16);
            const FrontendHostBinding *b = profile ? &profile->players[ui->settings_player][button] : NULL;
            snprintf(label, lc, "Player pad: %s", frontend_player_button_name(button));
            snprintf(value, vc, "%s", b ? gamepad_value(b->gamepad_button) : "Unbound");
        } else if (row < 24 + FRONTEND_SHORTCUT_COUNT) {
            unsigned shortcut = (unsigned)(row - 24);
            const FrontendHostBinding *b = profile ? &profile->shortcuts[shortcut] : NULL;
            snprintf(label, lc, "Shortcut key: %s", frontend_shortcut_name((FrontendShortcut)shortcut));
            key_binding_text(b,value,vc);
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

void desktop_start_text_edit(FrontendDesktopUi *ui, unsigned control, const char *text) {
    if (!text) text = "";
    size_t size = strlen(text);
    if (size >= sizeof(ui->edit_text)) {
        desktop_copy_status(ui, "This value is too long to edit here");
        return;
    }
    ui->edit_control = control;
    memcpy(ui->edit_text, text, size + 1);
    ui->edit_text_active = true;
    ui->edit_select_all = true;
    SDL_StartTextInput();
}

void desktop_begin_edit(FrontendDesktopUi *ui, unsigned row, const char *text) {
    desktop_start_text_edit(ui, 0x80000000u | row, text);
}

void desktop_adjust_setting(FrontendDesktopUi *ui, int row, int direction) {
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
        else if(row==4) { int v = (int)s->run_ahead_frames + direction; s->run_ahead_frames = (unsigned)(v < 0 ? 0 : v > 4 ? 4 : v); }
        else {int v=(int)s->rewind_step_frames+direction;s->rewind_step_frames=(unsigned)(v<1?1:v>30?30:v);}
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
        else if (row == 2) desktop_begin_edit(ui, (unsigned)row, s->audio_device);
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
        else if (row == 7) desktop_begin_edit(ui, (unsigned)row, s->device_guid[ui->settings_player]);
        else {
            ui->capture_binding = true;
            ui->capture_gamepad = row >= 16 && row < 24
                ? true : row >= 24 + FRONTEND_SHORTCUT_COUNT;
            ui->capture_shortcut = row >= 24;
            ui->capture_index = row < 16 ? (unsigned)(row - 8)
                : row < 24 ? (unsigned)(row - 16)
                : row < 24 + FRONTEND_SHORTCUT_COUNT ? (unsigned)(row - 24)
                : (unsigned)(row - 24 - FRONTEND_SHORTCUT_COUNT);
            desktop_copy_status(ui, ui->capture_gamepad
                ? "Press a controller button for this binding (Backspace clears)"
                : "Press a key for this binding (Backspace clears)");
        }
    } else if (ui->settings_category == 5) {
        if (row <= 4 || row == 9 || row == 10) {
            const char *paths[] = {s->fds_bios_path, s->studybox_bios_path, s->epsm_adpcm_path,
                s->fcns_kanji_path, s->disk_overlay_path, s->tape_play_path, s->tape_record_path};
            desktop_begin_edit(ui, (unsigned)row, paths[row <= 4 ? row : row - 4]);
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
        else if (row <= 4) { const char *text = row == 1 ? s->state_file_path : s->capture_paths[row - 2]; desktop_begin_edit(ui, (unsigned)row, text); }
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

void desktop_commit_edit(FrontendDesktopUi *ui) {
    if (!ui || !ui->edit_text_active) return;
    unsigned row = ui->edit_control & 0x7fffffffu;
    FrontendSettings *s = &ui->staged;
    if (ui->settings_open) {
        if (ui->settings_category == 3 && row == 2) {
            if (strlen(ui->edit_text) >= sizeof(s->audio_device)) {
                desktop_copy_status(ui, "Audio device name is too long");
                return;
            }
            snprintf(s->audio_device, sizeof(s->audio_device), "%s", ui->edit_text);
        } else if (ui->settings_category == 4 && row == 7) {
            size_t size = strlen(ui->edit_text);
            if (size >= FRONTEND_SETTINGS_GUID_TEXT) {
                desktop_copy_status(ui, "Controller identifier is too long");
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
        if (!frontend_panel_action(ui->panel_id, ui->edit_control, ui->edit_text, -1, error, sizeof(error))) desktop_copy_status(ui, error);
    }
    ui->edit_text_active = false; SDL_StopTextInput();
}

void desktop_binding_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *key) {
    FrontendBindingProfile *profile = frontend_settings_active_profile(&ui->staged);
    if (!profile) return;
    SDL_Scancode sc = key->keysym.scancode;
    if (key->type == SDL_KEYDOWN && (sc == SDL_SCANCODE_LCTRL || sc == SDL_SCANCODE_RCTRL || sc == SDL_SCANCODE_LSHIFT ||
        sc == SDL_SCANCODE_RSHIFT || sc == SDL_SCANCODE_LALT || sc == SDL_SCANCODE_RALT ||
        sc == SDL_SCANCODE_LGUI || sc == SDL_SCANCODE_RGUI)) return;
    FrontendHostBinding *binding = ui->capture_shortcut ? &profile->shortcuts[ui->capture_index] : &profile->players[ui->settings_player][ui->capture_index];
    if (key->keysym.scancode == SDL_SCANCODE_BACKSPACE) { binding->key = SDL_SCANCODE_UNKNOWN; binding->modifiers = KMOD_NONE; }
    else { binding->key = key->keysym.scancode; binding->modifiers = (SDL_Keymod)(key->keysym.mod & (KMOD_CTRL|KMOD_SHIFT|KMOD_ALT|KMOD_GUI)); }
    ui->capture_binding = false; desktop_copy_status(ui, "Binding updated");
}

void desktop_binding_gamepad(FrontendDesktopUi *ui, SDL_GameControllerButton button) {
    FrontendBindingProfile *profile = frontend_settings_active_profile(&ui->staged);
    if (!profile) return;
    FrontendHostBinding *binding = ui->capture_shortcut
        ? &profile->shortcuts[ui->capture_index]
        : &profile->players[ui->settings_player][ui->capture_index];
    binding->gamepad_button = button;
    ui->capture_binding = false;
    desktop_copy_status(ui, "Binding updated");
}

int desktop_menu_items(FrontendDesktopUi *ui, DesktopMenuItem items[128]) {
    if (ui->open_menu < 0 || ui->open_menu >= 7) return 0;
    const char *menu = menu_names[ui->open_menu];
    int count = 0;
    FrontendCommandInfo command;
    for (size_t i=0; i<frontend_command_count() && count<100; ++i) {
        if (!frontend_command_at(i,&command) || strcmp(command.menu,menu)) continue;
        items[count]=(DesktopMenuItem){.id=command.id,.enabled=command.enabled};
        snprintf(items[count].shortcut,sizeof(items[count].shortcut),"%s",command.shortcut?command.shortcut:"");
        const FrontendBindingProfile *profile=frontend_settings_active_profile_const(ui->settings);
        if(profile)for(unsigned shortcut=0;shortcut<FRONTEND_SHORTCUT_COUNT;++shortcut){
            if(frontend_execution_shortcut_command((FrontendShortcut)shortcut)==command.id){
                key_binding_text(&profile->shortcuts[shortcut],items[count].shortcut,sizeof(items[count].shortcut));break;
            }
        }
        snprintf(items[count++].label,128,"%s%s",command.checked?"[x] ":"",command.label);
    }
    if (ui->open_menu==5 || ui->open_menu==4) {
        FrontendPanelInfo panel;
        for (size_t i=0; i<frontend_panel_count() && count<120; ++i) {
            if (!frontend_panel_at(i,&panel)) continue;
            bool media = !strcmp(panel.category,"Media");
            if (media != (ui->open_menu==4)) continue;
            items[count]=(DesktopMenuItem){.id=panel.id,.kind=1,.enabled=panel.enabled};
            snprintf(items[count++].label,128,"%s",panel.title);
        }
    }
    if (ui->open_menu==0) {
        if (ui->sessions) for (size_t i=0; i<frontend_session_recent_count(ui->sessions->session) && count<126; ++i) {
            const FrontendImageRequest *recent=frontend_session_recent(ui->sessions->session,i);
            items[count]=(DesktopMenuItem){.id=(unsigned)i,.kind=2,.enabled=true};
            snprintf(items[count++].label,128,"Recent: %.90s",recent->archive_member[0]?recent->archive_member:recent->path);
        }
        items[count]=(DesktopMenuItem){.kind=3,.enabled=true};
        snprintf(items[count++].label,128,"Exit");
    } else if(ui->open_menu==6) {
        items[count]=(DesktopMenuItem){.kind=4,.enabled=true}; snprintf(items[count++].label,128,"Game information");
        items[count]=(DesktopMenuItem){.kind=5,.enabled=true}; snprintf(items[count++].label,128,"Documentation");
        items[count]=(DesktopMenuItem){.kind=6,.enabled=true};snprintf(items[count++].label,128,"Recent messages");
    }
    if (ui->open_menu == 5) {
        /* Put the tools themselves before their less common direct commands. */
        int next = 0;
        for (int i = 0; i < count; ++i) if (items[i].kind == 1) {
            DesktopMenuItem item = items[i];
            memmove(&items[next + 1], &items[next], (size_t)(i - next) * sizeof(items[0]));
            items[next++] = item;
        }
    }
    return count;
}

void frontend_desktop_render(FrontendDesktopUi *ui,int vw,int vh,const char *title,const char *region,const char *state) {
    (void)vw;(void)vh;
    if(ui&&ui->window&&ui->renderer) { desktop_layout(ui,title,region,state); desktop_render_windows(ui); }
}

static void category_defaults(FrontendDesktopUi *ui) {
    FrontendSettings d;
    frontend_settings_defaults(&d);
    switch (ui->settings_category) {
        case 0:
            memcpy(&ui->staged.reopen_last_image, &d.reopen_last_image, sizeof(d.reopen_last_image));
            memcpy(&ui->staged.remember_window_size, &d.remember_window_size, sizeof(d.remember_window_size));
            memcpy(&ui->staged.recent_file_limit, &d.recent_file_limit, sizeof(d.recent_file_limit));
            memcpy(&ui->staged.pause_on_focus_loss, &d.pause_on_focus_loss, sizeof(d.pause_on_focus_loss));
            memcpy(&ui->staged.pause_on_ui, &d.pause_on_ui, sizeof(d.pause_on_ui));
            memcpy(&ui->staged.show_fps, &d.show_fps, sizeof(d.show_fps));
            memcpy(&ui->staged.window_width, &d.window_width, sizeof(d.window_width));
            memcpy(&ui->staged.window_height, &d.window_height, sizeof(d.window_height));
            break;
        case 1:
            memcpy(&ui->staged.region_mode, &d.region_mode, sizeof(d.region_mode));
            memcpy(&ui->staged.speed, &d.speed, sizeof(d.speed));
            memcpy(&ui->staged.fast_forward_speed, &d.fast_forward_speed, sizeof(d.fast_forward_speed));
            memcpy(&ui->staged.rewind_seconds, &d.rewind_seconds, sizeof(d.rewind_seconds));
            ui->staged.rewind_step_frames=d.rewind_step_frames;
            memcpy(&ui->staged.run_ahead_frames, &d.run_ahead_frames, sizeof(d.run_ahead_frames));
            break;
        case 2:
            memcpy(&ui->staged.fullscreen, &d.fullscreen, sizeof(d.fullscreen));
            memcpy(&ui->staged.integer_scaling, &d.integer_scaling, sizeof(d.integer_scaling));
            memcpy(&ui->staged.vsync, &d.vsync, sizeof(d.vsync));
            memcpy(&ui->staged.aspect_mode, &d.aspect_mode, sizeof(d.aspect_mode));
            memcpy(&ui->staged.ntsc_composite, &d.ntsc_composite, sizeof(d.ntsc_composite));
            memcpy(&ui->staged.presentation, &d.presentation, sizeof(d.presentation));
            break;
        case 3:
            memcpy(&ui->staged.audio_mix, &d.audio_mix, sizeof(d.audio_mix));
            memcpy(&ui->staged.muted, &d.muted, sizeof(d.muted));
            memcpy(&ui->staged.audio_device, &d.audio_device, sizeof(d.audio_device));
            memcpy(&ui->staged.audio_sample_rate, &d.audio_sample_rate, sizeof(d.audio_sample_rate));
            memcpy(&ui->staged.audio_buffer_samples, &d.audio_buffer_samples, sizeof(d.audio_buffer_samples));
            break;
        case 4:
            memcpy(&ui->staged.input, &d.input, sizeof(d.input));
            memcpy(&ui->staged.zapper_radius, &d.zapper_radius, sizeof(d.zapper_radius));
            memcpy(&ui->staged.saved_input_overrides, &d.saved_input_overrides, sizeof(d.saved_input_overrides));
            memcpy(&ui->staged.active_profile, &d.active_profile, sizeof(d.active_profile));
            memcpy(&ui->staged.profiles, &d.profiles, sizeof(d.profiles));
            memcpy(&ui->staged.profile_count, &d.profile_count, sizeof(d.profile_count));
            memcpy(&ui->staged.device_guid, &d.device_guid, sizeof(d.device_guid));
            break;
        case 5:
            memcpy(&ui->staged.disk_save_mode, &d.disk_save_mode, sizeof(d.disk_save_mode));
            memcpy(&ui->staged.disk_overlay_path, &d.disk_overlay_path, sizeof(d.disk_overlay_path));
            memcpy(&ui->staged.fds_bios_path, &d.fds_bios_path, sizeof(d.fds_bios_path));
            memcpy(&ui->staged.studybox_bios_path, &d.studybox_bios_path, sizeof(d.studybox_bios_path));
            memcpy(&ui->staged.epsm_adpcm_path, &d.epsm_adpcm_path, sizeof(d.epsm_adpcm_path));
            memcpy(&ui->staged.fcns_kanji_path, &d.fcns_kanji_path, sizeof(d.fcns_kanji_path));
            memcpy(&ui->staged.tape_play_path, &d.tape_play_path, sizeof(d.tape_play_path));
            memcpy(&ui->staged.tape_record_path, &d.tape_record_path, sizeof(d.tape_record_path));
            memcpy(&ui->staged.fds_write_protected, &d.fds_write_protected, sizeof(d.fds_write_protected));
            memcpy(&ui->staged.fds_auto_insert, &d.fds_auto_insert, sizeof(d.fds_auto_insert));
            memcpy(&ui->staged.fds_loading_fast_forward, &d.fds_loading_fast_forward, sizeof(d.fds_loading_fast_forward));
            memcpy(&ui->staged.nsf_player, &d.nsf_player, sizeof(d.nsf_player));
            break;
        case 6:
            memcpy(&ui->staged.capture, &d.capture, sizeof(d.capture));
            memcpy(&ui->staged.capture_paths, &d.capture_paths, sizeof(d.capture_paths));
            memcpy(&ui->staged.state_slot, &d.state_slot, sizeof(d.state_slot));
            memcpy(&ui->staged.state_file_path, &d.state_file_path, sizeof(d.state_file_path));
            break;
        case 7:
            memcpy(&ui->staged.console_model, &d.console_model, sizeof(d.console_model));
            memcpy(&ui->staged.cpu_revision, &d.cpu_revision, sizeof(d.cpu_revision));
            memcpy(&ui->staged.ppu_revision, &d.ppu_revision, sizeof(d.ppu_revision));
            memcpy(&ui->staged.ram_power_state, &d.ram_power_state, sizeof(d.ram_power_state));
            memcpy(&ui->staged.randomize_vblank, &d.randomize_vblank, sizeof(d.randomize_vblank));
            memcpy(&ui->staged.apu_disable_noise_mode, &d.apu_disable_noise_mode, sizeof(d.apu_disable_noise_mode));
            memcpy(&ui->staged.apu_swap_duty_cycles, &d.apu_swap_duty_cycles, sizeof(d.apu_swap_duty_cycles));
            memcpy(&ui->staged.ppu_oam_row_corruption, &d.ppu_oam_row_corruption, sizeof(d.ppu_oam_row_corruption));
            memcpy(&ui->staged.ppu_startup_restriction, &d.ppu_startup_restriction, sizeof(d.ppu_startup_restriction));
            memcpy(&ui->staged.ppu_oam_decay, &d.ppu_oam_decay, sizeof(d.ppu_oam_decay));
            memcpy(&ui->staged.ppu_sprite_eval_wrap_bug, &d.ppu_sprite_eval_wrap_bug, sizeof(d.ppu_sprite_eval_wrap_bug));
            memcpy(&ui->staged.ppu_oamdata_read_disabled, &d.ppu_oamdata_read_disabled, sizeof(d.ppu_oamdata_read_disabled));
            memcpy(&ui->staged.ppu_palette_readback_disabled, &d.ppu_palette_readback_disabled, sizeof(d.ppu_palette_readback_disabled));
            memcpy(&ui->staged.ppu_reset_suppression, &d.ppu_reset_suppression, sizeof(d.ppu_reset_suppression));
            memcpy(&ui->staged.mmc3_revision_a, &d.mmc3_revision_a, sizeof(d.mmc3_revision_a));
            memcpy(&ui->staged.cart_dips, &d.cart_dips, sizeof(d.cart_dips));
            memcpy(&ui->staged.vs_dips, &d.vs_dips, sizeof(d.vs_dips));
            memcpy(&ui->staged.startup_phase_set, &d.startup_phase_set, sizeof(d.startup_phase_set));
            memcpy(&ui->staged.startup_cpu_offset, &d.startup_cpu_offset, sizeof(d.startup_cpu_offset));
            memcpy(&ui->staged.startup_ppu_phase, &d.startup_ppu_phase, sizeof(d.startup_ppu_phase));
            memcpy(&ui->staged.startup_seed_set, &d.startup_seed_set, sizeof(d.startup_seed_set));
            memcpy(&ui->staged.startup_seed, &d.startup_seed, sizeof(d.startup_seed));
            memcpy(&ui->staged.power_on_seed_set, &d.power_on_seed_set, sizeof(d.power_on_seed_set));
            memcpy(&ui->staged.power_on_seed, &d.power_on_seed, sizeof(d.power_on_seed));
            break;
    }
}

void desktop_settings_button(FrontendDesktopUi *ui, int button) {
    if (button == 0) (void)save_settings(ui);
    else if (button == 1) { if (save_settings(ui)) desktop_settings_open(ui, false); }
    else if (button == 2) desktop_settings_open(ui, false);
    else if (button == 3) category_defaults(ui);
    else if (button == 4) {
        if (ui->confirm_restore_all) {
            uint32_t overrides = ui->staged.cli_overrides;
            frontend_settings_defaults(&ui->staged);
            ui->staged.cli_overrides = overrides;
            ui->confirm_restore_all = false;
        } else { ui->confirm_restore_all = true; desktop_copy_status(ui, "Activate Reset all again to confirm"); }
    }
}

bool frontend_desktop_input_captured(const FrontendDesktopUi *ui) {
    if (!ui) return false;
    for (const FrontendDesktopUi *tool = ui->tools; tool; tool = tool->next)
        if (tool->focused && (tool->settings_open || tool->edit_text_active || tool->capture_binding)) return true;
    return ui->settings_open || ui->panel_open || ui->info_open || ui->edit_text_active ||
        ui->capture_binding || ui->open_menu >= 0 || desktop_palette_visible(ui);
}
bool frontend_desktop_quit_requested(const FrontendDesktopUi *ui){return ui&&ui->quit_requested;}
void frontend_desktop_update_window_settings(FrontendDesktopUi *ui){if(!ui||ui->parent||!ui->window||!ui->settings||!ui->settings->remember_window_size)return;int w,h;SDL_GetWindowSize(ui->window,&w,&h);if(w>0&&h>0){ui->settings->window_width=(unsigned)w;ui->settings->window_height=(unsigned)h;}}
void frontend_desktop_shutdown(FrontendDesktopUi *ui){if(!ui)return;desktop_close_windows(ui);desktop_settings_open(ui,false);SDL_StopTextInput();desktop_clay_destroy(ui->clay);ui->clay=NULL;}

bool desktop_palette_visible(const FrontendDesktopUi *ui) {
    return ui && (ui->palette_window || (!ui->parent && palette_tool_is_visible()));
}
