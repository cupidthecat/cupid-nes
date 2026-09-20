/*
 * desktop_ui.c - SDL desktop menus, toolbar, status, and settings interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "desktop_ui.h"
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

static bool save_settings(FrontendDesktopUi *ui) {
    if (!ui || !ui->settings) return false;
    char error[256] = {0};
    if (!frontend_settings_apply_core(&ui->staged, error, sizeof(error))) {
        copy_status(ui, error[0] ? error : "Settings are invalid");
        return false;
    }
    *ui->settings = ui->staged;
    if (ui->execution) {
        if (!frontend_execution_set_speeds(ui->execution, ui->settings->speed,
                                           ui->settings->fast_forward_speed)) {
            copy_status(ui, "Emulation speed is invalid");
            return false;
        }
        frontend_execution_set_muted(ui->execution, ui->settings->muted);
    }
    if (ui->window) {
        Uint32 flag = ui->settings->fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0;
        if (SDL_SetWindowFullscreen(ui->window, flag) != 0) copy_status(ui, SDL_GetError());
    }
    if (ui->settings_path) {
        FrontendSettingsReport report;
        if (!frontend_settings_save(ui->settings_path, ui->settings, &report)) {
            copy_status(ui, report.message);
            return false;
        }
    }
    copy_status(ui, "Settings applied");
    return true;
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

static int setting_rows(const FrontendDesktopUi *ui) {
    (void)ui;
    switch (ui->settings_category) {
        case 0: return 6;
        case 1: return 3;
        case 2: return 3;
        case 3: return 1;
        case 4: return 8 + 8 + FRONTEND_SHORTCUT_COUNT;
        case 5: return 11;
        case 6: return 5;
        case 7: return 2;
        default: return 0;
    }
}

static void setting_text(FrontendDesktopUi *ui, int row, char *label, size_t lc,
                         char *value, size_t vc) {
    FrontendSettings *s = &ui->staged;
    label[0] = value[0] = '\0';
    if (ui->settings_category == 0) {
        const char *labels[] = {"Reopen last image", "Pause on focus loss", "Pause while UI is open",
                                "Show frame rate", "Window width", "Window height"};
        snprintf(label, lc, "%s", labels[row]);
        if (row < 4) {
            bool flags[] = {s->reopen_last_image, s->pause_on_focus_loss, s->pause_on_ui, s->show_fps};
            snprintf(value, vc, "%s", flags[row] ? "On" : "Off");
        } else snprintf(value, vc, "%u", row == 4 ? s->window_width : s->window_height);
    } else if (ui->settings_category == 1) {
        const char *labels[] = {"Timing selection", "Emulation speed", "Fast-forward speed"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", region_value(s->region_mode));
        else snprintf(value, vc, "%.2gx", row == 1 ? s->speed : s->fast_forward_speed);
    } else if (ui->settings_category == 2) {
        const char *labels[] = {"Fullscreen", "Integer scaling", "Composite output"};
        bool flags[] = {s->fullscreen, s->integer_scaling, s->ntsc_composite};
        snprintf(label, lc, "%s", labels[row]); snprintf(value, vc, "%s", flags[row] ? "On" : "Off");
    } else if (ui->settings_category == 3) {
        snprintf(label, lc, "Mute audio"); snprintf(value, vc, "%s", s->muted ? "On" : "Off");
    } else if (ui->settings_category == 4) {
        const FrontendBindingProfile *profile = frontend_settings_active_profile_const(s);
        if (row == 0) { snprintf(label, lc, "Binding profile"); snprintf(value, vc, "%s", s->active_profile); }
        else if (row == 1) { snprintf(label, lc, "Input adapter"); snprintf(value, vc, "%d", (int)s->input.adapter); }
        else if (row == 2 || row == 3) { snprintf(label, lc, "Port %d device", row - 1); snprintf(value, vc, "%d", (int)s->input.ports[row - 2]); }
        else if (row == 4) { snprintf(label, lc, "Expansion device"); snprintf(value, vc, "%d", (int)s->input.expansion); }
        else if (row == 5) { snprintf(label, lc, "Light radius"); snprintf(value, vc, "%u", s->zapper_radius); }
        else if (row == 6) { snprintf(label, lc, "Editing player"); snprintf(value, vc, "%u", ui->settings_player + 1u); }
        else if (row == 7) { snprintf(label, lc, "Host controller GUID"); snprintf(value, vc, "%s", s->device_guid[ui->settings_player][0] ? s->device_guid[ui->settings_player] : "Auto"); }
        else if (row < 16) {
            unsigned button = (unsigned)(row - 8);
            const FrontendHostBinding *b = profile ? &profile->players[ui->settings_player][button] : NULL;
            snprintf(label, lc, "Player key: %s", frontend_player_button_name(button));
            snprintf(value, vc, "%s", b && b->key != SDL_SCANCODE_UNKNOWN ? SDL_GetScancodeName(b->key) : "Unbound");
        } else {
            unsigned shortcut = (unsigned)(row - 16);
            const FrontendHostBinding *b = profile ? &profile->shortcuts[shortcut] : NULL;
            snprintf(label, lc, "Shortcut: %s", frontend_shortcut_name((FrontendShortcut)shortcut));
            snprintf(value, vc, "%s", b && b->key != SDL_SCANCODE_UNKNOWN ? SDL_GetScancodeName(b->key) : "Unbound");
        }
    } else if (ui->settings_category == 5) {
        const char *labels[] = {"FDS BIOS path", "StudyBox BIOS path", "Disk overlay path", "Disk save mode",
            "Disk write protection", "Automatic disk insertion", "Fast-forward disk loading",
            "Music auto advance", "Music repeat", "Music shuffle", "Music silence detection"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "%s", s->fds_bios_path);
        else if (row == 1) snprintf(value, vc, "%s", s->studybox_bios_path);
        else if (row == 2) snprintf(value, vc, "%s", s->disk_overlay_path);
        else if (row == 3) snprintf(value, vc, "%s", s->disk_save_mode == FDS_SAVE_OVERLAY ? "Overlay" : "In place");
        else { bool flags[] = {s->fds_write_protected, s->fds_auto_insert, s->fds_loading_fast_forward,
                    s->nsf_player.automatic, s->nsf_player.repeat, s->nsf_player.shuffle, s->nsf_player.detect_silence};
            snprintf(value, vc, "%s", flags[row - 4] ? "On" : "Off"); }
    } else if (ui->settings_category == 6) {
        const char *labels[] = {"State slot", "State file path", "Screenshot path", "Audio capture path", "Video capture path"};
        snprintf(label, lc, "%s", labels[row]);
        if (row == 0) snprintf(value, vc, "Slot %u", s->state_slot + 1u);
        else if (row == 1) snprintf(value, vc, "%s", s->state_file_path);
        else snprintf(value, vc, "%s", s->capture_paths[row - 2]);
    } else if (ui->settings_category == 7) {
        snprintf(label, lc, "%s", row == 0 ? "Console wiring" : "Effective timing");
        snprintf(value, vc, "%s", row == 0 ? console_value(s->console_model) : nes_region_name(nes_timing()->region));
    }
}

static void begin_edit(FrontendDesktopUi *ui, unsigned row, const char *text) {
    ui->edit_control = 0x80000000u | row;
    snprintf(ui->edit_text, sizeof(ui->edit_text), "%s", text ? text : "");
    ui->edit_text_active = true;
    SDL_StartTextInput();
}

static void adjust_setting(FrontendDesktopUi *ui, int row, int direction) {
    FrontendSettings *s = &ui->staged;
    if (ui->settings_category == 0) {
        if (row == 0) s->reopen_last_image = !s->reopen_last_image;
        else if (row == 1) s->pause_on_focus_loss = !s->pause_on_focus_loss;
        else if (row == 2) s->pause_on_ui = !s->pause_on_ui;
        else if (row == 3) s->show_fps = !s->show_fps;
        else if (row == 4) s->window_width = (unsigned)((int)s->window_width + direction * 32 < 320 ? 320 : (int)s->window_width + direction * 32);
        else s->window_height = (unsigned)((int)s->window_height + direction * 24 < 240 ? 240 : (int)s->window_height + direction * 24);
    } else if (ui->settings_category == 1) {
        if (row == 0) s->region_mode = (NesRegionMode)(((int)s->region_mode + direction + 4) % 4);
        else if (row == 1) { s->speed += direction * 0.25; if (s->speed < .25) s->speed = .25; if (s->speed > 4) s->speed = 4; }
        else { s->fast_forward_speed += direction; if (s->fast_forward_speed < 1) s->fast_forward_speed = 1; if (s->fast_forward_speed > 16) s->fast_forward_speed = 16; }
    } else if (ui->settings_category == 2) {
        if (row == 0) s->fullscreen = !s->fullscreen;
        else if (row == 1) s->integer_scaling = !s->integer_scaling;
        else s->ntsc_composite = !s->ntsc_composite;
    } else if (ui->settings_category == 3) s->muted = !s->muted;
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
        else { ui->capture_binding = true; ui->capture_shortcut = row >= 16; ui->capture_index = (unsigned)(row - (row >= 16 ? 16 : 8)); copy_status(ui, "Press a key for this binding (Backspace clears)"); }
    } else if (ui->settings_category == 5) {
        if (row <= 2) { const char *texts[] = {s->fds_bios_path, s->studybox_bios_path, s->disk_overlay_path}; begin_edit(ui, (unsigned)row, texts[row]); }
        else if (row == 3) s->disk_save_mode = s->disk_save_mode == FDS_SAVE_OVERLAY ? FDS_SAVE_IN_PLACE : FDS_SAVE_OVERLAY;
        else { bool *flags[] = {&s->fds_write_protected, &s->fds_auto_insert, &s->fds_loading_fast_forward,
                &s->nsf_player.automatic, &s->nsf_player.repeat, &s->nsf_player.shuffle, &s->nsf_player.detect_silence}; *flags[row - 4] = !*flags[row - 4]; }
    } else if (ui->settings_category == 6) {
        if (row == 0) s->state_slot = (s->state_slot + NES_STATE_SLOT_COUNT + direction) % NES_STATE_SLOT_COUNT;
        else { const char *text = row == 1 ? s->state_file_path : s->capture_paths[row - 2]; begin_edit(ui, (unsigned)row, text); }
    } else if (ui->settings_category == 7 && row == 0) s->console_model = (NesConsoleModel)(((int)s->console_model + direction + 4) % 4);
}

static void commit_edit(FrontendDesktopUi *ui) {
    if (!ui || !ui->edit_text_active) return;
    unsigned row = ui->edit_control & 0x7fffffffu;
    FrontendSettings *s = &ui->staged;
    if (ui->settings_open) {
        if (ui->settings_category == 4 && row == 7) snprintf(s->device_guid[ui->settings_player], FRONTEND_SETTINGS_GUID_TEXT, "%s", ui->edit_text);
        else if (ui->settings_category == 5 && row <= 2) {
            char *targets[] = {s->fds_bios_path, s->studybox_bios_path, s->disk_overlay_path}; snprintf(targets[row], FRONTEND_SETTINGS_PATH_TEXT, "%s", ui->edit_text);
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
    if (ui->capture_binding) frontend_draw_text(ui->renderer,box.x+180,box.y+370,1,"Press key; Backspace clears",255,210,120,255);
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
    if(ui->open_menu==4) count+=2; if(!count)return;
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

static void handle_panel_mouse(FrontendDesktopUi *ui,int x,int y){int ww=0,wh=0;SDL_GetWindowSize(ui->window,&ww,&wh);SDL_Rect box={(ww-600)/2,(wh-430)/2,600,430};if(!inside(x,y,box))return;int row=(y-(box.y+52))/23;if(row<0)return;FrontendPanelControl cs[64];FrontendPanelModel m={.controls=cs,.capacity=64};char e[256]={0};if(!frontend_panel_snapshot(ui->panel_id,&m,e,sizeof(e))||(size_t)row>=m.count)return;FrontendPanelControl*c=&cs[row];if(!c->enabled)return;if(c->type==FRONTEND_PANEL_TEXT&&!c->read_only){ui->edit_control=c->id;snprintf(ui->edit_text,sizeof(ui->edit_text),"%s",c->value?c->value:"");ui->edit_text_active=true;SDL_StartTextInput();return;}int selected=c->selected;if(c->type==FRONTEND_PANEL_CHOICE&&c->item_count)selected=(selected+1)%(int)c->item_count;if(!frontend_panel_action(ui->panel_id,c->id,NULL,selected,e,sizeof(e))&&e[0])copy_status(ui,e);}

bool frontend_desktop_handle_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui || !event) return false;
    if(event->type==SDL_WINDOWEVENT&&ui->window&&event->window.windowID==SDL_GetWindowID(ui->window)){if(event->window.event==SDL_WINDOWEVENT_SIZE_CHANGED){ui->settings->window_width=(unsigned)event->window.data1;ui->settings->window_height=(unsigned)event->window.data2;sync_scale(ui);}if(event->window.event==SDL_WINDOWEVENT_FOCUS_LOST&&ui->settings->pause_on_focus_loss&&ui->execution&&!frontend_execution_paused(ui->execution)){ui->paused_for_focus=invoke_command(ui,FRONTEND_COMMAND_PAUSE);}if(event->window.event==SDL_WINDOWEVENT_FOCUS_GAINED&&ui->paused_for_focus&&ui->execution&&frontend_execution_paused(ui->execution)){(void)invoke_command(ui,FRONTEND_COMMAND_PAUSE);ui->paused_for_focus=false;}}
    if(event->type==SDL_TEXTINPUT&&ui->edit_text_active){size_t n=strlen(ui->edit_text),a=strlen(event->text.text);if(n+a<sizeof(ui->edit_text)){memcpy(ui->edit_text+n,event->text.text,a+1);}return true;}
    if((event->type==SDL_KEYDOWN||event->type==SDL_KEYUP)&&ui->capture_binding){if(event->type==SDL_KEYDOWN&&!event->key.repeat)handle_binding_key(ui,&event->key);return true;}
    if(event->type==SDL_KEYDOWN&&!event->key.repeat){SDL_Scancode sc=event->key.keysym.scancode;if(ui->edit_text_active){if(sc==SDL_SCANCODE_RETURN)commit_edit(ui);else if(sc==SDL_SCANCODE_ESCAPE){ui->edit_text_active=false;SDL_StopTextInput();}else if(sc==SDL_SCANCODE_BACKSPACE){size_t n=strlen(ui->edit_text);if(n)ui->edit_text[n-1]='\0';}return true;}if(sc==SDL_SCANCODE_ESCAPE){if(ui->settings_open)set_settings_open(ui,false);else if(ui->panel_open)ui->panel_open=false;else if(ui->info_open)ui->info_open=false;else ui->open_menu=-1;return true;}if(ui->settings_open){int rows=setting_rows(ui);if(sc==SDL_SCANCODE_TAB||sc==SDL_SCANCODE_DOWN)ui->settings_row=(ui->settings_row+1)%rows;else if(sc==SDL_SCANCODE_UP)ui->settings_row=(ui->settings_row+rows-1)%rows;else if(sc==SDL_SCANCODE_LEFT)adjust_setting(ui,ui->settings_row,-1);else if(sc==SDL_SCANCODE_RIGHT||sc==SDL_SCANCODE_RETURN)adjust_setting(ui,ui->settings_row,1);return true;}if((event->key.keysym.mod&KMOD_ALT)&&sc==SDL_SCANCODE_RETURN){(void)invoke_command(ui,FRONTEND_COMMAND_FULLSCREEN);return true;}if((event->key.keysym.mod&KMOD_CTRL)&&sc==SDL_SCANCODE_COMMA){set_settings_open(ui,true);return true;}}
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
