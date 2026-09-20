/* Desktop input and command routing. SPDX-License-Identifier: GPL-3.0-or-later */
#include "desktop_internal.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "palette_tool.h"
#include "platform_frontend.h"
#include <stdio.h>
#include <string.h>

static void activate_menu_row(FrontendDesktopUi *ui, int row) {
    DesktopMenuItem items[128];
    int count = desktop_menu_items(ui, items);
    if (row < 0 || row >= count) {
        ui->open_menu = -1;
        return;
    }
    DesktopMenuItem *item = &items[row];
    if (!item->enabled) {
        char message[256];
        snprintf(message, sizeof(message),
                 "%s is unavailable for this session. Check the loaded media and device settings.", item->label);
        if (item->id == DEVICE_COMMAND_DISK_TOGGLE || item->id == DEVICE_COMMAND_DISK_NEXT ||
            item->id == DEVICE_PANEL_DISK) {
            snprintf(message, sizeof(message),
                     "Disk controls require a loaded FDS image. Use File > Open to load a disk image.");
        } else if ((item->id >= DEVICE_COMMAND_TAPE_PLAY && item->id <= DEVICE_COMMAND_TAPE_STOP) ||
                   item->id == DEVICE_PANEL_TAPE) {
            snprintf(message, sizeof(message),
                     "Tape controls require the Family BASIC keyboard. Select it under Settings > Controllers > "
                     "Expansion device.");
        }
        desktop_copy_status(ui, message);
        ui->open_menu = -1;
        return;
    }
    char error[256] = {0};
    if (item->kind == 0) {
        if (item->id == FRONTEND_COMMAND_FAST_FORWARD_HOLD) {
            item->id = FRONTEND_COMMAND_FAST_FORWARD_TOGGLE;
        }
        if (desktop_invoke_command(ui, item->id) && ui->devices && item->id >= DEVICE_COMMAND_DISK_TOGGLE &&
            item->id <= DEVICE_COMMAND_BARCODE_SCAN) {
            desktop_copy_status(ui, ui->devices->status);
        }
    } else if (item->kind == 1 && ui->native_windows) {
        (void)desktop_open_window(ui, 1, item->id);
    } else if (item->kind == 1) {
        ui->panel_id = item->id;
        ui->panel_row = ui->panel_scroll = 0;
        ui->panel_open = true;
    } else if (item->kind == 2) {
        if (!frontend_session_action_open_recent(ui->sessions, item->id, error, sizeof(error))) {
            desktop_copy_status(ui, error);
        }
    } else if (item->kind == 3) {
        SDL_Event quit = {.type = SDL_QUIT};
        SDL_PushEvent(&quit);
    } else if ((item->kind == 4 || item->kind == 6) && ui->native_windows) {
        (void)desktop_open_window(ui, 2, item->kind == 6);
    } else if (item->kind == 4) {
        ui->info_open = true;
        ui->log_open = false;
    } else if (item->kind == 6) {
        ui->info_open = true;
        ui->log_open = true;
    } else if (item->kind == 5) {
        (void)SDL_OpenURL("https://github.com/cupidthecat/cupid-nes/tree/main/docs");
    }
    ui->open_menu = -1;
}

static void menu_key(FrontendDesktopUi *ui, SDL_Scancode sc) {
    DesktopMenuItem items[128];
    int count = desktop_menu_items(ui, items);
    if (sc == SDL_SCANCODE_LEFT || sc == SDL_SCANCODE_RIGHT) {
        ui->open_menu = (ui->open_menu + (sc == SDL_SCANCODE_LEFT ? 6 : 1)) % 7;
        ui->menu_row = ui->menu_scroll = 0;
    } else if (count && (sc == SDL_SCANCODE_UP || sc == SDL_SCANCODE_DOWN)) {
        ui->menu_row = (ui->menu_row + count + (sc == SDL_SCANCODE_UP ? -1 : 1)) % count;
    } else if (sc == SDL_SCANCODE_RETURN || sc == SDL_SCANCODE_SPACE) {
        activate_menu_row(ui, ui->menu_row);
    }
}

static void activate_panel(FrontendDesktopUi *ui, int row, int direction) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    char error[256] = {0};
    if (!frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error)) || row < 0 || (size_t)row >= model.count) {
        return;
    }
    ui->panel_row = row;
    FrontendPanelControl *control = &controls[row];
    if (!control->enabled || control->read_only) {
        return;
    }
    if (!direction) {
        direction = 1;
    }
    if (control->type == FRONTEND_PANEL_TEXT) {
        desktop_start_text_edit(ui, control->id, control->value);
        return;
    }
    int selected = control->type == FRONTEND_PANEL_CHECKBOX ? !control->selected : control->selected;
    if ((control->type == FRONTEND_PANEL_CHOICE || control->type == FRONTEND_PANEL_LIST) && control->item_count) {
        selected = (selected + (int)control->item_count + direction) % (int)control->item_count;
    }
    if (!frontend_panel_action(ui->panel_id, control->id, NULL, selected, error, sizeof(error)) && error[0]) {
        desktop_copy_status(ui, error);
    }
}

static void panel_key(FrontendDesktopUi *ui, SDL_Scancode sc) {
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(ui->panel_id, &model, NULL, 0) || !model.count) {
        return;
    }
    if (ui->panel_row < 0 || (size_t)ui->panel_row >= model.count) {
        ui->panel_row = 0;
    }
    if (sc == SDL_SCANCODE_DOWN || sc == SDL_SCANCODE_TAB) {
        ui->panel_row = (ui->panel_row + 1) % (int)model.count;
    } else if (sc == SDL_SCANCODE_UP) {
        ui->panel_row = (ui->panel_row + (int)model.count - 1) % (int)model.count;
    } else if (sc == SDL_SCANCODE_LEFT) {
        activate_panel(ui, ui->panel_row, -1);
    } else if (sc == SDL_SCANCODE_RETURN || sc == SDL_SCANCODE_SPACE || sc == SDL_SCANCODE_RIGHT) {
        activate_panel(ui, ui->panel_row, 1);
    }
    if (ui->panel_row < ui->panel_scroll) {
        ui->panel_scroll = ui->panel_row;
    }
    if (ui->panel_row >= ui->panel_scroll + ui->visible_rows) {
        ui->panel_scroll = ui->panel_row - ui->visible_rows + 1;
    }
}

static void browse_setting(FrontendDesktopUi *ui) {
    int row = ui->settings_row;
    char path[FRONTEND_SETTINGS_PATH_TEXT] = {0}, error[256] = {0};
    bool chosen = false;
    if (ui->settings_category == 5 && (row <= 3 || row == 9)) {
        chosen = frontend_open_file_dialog(row == 9 ? FRONTEND_OPEN_TAPE : FRONTEND_OPEN_FIRMWARE, path, sizeof(path),
                                           error, sizeof(error));
    } else if (ui->settings_category == 5 && row == 10) {
        chosen = frontend_save_file_dialog(FRONTEND_SAVE_TAPE, path, sizeof(path), error, sizeof(error));
    } else if (ui->settings_category == 6 && row >= 1 && row <= 4) {
        chosen = frontend_save_file_dialog(row == 1 ? FRONTEND_SAVE_STATE : (FrontendSaveFileType)(row - 2), path,
                                           sizeof(path), error, sizeof(error));
    }
    if (chosen) {
        desktop_begin_edit(ui, (unsigned)row, path);
        desktop_commit_edit(ui);
    } else if (error[0]) {
        desktop_copy_status(ui, error);
    }
}

static void settings_key(FrontendDesktopUi *ui, SDL_Scancode sc, SDL_Keymod mod) {
    if (sc == SDL_SCANCODE_F4) {
        browse_setting(ui);
    } else if (sc == SDL_SCANCODE_TAB && (mod & KMOD_CTRL)) {
        ui->settings_category = (ui->settings_category + ((mod & KMOD_SHIFT) ? 7 : 1)) % 8;
        ui->settings_row = ui->settings_scroll = 0;
    } else if (sc == SDL_SCANCODE_TAB) {
        ui->settings_focus = (ui->settings_focus + ((mod & KMOD_SHIFT) ? 2 : 1)) % 3;
    } else if (ui->settings_focus == 2) {
        if (sc == SDL_SCANCODE_UP || sc == SDL_SCANCODE_DOWN) {
            ui->settings_category = (ui->settings_category + (sc == SDL_SCANCODE_UP ? 7 : 1)) % 8;
            ui->settings_row = ui->settings_scroll = 0;
        } else if (sc == SDL_SCANCODE_RETURN || sc == SDL_SCANCODE_RIGHT) {
            ui->settings_focus = 0;
        }
    } else if (ui->settings_focus == 1) {
        if (sc == SDL_SCANCODE_LEFT || sc == SDL_SCANCODE_RIGHT) {
            ui->settings_button = (ui->settings_button + (sc == SDL_SCANCODE_LEFT ? 4 : 1)) % 5;
        } else if (sc == SDL_SCANCODE_RETURN || sc == SDL_SCANCODE_SPACE) {
            desktop_settings_button(ui, ui->settings_button);
        }
    } else {
        int rows = desktop_setting_rows(ui);
        if (sc == SDL_SCANCODE_DOWN) {
            ui->settings_row = (ui->settings_row + 1) % rows;
        } else if (sc == SDL_SCANCODE_UP) {
            ui->settings_row = (ui->settings_row + rows - 1) % rows;
        } else if (sc == SDL_SCANCODE_LEFT) {
            desktop_adjust_setting(ui, ui->settings_row, -1);
        } else if (sc == SDL_SCANCODE_RIGHT || sc == SDL_SCANCODE_RETURN) {
            desktop_adjust_setting(ui, ui->settings_row, 1);
        }
    }
}

static bool handle_click(FrontendDesktopUi *ui, int x, int y) {
    const DesktopHit *found = desktop_clay_at(ui->clay, (float)x, (float)y);
    if (!found) {
        if (ui->open_menu >= 0) {
            ui->open_menu = -1;
            return true;
        }
        return frontend_desktop_input_captured(ui);
    }
    DesktopHit hit = *found;
    switch (hit.kind) {
    case HIT_MENU:
        ui->open_menu = ui->open_menu == hit.index ? -1 : hit.index;
        ui->menu_row = ui->menu_scroll = 0;
        break;
    case HIT_MENU_ROW:
        activate_menu_row(ui, hit.index);
        break;
    case HIT_COMMAND:
        (void)desktop_invoke_command(ui, (unsigned)hit.index);
        break;
    case HIT_CATEGORY:
        ui->settings_category = hit.index;
        ui->settings_row = ui->settings_scroll = 0;
        ui->settings_focus = 2;
        break;
    case HIT_SETTING:
        ui->settings_row = hit.index;
        ui->settings_focus = 0;
        desktop_adjust_setting(ui, hit.index, hit.direction);
        break;
    case HIT_SETTINGS_BUTTON:
        desktop_settings_button(ui, hit.index);
        break;
    case HIT_PANEL:
        activate_panel(ui, hit.index, hit.direction);
        break;
    case HIT_BROWSE:
        browse_setting(ui);
        break;
    case HIT_RECENT:
        ui->idle_recent_index = hit.index;
        break;
    case HIT_CLOSE:
        if (ui->settings_open) {
            desktop_settings_open(ui, false);
        } else if (ui->panel_open) {
            ui->panel_open = false;
        } else if (ui->info_open) {
            ui->info_open = false;
        } else {
            ui->palette_window = false;
            palette_tool_hide_overlay();
        }
        break;
    case HIT_EDIT_OK:
        desktop_commit_edit(ui);
        break;
    case HIT_EDIT_CANCEL:
        ui->edit_text_active = ui->capture_binding = false;
        SDL_StopTextInput();
        break;
    case HIT_COLOR:
        ui->palette_index = hit.index;
        break;
    case HIT_CHANNEL: {
        uint32_t colors[64];
        ppu_palette_get(colors);
        uint32_t color = colors[ui->palette_index];
        int rgb[3] = {(int)((color >> 16) & 255), (int)((color >> 8) & 255), (int)(color & 255)};
        int value = hit.direction ? rgb[hit.index] + hit.direction : (int)((x - hit.bounds.x) * 255 / hit.bounds.width);
        rgb[hit.index] = value < 0 ? 0 : value > 255 ? 255 : value;
        ppu_palette_set_color(ui->palette_index, (uint8_t)rgb[0], (uint8_t)rgb[1], (uint8_t)rgb[2]);
        break;
    }
    case HIT_SCROLL:
        if (ui->open_menu >= 0) {
            menu_key(ui, hit.direction < 0 ? SDL_SCANCODE_UP : SDL_SCANCODE_DOWN);
        } else if (ui->settings_open) {
            ui->settings_focus = 0;
            settings_key(ui, hit.direction < 0 ? SDL_SCANCODE_UP : SDL_SCANCODE_DOWN, KMOD_NONE);
        } else if (ui->panel_open) {
            panel_key(ui, hit.direction < 0 ? SDL_SCANCODE_UP : SDL_SCANCODE_DOWN);
        }
        break;
    default:
        break;
    }
    return true;
}

static void append_text(FrontendDesktopUi *ui, const char *text) {
    size_t length = strlen(text), used = ui->edit_select_all ? 0 : strlen(ui->edit_text);
    if (length >= sizeof(ui->edit_text) - used) {
        desktop_copy_status(ui, "Text is too long for this field");
        return;
    }
    memcpy(ui->edit_text + used, text, length + 1);
    ui->edit_select_all = false;
}

static bool handle_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui || !event) {
        return false;
    }
    if (event->type == SDL_QUIT) {
        return false;
    }
    if (event->type == SDL_DROPFILE && frontend_desktop_input_captured(ui)) {
        if (event->drop.file) {
            if (desktop_palette_visible(ui)) {
                desktop_copy_status(ui, ppu_palette_load_pal_file(event->drop.file) == 0 ? "Palette loaded"
                                                                                         : "Invalid palette file");
            } else {
                desktop_copy_status(ui, "Close the dialog before opening another image");
            }
            SDL_free(event->drop.file);
        }
        return true;
    }
    if (ui->execution && ui->settings && (event->type == SDL_KEYDOWN || event->type == SDL_KEYUP)) {
        const FrontendBindingProfile *profile = frontend_settings_active_profile_const(ui->settings);
        FrontendShortcut shortcut;
        if (profile && event->type == SDL_KEYUP) {
            const FrontendShortcut held[] = {FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, FRONTEND_SHORTCUT_REWIND};
            for (unsigned i = 0; i < 2; ++i) {
                if (profile->shortcuts[held[i]].key == event->key.keysym.scancode) {
                    (void)frontend_execution_handle_shortcut_action(ui->execution, held[i], false, false);
                }
            }
        }
        if (profile && !ui->edit_text_active && !ui->capture_binding && !ui->settings_open &&
            !desktop_palette_visible(ui) && frontend_profile_shortcut_key(profile, &event->key, &shortcut)) {
            ui->open_menu = -1;
            if (shortcut == FRONTEND_SHORTCUT_FAST_FORWARD_HOLD || shortcut == FRONTEND_SHORTCUT_REWIND) {
                return frontend_execution_handle_shortcut_action(ui->execution, shortcut, event->type == SDL_KEYDOWN,
                                                                 event->key.repeat != 0);
            }
            if (event->type == SDL_KEYDOWN && !event->key.repeat) {
                (void)desktop_invoke_command(ui, frontend_execution_shortcut_command(shortcut));
            }
            return true;
        }
    }
    if (event->type == SDL_KEYDOWN && !event->key.repeat && !ui->edit_text_active && !ui->capture_binding) {
        if ((event->key.keysym.mod & KMOD_CTRL) && event->key.keysym.scancode == SDL_SCANCODE_COMMA) {
            desktop_settings_open(ui->parent ? ui->parent : ui, true);
            return true;
        }
        if ((event->key.keysym.mod & KMOD_ALT) && event->key.keysym.scancode == SDL_SCANCODE_RETURN) {
            (void)desktop_invoke_command(ui, FRONTEND_COMMAND_FULLSCREEN);
            return true;
        }
    }
    if (event->type == SDL_KEYDOWN && !event->key.repeat && !ui->edit_text_active && !ui->capture_binding &&
        !ui->settings_open && !ui->panel_open && !ui->info_open) {
        SDL_Scancode sc = event->key.keysym.scancode;
        if (sc == SDL_SCANCODE_F7) {
            if (ui->native_windows) {
                (void)desktop_invoke_command(ui, 0x1B00);
            } else if (ui->palette_window) {
                ui->palette_window = false;
            } else {
                palette_tool_toggle_overlay();
            }
            return true;
        }
        if (desktop_palette_visible(ui) && sc == SDL_SCANCODE_F6) {
            ppu_palette_reset_default();
            return true;
        }
        if (desktop_palette_visible(ui) && sc == SDL_SCANCODE_V && (event->key.keysym.mod & KMOD_CTRL)) {
            char *text = SDL_GetClipboardText();
            if (text) {
                desktop_copy_status(ui, ppu_palette_load_hex_string(text) == 0
                                            ? "Palette loaded"
                                            : "Clipboard does not contain a palette");
                SDL_free(text);
            }
            return true;
        }
    }
    if (ui->edit_text_active && event->type == SDL_KEYDOWN && (event->key.keysym.mod & KMOD_CTRL)) {
        if (event->key.keysym.scancode == SDL_SCANCODE_V) {
            char *text = SDL_GetClipboardText();
            if (text) {
                append_text(ui, text);
                SDL_free(text);
            }
            return true;
        }
        if (event->key.keysym.scancode == SDL_SCANCODE_C && ui->edit_select_all) {
            SDL_SetClipboardText(ui->edit_text);
            return true;
        }
    }
    SDL_Event adjusted = *event;
    if (ui->ui_scale > 1 && (event->type == SDL_MOUSEBUTTONDOWN || event->type == SDL_MOUSEBUTTONUP)) {
        adjusted.button.x /= ui->ui_scale;
        adjusted.button.y /= ui->ui_scale;
        event = &adjusted;
    }
    if (event->type == SDL_WINDOWEVENT && ui->window && event->window.windowID == SDL_GetWindowID(ui->window)) {
        if (event->window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
            event->window.event == SDL_WINDOWEVENT_DISPLAY_CHANGED) {
            frontend_desktop_update_window_settings(ui);
            desktop_sync_scale(ui);
        }
        if (event->window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
            ui->focused = false;
            frontend_execution_release_host_input(ui->execution);
        }
        if (event->window.event == SDL_WINDOWEVENT_FOCUS_GAINED) {
            ui->focused = true;
        }
        if (event->window.event == SDL_WINDOWEVENT_CLOSE && !ui->parent) {
            SDL_Event quit = {.type = SDL_QUIT};
            SDL_PushEvent(&quit);
            return true;
        }
    }
    if (event->type == SDL_MOUSEWHEEL && (ui->edit_text_active || ui->capture_binding)) {
        return true;
    }
    if (event->type == SDL_MOUSEWHEEL && ui->open_menu >= 0) {
        menu_key(ui, event->wheel.y > 0 ? SDL_SCANCODE_UP : SDL_SCANCODE_DOWN);
        return true;
    }
    if (event->type == SDL_MOUSEWHEEL && (ui->settings_open || ui->panel_open)) {
        SDL_Scancode direction = event->wheel.y > 0 ? SDL_SCANCODE_UP : SDL_SCANCODE_DOWN;
        if (ui->settings_open) {
            ui->settings_focus = 0;
            settings_key(ui, direction, KMOD_NONE);
        } else {
            panel_key(ui, direction);
        }
        return true;
    }
    if (event->type == SDL_TEXTINPUT && ui->edit_text_active) {
        append_text(ui, event->text.text);
        return true;
    }
    if (ui->capture_binding && event->type == SDL_CONTROLLERBUTTONDOWN && ui->capture_gamepad) {
        desktop_binding_gamepad(ui, (SDL_GameControllerButton)event->cbutton.button);
        return true;
    }
    if ((event->type == SDL_KEYDOWN || event->type == SDL_KEYUP) && ui->capture_binding) {
        if (event->type == SDL_KEYUP && !ui->capture_gamepad && event->key.keysym.scancode >= SDL_SCANCODE_LCTRL &&
            event->key.keysym.scancode <= SDL_SCANCODE_RGUI) {
            desktop_binding_key(ui, &event->key);
        }
        if (event->type == SDL_KEYDOWN && !event->key.repeat) {
            if (event->key.keysym.scancode == SDL_SCANCODE_BACKSPACE && ui->capture_gamepad) {
                desktop_binding_gamepad(ui, SDL_CONTROLLER_BUTTON_INVALID);
            } else if (!ui->capture_gamepad) {
                desktop_binding_key(ui, &event->key);
            }
        }
        return true;
    }
    if (event->type == SDL_KEYDOWN && !event->key.repeat) {
        SDL_Scancode sc = event->key.keysym.scancode;
        if (ui->edit_text_active) {
            if (sc == SDL_SCANCODE_END || sc == SDL_SCANCODE_RIGHT) {
                ui->edit_select_all = false;
            } else if (sc == SDL_SCANCODE_A && (event->key.keysym.mod & KMOD_CTRL)) {
                ui->edit_select_all = true;
            } else if (sc == SDL_SCANCODE_F4 && ui->settings_open) {
                browse_setting(ui);
            } else if (sc == SDL_SCANCODE_RETURN) {
                desktop_commit_edit(ui);
            } else if (sc == SDL_SCANCODE_ESCAPE) {
                ui->edit_text_active = false;
                SDL_StopTextInput();
            } else if (sc == SDL_SCANCODE_BACKSPACE) {
                size_t n = strlen(ui->edit_text);
                if (ui->edit_select_all) {
                    ui->edit_text[0] = '\0';
                    ui->edit_select_all = false;
                } else if (n) {
                    --n;
                    while (n && ((unsigned char)ui->edit_text[n] & 0xC0u) == 0x80u) {
                        --n;
                    }
                    ui->edit_text[n] = '\0';
                }
            }
            return true;
        }
        if (sc == SDL_SCANCODE_ESCAPE && frontend_desktop_input_captured(ui)) {
            if (ui->settings_open) {
                desktop_settings_open(ui, false);
            } else if (ui->panel_open) {
                ui->panel_open = false;
            } else if (ui->info_open) {
                ui->info_open = false;
            } else if (desktop_palette_visible(ui)) {
                ui->palette_window = false;
                palette_tool_hide_overlay();
            } else {
                ui->open_menu = -1;
            }
            return true;
        }
        if (ui->settings_open) {
            settings_key(ui, sc, (SDL_Keymod)event->key.keysym.mod);
            return true;
        }
        if (ui->panel_open) {
            panel_key(ui, sc);
            return true;
        }
        if (ui->open_menu >= 0) {
            menu_key(ui, sc);
            return true;
        }
        if ((event->key.keysym.mod & KMOD_ALT) && sc == SDL_SCANCODE_F) {
            ui->open_menu = 0;
            ui->menu_row = ui->menu_scroll = 0;
            return true;
        }
        if ((event->key.keysym.mod & KMOD_ALT) && sc == SDL_SCANCODE_RETURN) {
            (void)desktop_invoke_command(ui, FRONTEND_COMMAND_FULLSCREEN);
            return true;
        }
        if ((event->key.keysym.mod & KMOD_CTRL) && sc == SDL_SCANCODE_COMMA) {
            desktop_settings_open(ui, true);
            return true;
        }
    }
    if (event->type == SDL_MOUSEBUTTONDOWN && event->button.button == SDL_BUTTON_LEFT) {
        return handle_click(ui, event->button.x, event->button.y);
    }
    if (event->type == SDL_MOUSEMOTION && (event->motion.state & SDL_BUTTON_LMASK) && desktop_palette_visible(ui)) {
        float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
        int x = (int)(event->motion.x / scale), y = (int)(event->motion.y / scale);
        const DesktopHit *hit = desktop_clay_at(ui->clay, (float)x, (float)y);
        if (hit && hit->kind == HIT_CHANNEL && !hit->direction) {
            return handle_click(ui, x, y);
        }
    }
    return frontend_desktop_input_captured(ui);
}

bool frontend_desktop_handle_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!ui || !event) {
        return false;
    }
    if (desktop_route_window(ui, event)) {
        return true;
    }
    bool handled = handle_event(ui, event);
    if (!ui->native_windows && !ui->parent) {
        frontend_desktop_update_activity(ui);
    }
    return handled;
}
