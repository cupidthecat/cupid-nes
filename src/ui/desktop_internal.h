/* Desktop view model. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_INTERNAL_H
#define CUPID_DESKTOP_INTERNAL_H
#include "clay_backend.h"
#include "desktop_ui.h"
typedef struct {
    unsigned id, kind;
    bool enabled;
    char label[128], shortcut[80];
} DesktopMenuItem;
int desktop_setting_rows(const FrontendDesktopUi *ui);
void desktop_setting_text(FrontendDesktopUi *ui, int row, char *label, size_t label_size, char *value,
                          size_t value_size);
int desktop_menu_items(FrontendDesktopUi *ui, DesktopMenuItem items[128]);
void desktop_layout(FrontendDesktopUi *ui, const char *title, const char *region, const char *run_state);
void desktop_copy_status(FrontendDesktopUi *ui, const char *text);
bool desktop_invoke_command(FrontendDesktopUi *ui, unsigned id);
void desktop_settings_open(FrontendDesktopUi *ui, bool open);
void desktop_sync_scale(FrontendDesktopUi *ui);
void desktop_adjust_setting(FrontendDesktopUi *ui, int row, int direction);
void desktop_start_text_edit(FrontendDesktopUi *ui, unsigned control, const char *text);
void desktop_begin_edit(FrontendDesktopUi *ui, unsigned row, const char *text);
void desktop_commit_edit(FrontendDesktopUi *ui);
void desktop_settings_button(FrontendDesktopUi *ui, int button);
void desktop_binding_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *key);
void desktop_binding_gamepad(FrontendDesktopUi *ui, SDL_GameControllerButton button);
#endif
