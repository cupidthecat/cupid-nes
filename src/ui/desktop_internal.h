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
int desktop_menu_all(FrontendDesktopUi *ui, DesktopMenuItem items[128]);
int desktop_menu_level(FrontendDesktopUi *ui, int depth, DesktopMenuItem items[128]);
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
bool desktop_palette_visible(const FrontendDesktopUi *ui);
void desktop_window_context(const FrontendDesktopUi *ui, const char **title, const char **region, const char **state);
FrontendDesktopUi *desktop_open_window(FrontendDesktopUi *ui, int kind, unsigned id);
bool desktop_route_window(FrontendDesktopUi *ui, const SDL_Event *event);
void desktop_render_windows(FrontendDesktopUi *ui);
void desktop_close_windows(FrontendDesktopUi *ui);

typedef enum {
    SETTING_TOGGLE,
    SETTING_CHOICE,
    SETTING_NUMBER,
    SETTING_TEXT,
    SETTING_FILE,
    SETTING_BINDING,
    SETTING_READONLY
} DesktopSettingKind;

DesktopSettingKind desktop_setting_kind(const FrontendDesktopUi *ui, int row);
int desktop_setting_choices(FrontendDesktopUi *ui, int row, int *selected);
void desktop_setting_choice_text(FrontendDesktopUi *ui, int row, int option, char *text, size_t size);
void desktop_setting_choose(FrontendDesktopUi *ui, int row, int option);
void desktop_browse_setting(FrontendDesktopUi *ui);
void desktop_activate_setting(FrontendDesktopUi *ui, int row);
bool desktop_setting_commit_number(FrontendDesktopUi *ui, int row, const char *text);
int desktop_choice_count(FrontendDesktopUi *ui);
void desktop_choice_text(FrontendDesktopUi *ui, int option, char *text, size_t size);
void desktop_choice_select(FrontendDesktopUi *ui, int option);
#endif
