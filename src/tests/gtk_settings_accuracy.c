/*
 * gtk_settings_accuracy.c - Standalone native settings transaction regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifdef CUPID_GTK_SETTINGS_TEST_MAIN
/* Compile this translation unit alone with GTK4 and SDL2. The native controls and
 * typed model are real; desktop I/O and renderer-dependent adapters are isolated. */
#include "../ui/gtk_settings.c"
#include "../ui/desktop_settings_model.c"
#include <assert.h>

static unsigned activations;

GtkWidget *cupid_gtk_label(const char *text) {
    return gtk_label_new(text);
}

GtkWidget *cupid_gtk_scroll(GtkWidget *child) {
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), child);
    return scroll;
}

void cupid_gtk_margins(GtkWidget *widget, int margin) {
    (void)widget;
    (void)margin;
}

SDL_Scancode cupid_gtk_scancode(guint key) {
    (void)key;
    return SDL_SCANCODE_A;
}

SDL_Keymod cupid_gtk_modifiers(GdkModifierType mods) {
    (void)mods;
    return KMOD_NONE;
}

void desktop_copy_status(FrontendDesktopUi *ui, const char *text) {
    g_strlcpy(ui->status, text, sizeof(ui->status));
}

int desktop_setting_rows(const FrontendDesktopUi *ui) {
    return ui->settings_category == 4 ? 9 : ui->settings_category == 2 ? 36 : ui->settings_category == 5 ? 17 : 8;
}

void desktop_setting_text(FrontendDesktopUi *ui, int row, char *label, size_t lc, char *value, size_t vc) {
    g_snprintf(label, lc, "Row %d", row);
    if (desktop_setting_kind(ui, row) == SETTING_TOGGLE) {
        g_strlcpy(value, ui->staged.pause_on_ui ? "On" : "Off", vc);
    } else if (!desktop_setting_edit_text(ui, row, value, vc)) {
        g_snprintf(value, vc, "Player %u row %d", ui->settings_player, row);
    }
}

void desktop_adjust_setting(FrontendDesktopUi *ui, int row, int delta) {
    if (ui->settings_category == 4 && row == 6) {
        ui->settings_player = (ui->settings_player + NES_INPUT_PLAYERS + delta) % NES_INPUT_PLAYERS;
    } else if (desktop_setting_kind(ui, row) == SETTING_TOGGLE) {
        ui->staged.pause_on_ui = !ui->staged.pause_on_ui;
    }
}

void desktop_begin_edit(FrontendDesktopUi *ui, unsigned row, const char *text) {
    ++activations;
    ui->edit_control = row;
    ui->edit_text_active = true;
    g_strlcpy(ui->edit_text, text, sizeof(ui->edit_text));
}

void desktop_commit_edit(FrontendDesktopUi *ui) {
    unsigned row = ui->edit_control & 0x7fffffff;
    if (ui->settings_category == 4 && row == 7) {
        if (strlen(ui->edit_text) >= FRONTEND_SETTINGS_GUID_TEXT) {
            desktop_copy_status(ui, "GUID too long");
            return;
        }
        g_strlcpy(ui->staged.device_guid[ui->settings_player], ui->edit_text, FRONTEND_SETTINGS_GUID_TEXT);
    }
    ui->edit_text_active = false;
    SDL_StopTextInput();
}

void desktop_browse_setting(FrontendDesktopUi *ui) {
    ++activations;
    g_strlcpy(ui->staged.fds_bios_path, "chosen.nes", sizeof(ui->staged.fds_bios_path));
}

void desktop_binding_key(FrontendDesktopUi *ui, const SDL_KeyboardEvent *event) {
    (void)event;
    ui->capture_binding = false;
}

void desktop_settings_button(FrontendDesktopUi *ui, int id) {
    if (id == 3 || (id == 4 && ui->confirm_restore_all)) {
        ui->staged.zapper_radius = 12;
        ui->staged.device_guid[ui->settings_player][0] = 0;
        ui->confirm_restore_all = false;
    } else if (id == 4) {
        ui->confirm_restore_all = true;
    }
}

bool frontend_panel_snapshot(unsigned id, FrontendPanelModel *model, char *error, size_t size) {
    (void)id;
    (void)model;
    (void)error;
    (void)size;
    return false;
}

bool frontend_panel_action(unsigned id, unsigned control, const char *text, int chosen, char *error, size_t size) {
    (void)id;
    (void)control;
    (void)text;
    (void)chosen;
    (void)error;
    (void)size;
    return false;
}

const NtscCompositeControl *ntsc_composite_control_info(unsigned id) {
    (void)id;
    static const NtscCompositeControl c = {"hue", "Hue", "", -100, 100};
    return &c;
}

int *ntsc_composite_control_value(NtscCompositeSettings *settings, unsigned id) {
    (void)id;
    return &settings->hue;
}

bool ntsc_composite_preset(NtscCompositeSettings *settings, unsigned preset) {
    settings->hue = (int)preset * 10;
    return true;
}

int ntsc_composite_preset_index(const NtscCompositeSettings *settings) {
    return settings->hue / 10;
}

const char *ntsc_composite_preset_name(unsigned preset) {
    static const char *names[] = {"A", "B", "C", "D"};
    return names[preset];
}

int main(void) {
    gtk_init();
    CupidGtkTool *tool = g_new0(CupidGtkTool, 1);
    tool->window = gtk_window_new();
    tool->ui.settings_open = true;
    tool->ui.settings_category = 4;
    tool->ui.staged.zapper_radius = 3;
    g_strlcpy(tool->ui.staged.device_guid[1], "second", FRONTEND_SETTINGS_GUID_TEXT);
    tool->ui.capture_binding = true;
    tool->ui.settings_row = 8;
    tool->ui.edit_number = true;
    g_strlcpy(tool->ui.edit_text, "untouched", sizeof(tool->ui.edit_text));
    GtkWidget *root = cupid_gtk_settings_new(tool);
    g_object_ref_sink(root);
    Settings *s = g_object_get_data(G_OBJECT(root), "settings");
    assert(!activations && tool->ui.capture_binding && tool->ui.settings_row == 8 && tool->ui.edit_number &&
           !strcmp(tool->ui.edit_text, "untouched"));
    GtkWidget *player = s->rows[6].widget, *radius = s->rows[5].widget, *guid = s->rows[7].widget;
    gtk_editable_set_text(GTK_EDITABLE(guid), "first");
    gtk_drop_down_set_selected(GTK_DROP_DOWN(player), 1);
    assert(tool->ui.settings_player == 1 && !strcmp(tool->ui.staged.device_guid[0], "first"));
    assert(!strcmp(gtk_editable_get_text(GTK_EDITABLE(guid)), "second") && s->rows[6].widget == player);
    char too_long[FRONTEND_SETTINGS_GUID_TEXT + 8];
    memset(too_long, 'X', sizeof(too_long) - 1);
    too_long[sizeof(too_long) - 1] = 0;
    gtk_editable_set_text(GTK_EDITABLE(radius), "9");
    gtk_editable_set_text(GTK_EDITABLE(guid), too_long);
    gtk_drop_down_set_selected(GTK_DROP_DOWN(player), 2);
    assert(tool->ui.settings_player == 1 && gtk_drop_down_get_selected(GTK_DROP_DOWN(player)) == 1);
    assert(tool->ui.staged.zapper_radius == 3 && s->rows[5].dirty && s->rows[7].dirty);
    assert(!strcmp(gtk_editable_get_text(GTK_EDITABLE(guid)), too_long));
    gtk_list_box_select_row(GTK_LIST_BOX(s->categories), gtk_list_box_get_row_at_index(GTK_LIST_BOX(s->categories), 2));
    assert(tool->ui.settings_category == 4 &&
           gtk_list_box_row_get_index(gtk_list_box_get_selected_row(GTK_LIST_BOX(s->categories))) == 4);
    GtkWidget *reset = gtk_button_new();
    g_object_ref_sink(reset);
    g_object_set_data(G_OBJECT(reset), "action", GINT_TO_POINTER(4));
    action(GTK_BUTTON(reset), s);
    assert(s->rows[7].dirty);
    action(GTK_BUTTON(reset), s);
    assert(!s->rows[7].dirty && !strcmp(gtk_editable_get_text(GTK_EDITABLE(radius)), "12") &&
           s->rows[6].widget == player);
    gtk_list_box_select_row(GTK_LIST_BOX(s->categories), gtk_list_box_get_row_at_index(GTK_LIST_BOX(s->categories), 2));
    GtkWidget *hue = s->rows[26].widget;
    gtk_editable_set_text(GTK_EDITABLE(hue), "7");
    gtk_drop_down_set_selected(GTK_DROP_DOWN(s->rows[25].widget), 2);
    assert(tool->ui.staged.ntsc_picture.hue == 20 && !strcmp(gtk_editable_get_text(GTK_EDITABLE(hue)), "20"));
    gtk_editable_set_text(GTK_EDITABLE(hue), "bad");
    gtk_drop_down_set_selected(GTK_DROP_DOWN(s->rows[25].widget), 3);
    assert(tool->ui.staged.ntsc_picture.hue == 20 &&
           gtk_drop_down_get_selected(GTK_DROP_DOWN(s->rows[25].widget)) == 2);
    assert(!strcmp(gtk_editable_get_text(GTK_EDITABLE(hue)), "bad"));
    cupid_gtk_settings_reset(root);
    gtk_list_box_select_row(GTK_LIST_BOX(s->categories), gtk_list_box_get_row_at_index(GTK_LIST_BOX(s->categories), 5));
    assert(!strcmp(gtk_editable_get_text(GTK_EDITABLE(s->rows[0].widget)), ""));
    activate(NULL, &s->rows[0]);
    assert(!strcmp(gtk_editable_get_text(GTK_EDITABLE(s->rows[0].widget)), "chosen.nes"));
    tool->ui.staged.pause_on_ui = true;
    cupid_gtk_settings_refresh(root);
    assert(gtk_check_button_get_active(GTK_CHECK_BUTTON(s->rows[6].widget)));
    tool->ui.settings_category = 4;
    tool->ui.settings_player = 0;
    cupid_gtk_settings_reset(root);
    assert(s->built_category == 4 && s->rows[7].kind == SETTING_TEXT);
    assert(gtk_list_box_row_get_index(gtk_list_box_get_selected_row(GTK_LIST_BOX(s->categories))) == 4);
    g_object_unref(reset);
    g_object_unref(root);
    gtk_window_destroy(GTK_WINDOW(tool->window));
    g_free(tool);
    puts("GTK settings context rollback, atomic pending edits, raw reads, presets, browse and reset: PASS");
    return 0;
}
#endif
