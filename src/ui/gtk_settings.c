/*
 * gtk_settings.c - Native settings pages with staged, validated changes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include <string.h>
typedef struct Settings Settings;

typedef struct {
    Settings *settings;
    int row;
    DesktopSettingKind kind;
    GtkWidget *widget, *label;
    bool dirty;
} Setting;

struct Settings {
    CupidGtkTool *tool;
    GtkWidget *root, *page, *categories;
    Setting rows[256];
    int count, built_category;
    bool updating;
};

static bool commit(Settings *s) {
    FrontendDesktopUi *ui = &s->tool->ui;
    /* Commit to a copy: an invalid later row must not leave earlier rows applied. */
    FrontendDesktopUi *preview = g_new(FrontendDesktopUi, 1);
    *preview = *ui;
    bool valid = true;
    SDL_bool input_active = SDL_IsTextInputActive();
    for (int i = 0; i < s->count; i++) {
        Setting *r = &s->rows[i];
        if (!r->dirty) {
            continue;
        }
        const char *text = gtk_editable_get_text(GTK_EDITABLE(r->widget));
        if (r->kind == SETTING_NUMBER) {
            valid = desktop_setting_commit_number(preview, r->row, text);
        } else if (strlen(text) >= sizeof(preview->edit_text)) {
            desktop_copy_status(preview, "This value is too long.");
            valid = false;
        } else {
            /* Use the shared validator without starting a second text-input owner. */
            preview->settings_open = true;
            preview->edit_text_active = true;
            preview->edit_number = false;
            preview->edit_control = 0x80000000u | (unsigned)r->row;
            g_strlcpy(preview->edit_text, text, sizeof(preview->edit_text));
            desktop_commit_edit(preview);
            valid = !preview->edit_text_active;
        }
        if (!valid) {
            desktop_copy_status(ui, preview->status);
            gtk_widget_grab_focus(r->widget);
            break;
        }
    }
    if (SDL_IsTextInputActive() != input_active) {
        if (input_active) {
            SDL_StartTextInput();
        } else {
            SDL_StopTextInput();
        }
    }
    if (valid) {
        ui->staged = preview->staged;
        for (int i = 0; i < s->count; ++i) {
            s->rows[i].dirty = false;
        }
    }
    g_free(preview);
    return valid;
}

static void build(Settings *s);

static void changed(GtkEditable *w, gpointer data) {
    (void)w;
    Setting *r = data;
    if (!r->settings->updating) {
        r->dirty = true;
    }
}

static void entered(GtkEntry *w, gpointer data) {
    (void)w;
    Setting *r = data;
    commit(r->settings);
    cupid_gtk_settings_refresh(r->settings->root);
}

static void toggled(GtkCheckButton *w, gpointer data) {
    (void)w;
    Setting *r = data;
    Settings *s = r->settings;
    if (s->updating) {
        return;
    }
    if (commit(s)) {
        desktop_adjust_setting(&s->tool->ui, r->row, 1);
    }
    cupid_gtk_settings_refresh(s->root);
}

static void selected(GObject *w, GParamSpec *spec, gpointer data) {
    (void)spec;
    Setting *r = data;
    Settings *s = r->settings;
    if (s->updating) {
        return;
    }
    guint desired = gtk_drop_down_get_selected(GTK_DROP_DOWN(w));
    if (commit(s) && desired != GTK_INVALID_LIST_POSITION) {
        s->tool->ui.capture_binding = false;
        desktop_setting_choose(&s->tool->ui, r->row, (int)desired);
    }
    /* Also restores the old selection after a rejected edit or unavailable choice. */
    cupid_gtk_settings_refresh(s->root);
}

static void activate(GtkButton *w, gpointer data) {
    (void)w;
    Setting *r = data;
    Settings *s = r->settings;
    if (commit(s)) {
        desktop_activate_setting(&s->tool->ui, r->row);
    }
    cupid_gtk_settings_refresh(s->root);
}

static gboolean binding(GtkEventControllerKey *controller, guint key, guint code, GdkModifierType mods, gpointer data) {
    (void)controller;
    (void)code;
    Settings *s = data;
    FrontendDesktopUi *ui = &s->tool->ui;
    if (!ui->capture_binding) {
        return FALSE;
    }
    if (key == GDK_KEY_Escape) {
        ui->capture_binding = false;
        return TRUE;
    }
    SDL_KeyboardEvent event = {.type = SDL_KEYDOWN};
    event.keysym.scancode = cupid_gtk_scancode(key);
    event.keysym.mod = cupid_gtk_modifiers(mods);
    desktop_binding_key(ui, &event);
    return TRUE;
}

static void category(GtkListBox *box, GtkListBoxRow *row, gpointer data) {
    (void)box;
    Settings *s = data;
    if (!row || s->updating) {
        return;
    }
    if (!commit(s)) {
        s->updating = true;
        gtk_list_box_select_row(box, gtk_list_box_get_row_at_index(box, s->tool->ui.settings_category));
        s->updating = false;
        return;
    }
    s->tool->ui.capture_binding = false;
    s->tool->ui.settings_category = gtk_list_box_row_get_index(row);
    build(s);
}

static void action(GtkButton *button, gpointer data) {
    Settings *s = data;
    int id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(button), "action"));
    if (id <= 1 && !commit(s)) {
        return;
    }
    bool reset = id == 3 || (id == 4 && s->tool->ui.confirm_restore_all);
    desktop_settings_button(&s->tool->ui, id);
    if (reset) {
        for (int i = 0; i < s->count; ++i) {
            s->rows[i].dirty = false;
        }
        s->tool->ui.capture_binding = false;
    }
    if (!s->tool->ui.settings_open) {
        gtk_widget_set_visible(s->tool->window, FALSE);
        return;
    }
    cupid_gtk_settings_refresh(s->root);
}

static void build(Settings *s) {
    FrontendDesktopUi *ui = &s->tool->ui;
    s->updating = true;
    s->built_category = ui->settings_category;
    GtkWidget *child;
    while ((child = gtk_widget_get_first_child(s->page))) {
        gtk_box_remove(GTK_BOX(s->page), child);
    }
    s->count = desktop_setting_rows(ui);
    if (s->count > 256) {
        s->count = 256;
    }
    for (int i = 0; i < s->count; i++) {
        char label[256], value[4096];
        desktop_setting_text(ui, i, label, sizeof(label), value, sizeof(value));
        Setting *r = &s->rows[i];
        *r = (Setting){.settings = s, .row = i, .kind = desktop_setting_kind(ui, i)};
        GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 20), *name = cupid_gtk_label(label);
        r->label = name;
        gtk_label_set_wrap(GTK_LABEL(name), TRUE);
        gtk_widget_set_size_request(name, 240, -1);
        gtk_widget_set_halign(name, GTK_ALIGN_START);
        gtk_box_append(GTK_BOX(row), name);
        switch (r->kind) {
        case SETTING_TOGGLE:
            r->widget = gtk_check_button_new();
            gtk_check_button_set_active(GTK_CHECK_BUTTON(r->widget),
                                        !strcmp(value, "On") || !strcmp(value, "Yes") || !strcmp(value, "Enabled"));
            g_signal_connect(r->widget, "toggled", G_CALLBACK(toggled), r);
            break;
        case SETTING_CHOICE: {
            int chosen = 0, n = desktop_setting_choices(ui, i, &chosen);
            GtkStringList *list = gtk_string_list_new(NULL);
            for (int j = 0; j < n; j++) {
                char option[512];
                desktop_setting_choice_text(ui, i, j, option, sizeof(option));
                gtk_string_list_append(list, option);
            }
            r->widget = gtk_drop_down_new(G_LIST_MODEL(list), NULL);
            gtk_drop_down_set_selected(GTK_DROP_DOWN(r->widget), (guint)chosen);
            g_signal_connect(r->widget, "notify::selected", G_CALLBACK(selected), r);
            break;
        }
        case SETTING_NUMBER:
        case SETTING_TEXT:
        case SETTING_FILE:
            r->widget = gtk_entry_new();
            gtk_entry_set_max_length(GTK_ENTRY(r->widget), FRONTEND_SETTINGS_PATH_TEXT - 1);
            (void)desktop_setting_edit_text(ui, i, value, sizeof(value));
            gtk_editable_set_text(GTK_EDITABLE(r->widget), value);
            g_signal_connect(r->widget, "changed", G_CALLBACK(changed), r);
            g_signal_connect(r->widget, "activate", G_CALLBACK(entered), r);
            break;
        case SETTING_BINDING:
            r->widget = gtk_button_new_with_label(value);
            g_signal_connect(r->widget, "clicked", G_CALLBACK(activate), r);
            break;
        default:
            r->widget = cupid_gtk_label(value);
            break;
        }
        gtk_widget_set_hexpand(r->widget, TRUE);
        gtk_box_append(GTK_BOX(row), r->widget);
        if (r->kind == SETTING_FILE) {
            GtkWidget *browse = gtk_button_new_with_label("Browse...");
            g_signal_connect(browse, "clicked", G_CALLBACK(activate), r);
            gtk_box_append(GTK_BOX(row), browse);
        }
        gtk_box_append(GTK_BOX(s->page), row);
    }
    s->updating = false;
}

GtkWidget *cupid_gtk_settings_new(CupidGtkTool *tool) {
    Settings *s = g_new0(Settings, 1);
    s->tool = tool;
    s->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    GtkWidget *paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL), *categories = gtk_list_box_new();
    static const char *names[] = {"General",
                                  "Emulation",
                                  "Video",
                                  "Audio",
                                  "Controllers and shortcuts",
                                  "Media and firmware",
                                  "Capture and states",
                                  "Hardware"};
    for (unsigned i = 0; i < G_N_ELEMENTS(names); i++) {
        GtkWidget *label = cupid_gtk_label(names[i]);
        gtk_label_set_selectable(GTK_LABEL(label), FALSE);
        cupid_gtk_margins(label, 10);
        gtk_list_box_append(GTK_LIST_BOX(categories), label);
    }
    s->categories = categories;
    gtk_widget_set_size_request(categories, 205, -1);
    gtk_paned_set_start_child(GTK_PANED(paned), cupid_gtk_scroll(categories));
    s->page = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);
    cupid_gtk_margins(s->page, 18);
    gtk_paned_set_end_child(GTK_PANED(paned), cupid_gtk_scroll(s->page));
    gtk_paned_set_position(GTK_PANED(paned), 220);
    gtk_box_append(GTK_BOX(s->root), paned);
    GtkWidget *buttons = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    cupid_gtk_margins(buttons, 10);
    gtk_box_append(GTK_BOX(s->root), buttons);
    static const char *actions[] = {"Apply", "OK", "Cancel", "Reset page", "Reset all"};
    for (int i = 0; i < 5; i++) {
        GtkWidget *b = gtk_button_new_with_label(actions[i]);
        g_object_set_data(G_OBJECT(b), "action", GINT_TO_POINTER(i));
        g_signal_connect(b, "clicked", G_CALLBACK(action), s);
        gtk_box_append(GTK_BOX(buttons), b);
    }
    build(s);
    gtk_list_box_select_row(GTK_LIST_BOX(categories),
                            gtk_list_box_get_row_at_index(GTK_LIST_BOX(categories), tool->ui.settings_category));
    g_signal_connect(categories, "row-selected", G_CALLBACK(category), s);
    GtkEventController *keys = gtk_event_controller_key_new();
    gtk_event_controller_set_propagation_phase(keys, GTK_PHASE_CAPTURE);
    g_signal_connect(keys, "key-pressed", G_CALLBACK(binding), s);
    gtk_widget_add_controller(s->root, keys);
    g_object_set_data_full(G_OBJECT(s->root), "settings", s, g_free);
    return s->root;
}

void cupid_gtk_settings_refresh(GtkWidget *root) {
    Settings *s = g_object_get_data(G_OBJECT(root), "settings");
    if (!s || s->updating) {
        return;
    }
    FrontendDesktopUi *ui = &s->tool->ui;
    s->updating = true;
    for (int i = 0; i < s->count; i++) {
        Setting *r = &s->rows[i];
        char label[256], value[4096];
        desktop_setting_text(ui, i, label, sizeof(label), value, sizeof(value));
        gtk_label_set_text(GTK_LABEL(r->label), label);
        if (r->dirty) {
            continue;
        }
        switch (r->kind) {
        case SETTING_TOGGLE:
            gtk_check_button_set_active(GTK_CHECK_BUTTON(r->widget),
                                        !strcmp(value, "On") || !strcmp(value, "Yes") || !strcmp(value, "Enabled"));
            break;
        case SETTING_CHOICE: {
            int chosen = 0, n = desktop_setting_choices(ui, i, &chosen);
            GtkStringList *list = GTK_STRING_LIST(gtk_drop_down_get_model(GTK_DROP_DOWN(r->widget)));
            unsigned old_count = g_list_model_get_n_items(G_LIST_MODEL(list));
            bool same = n >= 0 && old_count == (unsigned)n;
            for (int j = 0; same && j < n; ++j) {
                char option[512];
                desktop_setting_choice_text(ui, i, j, option, sizeof(option));
                same = !strcmp(option, gtk_string_list_get_string(list, (guint)j));
            }
            if (!same) {
                char **options = g_new0(char *, (size_t)MAX(n, 0) + 1);
                for (int j = 0; j < n; ++j) {
                    char option[512];
                    desktop_setting_choice_text(ui, i, j, option, sizeof(option));
                    options[j] = g_strdup(option);
                }
                gtk_string_list_splice(list, 0, old_count, (const char *const *)options);
                g_strfreev(options);
            }
            gtk_drop_down_set_selected(GTK_DROP_DOWN(r->widget),
                                       chosen >= 0 && chosen < n ? (guint)chosen : GTK_INVALID_LIST_POSITION);
            break;
        }
        case SETTING_NUMBER:
        case SETTING_TEXT:
        case SETTING_FILE:
            (void)desktop_setting_edit_text(ui, i, value, sizeof(value));
            if (strcmp(gtk_editable_get_text(GTK_EDITABLE(r->widget)), value)) {
                gtk_editable_set_text(GTK_EDITABLE(r->widget), value);
            }
            break;
        case SETTING_BINDING:
            gtk_button_set_label(GTK_BUTTON(r->widget), ui->capture_binding && ui->settings_row == i
                                                            ? "Press a key or controller button..."
                                                            : value);
            break;
        default:
            gtk_label_set_text(GTK_LABEL(r->widget), value);
            break;
        }
    }
    s->updating = false;
}

void cupid_gtk_settings_reset(GtkWidget *root) {
    Settings *s = g_object_get_data(G_OBJECT(root), "settings");
    if (!s) {
        return;
    }
    for (int i = 0; i < s->count; ++i) {
        s->rows[i].dirty = false;
    }
    s->updating = true;
    gtk_list_box_select_row(GTK_LIST_BOX(s->categories),
                            gtk_list_box_get_row_at_index(GTK_LIST_BOX(s->categories), s->tool->ui.settings_category));
    s->updating = false;
    /* Reopening can reset the category. This external reset is outside row callbacks. */
    if (s->built_category != s->tool->ui.settings_category) {
        build(s);
    } else {
        cupid_gtk_settings_refresh(root);
    }
}
