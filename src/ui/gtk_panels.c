/*
 * gtk_panels.c - Resizable feature panels with native controls and data views
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "debug_frontend.h"
#include "game_genie_frontend.h"
#include "../cheats/cheats.h"
#include "watch_frontend.h"
#include "hex_frontend.h"
#include "memory_search_frontend.h"
#include "platform_frontend.h"
#include "../debugger/debugger.h"
#include <string.h>
typedef struct Panel Panel;

typedef struct {
    Panel *panel;
    unsigned id;
    FrontendPanelControlType type;
    GtkWidget *widget, *label;
    GtkNativeDialog *dialog;
    GtkWidget *cells[8];
    unsigned cell_count;
    char *last;
    bool dirty, readonly, explicit_commit;
    int file_type;
} Control;

struct Panel {
    CupidGtkTool *tool;
    GtkWidget *root, *data, *fields, *debug;
    Control controls[256];
    size_t count;
    bool updating, positioned;
};

static bool invoke(Control *c, const char *text, int selected) {
    char error[256] = {0};
    bool ok = frontend_panel_action(c->panel->tool->id, c->id, text, selected, error, sizeof(error));
    if (!ok) {
        cupid_gtk_tool_status(c->panel->tool, error);
    } else {
        unsigned panel = c->panel->tool->id, destination = 0;
        if ((panel == CHEATS_GAME_GENIE_PANEL && c->id == GAME_GENIE_SEND) ||
            (memory_tools_panel(panel) && (c->id == MEMORY_SEARCH_SEND || c->id == MEMORY_SEARCH_TEST))) {
            destination = CHEATS_FRONTEND_PANEL;
        } else if ((memory_tools_panel(panel) && c->id == MEMORY_SEARCH_WATCH) ||
                   (panel == HEX_FRONTEND_PANEL && c->id == HEX_WATCH)) {
            destination = WATCH_FRONTEND_PANEL;
        }
        if (destination) {
            frontend_desktop_open_panel(&c->panel->tool->ui, destination);
        }
    }
    return ok;
}

static bool commit(Panel *p) {
    bool ok = true;
    for (size_t i = 0; i < p->count; i++) {
        Control *c = &p->controls[i];
        if (c->dirty && !c->readonly && !c->explicit_commit && GTK_IS_ENTRY(c->widget)) {
            if (invoke(c, gtk_editable_get_text(GTK_EDITABLE(c->widget)), -1)) {
                c->dirty = false;
            } else {
                ok = false;
            }
        }
    }
    return ok;
}

static void changed(GtkEditable *entry, gpointer data) {
    Control *c = data;
    if (!c->panel->updating) {
        c->dirty = c->explicit_commit ||
                   !frontend_panel_action(c->panel->tool->id, c->id, gtk_editable_get_text(entry), -1, NULL, 0);
    }
}

static void entered(GtkEntry *entry, gpointer data) {
    Control *c = data;
    if (c->explicit_commit) {
        c->dirty = !invoke(c, gtk_editable_get_text(GTK_EDITABLE(entry)), -1);
    } else {
        commit(c->panel);
    }
}

static void clicked(GtkButton *button, gpointer data) {
    (void)button;
    Control *c = data;
    if (commit(c->panel)) {
        invoke(c, NULL, 0);
    }
}

static void toggled(GtkCheckButton *button, gpointer data) {
    Control *c = data;
    if (!c->panel->updating) {
        invoke(c, NULL, gtk_check_button_get_active(button));
    }
}

static void choice_changed(GObject *object, GParamSpec *spec, gpointer data) {
    (void)spec;
    Control *c = data;
    if (!c->panel->updating) {
        invoke(c, NULL, (int)gtk_drop_down_get_selected(GTK_DROP_DOWN(object)));
    }
}

static void row_selected(GtkListBox *box, GtkListBoxRow *row, gpointer data) {
    (void)box;
    Control *c = data;
    if (row && !c->panel->updating) {
        invoke(c, NULL, gtk_list_box_row_get_index(row));
    }
}

static void file_response(GtkNativeDialog *dialog, int response, gpointer data) {
    Control *c = data;
    c->dialog = NULL;
    if (response == GTK_RESPONSE_ACCEPT) {
        GFile *f = gtk_file_chooser_get_file(GTK_FILE_CHOOSER(dialog));
        char *path = f ? g_file_get_path(f) : NULL;
        if (path) {
            invoke(c, path, c->file_type);
        }
        g_free(path);
        g_clear_object(&f);
    }
    gtk_native_dialog_destroy(dialog);
    g_object_unref(dialog);
}

static void browse(GtkButton *button, gpointer data) {
    (void)button;
    Control *c = data;
    if (c->dialog) {
        gtk_native_dialog_show(c->dialog);
        return;
    }
    GtkFileChooserAction action = c->type == FRONTEND_PANEL_DIRECTORY   ? GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER
                                  : c->type == FRONTEND_PANEL_FILE_SAVE ? GTK_FILE_CHOOSER_ACTION_SAVE
                                                                        : GTK_FILE_CHOOSER_ACTION_OPEN;
    GtkFileChooserNative *dialog =
        gtk_file_chooser_native_new("Choose a file", GTK_WINDOW(c->panel->tool->window), action, "Select", "Cancel");
    c->dialog = GTK_NATIVE_DIALOG(dialog);
    g_signal_connect(dialog, "response", G_CALLBACK(file_response), c);
    gtk_native_dialog_show(GTK_NATIVE_DIALOG(dialog));
}

static void cleanup(gpointer data) {
    Panel *p = data;
    for (size_t i = 0; i < p->count; i++) {
        if (p->controls[i].dialog) {
            g_signal_handlers_disconnect_by_data(p->controls[i].dialog, &p->controls[i]);
            gtk_native_dialog_destroy(p->controls[i].dialog);
            g_clear_object(&p->controls[i].dialog);
        }
        g_free(p->controls[i].last);
    }
    g_free(p);
}

static GtkWidget *frame(const char *title, GtkWidget *child) {
    GtkWidget *w = gtk_frame_new(title);
    gtk_frame_set_child(GTK_FRAME(w), child);
    return w;
}

static void first_map(GtkWidget *widget, gpointer data) {
    Panel *p = data;
    if (!p->positioned && gtk_widget_get_width(widget) > 0) {
        gtk_paned_set_position(GTK_PANED(widget), MAX(240, gtk_widget_get_width(widget) - 320));
        p->positioned = true;
    }
}

static gboolean panel_key(GtkEventControllerKey *controller, guint key, guint code, GdkModifierType mods,
                          gpointer data) {
    (void)controller;
    (void)code;
    Panel *p = data;
    if (p->tool->id != HEX_FRONTEND_PANEL) {
        return FALSE;
    }
    bool find = (mods & GDK_CONTROL_MASK) && gdk_keyval_to_lower(key) == GDK_KEY_f;
    if (!find && key != GDK_KEY_F3) {
        return FALSE;
    }
    for (size_t i = 0; i < p->count; ++i) {
        Control *c = &p->controls[i];
        if (find && c->id == HEX_PATTERN) {
            gtk_widget_grab_focus(c->widget);
            gtk_editable_select_region(GTK_EDITABLE(c->widget), 0, -1);
            return TRUE;
        }
        if (!find && c->id == HEX_FIND) {
            if (commit(p)) {
                invoke(c, NULL, 0);
            }
            return TRUE;
        }
    }
    return FALSE;
}

static GtkWidget *control_new(Panel *p, const FrontendPanelControl *m) {
    Control *c = &p->controls[p->count++];
    *c = (Control){.panel = p, .id = m->id, .type = m->type, .readonly = m->read_only, .file_type = m->selected};
    c->explicit_commit = p->tool->id == DEBUGGER_FRONTEND_PANEL && m->id == DEBUG_CONTROL_REGISTER;
    c->explicit_commit |= p->tool->id == CHEATS_GAME_GENIE_PANEL && m->type == FRONTEND_PANEL_TEXT && !m->read_only;
    GtkWidget *row = gtk_box_new(GTK_ORIENTATION_VERTICAL, 3);
    c->label = cupid_gtk_label(m->label);
    gtk_label_set_wrap(GTK_LABEL(c->label), TRUE);
    switch (m->type) {
    case FRONTEND_PANEL_ACTION:
        c->widget = gtk_button_new_with_label(m->label);
        if (m->items && m->item_count && m->item_count <= 8) {
            GtkWidget *table = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
            c->cell_count = (unsigned)m->item_count;
            for (unsigned col = 0; col < c->cell_count; col++) {
                c->cells[col] = cupid_gtk_label(m->items[col]);
                gtk_label_set_selectable(GTK_LABEL(c->cells[col]), FALSE);
                gtk_widget_add_css_class(c->cells[col], "monospace");
                gtk_widget_set_size_request(c->cells[col], col == 0 ? 100 : 90, -1);
                gtk_widget_set_hexpand(c->cells[col], TRUE);
                gtk_box_append(GTK_BOX(table), c->cells[col]);
            }
            gtk_button_set_child(GTK_BUTTON(c->widget), table);
        }
        g_signal_connect(c->widget, "clicked", G_CALLBACK(clicked), c);
        break;
    case FRONTEND_PANEL_CHECKBOX:
        c->widget = gtk_check_button_new_with_label(m->label);
        g_signal_connect(c->widget, "toggled", G_CALLBACK(toggled), c);
        break;
    case FRONTEND_PANEL_CHOICE:
        c->widget = gtk_drop_down_new(NULL, NULL);
        g_signal_connect(c->widget, "notify::selected", G_CALLBACK(choice_changed), c);
        break;
    case FRONTEND_PANEL_LIST:
        c->widget = gtk_list_box_new();
        gtk_widget_add_css_class(c->widget, "monospace");
        gtk_list_box_set_selection_mode(GTK_LIST_BOX(c->widget), GTK_SELECTION_SINGLE);
        g_signal_connect(c->widget, "row-selected", G_CALLBACK(row_selected), c);
        break;
    case FRONTEND_PANEL_FILE_OPEN:
    case FRONTEND_PANEL_FILE_SAVE:
    case FRONTEND_PANEL_DIRECTORY:
        c->widget = gtk_button_new_with_label(m->value && *m->value ? m->value : "Browse...");
        g_signal_connect(c->widget, "clicked", G_CALLBACK(browse), c);
        break;
    default:
        if (m->read_only && m->value && strchr(m->value, '\n')) {
            c->widget = gtk_text_view_new();
            gtk_text_view_set_editable(GTK_TEXT_VIEW(c->widget), FALSE);
            gtk_text_view_set_monospace(GTK_TEXT_VIEW(c->widget), TRUE);
            gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(c->widget), GTK_WRAP_NONE);
        } else if (m->read_only) {
            c->widget = cupid_gtk_label(m->value);
            gtk_label_set_wrap(GTK_LABEL(c->widget), TRUE);
            gtk_widget_add_css_class(c->widget, "monospace");
        } else {
            c->widget = gtk_entry_new();
            gtk_entry_set_max_length(GTK_ENTRY(c->widget), 4095);
            if (c->explicit_commit) {
                gtk_entry_set_placeholder_text(GTK_ENTRY(c->widget), "Press Enter to apply");
            }
            g_signal_connect(c->widget, "changed", G_CALLBACK(changed), c);
            g_signal_connect(c->widget, "activate", G_CALLBACK(entered), c);
        }
        break;
    }
    bool data_view = m->type == FRONTEND_PANEL_LIST || GTK_IS_TEXT_VIEW(c->widget);
    if (m->type != FRONTEND_PANEL_ACTION && m->type != FRONTEND_PANEL_CHECKBOX) {
        gtk_box_append(GTK_BOX(row), c->label);
    } else {
        g_object_ref_sink(c->label);
        g_object_unref(c->label);
        c->label = NULL;
    }
    if (data_view) {
        GtkWidget *scroll = cupid_gtk_scroll(c->widget);
        gtk_widget_set_size_request(scroll, 240, m->type == FRONTEND_PANEL_LIST ? 160 : 65);
        gtk_box_append(GTK_BOX(row), scroll);
        gtk_widget_set_vexpand(row, TRUE);
    } else {
        gtk_box_append(GTK_BOX(row), c->widget);
    }
    gtk_widget_set_hexpand(row, TRUE);
    return row;
}

GtkWidget *cupid_gtk_panel_new(CupidGtkTool *tool) {
    Panel *p = g_new0(Panel, 1);
    p->tool = tool;
    p->root = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_paned_set_position(GTK_PANED(p->root), 830);
    g_signal_connect(p->root, "map", G_CALLBACK(first_map), p);
    GtkEventController *keys = gtk_event_controller_key_new();
    gtk_event_controller_set_propagation_phase(keys, GTK_PHASE_CAPTURE);
    g_signal_connect(keys, "key-pressed", G_CALLBACK(panel_key), p);
    gtk_widget_add_controller(p->root, keys);
    p->data = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    p->fields = gtk_box_new(GTK_ORIENTATION_VERTICAL, 9);
    cupid_gtk_margins(p->data, 8);
    cupid_gtk_margins(p->fields, 10);
    gtk_widget_set_size_request(p->fields, 280, -1);
    FrontendPanelControl controls[256];
    FrontendPanelModel model = {.controls = controls, .capacity = 256};
    if (frontend_panel_snapshot(tool->id, &model, NULL, 0)) {
        for (size_t i = 0; i < model.count; i++) {
            FrontendPanelControl *c = &controls[i];
            if (tool->id == HEX_FRONTEND_PANEL && c->id >= HEX_ROW) {
                continue;
            }
            GtkWidget *row = control_new(p, c);
            bool data_view = (c->type == FRONTEND_PANEL_ACTION && c->item_count) || c->type == FRONTEND_PANEL_LIST ||
                             (c->type == FRONTEND_PANEL_TEXT && c->read_only);
            if (memory_tools_panel(tool->id) && c->id >= MEMORY_SEARCH_ROW &&
                c->id < MEMORY_SEARCH_ROW + MEMORY_SEARCH_PAGE_SIZE) {
                data_view = true;
            }
            gtk_box_append(GTK_BOX(data_view ? p->data : p->fields), row);
        }
    }
    GtkWidget *fields = cupid_gtk_scroll(p->fields);
    if (tool->id == DEBUGGER_FRONTEND_PANEL) {
        p->debug = cupid_gtk_debug_new(tool);
        gtk_box_append(GTK_BOX(p->data), p->debug);
    }
    if (tool->id == HEX_FRONTEND_PANEL) {
        GtkWidget *hex = cupid_gtk_hex_new(tool, p->data);
        gtk_paned_set_start_child(GTK_PANED(p->root), hex);
    } else {
        gtk_paned_set_start_child(GTK_PANED(p->root), cupid_gtk_scroll(p->data));
    }
    gtk_paned_set_end_child(GTK_PANED(p->root), frame("Controls", fields));
    gtk_paned_set_resize_start_child(GTK_PANED(p->root), TRUE);
    gtk_paned_set_resize_end_child(GTK_PANED(p->root), FALSE);
    gtk_paned_set_shrink_start_child(GTK_PANED(p->root), FALSE);
    gtk_paned_set_shrink_end_child(GTK_PANED(p->root), FALSE);
    g_object_set_data_full(G_OBJECT(p->root), "panel", p, cleanup);
    cupid_gtk_panel_refresh(p->root);
    return p->root;
}

static GString *items_text(const FrontendPanelControl *m) {
    GString *s = g_string_new("");
    for (size_t i = 0; i < m->item_count; i++) {
        g_string_append_printf(s, "%s\n", m->items[i] ? m->items[i] : "");
    }
    return s;
}

void cupid_gtk_panel_refresh(GtkWidget *root) {
    Panel *p = g_object_get_data(G_OBJECT(root), "panel");
    if (!p) {
        return;
    }
    FrontendPanelControl controls[256];
    FrontendPanelModel model = {.controls = controls, .capacity = 256};
    char error[256] = {0};
    if (!frontend_panel_snapshot(p->tool->id, &model, error, sizeof(error))) {
        gtk_label_set_text(GTK_LABEL(p->tool->status), error);
        return;
    }
    p->updating = true;
    for (size_t i = 0; i < p->count; i++) {
        Control *c = &p->controls[i];
        const FrontendPanelControl *m = NULL;
        for (size_t j = 0; j < model.count; j++) {
            if (controls[j].id == c->id) {
                m = &controls[j];
                break;
            }
        }
        if (!m) {
            continue;
        }
        gtk_widget_set_sensitive(c->widget, m->enabled);
        c->readonly = m->read_only;
        if (c->type == FRONTEND_PANEL_CHECKBOX) {
            gtk_check_button_set_active(GTK_CHECK_BUTTON(c->widget), m->selected != 0);
            continue;
        }
        if (c->type == FRONTEND_PANEL_LIST || c->type == FRONTEND_PANEL_CHOICE) {
            GString *text = items_text(m);
            if (!c->last || strcmp(c->last, text->str)) {
                if (c->type == FRONTEND_PANEL_CHOICE) {
                    GtkStringList *list = gtk_string_list_new(NULL);
                    for (size_t j = 0; j < m->item_count; j++) {
                        gtk_string_list_append(list, m->items[j] ? m->items[j] : "");
                    }
                    gtk_drop_down_set_model(GTK_DROP_DOWN(c->widget), G_LIST_MODEL(list));
                    g_object_unref(list);
                } else {
                    GtkWidget *child;
                    while ((child = gtk_widget_get_first_child(c->widget))) {
                        gtk_list_box_remove(GTK_LIST_BOX(c->widget), child);
                    }
                    for (size_t j = 0; j < m->item_count; j++) {
                        GtkWidget *label = cupid_gtk_label(m->items[j]);
                        gtk_label_set_selectable(GTK_LABEL(label), FALSE);
                        cupid_gtk_margins(label, 4);
                        gtk_list_box_append(GTK_LIST_BOX(c->widget), label);
                    }
                }
                g_free(c->last);
                c->last = g_strdup(text->str);
            }
            g_string_free(text, TRUE);
            if (c->type == FRONTEND_PANEL_CHOICE) {
                gtk_drop_down_set_selected(GTK_DROP_DOWN(c->widget),
                                           m->selected < 0 ? GTK_INVALID_LIST_POSITION : (guint)m->selected);
            } else {
                gtk_list_box_select_row(GTK_LIST_BOX(c->widget),
                                        gtk_list_box_get_row_at_index(GTK_LIST_BOX(c->widget), m->selected));
            }
        } else if (GTK_IS_TEXT_VIEW(c->widget)) {
            const char *text = m->value ? m->value : "";
            if (!c->last || strcmp(c->last, text)) {
                gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(c->widget)), text, -1);
                g_free(c->last);
                c->last = g_strdup(text);
            }
        } else if (GTK_IS_LABEL(c->widget)) {
            gtk_label_set_text(GTK_LABEL(c->widget), m->value ? m->value : "");
        } else if (GTK_IS_ENTRY(c->widget)) {
            const char *text = m->value ? m->value : "";
            if (!c->dirty && !gtk_widget_has_focus(c->widget) &&
                strcmp(gtk_editable_get_text(GTK_EDITABLE(c->widget)), text)) {
                gtk_editable_set_text(GTK_EDITABLE(c->widget), text);
            }
        } else if (c->type == FRONTEND_PANEL_ACTION) {
            if (c->cell_count) {
                for (unsigned col = 0; col < c->cell_count; col++) {
                    gtk_label_set_text(GTK_LABEL(c->cells[col]), col < m->item_count ? m->items[col] : "");
                }
                if (m->selected) {
                    gtk_widget_add_css_class(c->widget, "suggested-action");
                } else {
                    gtk_widget_remove_css_class(c->widget, "suggested-action");
                }
            } else {
                gtk_button_set_label(GTK_BUTTON(c->widget), m->label);
            }
        } else if (GTK_IS_BUTTON(c->widget)) {
            gtk_button_set_label(GTK_BUTTON(c->widget), m->value && *m->value ? m->value : "Browse...");
        }
    }
    if (model.status) {
        gtk_label_set_text(GTK_LABEL(p->tool->status), model.status);
    }
    p->updating = false;
    if (p->debug) {
        cupid_gtk_debug_refresh(p->debug);
    }
    if (p->tool->id == HEX_FRONTEND_PANEL) {
        cupid_gtk_hex_refresh(gtk_paned_get_start_child(GTK_PANED(root)));
    }
}
