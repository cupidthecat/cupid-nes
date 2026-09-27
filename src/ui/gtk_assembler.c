/*
 * gtk_assembler.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_assembler.h"
#ifdef CUPID_GTK
#include "assembler_frontend.h"
#include "frontend_panels.h"
#include "../debugger/opcodes.h"
#include <string.h>

typedef struct {
    unsigned panel_id;
    GtkWidget *root, *address, *target, *apply, *preview, *status, *backing;
    GtkWidget *undo, *redo;
    GtkTextBuffer *source, *output;
    gboolean updating;
    char source_error[256], address_error[256];
} GtkAssembler;

static void set_text(GtkTextBuffer *buffer, const char *text) {
    GtkTextIter start, end;
    gtk_text_buffer_get_bounds(buffer, &start, &end);
    char *old = gtk_text_buffer_get_text(buffer, &start, &end, FALSE);
    if (strcmp(old, text ? text : "")) {
        gtk_text_buffer_set_text(buffer, text ? text : "", -1);
    }
    g_free(old);
}

static void tag(GtkTextBuffer *buffer, const char *name, const char *text, const char *first, const char *last) {
    GtkTextIter a, b;
    gtk_text_buffer_get_iter_at_offset(buffer, &a, (int)g_utf8_pointer_to_offset(text, first));
    gtk_text_buffer_get_iter_at_offset(buffer, &b, (int)g_utf8_pointer_to_offset(text, last));
    gtk_text_buffer_apply_tag_by_name(buffer, name, &a, &b);
}

static void highlight(GtkAssembler *ui) {
    GtkTextIter start, end;
    gtk_text_buffer_get_bounds(ui->source, &start, &end);
    char *text = gtk_text_buffer_get_text(ui->source, &start, &end, FALSE);
    gtk_text_buffer_remove_all_tags(ui->source, &start, &end);
    const char *p = text;
    while (*p) {
        const char *first = p;
        if (*p == ';') {
            while (*p && *p != '\n' && *p != '\r') {
                p = g_utf8_next_char(p);
            }
            tag(ui->source, "comment", text, first, p);
        } else if (g_ascii_isalpha(*p) || *p == '_') {
            while (g_ascii_isalnum(*p) || *p == '_') {
                ++p;
            }
            const char *next = p;
            while (*next == ' ' || *next == '\t') {
                ++next;
            }
            gboolean opcode = FALSE;
            if (p - first == 3) {
                char name[4] = {g_ascii_toupper(first[0]), g_ascii_toupper(first[1]), g_ascii_toupper(first[2]), 0};
                for (unsigned i = 0; i < 256; ++i) {
                    if (!strcmp(debugger_opcode((uint8_t)i).name, name)) {
                        opcode = TRUE;
                        break;
                    }
                }
            }
            tag(ui->source, *next == ':' || !opcode ? "label" : "opcode", text, first, p);
        } else if (g_ascii_isdigit(*p) || *p == '$' || *p == '%') {
            ++p;
            while (g_ascii_isalnum(*p)) {
                ++p;
            }
            tag(ui->source, "number", text, first, p);
        } else {
            p = g_utf8_next_char(p);
        }
    }
    g_free(text);
}

void cupid_gtk_assembler_refresh(GtkWidget *root) {
    GtkAssembler *ui = g_object_get_data(G_OBJECT(root), "cupid-assembler");
    if (!ui) {
        return;
    }
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = G_N_ELEMENTS(controls)};
    char error[256] = "";
    ui->updating = TRUE;
    gboolean available = frontend_panel_snapshot(ui->panel_id, &model, error, sizeof(error));
    gtk_widget_set_sensitive(ui->apply, FALSE);
    gtk_widget_set_sensitive(ui->preview, available && !ui->source_error[0] && !ui->address_error[0]);
    if (available) {
        for (size_t i = 0; i < model.count; ++i) {
            FrontendPanelControl *c = &controls[i];
            switch (c->id) {
            case ASSEMBLER_SOURCE:
                if (!ui->source_error[0]) {
                    GtkTextIter start, end;
                    gtk_text_buffer_get_bounds(ui->source, &start, &end);
                    char *local = gtk_text_buffer_get_text(ui->source, &start, &end, FALSE);
                    if (strcmp(local, c->value ? c->value : "")) {
                        gtk_text_buffer_set_enable_undo(ui->source, FALSE);
                        set_text(ui->source, c->value);
                        gtk_text_buffer_set_enable_undo(ui->source, TRUE);
                        highlight(ui);
                    }
                    g_free(local);
                }
                break;
            case ASSEMBLER_ADDRESS:
                if (!ui->address_error[0] &&
                    strcmp(gtk_editable_get_text(GTK_EDITABLE(ui->address)), c->value ? c->value : "")) {
                    gtk_editable_set_text(GTK_EDITABLE(ui->address), c->value ? c->value : "");
                }
                break;
            case ASSEMBLER_SPACE:
                gtk_drop_down_set_selected(GTK_DROP_DOWN(ui->target), (guint)c->selected);
                break;
            case ASSEMBLER_BYTES:
                set_text(ui->output, c->value);
                break;
            case ASSEMBLER_BACKING:
                gtk_label_set_text(GTK_LABEL(ui->backing), c->value ? c->value : "");
                break;
            case ASSEMBLER_APPLY:
                gtk_widget_set_sensitive(ui->apply, c->enabled && !ui->source_error[0] && !ui->address_error[0]);
                break;
            default:
                break;
            }
        }
    } else {
        set_text(ui->output, "Preview unavailable. Open a game to assemble a program.");
        gtk_label_set_text(GTK_LABEL(ui->backing), "");
    }
    const char *status = ui->source_error[0]    ? ui->source_error
                         : ui->address_error[0] ? ui->address_error
                         : available            ? model.status
                                                : error;
    gtk_label_set_text(GTK_LABEL(ui->status),
                       status && *status ? status : "Enter source, then preview the generated bytes.");
    gtk_widget_set_sensitive(ui->undo, gtk_text_buffer_get_can_undo(ui->source));
    gtk_widget_set_sensitive(ui->redo, gtk_text_buffer_get_can_redo(ui->source));
    ui->updating = FALSE;
}

static void undo_state_changed(GObject *object, GParamSpec *spec, gpointer data) {
    GtkAssembler *ui = data;
    (void)object;
    (void)spec;
    gtk_widget_set_sensitive(ui->undo, gtk_text_buffer_get_can_undo(ui->source));
    gtk_widget_set_sensitive(ui->redo, gtk_text_buffer_get_can_redo(ui->source));
}

static void source_changed(GtkTextBuffer *buffer, gpointer data) {
    GtkAssembler *ui = data;
    if (ui->updating) {
        return;
    }
    GtkTextIter start, end;
    gtk_text_buffer_get_bounds(buffer, &start, &end);
    char *source = gtk_text_buffer_get_text(buffer, &start, &end, FALSE);
    ui->source_error[0] = 0;
    frontend_panel_action(ui->panel_id, ASSEMBLER_SOURCE, source, 0, ui->source_error, sizeof(ui->source_error));
    g_free(source);
    highlight(ui);
    cupid_gtk_assembler_refresh(ui->root);
}

static void address_changed(GtkEditable *editable, gpointer data) {
    GtkAssembler *ui = data;
    if (ui->updating) {
        return;
    }
    ui->address_error[0] = 0;
    frontend_panel_action(ui->panel_id, ASSEMBLER_ADDRESS, gtk_editable_get_text(editable), 0, ui->address_error,
                          sizeof(ui->address_error));
    cupid_gtk_assembler_refresh(ui->root);
}

static void target_changed(GObject *object, GParamSpec *spec, gpointer data) {
    GtkAssembler *ui = data;
    (void)spec;
    if (ui->updating) {
        return;
    }
    char error[256] = "";
    gboolean ok = frontend_panel_action(ui->panel_id, ASSEMBLER_SPACE, NULL,
                                        (int)gtk_drop_down_get_selected(GTK_DROP_DOWN(object)), error, sizeof(error));
    cupid_gtk_assembler_refresh(ui->root);
    if (!ok) {
        gtk_label_set_text(GTK_LABEL(ui->status), error);
    }
}

static void clicked(GtkButton *button, gpointer data) {
    GtkAssembler *ui = data;
    unsigned action = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "assembler-action"));
    char error[256] = "";
    gboolean ok = frontend_panel_action(ui->panel_id, action, NULL, 0, error, sizeof(error));
    if (ok && action == ASSEMBLER_PC) {
        ui->address_error[0] = 0;
    }
    cupid_gtk_assembler_refresh(ui->root);
    if (!ok) {
        gtk_label_set_text(GTK_LABEL(ui->status), error);
    }
}

static void undo_clicked(GtkButton *button, gpointer data) {
    GtkAssembler *ui = data;
    (void)button;
    if (gtk_text_buffer_get_can_undo(ui->source)) {
        gtk_text_buffer_undo(ui->source);
    }
    cupid_gtk_assembler_refresh(ui->root);
}

static void redo_clicked(GtkButton *button, gpointer data) {
    GtkAssembler *ui = data;
    (void)button;
    if (gtk_text_buffer_get_can_redo(ui->source)) {
        gtk_text_buffer_redo(ui->source);
    }
    cupid_gtk_assembler_refresh(ui->root);
}

static GtkWidget *action_button(GtkAssembler *ui, GtkWidget *bar, const char *label, unsigned action) {
    GtkWidget *button = gtk_button_new_with_label(label);
    g_object_set_data(G_OBJECT(button), "assembler-action", GUINT_TO_POINTER(action));
    g_signal_connect(button, "clicked", G_CALLBACK(clicked), ui);
    gtk_box_append(GTK_BOX(bar), button);
    return button;
}

static GtkWidget *editor_pane(const char *title, GtkTextBuffer **buffer, gboolean editable) {
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    GtkWidget *label = gtk_label_new(title);
    gtk_label_set_xalign(GTK_LABEL(label), 0);
    gtk_box_append(GTK_BOX(box), label);
    GtkWidget *view = gtk_text_view_new();
    *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
    gtk_text_view_set_monospace(GTK_TEXT_VIEW(view), TRUE);
    gtk_text_view_set_editable(GTK_TEXT_VIEW(view), editable);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(view), editable);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(view), GTK_WRAP_NONE);
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(view), 10);
    gtk_text_view_set_right_margin(GTK_TEXT_VIEW(view), 10);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(view), 8);
    gtk_widget_set_tooltip_text(view, title);
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), view);
    gtk_scrolled_window_set_has_frame(GTK_SCROLLED_WINDOW(scroll), TRUE);
    gtk_widget_set_hexpand(scroll, TRUE);
    gtk_widget_set_vexpand(scroll, TRUE);
    gtk_box_append(GTK_BOX(box), scroll);
    return box;
}

GtkWidget *cupid_gtk_assembler_new(unsigned panel_id) {
    GtkAssembler *ui = g_new0(GtkAssembler, 1);
    ui->panel_id = panel_id;
    ui->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    g_object_set_data_full(G_OBJECT(ui->root), "cupid-assembler", ui, g_free);
    gtk_widget_set_margin_start(ui->root, 12);
    gtk_widget_set_margin_end(ui->root, 12);
    gtk_widget_set_margin_top(ui->root, 12);
    gtk_widget_set_margin_bottom(ui->root, 12);
    GtkWidget *bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_append(GTK_BOX(ui->root), bar);
    gtk_box_append(GTK_BOX(bar), gtk_label_new("Target"));
    const char *targets[] = {"CPU address space", "Internal CPU RAM", "Cartridge RAM", NULL};
    ui->target = gtk_drop_down_new_from_strings(targets);
    gtk_box_append(GTK_BOX(bar), ui->target);
    gtk_box_append(GTK_BOX(bar), gtk_label_new("Address"));
    ui->address = gtk_entry_new();
    gtk_editable_set_width_chars(GTK_EDITABLE(ui->address), 12);
    gtk_widget_set_tooltip_text(ui->address, "Start address or debugger expression, for example $0600");
    gtk_box_append(GTK_BOX(bar), ui->address);
    action_button(ui, bar, "Use PC", ASSEMBLER_PC);
    GtkWidget *edit_bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_append(GTK_BOX(ui->root), edit_bar);
    ui->undo = gtk_button_new_with_label("Undo");
    ui->redo = gtk_button_new_with_label("Redo");
    gtk_box_append(GTK_BOX(edit_bar), ui->undo);
    gtk_box_append(GTK_BOX(edit_bar), ui->redo);
    GtkWidget *hint = gtk_label_new("6502 source • labels • $hex / %binary / decimal • ; comments");
    gtk_label_set_wrap(GTK_LABEL(hint), TRUE);
    gtk_box_append(GTK_BOX(edit_bar), hint);
    GtkWidget *split = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_paned_set_start_child(GTK_PANED(split), editor_pane("Source", &ui->source, TRUE));
    gtk_paned_set_end_child(GTK_PANED(split), editor_pane("Generated bytes / disassembly", &ui->output, FALSE));
    gtk_paned_set_position(GTK_PANED(split), 600);
    gtk_paned_set_shrink_start_child(GTK_PANED(split), FALSE);
    gtk_paned_set_shrink_end_child(GTK_PANED(split), FALSE);
    gtk_widget_set_vexpand(split, TRUE);
    gtk_widget_set_size_request(split, -1, 280);
    gtk_box_append(GTK_BOX(ui->root), split);
    GtkWidget *actions = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_set_halign(actions, GTK_ALIGN_END);
    gtk_box_append(GTK_BOX(ui->root), actions);
    ui->preview = action_button(ui, actions, "Assemble / preview", ASSEMBLER_PREVIEW);
    ui->apply = action_button(ui, actions, "Apply to memory", ASSEMBLER_APPLY);
    gtk_widget_add_css_class(ui->apply, "suggested-action");
    ui->backing = gtk_label_new("");
    ui->status = gtk_label_new("");
    GtkWidget *labels[] = {ui->backing, ui->status};
    for (unsigned i = 0; i < G_N_ELEMENTS(labels); ++i) {
        gtk_label_set_xalign(GTK_LABEL(labels[i]), 0);
        gtk_label_set_wrap(GTK_LABEL(labels[i]), TRUE);
        gtk_label_set_selectable(GTK_LABEL(labels[i]), TRUE);
        gtk_box_append(GTK_BOX(ui->root), labels[i]);
    }
    gtk_text_buffer_create_tag(ui->source, "opcode", "foreground", "#3584e4", "weight", PANGO_WEIGHT_BOLD, NULL);
    gtk_text_buffer_create_tag(ui->source, "number", "foreground", "#c64600", NULL);
    gtk_text_buffer_create_tag(ui->source, "comment", "foreground", "#77767b", "style", PANGO_STYLE_ITALIC, NULL);
    gtk_text_buffer_create_tag(ui->source, "label", "foreground", "#9141ac", NULL);
    gtk_text_buffer_set_enable_undo(ui->source, TRUE);
    gtk_text_buffer_set_max_undo_levels(ui->source, 200);
    g_signal_connect(ui->source, "notify::can-undo", G_CALLBACK(undo_state_changed), ui);
    g_signal_connect(ui->source, "notify::can-redo", G_CALLBACK(undo_state_changed), ui);
    g_signal_connect(ui->source, "changed", G_CALLBACK(source_changed), ui);
    g_signal_connect(ui->address, "changed", G_CALLBACK(address_changed), ui);
    g_signal_connect(ui->target, "notify::selected", G_CALLBACK(target_changed), ui);
    g_signal_connect(ui->undo, "clicked", G_CALLBACK(undo_clicked), ui);
    g_signal_connect(ui->redo, "clicked", G_CALLBACK(redo_clicked), ui);
    cupid_gtk_assembler_refresh(ui->root);
    return ui->root;
}
#endif
