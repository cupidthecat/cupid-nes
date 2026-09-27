/*
 * gtk_hex.c - Scrollable memory grid with stable columns and byte selection
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "hex_frontend.h"
#include "../debugger/memory_editor.h"
#include "../debugger/debugger.h"
#include <string.h>

enum { VIEW_ROWS = 128, ROW_BYTES = 16, HEX_START = 9, ASCII_START = 59 };

typedef struct {
    CupidGtkTool *tool;
    GtkWidget *root, *view, *status;
    GtkAdjustment *scroll;
    GtkTextBuffer *buffer;
    DebugMemoryByte bytes[VIEW_ROWS * ROW_BYTES];
    bool chosen[VIEW_ROWS * ROW_BYTES];
    DebugMemorySpace space;
    uint32_t first, selected, low, high;
    uint64_t session;
    unsigned rows, nibble;
    double drag_x, drag_y;
    bool live, updating, half, dragging;
    char *last;
} Hex;

static bool action(Hex *h, unsigned id, const char *text, int selected) {
    char error[256] = {0};
    bool ok = frontend_panel_action(HEX_FRONTEND_PANEL, id, text, selected, error, sizeof(error));
    if (!ok) {
        cupid_gtk_tool_status(h->tool, error);
    }
    return ok;
}

static void select_byte(Hex *h, uint32_t address, bool extend) {
    h->half = false;
    if (address < h->low || address > h->high) {
        return;
    }
    if (action(h, extend ? HEX_EXTEND : HEX_PICK, NULL, (int)address)) {
        h->selected = address;
    }
    if (address < h->first || address >= h->first + h->rows * 16) {
        h->first = address & ~15u;
        h->updating = true;
        gtk_adjustment_set_value(h->scroll, (double)((address - h->low) / 16));
        h->updating = false;
    }
}

static void scroll_changed(GtkAdjustment *a, gpointer data) {
    Hex *h = data;
    if (h->updating) {
        return;
    }
    h->first = h->low + (uint32_t)gtk_adjustment_get_value(a) * 16;
    h->half = false;
}

static gboolean wheel(GtkEventControllerScroll *c, double dx, double dy, gpointer data) {
    (void)c;
    (void)dx;
    Hex *h = data;
    gtk_adjustment_set_value(h->scroll, gtk_adjustment_get_value(h->scroll) + dy * 3);
    return TRUE;
}

static bool byte_at(Hex *h, double x, double y, uint32_t *address) {
    GtkTextIter iter;
    int bx, by;
    gtk_text_view_window_to_buffer_coords(GTK_TEXT_VIEW(h->view), GTK_TEXT_WINDOW_WIDGET, (int)x, (int)y, &bx, &by);
    gtk_text_view_get_iter_at_location(GTK_TEXT_VIEW(h->view), &iter, bx, by);
    int row = gtk_text_iter_get_line(&iter) - 1, col = gtk_text_iter_get_line_offset(&iter);
    if (row < 0 || row >= (int)h->rows) {
        return false;
    }
    int byte = col >= ASCII_START ? col - ASCII_START : (col - HEX_START) / 3;
    if (col < HEX_START || byte < 0 || byte >= 16) {
        return false;
    }
    *address = h->first + (unsigned)row * 16 + (unsigned)byte;
    return *address <= h->high;
}

static void pointer_select(Hex *h, double x, double y, bool extend) {
    uint32_t address;
    h->half = false;
    if (byte_at(h, x, y, &address)) {
        select_byte(h, address, extend);
        gtk_widget_grab_focus(h->view);
    }
}

static void clicked(GtkGestureClick *click, int count, double x, double y, gpointer data) {
    (void)count;
    GdkModifierType mods = gtk_event_controller_get_current_event_state(GTK_EVENT_CONTROLLER(click));
    pointer_select(data, x, y, (mods & GDK_SHIFT_MASK) != 0);
}

static void drag_begin(GtkGestureDrag *drag, double x, double y, gpointer data) {
    Hex *h = data;
    uint32_t address;
    h->dragging = byte_at(h, x, y, &address);
    h->drag_x = x;
    h->drag_y = y;
    GdkModifierType mods = gtk_event_controller_get_current_event_state(GTK_EVENT_CONTROLLER(drag));
    pointer_select(h, x, y, (mods & GDK_SHIFT_MASK) != 0);
}

static void drag_update(GtkGestureDrag *drag, double dx, double dy, gpointer data) {
    (void)drag;
    Hex *h = data;
    if (h->dragging) {
        pointer_select(h, h->drag_x + dx, h->drag_y + dy, true);
    }
}

static gboolean key(GtkEventControllerKey *controller, guint keyval, guint code, GdkModifierType mods, gpointer data) {
    (void)controller;
    (void)code;
    Hex *h = data;
    int delta = 0;
    switch (keyval) {
    case GDK_KEY_Left:
        delta = -1;
        break;
    case GDK_KEY_Right:
        delta = 1;
        break;
    case GDK_KEY_Up:
        delta = -16;
        break;
    case GDK_KEY_Down:
        delta = 16;
        break;
    case GDK_KEY_Page_Up:
        delta = -(int)(h->rows * 16);
        break;
    case GDK_KEY_Page_Down:
        delta = (int)(h->rows * 16);
        break;
    case GDK_KEY_Home:
        select_byte(h, (mods & GDK_CONTROL_MASK) ? h->low : h->selected & ~15u, mods & GDK_SHIFT_MASK);
        return TRUE;
    case GDK_KEY_End:
        select_byte(h, (mods & GDK_CONTROL_MASK) ? h->high : (h->selected | 15u), mods & GDK_SHIFT_MASK);
        return TRUE;
    default:
        break;
    }
    if (delta) {
        int64_t next = (int64_t)h->selected + delta;
        if (next < h->low) {
            next = h->low;
        }
        if (next > h->high) {
            next = h->high;
        }
        select_byte(h, (uint32_t)next, mods & GDK_SHIFT_MASK);
        h->half = false;
        return TRUE;
    }
    if (mods & GDK_CONTROL_MASK) {
        if (keyval == GDK_KEY_c) {
            action(h, HEX_COPY, NULL, 0);
            return TRUE;
        }
        if (keyval == GDK_KEY_v) {
            action(h, HEX_PASTE, NULL, 0);
            return TRUE;
        }
        return FALSE;
    }
    int digit = g_ascii_xdigit_value((gchar)keyval);
    if (keyval < 128 && digit >= 0) {
        if (!h->half) {
            h->nibble = (unsigned)digit;
            h->half = true;
            gtk_label_set_text(GTK_LABEL(h->status), "Type the second hex digit to write this byte. Escape cancels.");
        } else {
            char value[3];
            g_snprintf(value, sizeof(value), "%02X", h->nibble * 16 + (unsigned)digit);
            h->half = false;
            /* Direct typing replaces the cursor byte, not the range anchor. */
            if (action(h, HEX_PICK, NULL, (int)h->selected) && action(h, HEX_TEXT_MODE, NULL, 0) &&
                action(h, HEX_BYTES, value, 0) && action(h, HEX_APPLY, NULL, 0)) {
                select_byte(h, h->selected + 1, false);
            }
        }
        return TRUE;
    }
    if (keyval == GDK_KEY_Escape) {
        h->half = false;
        return TRUE;
    }
    return FALSE;
}

static void cleanup(gpointer data) {
    Hex *h = data;
    g_free(h->last);
    g_free(h);
}

GtkWidget *cupid_gtk_hex_new(CupidGtkTool *tool, GtkWidget *controls) {
    Hex *h = g_new0(Hex, 1);
    h->tool = tool;
    h->live = true;
    h->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_vexpand(row, TRUE);
    gtk_box_append(GTK_BOX(h->root), row);
    h->view = gtk_text_view_new();
    h->buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(h->view));
    gtk_text_view_set_monospace(GTK_TEXT_VIEW(h->view), TRUE);
    gtk_text_view_set_editable(GTK_TEXT_VIEW(h->view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(h->view), FALSE);
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(h->view), 10);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(h->view), 8);
    gtk_text_buffer_create_tag(h->buffer, "selected", "background", "#315d99", "foreground", "#ffffff", NULL);
    gtk_text_buffer_create_tag(h->buffer, "changed", "foreground", "#ce6c18", "weight", PANGO_WEIGHT_BOLD, NULL);
    GtkWidget *scrolled = cupid_gtk_scroll(h->view);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled), GTK_POLICY_AUTOMATIC, GTK_POLICY_EXTERNAL);
    gtk_box_append(GTK_BOX(row), scrolled);
    h->scroll = gtk_adjustment_new(0, 0, 4096, 1, 16, 16);
    GtkWidget *bar = gtk_scrollbar_new(GTK_ORIENTATION_VERTICAL, h->scroll);
    gtk_box_append(GTK_BOX(row), bar);
    g_signal_connect(h->scroll, "value-changed", G_CALLBACK(scroll_changed), h);
    GtkEventController *wheel_ctl = gtk_event_controller_scroll_new(GTK_EVENT_CONTROLLER_SCROLL_VERTICAL);
    g_signal_connect(wheel_ctl, "scroll", G_CALLBACK(wheel), h);
    gtk_widget_add_controller(h->view, wheel_ctl);
    GtkGesture *click = gtk_gesture_click_new();
    g_signal_connect(click, "pressed", G_CALLBACK(clicked), h);
    gtk_widget_add_controller(h->view, GTK_EVENT_CONTROLLER(click));
    GtkGesture *drag = gtk_gesture_drag_new();
    gtk_widget_add_controller(h->view, GTK_EVENT_CONTROLLER(drag));
    gtk_gesture_group(click, drag);
    g_signal_connect(drag, "drag-begin", G_CALLBACK(drag_begin), h);
    g_signal_connect(drag, "drag-update", G_CALLBACK(drag_update), h);
    GtkEventController *keys = gtk_event_controller_key_new();
    g_signal_connect(keys, "key-pressed", G_CALLBACK(key), h);
    gtk_widget_add_controller(h->view, keys);
    h->status = cupid_gtk_label("Select a byte. Arrows move; Shift extends; type two hex digits to edit.");
    gtk_label_set_wrap(GTK_LABEL(h->status), TRUE);
    cupid_gtk_margins(h->status, 6);
    gtk_box_append(GTK_BOX(h->root), h->status);
    GtkWidget *details = gtk_expander_new("Selection and backing details");
    gtk_expander_set_child(GTK_EXPANDER(details), controls);
    gtk_box_append(GTK_BOX(h->root), details);
    g_object_set_data_full(G_OBJECT(h->root), "hex", h, cleanup);
    return h->root;
}

void cupid_gtk_hex_refresh(GtkWidget *root) {
    Hex *h = g_object_get_data(G_OBJECT(root), "hex");
    if (!h) {
        return;
    }
    FrontendPanelControl controls[64];
    FrontendPanelModel model = {.controls = controls, .capacity = 64};
    if (!frontend_panel_snapshot(HEX_FRONTEND_PANEL, &model, NULL, 0)) {
        return;
    }
    DebugMemorySpace space = h->space;
    uint32_t selected = h->selected;
    bool live = h->live;
    for (size_t i = 0; i < model.count; i++) {
        if (controls[i].id == HEX_SPACE) {
            space = (DebugMemorySpace)controls[i].selected;
        }
        if (controls[i].id == HEX_SELECTION) {
            selected = (uint32_t)controls[i].selected;
        }
        if (controls[i].id == HEX_LIVE) {
            live = controls[i].selected != 0;
        }
    }
    uint64_t session = debugger_session_revision();
    bool reset = space != h->space || session != h->session;
    h->space = space;
    h->session = session;
    h->live = live;
    if (!debug_memory_bounds(space, &h->low, &h->high)) {
        return;
    }
    /* The frontend owns frozen values, selection and baseline flags.
     * Never inspect memory separately: that would display different bytes
     * from the ones copied/exported from a frozen page. */
    unsigned rows = (unsigned)(gtk_widget_get_height(h->view) > 50 ? (gtk_widget_get_height(h->view) - 35) / 20 : 16);
    rows = CLAMP(rows, 8u, (unsigned)VIEW_ROWS);
    h->rows = rows;
    if (reset || selected != h->selected) {
        h->half = false;
        if (reset || selected < h->first || selected >= h->first + rows * ROW_BYTES) {
            h->first = selected & ~15u;
        }
    }
    h->selected = selected;
    uint32_t last_row = h->high + 1 > rows * ROW_BYTES ? h->high + 1 - rows * ROW_BYTES : h->low;
    h->first = CLAMP(h->first, h->low, MAX(h->low, last_row));
    h->updating = true;
    gtk_adjustment_configure(h->scroll, (h->first - h->low) / ROW_BYTES, 0, (h->high - h->low + ROW_BYTES) / ROW_BYTES,
                             1, rows, rows);
    h->updating = false;
    memset(h->bytes, 0, sizeof(h->bytes));
    memset(h->chosen, 0, sizeof(h->chosen));
    size_t count = MIN((size_t)rows * ROW_BYTES, (size_t)(h->high - h->first) + 1);
    uint32_t selection_first, selection_last;
    if (!hex_frontend_view(h->first, count, h->bytes, &selection_first, &selection_last)) {
        return;
    }
    for (size_t i = 0; i < count; i++) {
        h->chosen[i] = h->first + i >= selection_first && h->first + i <= selection_last;
    }
    GString *text = g_string_new("Address  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F  ASCII\n");
    for (unsigned row = 0; row < rows; row++) {
        uint32_t address = h->first + row * 16;
        if (address > h->high) {
            break;
        }
        g_string_append_printf(text, "$%04X    ", address);
        for (unsigned col = 0; col < 16; col++) {
            DebugMemoryByte *b = &h->bytes[row * 16 + col];
            if (b->readable) {
                g_string_append_printf(text, "%02X ", b->value);
            } else {
                g_string_append(text, "?? ");
            }
        }
        g_string_append(text, "  ");
        for (unsigned col = 0; col < 16; col++) {
            DebugMemoryByte *b = &h->bytes[row * 16 + col];
            g_string_append_c(text, b->readable && b->value >= 32 && b->value < 127 ? (char)b->value : '.');
        }
        g_string_append_c(text, '\n');
    }
    if (!h->last || strcmp(h->last, text->str)) {
        gtk_text_buffer_set_text(h->buffer, text->str, -1);
        g_free(h->last);
        h->last = g_strdup(text->str);
    }
    g_string_free(text, TRUE);
    GtkTextIter a, b;
    gtk_text_buffer_get_bounds(h->buffer, &a, &b);
    gtk_text_buffer_remove_all_tags(h->buffer, &a, &b);
    for (unsigned i = 0; i < rows * 16; i++) {
        bool chosen = h->chosen[i], changed = h->bytes[i].changed;
        if (!chosen && !changed) {
            continue;
        }
        gtk_text_buffer_get_iter_at_line_offset(h->buffer, &a, (int)(i / 16 + 1), HEX_START + (int)(i % 16) * 3);
        b = a;
        gtk_text_iter_forward_chars(&b, 2);
        gtk_text_buffer_apply_tag_by_name(h->buffer, chosen ? "selected" : "changed", &a, &b);
        if (chosen) {
            gtk_text_buffer_get_iter_at_line_offset(h->buffer, &a, (int)(i / 16 + 1), ASCII_START + (int)(i % 16));
            b = a;
            gtk_text_iter_forward_char(&b);
            gtk_text_buffer_apply_tag_by_name(h->buffer, "selected", &a, &b);
        }
    }
}
