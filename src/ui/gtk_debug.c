/*
 * gtk_debug.c - Live disassembly and breakpoint navigation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "frontend_commands.h"
#include "../debugger/debugger.h"
#include "../debugger/expression.h"
#include <string.h>

typedef struct {
    CupidGtkTool *tool;
    GtkWidget *root, *view, *address, *follow;
    uint16_t first, addresses[96];
    char *last;
} Code;

static void jump(GtkEntry *entry, gpointer data) {
    Code *c = data;
    uint32_t address;
    char error[256] = {0};
    if (!debug_expression_eval(gtk_editable_get_text(GTK_EDITABLE(entry)), &address, error, sizeof(error)) ||
        address > 65535) {
        cupid_gtk_tool_status(c->tool, error[0] ? error : "Address must fit in 16 bits");
        return;
    }
    c->first = (uint16_t)address;
    gtk_check_button_set_active(GTK_CHECK_BUTTON(c->follow), FALSE);
    cupid_gtk_debug_refresh(c->root);
}

static void control(GtkButton *button, gpointer data) {
    Code *c = data;
    desktop_invoke_command(&c->tool->ui, GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "command")));
    cupid_gtk_debug_refresh(c->root);
}

static void clicked(GtkGestureClick *gesture, int count, double x, double y, gpointer data) {
    (void)gesture;
    if (count != 2) {
        return;
    }
    Code *c = data;
    GtkTextIter iter;
    int bx, by;
    gtk_text_view_window_to_buffer_coords(GTK_TEXT_VIEW(c->view), GTK_TEXT_WINDOW_WIDGET, (int)x, (int)y, &bx, &by);
    gtk_text_view_get_iter_at_location(GTK_TEXT_VIEW(c->view), &iter, bx, by);
    int row = gtk_text_iter_get_line(&iter);
    if (row < 0 || row >= 96) {
        return;
    }
    uint16_t address = c->addresses[row];
    for (size_t i = 0; i < debugger_breakpoint_count(); i++) {
        DebugBreakpoint bp;
        if (debugger_breakpoint_at(i, &bp) && bp.type == DEBUG_BREAK_EXECUTE && bp.first == address &&
            bp.last == address) {
            debugger_remove_breakpoint(bp.id);
            cupid_gtk_debug_refresh(c->root);
            return;
        }
    }
    if (!debugger_add_breakpoint(DEBUG_BREAK_EXECUTE, address, address)) {
        cupid_gtk_tool_status(c->tool, "Cannot add another breakpoint");
    }
    cupid_gtk_debug_refresh(c->root);
}

static void cleanup(gpointer data) {
    Code *c = data;
    g_free(c->last);
    g_free(c);
}

GtkWidget *cupid_gtk_debug_new(CupidGtkTool *tool) {
    Code *c = g_new0(Code, 1);
    c->tool = tool;
    c->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_vexpand(c->root, TRUE);
    GtkWidget *bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    const char *labels[] = {"Pause", "Resume", "Step into", "Step over", "Step out"};
    unsigned commands[] = {DEBUGGER_PAUSE_COMMAND, DEBUGGER_RESUME_COMMAND, DEBUGGER_STEP_INTO_COMMAND,
                           DEBUGGER_STEP_OVER_COMMAND, DEBUGGER_STEP_OUT_COMMAND};
    for (unsigned i = 0; i < G_N_ELEMENTS(commands); i++) {
        GtkWidget *b = gtk_button_new_with_label(labels[i]);
        g_object_set_data(G_OBJECT(b), "command", GUINT_TO_POINTER(commands[i]));
        g_signal_connect(b, "clicked", G_CALLBACK(control), c);
        gtk_box_append(GTK_BOX(bar), b);
    }
    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll), GTK_POLICY_AUTOMATIC, GTK_POLICY_NEVER);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), bar);
    gtk_box_append(GTK_BOX(c->root), scroll);
    GtkWidget *nav = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_append(GTK_BOX(c->root), nav);
    c->address = gtk_entry_new();
    gtk_entry_set_placeholder_text(GTK_ENTRY(c->address), "Go to address or expression; press Enter");
    gtk_widget_set_hexpand(c->address, TRUE);
    gtk_box_append(GTK_BOX(nav), c->address);
    g_signal_connect(c->address, "activate", G_CALLBACK(jump), c);
    c->follow = gtk_check_button_new_with_label("Follow PC");
    gtk_check_button_set_active(GTK_CHECK_BUTTON(c->follow), TRUE);
    gtk_box_append(GTK_BOX(c->root), c->follow);
    c->view = gtk_text_view_new();
    gtk_text_view_set_monospace(GTK_TEXT_VIEW(c->view), TRUE);
    gtk_text_view_set_editable(GTK_TEXT_VIEW(c->view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(c->view), FALSE);
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(c->view), 8);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(c->view), 8);
    gtk_text_buffer_create_tag(gtk_text_view_get_buffer(GTK_TEXT_VIEW(c->view)), "pc", "background", "#315d99",
                               "foreground", "white", NULL);
    gtk_box_append(GTK_BOX(c->root), cupid_gtk_scroll(c->view));
    GtkWidget *hint = cupid_gtk_label("Double-click an instruction to toggle an execution breakpoint.");
    gtk_label_set_wrap(GTK_LABEL(hint), TRUE);
    gtk_box_append(GTK_BOX(c->root), hint);
    GtkGesture *click = gtk_gesture_click_new();
    g_signal_connect(click, "pressed", G_CALLBACK(clicked), c);
    gtk_widget_add_controller(c->view, GTK_EVENT_CONTROLLER(click));
    g_object_set_data_full(G_OBJECT(c->root), "code", c, cleanup);
    return c->root;
}

void cupid_gtk_debug_refresh(GtkWidget *root) {
    Code *c = g_object_get_data(G_OBJECT(root), "code");
    if (!c || !frontend_panel_session_active()) {
        return;
    }
    DebugCpuSnapshot cpu;
    debugger_get_cpu(&cpu);
    if (gtk_check_button_get_active(GTK_CHECK_BUTTON(c->follow))) {
        c->first = cpu.cpu.pc;
    }
    GString *text = g_string_new("");
    uint16_t address = c->first;
    int current = -1;
    for (unsigned i = 0; i < 96; i++) {
        DebugDisassembly line;
        debugger_disassemble(address, &line);
        c->addresses[i] = address;
        bool breakpoint = false;
        for (size_t j = 0; j < debugger_breakpoint_count(); j++) {
            DebugBreakpoint bp;
            if (debugger_breakpoint_at(j, &bp) && bp.enabled && (bp.type & DEBUG_BREAK_EXECUTE) &&
                address >= bp.first && address <= bp.last) {
                breakpoint = true;
                break;
            }
        }
        g_string_append_printf(text, "%s %s\n", breakpoint ? "B" : address == cpu.cpu.pc ? ">" : " ", line.text);
        if (address == cpu.cpu.pc) {
            current = (int)i;
        }
        address = (uint16_t)(address + (line.length ? line.length : 1));
    }
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(c->view));
    if (!c->last || strcmp(c->last, text->str)) {
        gtk_text_buffer_set_text(buffer, text->str, -1);
        g_free(c->last);
        c->last = g_strdup(text->str);
    }
    g_string_free(text, TRUE);
    GtkTextIter a, b;
    gtk_text_buffer_get_bounds(buffer, &a, &b);
    gtk_text_buffer_remove_all_tags(buffer, &a, &b);
    if (current >= 0) {
        gtk_text_buffer_get_iter_at_line(buffer, &a, current);
        b = a;
        gtk_text_iter_forward_to_line_end(&b);
        gtk_text_buffer_apply_tag_by_name(buffer, "pc", &a, &b);
    }
}
