/*
 * gtk_layout.h - Feature-specific desktop layouts
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GTK_LAYOUT_H
#define CUPID_GTK_LAYOUT_H
#include "gtk_internal.h"

typedef struct {
    unsigned id;
    int width, height;
    bool vertical;
    const char *fields;
} CupidGtkLayout;

CupidGtkLayout cupid_gtk_layout(unsigned id);
const char *cupid_gtk_control_group(unsigned panel, unsigned control);
const char *cupid_gtk_setting_group(int category, int row);
GtkWidget *cupid_gtk_group(GtkWidget *parent, const char *title);
#endif
