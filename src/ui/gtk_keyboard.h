/*
 * gtk_keyboard.h - Native Family BASIC keyboard
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GTK_KEYBOARD_H
#define CUPID_GTK_KEYBOARD_H
#ifdef CUPID_GTK
#include "gtk_internal.h"
GtkWidget *cupid_gtk_keyboard_new(CupidGtkTool *tool);
void cupid_gtk_keyboard_refresh(GtkWidget *widget);
#endif
#endif
