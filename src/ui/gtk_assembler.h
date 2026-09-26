/*
 * gtk_assembler.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GTK_ASSEMBLER_H
#define CUPID_GTK_ASSEMBLER_H
#ifdef CUPID_GTK
#include <gtk/gtk.h>
GtkWidget *cupid_gtk_assembler_new(unsigned panel_id);
void cupid_gtk_assembler_refresh(GtkWidget *root);
#endif
#endif
