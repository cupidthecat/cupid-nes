/*
 * gtk_desktop.h - Native desktop host
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GTK_DESKTOP_H
#define CUPID_GTK_DESKTOP_H
#include "desktop_ui.h"
#ifdef CUPID_GTK
bool cupid_gtk_init(FrontendDesktopUi *ui);
void cupid_gtk_render(FrontendDesktopUi *ui, const char *title, const char *region, const char *state);
void cupid_gtk_shutdown(FrontendDesktopUi *ui);
FrontendDesktopUi *cupid_gtk_open(FrontendDesktopUi *ui, int kind, unsigned id);
bool cupid_gtk_event(FrontendDesktopUi *ui, const SDL_Event *event);
bool cupid_gtk_captured(const FrontendDesktopUi *ui);
void cupid_gtk_activity(FrontendDesktopUi *ui);
void cupid_gtk_save_size(FrontendDesktopUi *ui);
void cupid_gtk_dispatch(void);
bool cupid_gtk_draw_stats(FrontendDesktopUi *ui, NesFrameTimingSummary *summary, bool reset);
uint32_t cupid_gtk_pointer(FrontendDesktopUi *ui, const SDL_Event *event, int *x, int *y);
#endif
#endif
