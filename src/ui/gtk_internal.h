/*
 * gtk_internal.h - Shared native desktop widgets
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_GTK_INTERNAL_H
#define CUPID_GTK_INTERNAL_H
#include <gtk/gtk.h>
#include "desktop_internal.h"
#include "frontend_panels.h"
#include "../video/frame_timing.h"
typedef struct CupidGtkTool CupidGtkTool;

typedef struct CupidGtkDesktop {
    FrontendDesktopUi *ui;
    GtkWidget *window, *picture, *status, *status_info, *menubar, *empty;
    GSimpleActionGroup *actions;
    CupidGtkTool *tools;
    size_t commands, panels, recent;
    bool session, held[SDL_NUM_SCANCODES], fullscreen, joystick_background;
    uint32_t refreshed, theme_checked;
    cairo_surface_t *frame;
    GdkTexture *frame_texture;
    struct CupidGtkNativeVideo *native_video;
    bool native_video_failed;
    unsigned frame_width, frame_height;
    uint64_t submitted_frames, drawn_frame, drawn_frames, last_draw;
    NesFrameTiming draw_timing;
    int mouse_x, mouse_y;
    uint32_t mouse_buttons, consumed_mouse_buttons;
} CupidGtkDesktop;

struct CupidGtkTool {
    CupidGtkDesktop *desktop;
    CupidGtkTool *next;
    FrontendDesktopUi ui;
    GtkWidget *window, *content, *status, *prompt, *prompt_entry, *info_view;
    int kind;
    unsigned id;
};

GtkWidget *cupid_gtk_scroll(GtkWidget *child);
GtkWidget *cupid_gtk_label(const char *text);
void cupid_gtk_margins(GtkWidget *widget, int margin);
void cupid_gtk_picture(GtkWidget *picture, const uint32_t *pixels, unsigned w, unsigned h, unsigned stride);
void cupid_gtk_tool_status(CupidGtkTool *tool, const char *text);
GtkWidget *cupid_gtk_panel_new(CupidGtkTool *tool);
void cupid_gtk_panel_refresh(GtkWidget *widget);
GtkWidget *cupid_gtk_settings_new(CupidGtkTool *tool);
void cupid_gtk_settings_refresh(GtkWidget *widget);
void cupid_gtk_settings_reset(GtkWidget *widget);
GtkWidget *cupid_gtk_tas_new(CupidGtkTool *tool);
void cupid_gtk_tas_refresh(GtkWidget *widget);
/* Update only the live preview; safe to call once per presentation frame. */
void cupid_gtk_tas_video(GtkWidget *widget);
GtkWidget *cupid_gtk_ppu_new(CupidGtkTool *tool);
void cupid_gtk_ppu_refresh(GtkWidget *widget);
GtkWidget *cupid_gtk_hex_new(CupidGtkTool *tool, GtkWidget *controls);
void cupid_gtk_hex_refresh(GtkWidget *widget);
GtkWidget *cupid_gtk_debug_new(CupidGtkTool *tool);
void cupid_gtk_debug_refresh(GtkWidget *widget);
/* Release native host input without depending on window-manager focus reporting. */
void cupid_gtk_release_input(CupidGtkDesktop *desktop);
SDL_Scancode cupid_gtk_scancode(guint keyval);
SDL_Keymod cupid_gtk_modifiers(GdkModifierType state);
/* Install after main window creation; shut down before destroying that window. */
void cupid_gtk_files_install(GtkWindow *parent);
void cupid_gtk_files_shutdown(void);
void cupid_gtk_file_filters(GtkFileChooser *chooser, bool save, unsigned type);
bool cupid_gtk_native_video_present(CupidGtkDesktop *desktop, const uint32_t *pixels,
                                   unsigned width, unsigned height, unsigned stride);
void cupid_gtk_native_video_hide(CupidGtkDesktop *desktop);
void cupid_gtk_native_video_destroy(CupidGtkDesktop *desktop);
const char *cupid_gtk_native_video_name(CupidGtkDesktop *desktop);
bool cupid_gtk_native_video_read(CupidGtkDesktop *desktop, uint32_t **pixels, unsigned *width, unsigned *height);
void cupid_gtk_record_draw(CupidGtkDesktop *desktop);
#endif
