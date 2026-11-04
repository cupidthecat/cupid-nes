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

/* Setting identical label text still rebuilds its layout in GTK. */
static inline void cupid_gtk_set_label(GtkWidget *label, const char *text) {
    const char *value = text ? text : "";
    if (g_strcmp0(gtk_label_get_text(GTK_LABEL(label)), value)) {
        gtk_label_set_text(GTK_LABEL(label), value);
    }
}

static inline void cupid_gtk_set_button(GtkWidget *button, const char *text) {
    const char *value = text ? text : "";
    if (g_strcmp0(gtk_button_get_label(GTK_BUTTON(button)), value)) {
        gtk_button_set_label(GTK_BUTTON(button), value);
    }
}

typedef struct CupidGtkDesktop {
    FrontendDesktopUi *ui;
    GtkWidget *window, *picture, *status, *status_info, *menubar, *empty;
    GtkWindowGroup *window_group;
    GSimpleActionGroup *actions;
    CupidGtkTool *tools;
    CupidGtkTool *refresh_next;
    size_t commands, panels, recent;
    const FrontendSession *recent_session;
    uint64_t recent_revision;
    FrontendHostBinding shortcuts[FRONTEND_SHORTCUT_COUNT];
    bool session, held[SDL_NUM_SCANCODES], fullscreen, joystick_background;
    gboolean animations;
    bool animations_running;
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
    GtkWindowGroup *window_group;
    int kind;
    unsigned id;
    uint32_t refreshed;
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
GtkWidget *cupid_gtk_history_new(CupidGtkTool *tool);
void cupid_gtk_history_refresh(GtkWidget *widget);
void cupid_gtk_history_open(GtkWidget *widget);
GtkWidget *cupid_gtk_ppu_new(CupidGtkTool *tool);
void cupid_gtk_ppu_refresh(GtkWidget *widget);
GtkWidget *cupid_gtk_hex_new(CupidGtkTool *tool, GtkWidget *controls);
void cupid_gtk_hex_refresh(GtkWidget *widget);
GtkWidget *cupid_gtk_debug_new(CupidGtkTool *tool);
void cupid_gtk_debug_refresh(GtkWidget *widget);
/* Release native host input without depending on window-manager focus reporting. */
void cupid_gtk_release_input(CupidGtkDesktop *desktop);
SDL_Scancode cupid_gtk_scancode(guint keyval);
guint cupid_gtk_base_key(GtkEventControllerKey *controller, guint keyval, guint keycode);
SDL_Keymod cupid_gtk_modifiers(GdkModifierType state);
/* Install after main window creation; shut down before destroying that window. */
void cupid_gtk_files_install(GtkWindow *parent);
void cupid_gtk_files_shutdown(void);
void cupid_gtk_file_filters(GtkFileChooser *chooser, bool save, unsigned type);
GtkFileFilter *cupid_gtk_file_filter(bool save, unsigned type);
void cupid_gtk_file_location(GtkFileChooser *chooser, const char *path);
bool cupid_gtk_native_video_present(CupidGtkDesktop *desktop, const uint32_t *pixels,
                                   unsigned width, unsigned height, unsigned stride);
void cupid_gtk_native_video_hide(CupidGtkDesktop *desktop);
void cupid_gtk_native_video_destroy(CupidGtkDesktop *desktop);
const char *cupid_gtk_native_video_name(CupidGtkDesktop *desktop);
bool cupid_gtk_native_video_read(CupidGtkDesktop *desktop, uint32_t **pixels, unsigned *width, unsigned *height);
void cupid_gtk_record_draw(CupidGtkDesktop *desktop);
#endif
