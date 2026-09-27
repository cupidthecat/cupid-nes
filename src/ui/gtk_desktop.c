/*
 * gtk_desktop.c - GTK menus, game view and independent tool windows
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_desktop.h"
#include "gtk_internal.h"
#include "gtk_layout.h"
#include "gtk_assembler.h"
#include "gtk_keyboard.h"
#include "desktop_keyboard.h"
#include "assembler_frontend.h"
#include "hex_frontend.h"
#include "tas_frontend.h"
#include "frontend_commands.h"
#include "platform_frontend.h"
#include "update_checker.h"
#include "host_input.h"
#include "../debugger/ppu_inspector.h"
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

void cupid_gtk_margins(GtkWidget *w, int n) {
    gtk_widget_set_margin_start(w, n);
    gtk_widget_set_margin_end(w, n);
    gtk_widget_set_margin_top(w, n);
    gtk_widget_set_margin_bottom(w, n);
}

GtkWidget *cupid_gtk_label(const char *text) {
    GtkWidget *w = gtk_label_new(text ? text : "");
    gtk_label_set_xalign(GTK_LABEL(w), 0);
    gtk_label_set_selectable(GTK_LABEL(w), FALSE);
    return w;
}

GtkWidget *cupid_gtk_scroll(GtkWidget *child) {
    GtkWidget *w = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(w), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(w), child);
    gtk_widget_set_hexpand(w, TRUE);
    gtk_widget_set_vexpand(w, TRUE);
    return w;
}

void cupid_gtk_picture(GtkWidget *picture, const uint32_t *pixels, unsigned w, unsigned h, unsigned stride) {
    if (!pixels || !w || !h) {
        return;
    }
    GBytes *bytes = g_bytes_new(pixels, (size_t)stride * h);
    GdkTexture *texture = gdk_memory_texture_new((int)w, (int)h, GDK_MEMORY_B8G8R8A8, bytes, stride);
    gtk_picture_set_paintable(GTK_PICTURE(picture), GDK_PAINTABLE(texture));
    g_object_unref(texture);
    g_bytes_unref(bytes);
}

void cupid_gtk_tool_status(CupidGtkTool *tool, const char *text) {
    desktop_copy_status(&tool->ui, text);
    gtk_label_set_text(GTK_LABEL(tool->status), text ? text : "");
}

SDL_Keymod cupid_gtk_modifiers(GdkModifierType state) {
    return (SDL_Keymod)(((state & GDK_SHIFT_MASK) ? KMOD_SHIFT : 0) | ((state & GDK_CONTROL_MASK) ? KMOD_CTRL : 0) |
                        ((state & GDK_ALT_MASK) ? KMOD_ALT : 0) | ((state & GDK_SUPER_MASK) ? KMOD_GUI : 0));
}

SDL_Scancode cupid_gtk_scancode(guint key) {
    key = gdk_keyval_to_lower(key);
    if (key >= GDK_KEY_a && key <= GDK_KEY_z) {
        return (SDL_Scancode)(SDL_SCANCODE_A + key - GDK_KEY_a);
    }
    if (key >= GDK_KEY_1 && key <= GDK_KEY_9) {
        return (SDL_Scancode)(SDL_SCANCODE_1 + key - GDK_KEY_1);
    }
    if (key >= GDK_KEY_F1 && key <= GDK_KEY_F24) {
        return (SDL_Scancode)(key <= GDK_KEY_F12 ? SDL_SCANCODE_F1 + key - GDK_KEY_F1
                                                 : SDL_SCANCODE_F13 + key - GDK_KEY_F13);
    }
    switch (key) {
    case GDK_KEY_0:
        return SDL_SCANCODE_0;
    case GDK_KEY_Return:
        return SDL_SCANCODE_RETURN;
    case GDK_KEY_KP_Enter:
        return SDL_SCANCODE_KP_ENTER;
    case GDK_KEY_KP_0:
    case GDK_KEY_KP_Insert:
        return SDL_SCANCODE_KP_0;
    case GDK_KEY_KP_1:
    case GDK_KEY_KP_End:
        return SDL_SCANCODE_KP_1;
    case GDK_KEY_KP_2:
    case GDK_KEY_KP_Down:
        return SDL_SCANCODE_KP_2;
    case GDK_KEY_KP_3:
    case GDK_KEY_KP_Page_Down:
        return SDL_SCANCODE_KP_3;
    case GDK_KEY_KP_4:
    case GDK_KEY_KP_Left:
        return SDL_SCANCODE_KP_4;
    case GDK_KEY_KP_5:
    case GDK_KEY_KP_Begin:
        return SDL_SCANCODE_KP_5;
    case GDK_KEY_KP_6:
    case GDK_KEY_KP_Right:
        return SDL_SCANCODE_KP_6;
    case GDK_KEY_KP_7:
    case GDK_KEY_KP_Home:
        return SDL_SCANCODE_KP_7;
    case GDK_KEY_KP_8:
    case GDK_KEY_KP_Up:
        return SDL_SCANCODE_KP_8;
    case GDK_KEY_KP_9:
    case GDK_KEY_KP_Page_Up:
        return SDL_SCANCODE_KP_9;
    case GDK_KEY_KP_Decimal:
    case GDK_KEY_KP_Delete:
        return SDL_SCANCODE_KP_PERIOD;
    case GDK_KEY_KP_Add:
        return SDL_SCANCODE_KP_PLUS;
    case GDK_KEY_KP_Subtract:
        return SDL_SCANCODE_KP_MINUS;
    case GDK_KEY_KP_Multiply:
        return SDL_SCANCODE_KP_MULTIPLY;
    case GDK_KEY_KP_Divide:
        return SDL_SCANCODE_KP_DIVIDE;
    case GDK_KEY_KP_Equal:
        return SDL_SCANCODE_KP_EQUALS;
    case GDK_KEY_Num_Lock:
        return SDL_SCANCODE_NUMLOCKCLEAR;
    case GDK_KEY_Escape:
        return SDL_SCANCODE_ESCAPE;
    case GDK_KEY_space:
        return SDL_SCANCODE_SPACE;
    case GDK_KEY_BackSpace:
        return SDL_SCANCODE_BACKSPACE;
    case GDK_KEY_Tab:
        return SDL_SCANCODE_TAB;
    case GDK_KEY_comma:
        return SDL_SCANCODE_COMMA;
    case GDK_KEY_period:
        return SDL_SCANCODE_PERIOD;
    case GDK_KEY_slash:
        return SDL_SCANCODE_SLASH;
    case GDK_KEY_backslash:
        return SDL_SCANCODE_BACKSLASH;
    case GDK_KEY_semicolon:
        return SDL_SCANCODE_SEMICOLON;
    case GDK_KEY_apostrophe:
        return SDL_SCANCODE_APOSTROPHE;
    case GDK_KEY_bracketleft:
        return SDL_SCANCODE_LEFTBRACKET;
    case GDK_KEY_bracketright:
        return SDL_SCANCODE_RIGHTBRACKET;
    case GDK_KEY_minus:
        return SDL_SCANCODE_MINUS;
    case GDK_KEY_equal:
        return SDL_SCANCODE_EQUALS;
    case GDK_KEY_grave:
        return SDL_SCANCODE_GRAVE;
    case GDK_KEY_Left:
        return SDL_SCANCODE_LEFT;
    case GDK_KEY_Right:
        return SDL_SCANCODE_RIGHT;
    case GDK_KEY_Up:
        return SDL_SCANCODE_UP;
    case GDK_KEY_Down:
        return SDL_SCANCODE_DOWN;
    case GDK_KEY_Home:
        return SDL_SCANCODE_HOME;
    case GDK_KEY_End:
        return SDL_SCANCODE_END;
    case GDK_KEY_Page_Up:
        return SDL_SCANCODE_PAGEUP;
    case GDK_KEY_Page_Down:
        return SDL_SCANCODE_PAGEDOWN;
    case GDK_KEY_Insert:
        return SDL_SCANCODE_INSERT;
    case GDK_KEY_Delete:
        return SDL_SCANCODE_DELETE;
    case GDK_KEY_Shift_L:
        return SDL_SCANCODE_LSHIFT;
    case GDK_KEY_Shift_R:
        return SDL_SCANCODE_RSHIFT;
    case GDK_KEY_Control_L:
        return SDL_SCANCODE_LCTRL;
    case GDK_KEY_Control_R:
        return SDL_SCANCODE_RCTRL;
    case GDK_KEY_Alt_L:
        return SDL_SCANCODE_LALT;
    case GDK_KEY_Alt_R:
        return SDL_SCANCODE_RALT;
    default: {
        const char *name = gdk_keyval_name(key);
        return name ? SDL_GetScancodeFromName(name) : SDL_SCANCODE_UNKNOWN;
    }
    }
}

static void key_event(CupidGtkDesktop *d, guint key, GdkModifierType mods, bool down) {
    SDL_Scancode sc = cupid_gtk_scancode(key);
    if (sc == SDL_SCANCODE_UNKNOWN) {
        return;
    }
    SDL_Event e = {0};
    e.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    e.key.windowID = SDL_GetWindowID(d->ui->window);
    e.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    e.key.repeat = down && d->held[sc];
    e.key.keysym.scancode = sc;
    e.key.keysym.sym = SDL_GetKeyFromScancode(sc);
    e.key.keysym.mod = cupid_gtk_modifiers(mods);
    d->held[sc] = down;
    SDL_SetModState((SDL_Keymod)e.key.keysym.mod);
    SDL_PushEvent(&e);
}

static guint base_key(GtkEventControllerKey *c, guint key, guint code) {
    GdkKeymapKey *keys = NULL;
    guint *values = NULL;
    int count = 0;
    GdkDisplay *display = gtk_widget_get_display(gtk_event_controller_get_widget(GTK_EVENT_CONTROLLER(c)));
    if (gdk_display_map_keycode(display, code, &keys, &values, &count)) {
        for (int i = 0; i < count; i++) {
            if (keys[i].group == 0 && keys[i].level == 0) {
                key = values[i];
                break;
            }
        }
    }
    g_free(keys);
    g_free(values);
    return key;
}

static gboolean key_down(GtkEventControllerKey *c, guint key, guint code, GdkModifierType mods, gpointer data) {
    key_event(data, base_key(c, key, code), mods, true);
    return TRUE;
}

static void key_up(GtkEventControllerKey *c, guint key, guint code, GdkModifierType mods, gpointer data) {
    key_event(data, base_key(c, key, code), mods, false);
}

static void release_keys(CupidGtkDesktop *d) {
    for (unsigned i = 1; i < SDL_NUM_SCANCODES; i++) {
        if (d->held[i]) {
            SDL_Event e = {0};
            e.type = SDL_KEYUP;
            e.key.windowID = SDL_GetWindowID(d->ui->window);
            e.key.keysym.scancode = (SDL_Scancode)i;
            e.key.keysym.sym = SDL_GetKeyFromScancode((SDL_Scancode)i);
            SDL_PushEvent(&e);
            d->held[i] = false;
        }
    }
    SDL_SetModState(KMOD_NONE);
}

void cupid_gtk_release_input(CupidGtkDesktop *d) {
    release_keys(d);
    frontend_host_input_release_all();
    if (d->ui->execution) {
        frontend_execution_release_host_input(d->ui->execution);
    }
    for (unsigned button = 1; button <= 5; ++button) {
        if (d->mouse_buttons & SDL_BUTTON(button)) {
            SDL_Event e = {0};
            e.type = SDL_MOUSEBUTTONUP;
            e.button.windowID = SDL_GetWindowID(d->ui->window);
            e.button.button = (Uint8)button;
            e.button.x = d->mouse_x;
            e.button.y = d->mouse_y;
            SDL_PushEvent(&e);
        }
    }
    d->mouse_buttons = 0;
}

static void active_changed(GObject *object, GParamSpec *spec, gpointer data) {
    (void)spec;
    if (!gtk_window_is_active(GTK_WINDOW(object))) {
        cupid_gtk_release_input(data);
    }
}

static void picture_focus_changed(GObject *object, GParamSpec *spec, gpointer data) {
    (void)spec;
    if (!gtk_widget_has_focus(GTK_WIDGET(object))) {
        cupid_gtk_release_input(data);
    }
}

static gboolean main_close(GtkWindow *w, gpointer data) {
    (void)w;
    CupidGtkDesktop *d = data;
    d->ui->quit_requested = true;
    return TRUE;
}

static void theme(CupidGtkDesktop *d) {
    if (SDL_GetTicks() - d->theme_checked < 1000) {
        return;
    }
    d->theme_checked = SDL_GetTicks();
#ifdef _WIN32
    DWORD light = 1, bytes = sizeof(light);
    if (RegGetValueW(HKEY_CURRENT_USER, L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize",
                     L"AppsUseLightTheme", RRF_RT_REG_DWORD, NULL, &light, &bytes) == ERROR_SUCCESS) {
        GtkSettings *settings = gtk_settings_get_default();
        GParamSpec *property = g_object_class_find_property(G_OBJECT_GET_CLASS(settings), "gtk-interface-color-scheme");
        if (property && G_TYPE_IS_ENUM(G_PARAM_SPEC_VALUE_TYPE(property))) {
            GEnumClass *values = g_type_class_ref(G_PARAM_SPEC_VALUE_TYPE(property));
            GEnumValue *value = g_enum_get_value_by_nick(values, light ? "light" : "dark");
            if (value) {
                g_object_set(settings, "gtk-interface-color-scheme", value->value, NULL);
            }
            g_type_class_unref(values);
        } else {
            g_object_set(settings, "gtk-application-prefer-dark-theme", light == 0, NULL);
        }
    }
#endif
}

typedef struct {
    CupidGtkDesktop *desktop;
    DesktopMenuItem item;
} MenuAction;

static void menu_action(GSimpleAction *action, GVariant *value, gpointer data) {
    (void)action;
    (void)value;
    MenuAction *a = data;
    FrontendDesktopUi *ui = a->desktop->ui;
    char error[256] = {0};
    switch (a->item.kind) {
    case 0:
        desktop_invoke_command(
            ui, a->item.id == FRONTEND_COMMAND_FAST_FORWARD_HOLD ? FRONTEND_COMMAND_FAST_FORWARD_TOGGLE : a->item.id);
        if (a->item.id == UPDATE_CHECK_COMMAND) {
            cupid_gtk_open(ui, 1, UPDATE_PANEL);
        }
        break;
    case 1:
        frontend_desktop_open_panel(ui, a->item.id);
        break;
    case 2:
        if (ui->sessions) {
            if (!frontend_session_action_open_recent(ui->sessions, a->item.id, error, sizeof(error))) {
                desktop_copy_status(ui, error);
            }
        } else {
            ui->idle_recent_index = (int)a->item.id;
        }
        break;
    case 3:
        ui->quit_requested = true;
        break;
    case 4:
    case 6:
        cupid_gtk_open(ui, 2, a->item.kind == 6);
        break;
    case 5:
        SDL_OpenURL("https://github.com/cupidthecat/cupid-nes/tree/main/docs");
        break;
    default:
        break;
    }
    gtk_widget_grab_focus(a->desktop->picture);
}

static void free_menu(gpointer p, GClosure *closure) {
    (void)closure;
    g_free(p);
}

static GMenu *menu_level(CupidGtkDesktop *d, int depth, unsigned *serial) {
    DesktopMenuItem rows[128];
    int count = desktop_menu_level(d->ui, depth, rows);
    GMenu *menu = g_menu_new();
    GMenu *section = g_menu_new();
    for (int i = 0; i < count; i++) {
        if (rows[i].kind == 7 && depth < 4) {
            if (g_menu_model_get_n_items(G_MENU_MODEL(section))) {
                g_menu_append_section(menu, NULL, G_MENU_MODEL(section));
                g_object_unref(section);
                section = g_menu_new();
            }
            d->ui->menu_path[depth] = rows[i].id;
            GMenu *sub = menu_level(d, depth + 1, serial);
            g_menu_append_submenu(menu, rows[i].label, G_MENU_MODEL(sub));
            g_object_unref(sub);
            continue;
        }
        char name[40], full[48];
        g_snprintf(name, sizeof(name), "item%u", (*serial)++);
        g_snprintf(full, sizeof(full), "desktop.%s", name);
        FrontendCommandInfo command;
        bool checkable = rows[i].kind == 0 && frontend_command_get(rows[i].id, &command) &&
                         (command.flags & FRONTEND_COMMAND_CHECKABLE);
        GSimpleAction *act = checkable
                                 ? g_simple_action_new_stateful(name, NULL, g_variant_new_boolean(command.checked))
                                 : g_simple_action_new(name, NULL);
        MenuAction *a = g_new0(MenuAction, 1);
        a->desktop = d;
        a->item = rows[i];
        g_object_set_data(G_OBJECT(act), "menu-action", a);
        g_simple_action_set_enabled(act, rows[i].enabled);
        g_signal_connect_data(act, "activate", G_CALLBACK(menu_action), a, free_menu, 0);
        g_action_map_add_action(G_ACTION_MAP(d->actions), G_ACTION(act));
        g_object_unref(act);
        const char *label = rows[i].label;
        if (checkable && !strncmp(label, "[x] ", 4)) {
            label += 4;
        }
        GMenuItem *item = g_menu_item_new(label, full);
        if (rows[i].shortcut[0]) {
            gchar **parts = g_strsplit(rows[i].shortcut, "+", -1);
            GString *accel = g_string_new("");
            for (unsigned k = 0; parts[k]; ++k) {
                if (!g_ascii_strcasecmp(parts[k], "Ctrl")) {
                    g_string_append(accel, "<Control>");
                } else if (!g_ascii_strcasecmp(parts[k], "Alt")) {
                    g_string_append(accel, "<Alt>");
                } else if (!g_ascii_strcasecmp(parts[k], "Shift")) {
                    g_string_append(accel, "<Shift>");
                } else {
                    g_string_append(accel, parts[k]);
                }
            }
            guint key = 0;
            GdkModifierType mods = 0;
            if (gtk_accelerator_parse(accel->str, &key, &mods) && key) {
                g_menu_item_set_attribute(item, "accel", "s", accel->str);
            }
            g_string_free(accel, TRUE);
            g_strfreev(parts);
        }
        g_menu_append_item(section, item);
        g_object_unref(item);
    }
    if (g_menu_model_get_n_items(G_MENU_MODEL(section))) {
        g_menu_append_section(menu, NULL, G_MENU_MODEL(section));
    }
    g_object_unref(section);
    return menu;
}

static void rebuild_menus(CupidGtkDesktop *d) {
    static const char *names[] = {"File", "Emulation", "View", "Audio", "Media", "Tools", "Help"};
    g_clear_object(&d->actions);
    d->actions = g_simple_action_group_new();
    gtk_widget_insert_action_group(d->window, "desktop", G_ACTION_GROUP(d->actions));
    GMenu *root = g_menu_new();
    unsigned serial = 0;
    int old = d->ui->open_menu;
    for (int i = 0; i < 7; i++) {
        d->ui->open_menu = i;
        GMenu *menu = menu_level(d, 0, &serial);
        g_menu_append_submenu(root, names[i], G_MENU_MODEL(menu));
        g_object_unref(menu);
    }
    d->ui->open_menu = old;
    gtk_popover_menu_bar_set_menu_model(GTK_POPOVER_MENU_BAR(d->menubar), G_MENU_MODEL(root));
    g_object_unref(root);
}

static void sync_actions(CupidGtkDesktop *d) {
    gchar **names = g_action_group_list_actions(G_ACTION_GROUP(d->actions));
    for (unsigned i = 0; names[i]; i++) {
        GAction *action = g_action_map_lookup_action(G_ACTION_MAP(d->actions), names[i]);
        MenuAction *a = g_object_get_data(G_OBJECT(action), "menu-action");
        if (!a) {
            continue;
        }
        FrontendCommandInfo command;
        FrontendPanelInfo panel;
        if (a->item.kind == 0 && frontend_command_get(a->item.id, &command)) {
            g_simple_action_set_enabled(G_SIMPLE_ACTION(action), command.enabled);
            if (command.flags & FRONTEND_COMMAND_CHECKABLE) {
                g_simple_action_set_state(G_SIMPLE_ACTION(action), g_variant_new_boolean(command.checked));
            }
        } else if (a->item.kind == 1 && frontend_panel_get(a->item.id, &panel)) {
            g_simple_action_set_enabled(G_SIMPLE_ACTION(action), panel.enabled);
        }
    }
    g_strfreev(names);
}

static void toolbar_action(GtkButton *button, gpointer data) {
    CupidGtkDesktop *d = data;
    desktop_invoke_command(d->ui, GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(button), "command")));
    gtk_widget_grab_focus(d->picture);
}

#if GTK_CHECK_VERSION(4, 10, 0)
typedef struct {
    GtkWidget parent_instance;
    CupidGtkDesktop *desktop;
} CupidGtkGameView;

typedef GtkWidgetClass CupidGtkGameViewClass;

G_DEFINE_TYPE(CupidGtkGameView, cupid_gtk_game_view, GTK_TYPE_WIDGET)

static void game_snapshot(GtkWidget *widget, GtkSnapshot *snapshot) {
    CupidGtkDesktop *d = ((CupidGtkGameView *)widget)->desktop;
    int width = gtk_widget_get_width(widget), height = gtk_widget_get_height(widget);
    graphene_rect_t bounds = GRAPHENE_RECT_INIT(0, 0, width, height);
    const GdkRGBA background = {.025f, .028f, .035f, 1.f};
    gtk_snapshot_append_color(snapshot, &background, &bounds);
    if (!d->frame_texture || !d->session) {
        return;
    }

    unsigned vw, vh;
    frontend_video_runtime_display_size(d->ui->video, &vw, &vh);
    SDL_Rect r;
    frontend_desktop_game_rect(d->ui, width, height, (int)vw, (int)vh, d->ui->settings->integer_scaling, &r);
    bounds = GRAPHENE_RECT_INIT(r.x, r.y, r.w, r.h);
    gtk_snapshot_append_scaled_texture(snapshot, d->frame_texture,
        d->ui->settings->bilinear_interpolation ? GSK_SCALING_FILTER_LINEAR : GSK_SCALING_FILTER_NEAREST, &bounds);
}

static void cupid_gtk_game_view_class_init(CupidGtkGameViewClass *klass) {
    GTK_WIDGET_CLASS(klass)->snapshot = game_snapshot;
}

static void cupid_gtk_game_view_init(CupidGtkGameView *view) {
    gtk_widget_set_overflow(GTK_WIDGET(view), GTK_OVERFLOW_HIDDEN);
}

static void update_game(CupidGtkDesktop *d, const uint32_t *pixels, unsigned w, unsigned h, unsigned stride) {
    /* Upload the source frame once; GTK scales its texture at presentation.
     * The immutable copy outlives emulation's reusable pixel buffers. Like
     * Cairo RGB24, the game view ignores the source's unused alpha byte. */
    uint32_t *copy = g_new(uint32_t, (size_t)w * h);
    for (unsigned y = 0; y < h; ++y) {
        for (unsigned x = 0; x < w; ++x) {
            copy[(size_t)y * w + x] = pixels[(size_t)y * stride + x] | 0xff000000u;
        }
    }

    GBytes *bytes = g_bytes_new_take(copy, (size_t)w * h * sizeof(*copy));
    GdkTexture *texture = gdk_memory_texture_new((int)w, (int)h, GDK_MEMORY_DEFAULT, bytes, w * sizeof(*copy));
    g_bytes_unref(bytes);
    g_clear_object(&d->frame_texture);
    d->frame_texture = texture;
    d->frame_width = w;
    d->frame_height = h;
    gtk_widget_queue_draw(d->picture);
}
#else
/* GTK 4.8 lacks texture scaling filters; retain its pixel-exact Cairo path. */
static void draw_game(GtkDrawingArea *area, cairo_t *cr, int width, int height, gpointer data) {
    (void)area;
    CupidGtkDesktop *d = data;
    cairo_set_source_rgb(cr, .025, .028, .035);
    cairo_paint(cr);
    if (!d->frame || !d->session) {
        return;
    }
    unsigned vw, vh;
    frontend_video_runtime_display_size(d->ui->video, &vw, &vh);
    SDL_Rect r;
    frontend_desktop_game_rect(d->ui, width, height, (int)vw, (int)vh, d->ui->settings->integer_scaling, &r);
    cairo_translate(cr, r.x, r.y);
    cairo_scale(cr, (double)r.w / d->frame_width, (double)r.h / d->frame_height);
    cairo_set_source_surface(cr, d->frame, 0, 0);
    cairo_pattern_set_filter(cairo_get_source(cr),
                             d->ui->settings->bilinear_interpolation ? CAIRO_FILTER_BILINEAR : CAIRO_FILTER_NEAREST);
    cairo_paint(cr);
}

static void update_game(CupidGtkDesktop *d, const uint32_t *pixels, unsigned w, unsigned h, unsigned stride) {
    if (!d->frame || d->frame_width != w || d->frame_height != h) {
        if (d->frame) {
            cairo_surface_destroy(d->frame);
        }
        d->frame = cairo_image_surface_create(CAIRO_FORMAT_RGB24, (int)w, (int)h);
        d->frame_width = w;
        d->frame_height = h;
    }
    cairo_surface_flush(d->frame);
    unsigned char *out = cairo_image_surface_get_data(d->frame);
    int pitch = cairo_image_surface_get_stride(d->frame);
    for (unsigned y = 0; y < h; ++y) {
        memcpy(out + (size_t)y * pitch, pixels + (size_t)y * stride, w * sizeof(uint32_t));
    }
    cairo_surface_mark_dirty(d->frame);
    gtk_widget_queue_draw(d->picture);
}
#endif

static void pointer_event(CupidGtkDesktop *d, double x, double y, Uint32 type, guint button) {
    int w = gtk_widget_get_width(d->picture), h = gtk_widget_get_height(d->picture), sw, sh;
    SDL_GetWindowSize(d->ui->window, &sw, &sh);
    unsigned vw, vh;
    frontend_video_runtime_display_size(d->ui->video, &vw, &vh);
    SDL_Rect shown, hidden;
    frontend_desktop_game_rect(d->ui, w, h, (int)vw, (int)vh, d->ui->settings->integer_scaling, &shown);
    frontend_desktop_game_rect(d->ui, sw, sh, (int)vw, (int)vh, d->ui->settings->integer_scaling, &hidden);
    x = hidden.x + (x - shown.x) * hidden.w / (shown.w ? shown.w : 1);
    y = hidden.y + (y - shown.y) * hidden.h / (shown.h ? shown.h : 1);
    SDL_Event e = {0};
    e.type = type;
    if (type == SDL_MOUSEMOTION) {
        e.motion.windowID = SDL_GetWindowID(d->ui->window);
        e.motion.x = (int)x;
        e.motion.y = (int)y;
        e.motion.xrel = (int)x - d->mouse_x;
        e.motion.yrel = (int)y - d->mouse_y;
        e.motion.state = d->mouse_buttons;
    } else {
        e.button.windowID = SDL_GetWindowID(d->ui->window);
        e.button.x = (int)x;
        e.button.y = (int)y;
        e.button.button = (Uint8)button;
        e.button.state = type == SDL_MOUSEBUTTONDOWN ? SDL_PRESSED : SDL_RELEASED;
        if (type == SDL_MOUSEBUTTONDOWN) {
            d->mouse_buttons |= SDL_BUTTON(button);
        } else {
            d->mouse_buttons &= ~SDL_BUTTON(button);
        }
    }
    d->mouse_x = (int)x;
    d->mouse_y = (int)y;
    SDL_PushEvent(&e);
}

uint32_t cupid_gtk_pointer(FrontendDesktopUi *ui, const SDL_Event *event, int *x, int *y) {
    CupidGtkDesktop *d = ui->gtk;
    if (event->type == SDL_MOUSEMOTION) {
        *x = event->motion.x;
        *y = event->motion.y;
        d->consumed_mouse_buttons = event->motion.state;
    } else {
        *x = event->button.x;
        *y = event->button.y;
        if (event->type == SDL_MOUSEBUTTONDOWN) {
            d->consumed_mouse_buttons |= SDL_BUTTON(event->button.button);
        } else {
            d->consumed_mouse_buttons &= ~SDL_BUTTON(event->button.button);
        }
    }
    return d->consumed_mouse_buttons;
}

static void motion(GtkEventControllerMotion *c, double x, double y, gpointer p) {
    (void)c;
    pointer_event(p, x, y, SDL_MOUSEMOTION, 0);
}

static void pressed(GtkGestureClick *c, int n, double x, double y, gpointer p) {
    (void)n;
    CupidGtkDesktop *d = p;
    gtk_widget_grab_focus(d->picture);
    pointer_event(d, x, y, SDL_MOUSEBUTTONDOWN, gtk_gesture_single_get_current_button(GTK_GESTURE_SINGLE(c)));
}

static void released(GtkGestureClick *c, int n, double x, double y, gpointer p) {
    (void)n;
    pointer_event(p, x, y, SDL_MOUSEBUTTONUP, gtk_gesture_single_get_current_button(GTK_GESTURE_SINGLE(c)));
}

static gboolean dropped(GtkDropTarget *target, const GValue *value, double x, double y, gpointer data) {
    (void)target;
    (void)x;
    (void)y;
    (void)data;
    GFile *file = g_value_get_object(value);
    char *path = g_file_get_path(file);
    if (!path) {
        return FALSE;
    }
    SDL_Event e = {0};
    e.type = SDL_DROPFILE;
    e.drop.file = SDL_strdup(path);
    g_free(path);
    SDL_PushEvent(&e);
    return TRUE;
}

bool cupid_gtk_init(FrontendDesktopUi *ui) {
    const char *driver = SDL_GetCurrentVideoDriver();
#ifdef _WIN32
    /* Recent Win32 GTK renderers require DirectComposition for GPU output.
     * Without this opt-in they silently fall back to scaling with Cairo.
     * Keep explicit diagnostic overrides and GTK's device-failure fallback. */
    if (gtk_check_version(4, 24, 0) == NULL) {
        g_setenv("GDK_DEBUG", "dcomp", FALSE);
    }
#endif
    if (!ui->window || !driver || !strcmp(driver, "dummy") || !gtk_init_check()) {
        return false;
    }
    CupidGtkDesktop *d = g_new0(CupidGtkDesktop, 1);
    d->ui = ui;
    /* The SDL host is hidden; GTK focus decides whether queued controller input is accepted. */
    d->joystick_background = SDL_GetHintBoolean(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, SDL_FALSE);
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    ui->gtk = d;
    d->window = gtk_window_new();
    gtk_widget_add_css_class(d->window, "cupid-desktop");
    gtk_window_set_title(GTK_WINDOW(d->window), "Cupid NES");
    gtk_window_set_default_size(GTK_WINDOW(d->window), (int)ui->settings->window_width,
                                (int)ui->settings->window_height);
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_window_set_child(GTK_WINDOW(d->window), box);
    GMenu *empty = g_menu_new();
    d->menubar = gtk_popover_menu_bar_new_from_model(G_MENU_MODEL(empty));
    g_object_unref(empty);
    gtk_box_append(GTK_BOX(box), d->menubar);
    GtkWidget *bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_add_css_class(bar, "toolbar");
    cupid_gtk_margins(bar, 6);
    gtk_box_append(GTK_BOX(box), bar);
    const char *labels[] = {"Open game", "Pause / Resume", "Frame advance", "Reset", "Settings"};
    const char *icons[] = {"document-open-symbolic", "media-playback-pause-symbolic", "media-skip-forward-symbolic",
                           "view-refresh-symbolic", "preferences-system-symbolic"};
    unsigned ids[] = {FRONTEND_COMMAND_OPEN, FRONTEND_COMMAND_PAUSE, FRONTEND_COMMAND_FRAME_ADVANCE,
                      FRONTEND_COMMAND_SOFT_RESET, FRONTEND_COMMAND_SETTINGS};
    for (unsigned i = 0; i < G_N_ELEMENTS(ids); i++) {
        GtkWidget *b = gtk_button_new_from_icon_name(icons[i]);
        gtk_widget_set_tooltip_text(b, labels[i]);
        gtk_accessible_update_property(GTK_ACCESSIBLE(b), GTK_ACCESSIBLE_PROPERTY_LABEL, labels[i], -1);
        g_object_set_data(G_OBJECT(b), "command", GUINT_TO_POINTER(ids[i]));
        g_signal_connect(b, "clicked", G_CALLBACK(toolbar_action), d);
        gtk_box_append(GTK_BOX(bar), b);
    }
    GtkWidget *overlay = gtk_overlay_new();
    gtk_widget_set_vexpand(overlay, TRUE);
    gtk_box_append(GTK_BOX(box), overlay);
#if GTK_CHECK_VERSION(4, 10, 0)
    d->picture = g_object_new(cupid_gtk_game_view_get_type(), NULL);
    ((CupidGtkGameView *)d->picture)->desktop = d;
#else
    d->picture = gtk_drawing_area_new();
    gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(d->picture), draw_game, d, NULL);
#endif
    gtk_widget_set_focusable(d->picture, TRUE);
    g_signal_connect(d->picture, "notify::has-focus", G_CALLBACK(picture_focus_changed), d);
    gtk_widget_add_css_class(d->picture, "game-view");
    gtk_overlay_set_child(GTK_OVERLAY(overlay), d->picture);
    d->empty = gtk_label_new("Cupid NES\n\nOpen a game or drop a ROM here");
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), d->empty);
    d->status = cupid_gtk_label("Ready");
    gtk_widget_add_css_class(d->status, "tool-status");
    cupid_gtk_margins(d->status, 6);
    gtk_box_append(GTK_BOX(box), d->status);
    GtkEventController *keys = gtk_event_controller_key_new();
    g_signal_connect(keys, "key-pressed", G_CALLBACK(key_down), d);
    g_signal_connect(keys, "key-released", G_CALLBACK(key_up), d);
    gtk_widget_add_controller(d->picture, keys);
    GtkEventController *moves = gtk_event_controller_motion_new();
    g_signal_connect(moves, "motion", G_CALLBACK(motion), d);
    gtk_widget_add_controller(d->picture, moves);
    GtkGesture *click = gtk_gesture_click_new();
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click), 0);
    g_signal_connect(click, "pressed", G_CALLBACK(pressed), d);
    g_signal_connect(click, "released", G_CALLBACK(released), d);
    gtk_widget_add_controller(d->picture, GTK_EVENT_CONTROLLER(click));
    GtkDropTarget *drop = gtk_drop_target_new(G_TYPE_FILE, GDK_ACTION_COPY);
    g_signal_connect(drop, "drop", G_CALLBACK(dropped), d);
    gtk_widget_add_controller(d->window, GTK_EVENT_CONTROLLER(drop));
    g_signal_connect(d->window, "close-request", G_CALLBACK(main_close), d);
    g_signal_connect(d->window, "notify::is-active", G_CALLBACK(active_changed), d);
    GtkCssProvider *css = gtk_css_provider_new();
    gtk_css_provider_load_from_data(
        css,
        ".game-view { background: #08090b; } .monospace { font-family: monospace; }"
        ".cupid-desktop { font-size: 12px; }"
        ".cupid-desktop button { min-height: 22px; padding: 3px 8px; border-radius: 3px; }"
        ".cupid-desktop entry { min-height: 24px; padding: 2px 6px; border-radius: 2px; }"
        ".cupid-desktop dropdown button { min-height: 24px; }"
        ".cupid-desktop notebook > header tab { min-height: 24px; padding: 3px 9px; }"
        ".cupid-desktop frame { border-radius: 2px; }"
        ".cupid-desktop frame > label { margin: 2px 8px; font-weight: bold; }"
        ".cupid-desktop flowboxchild { padding: 0; }"
        ".cupid-desktop .toolbar button { padding: 4px 7px; }"
        ".tool-status { padding: 4px 8px; border-top: 1px solid alpha(currentColor, 0.18); }",
        -1);
    gtk_style_context_add_provider_for_display(gtk_widget_get_display(d->window), GTK_STYLE_PROVIDER(css),
                                               GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(css);
    SDL_HideWindow(ui->window);
    cupid_gtk_files_install(GTK_WINDOW(d->window));
    theme(d);
    gtk_window_present(GTK_WINDOW(d->window));
    gtk_widget_grab_focus(d->picture);
    return true;
}

static gboolean tool_close(GtkWindow *window, gpointer data) {
    (void)window;
    CupidGtkTool *t = data;
    gtk_widget_set_visible(t->window, FALSE);
    t->ui.capture_binding = false;
    t->ui.edit_text_active = false;
    if (t->prompt) {
        gtk_window_destroy(GTK_WINDOW(t->prompt));
        t->prompt = NULL;
    }
    return TRUE;
}

static void prompt_response(GtkDialog *dialog, int response, gpointer data) {
    CupidGtkTool *t = data;
    if (response == GTK_RESPONSE_ACCEPT) {
        g_snprintf(t->ui.edit_text, sizeof(t->ui.edit_text), "%s",
                   gtk_editable_get_text(GTK_EDITABLE(t->prompt_entry)));
        desktop_commit_edit(&t->ui);
        if (t->ui.edit_text_active) {
            return;
        }
    }
    t->ui.edit_text_active = false;
    t->ui.prompt_command = 0;
    t->prompt = NULL;
    gtk_window_destroy(GTK_WINDOW(dialog));
}

static void prompt_open(CupidGtkTool *t) {
    if (t->prompt || !t->ui.edit_text_active || t->id == TAS_PANEL || t->kind == 0) {
        return;
    }
    t->prompt = gtk_dialog_new_with_buttons("Edit value", GTK_WINDOW(t->window),
                                            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT, "Cancel",
                                            GTK_RESPONSE_CANCEL, "Apply", GTK_RESPONSE_ACCEPT, NULL);
    t->prompt_entry = gtk_entry_new();
    gtk_editable_set_text(GTK_EDITABLE(t->prompt_entry), t->ui.edit_text);
    gtk_entry_set_activates_default(GTK_ENTRY(t->prompt_entry), TRUE);
    gtk_dialog_set_default_response(GTK_DIALOG(t->prompt), GTK_RESPONSE_ACCEPT);
    cupid_gtk_margins(t->prompt_entry, 16);
    gtk_widget_set_size_request(t->prompt_entry, 380, -1);
    gtk_box_append(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(t->prompt))), t->prompt_entry);
    g_signal_connect(t->prompt, "response", G_CALLBACK(prompt_response), t);
    gtk_window_present(GTK_WINDOW(t->prompt));
}

FrontendDesktopUi *cupid_gtk_open(FrontendDesktopUi *ui, int kind, unsigned id) {
    CupidGtkDesktop *d = ui->gtk;
    for (CupidGtkTool *t = d->tools; t; t = t->next) {
        if (t->kind == kind && (t->id == id || kind == 3)) {
            if (kind == 0 && !gtk_widget_get_visible(t->window)) {
                t->ui.settings_open = false;
                desktop_settings_open(&t->ui, true);
                cupid_gtk_settings_reset(t->content);
            }
            gtk_window_present(GTK_WINDOW(t->window));
            return &t->ui;
        }
    }
    CupidGtkTool *t = g_new0(CupidGtkTool, 1);
    t->desktop = d;
    t->kind = kind;
    t->id = id;
    t->ui = *d->ui;
    t->ui.parent = d->ui;
    t->ui.tools = t->ui.next = NULL;
    t->ui.clay = NULL;
    t->ui.ppu_viewer = NULL;
    t->ui.tas_editor = NULL;
    t->ui.settings_open = false;
    t->ui.panel_open = kind == 1;
    t->ui.panel_id = id;
    t->ui.staged = *ui->settings;
    t->ui.status[0] = 0;
    t->ui.edit_text_active = false;
    t->ui.capture_binding = false;
    FrontendPanelInfo info;
    const char *title = kind == 0   ? "Settings"
                        : kind == 3 ? "Palette editor"
                        : id        ? "Recent messages"
                                    : "Game information";
    if (kind == 1 && frontend_panel_get(id, &info)) {
        title = info.title;
    }
    t->window = gtk_window_new();
    gtk_widget_add_css_class(t->window, "cupid-desktop");
    gtk_window_set_title(GTK_WINDOW(t->window), title);
    CupidGtkLayout layout = cupid_gtk_layout(id);
    gtk_window_set_default_size(GTK_WINDOW(t->window), kind == 0 ? 880 : layout.width, kind == 0 ? 620 : layout.height);
    if (kind == 2) {
        gtk_window_set_default_size(GTK_WINDOW(t->window), id ? 760 : 540, id ? 480 : 300);
    }
    if (kind == 3) {
        gtk_window_set_default_size(GTK_WINDOW(t->window), 760, 510);
    }
    gtk_window_set_transient_for(GTK_WINDOW(t->window), GTK_WINDOW(d->window));
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_window_set_child(GTK_WINDOW(t->window), box);
    t->status = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(t->status), TRUE);
    gtk_widget_add_css_class(t->status, "tool-status");
    if (kind == 0) {
        desktop_settings_open(&t->ui, true);
        t->content = cupid_gtk_settings_new(t);
    } else if (kind == 1 && id == DESKTOP_KEYBOARD_PANEL) {
        t->content = cupid_gtk_keyboard_new(t);
    } else if (kind == 1 && id == ASSEMBLER_FRONTEND_PANEL) {
        t->content = cupid_gtk_assembler_new(id);
    } else if (kind == 1 && id == TAS_PANEL) {
        t->content = cupid_gtk_tas_new(t);
    } else if (kind == 3 || (kind == 1 && desktop_ppu_panel(id))) {
        if (kind == 3) {
            t->id = DEBUG_PPU_PALETTE;
        }
        t->content = cupid_gtk_ppu_new(t);
    } else if (kind == 1) {
        t->content = cupid_gtk_panel_new(t);
    } else {
        GString *text = g_string_new("");
        if (id) {
            for (unsigned i = 0; i < ui->log_count; i++) {
                g_string_append_printf(text, "%s\n", ui->log_lines[i]);
            }
        } else {
            const char *game, *region, *state;
            desktop_window_context(ui, &game, &region, &state);
            g_string_append_printf(text, "%s\n%s | %s", game ? game : "No game loaded", region, state);
            GskRenderer *renderer = gtk_native_get_renderer(GTK_NATIVE(d->window));
            g_string_append_printf(text, "\nGTK %u.%u.%u\nVideo renderer: %s", gtk_get_major_version(),
                                   gtk_get_minor_version(), gtk_get_micro_version(),
                                   renderer ? G_OBJECT_TYPE_NAME(renderer) : "Unavailable");
        }
        GtkWidget *view = gtk_text_view_new();
        gtk_text_view_set_editable(GTK_TEXT_VIEW(view), FALSE);
        gtk_text_view_set_monospace(GTK_TEXT_VIEW(view), id != 0);
        gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(view), GTK_WRAP_WORD_CHAR);
        gtk_text_view_set_left_margin(GTK_TEXT_VIEW(view), 14);
        gtk_text_view_set_top_margin(GTK_TEXT_VIEW(view), 14);
        gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(view)), text->str, -1);
        t->content = cupid_gtk_scroll(view);
        g_string_free(text, TRUE);
    }
    gtk_widget_set_vexpand(t->content, TRUE);
    gtk_box_append(GTK_BOX(box), t->content);
    gtk_box_append(GTK_BOX(box), t->status);
    g_signal_connect(t->window, "close-request", G_CALLBACK(tool_close), t);
    t->next = d->tools;
    d->tools = t;
    gtk_window_present(GTK_WINDOW(t->window));
    return &t->ui;
}

void cupid_gtk_render(FrontendDesktopUi *ui, const char *title, const char *region, const char *state) {
    CupidGtkDesktop *d = ui->gtk;
    theme(d);
    size_t recent = frontend_session_recent_count(ui->sessions ? ui->sessions->session : ui->idle_session);
    bool session = frontend_panel_session_active();
    if (!d->actions || d->commands != frontend_command_count() || d->panels != frontend_panel_count() ||
        d->session != session || d->recent != recent) {
        d->commands = frontend_command_count();
        d->panels = frontend_panel_count();
        d->session = session;
        d->recent = recent;
        rebuild_menus(d);
    }
    if (d->fullscreen != ui->settings->fullscreen) {
        d->fullscreen = ui->settings->fullscreen;
        if (d->fullscreen) {
            gtk_window_fullscreen(GTK_WINDOW(d->window));
        } else {
            gtk_window_unfullscreen(GTK_WINDOW(d->window));
        }
    }
    const uint32_t *pixels;
    unsigned w, h, stride;
    if (session && ui->video && frontend_video_runtime_pixels(ui->video, true, &pixels, &w, &h, &stride)) {
        update_game(d, pixels, w, h, stride);
    }
    if (!session) {
        gtk_widget_queue_draw(d->picture);
    }
    gtk_widget_set_visible(d->empty, !session);
    char caption[512];
    g_snprintf(caption, sizeof(caption), "Cupid NES%s%s", title ? " | " : "", title ? title : "");
    if (g_strcmp0(gtk_window_get_title(GTK_WINDOW(d->window)), caption)) {
        gtk_window_set_title(GTK_WINDOW(d->window), caption);
    }
    g_snprintf(caption, sizeof(caption), "%s%s%s%s%s", state ? state : "Ready", region ? " | " : "",
               region ? region : "", ui->status[0] ? " | " : "", ui->status);
    if (ui->settings->show_fps) {
        size_t n = strlen(caption);
        g_snprintf(caption + n, sizeof(caption) - n, " | %.1f fps", ui->fps);
    }
    if (strcmp(gtk_label_get_text(GTK_LABEL(d->status)), caption)) {
        gtk_label_set_text(GTK_LABEL(d->status), caption);
    }
    for (CupidGtkTool *t = d->tools; t; t = t->next) {
        if (t->kind == 1 && t->id == TAS_PANEL && gtk_widget_get_visible(t->window)) {
            cupid_gtk_tas_video(t->content);
        }
    }
    if (SDL_GetTicks() - d->refreshed >= 100) {
        d->refreshed = SDL_GetTicks();
        sync_actions(d);
        for (CupidGtkTool *t = d->tools; t; t = t->next) {
            if (!gtk_widget_get_visible(t->window)) {
                continue;
            }
            t->ui.audio = ui->audio;
            t->ui.video = ui->video;
            t->ui.capture = ui->capture;
            t->ui.devices = ui->devices;
            t->ui.music = ui->music;
            if (t->kind == 1 && t->id == DESKTOP_KEYBOARD_PANEL) {
                cupid_gtk_keyboard_refresh(t->content);
            } else if (t->kind == 0) {
                cupid_gtk_settings_refresh(t->content);
            } else if (t->kind == 3 || desktop_ppu_panel(t->id)) {
                cupid_gtk_ppu_refresh(t->content);
            } else if (t->kind == 1 && t->id == ASSEMBLER_FRONTEND_PANEL) {
                cupid_gtk_assembler_refresh(t->content);
            } else if (t->kind == 1 && t->id == TAS_PANEL) {
                cupid_gtk_tas_refresh(t->content);
            } else if (t->kind == 1) {
                cupid_gtk_panel_refresh(t->content);
            }
            if (t->ui.status[0]) {
                gtk_label_set_text(GTK_LABEL(t->status), t->ui.status);
            }
            prompt_open(t);
        }
    }
    for (unsigned i = 0; i < 64 && g_main_context_pending(NULL); i++) {
        g_main_context_iteration(NULL, FALSE);
    }
}

bool cupid_gtk_captured(const FrontendDesktopUi *ui) {
    const CupidGtkDesktop *d = ui->gtk;
    if (!gtk_window_is_active(GTK_WINDOW(d->window)) || !gtk_widget_has_focus(d->picture)) {
        return true;
    }
    for (CupidGtkTool *t = d->tools; t; t = t->next) {
        if (gtk_widget_get_visible(t->window) && gtk_window_is_active(GTK_WINDOW(t->window))) {
            return true;
        }
    }
    return false;
}

void cupid_gtk_activity(FrontendDesktopUi *ui) {
    if (!ui->execution) {
        return;
    }
    CupidGtkDesktop *d = ui->gtk;
    bool focused = gtk_window_is_active(GTK_WINDOW(d->window)), modal = false;
    for (CupidGtkTool *t = d->tools; t; t = t->next) {
        if (gtk_widget_get_visible(t->window)) {
            focused |= gtk_window_is_active(GTK_WINDOW(t->window));
            modal |= t->kind == 0;
        }
    }
    unsigned reasons = ui->execution->suspend_reasons & ~(FRONTEND_SUSPEND_UI | FRONTEND_SUSPEND_FOCUS);
    if (!focused && ui->settings->pause_on_focus_loss) {
        reasons |= FRONTEND_SUSPEND_FOCUS;
    }
    if (modal && ui->settings->pause_on_ui) {
        reasons |= FRONTEND_SUSPEND_UI;
    }
    frontend_execution_set_suspension(ui->execution, reasons);
}

bool cupid_gtk_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    if (!event) {
        return false;
    }
    if (event->type == SDL_WINDOWEVENT) {
        return true;
    }
    for (CupidGtkTool *t = ui->gtk->tools; t; t = t->next) {
        if (t->ui.capture_binding && event->type == SDL_CONTROLLERBUTTONDOWN) {
            desktop_binding_gamepad(&t->ui, (SDL_GameControllerButton)event->cbutton.button);
            return true;
        }
    }
    return false;
}

void cupid_gtk_save_size(FrontendDesktopUi *ui) {
    if (!ui->settings->remember_window_size || ui->settings->fullscreen ||
        gtk_window_is_maximized(GTK_WINDOW(ui->gtk->window))) {
        return;
    }
    int w = gtk_widget_get_width(ui->gtk->window), h = gtk_widget_get_height(ui->gtk->window);
    if (w >= 640 && h >= 480) {
        ui->settings->window_width = (unsigned)w;
        ui->settings->window_height = (unsigned)h;
    }
}

void cupid_gtk_shutdown(FrontendDesktopUi *ui) {
    CupidGtkDesktop *d = ui->gtk;
    cupid_gtk_files_shutdown();
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, d->joystick_background ? "1" : "0");
    release_keys(d);
    while (d->tools) {
        CupidGtkTool *t = d->tools;
        d->tools = t->next;
        gtk_window_destroy(GTK_WINDOW(t->window));
        desktop_tas_destroy(&t->ui);
        g_free(t);
    }
    gtk_window_destroy(GTK_WINDOW(d->window));
    if (d->frame) {
        cairo_surface_destroy(d->frame);
    }
    g_clear_object(&d->frame_texture);
    g_clear_object(&d->actions);
    g_free(d);
    ui->gtk = NULL;
}
