/*
 * gtk_workflow_accuracy.c - Recent games, shortcut labels and file filters
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/platform_frontend.h"
#include <stdio.h>
#include <string.h>

static char *menu_attribute(GMenuModel *menu, const char *label, const char *attribute) {
    for (int i = 0; i < g_menu_model_get_n_items(menu); ++i) {
        char *text = NULL, *value = NULL;
        g_menu_model_get_item_attribute(menu, i, G_MENU_ATTRIBUTE_LABEL, "s", &text);
        if (text && strstr(text, label)) {
            g_menu_model_get_item_attribute(menu, i, attribute, "s", &value);
        }
        g_free(text);
        if (value) {
            return value;
        }

        const char *links[] = {G_MENU_LINK_SECTION, G_MENU_LINK_SUBMENU};
        for (unsigned j = 0; j < G_N_ELEMENTS(links); ++j) {
            GMenuModel *child = g_menu_model_get_item_link(menu, i, links[j]);
            if (child) {
                value = menu_attribute(child, label, attribute);
                g_object_unref(child);
                if (value) {
                    return value;
                }
            }
        }
    }
    return NULL;
}

static bool open_recent(void *context, const FrontendImageRequest *request, FrontendImageResult *result, char *error,
                        size_t size) {
    (void)context;
    (void)request;
    (void)result;
    (void)error;
    (void)size;
    return true;
}

bool test_gtk_workflows(FrontendDesktopUi *ui) {
    FrontendSession session;
    frontend_session_init(&session, open_recent, NULL);
    FrontendSessionActions *previous_actions = ui->sessions;
    const FrontendSession *previous_idle = ui->idle_session;
    ui->sessions = NULL;
    ui->idle_session = &session;
    FrontendImageRequest request;
    bool ok = frontend_image_request_init(&request, "first-workflow.nes") &&
              frontend_session_open(&session, &request, NULL, 0) &&
              frontend_image_request_init(&request, "second-workflow.nes") &&
              frontend_session_open(&session, &request, NULL, 0);
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    GMenuModel *menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    char *before = menu_attribute(menu, "first-workflow.nes", G_MENU_ATTRIBUTE_ACTION);
    ok &= frontend_session_open_recent(&session, 1, NULL, 0);
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    char *after = menu_attribute(menu, "first-workflow.nes", G_MENU_ATTRIBUTE_ACTION);
    bool reordered = before && after && strcmp(before, after);
    g_free(before);
    g_free(after);
    /* Keep the list length constant even when a new game replaces an old one. */
    session.recent_count = FRONTEND_RECENT_MAX;
    for (unsigned i = 2; i < FRONTEND_RECENT_MAX; ++i) {
        char path[64];
        g_snprintf(path, sizeof(path), "workflow-%u.nes", i);
        frontend_image_request_init(&session.recent[i], path);
    }
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    ok &= frontend_image_request_init(&request, "replacement-workflow.nes") &&
          frontend_session_open(&session, &request, NULL, 0);
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    after = menu_attribute(menu, "replacement-workflow.nes", G_MENU_ATTRIBUTE_ACTION);
    bool replaced = after != NULL;
    g_free(after);
    ui->sessions = previous_actions;
    ui->idle_session = previous_idle;

    FrontendBindingProfile *profile = frontend_settings_active_profile(ui->settings);
    FrontendHostBinding saved = profile->shortcuts[FRONTEND_SHORTCUT_PAUSE];
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE].key = SDL_SCANCODE_J;
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE].modifiers = KMOD_CTRL;
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    before = menu_attribute(menu, "Pause", "accel");
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE].key = SDL_SCANCODE_K;
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    after = menu_attribute(menu, "Pause", "accel");
    bool remapped = before && after && strcmp(before, after);
    g_free(before);
    g_free(after);
    profile->shortcuts[FRONTEND_SHORTCUT_PAUSE] = saved;
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");

    GtkFileFilter *filter = cupid_gtk_file_filter(false, FRONTEND_OPEN_IMAGE);
    const char *names[] = {"game.nes", "GAME.NES", "game.NeS", "archive.ZIP", "disk.FDS"};
    bool filtered = true;
    GFileInfo *info = g_file_info_new();
    for (unsigned i = 0; i < G_N_ELEMENTS(names); ++i) {
        g_file_info_set_display_name(info, names[i]);
        g_file_info_set_name(info, names[i]);
        filtered &= gtk_filter_match(GTK_FILTER(filter), info);
    }
    g_file_info_set_display_name(info, "wrong.exe");
    g_file_info_set_name(info, "wrong.exe");
    filtered &= !gtk_filter_match(GTK_FILTER(filter), info);
    g_object_unref(info);
    g_object_unref(filter);
    printf("GTK workflows: reordered=%d replaced=%d shortcuts=%d file extensions=%d\n", reordered, replaced, remapped,
           filtered);
    return ok && reordered && replaced && remapped && filtered;
}
