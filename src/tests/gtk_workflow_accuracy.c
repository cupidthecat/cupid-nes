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

static bool fair_refresh(FrontendDesktopUi *ui) {
    GList *visible = NULL;
    for (CupidGtkTool *t = ui->gtk->tools; t; t = t->next) {
        if (gtk_widget_get_visible(t->window)) {
            visible = g_list_prepend(visible, t);
        }
        gtk_widget_set_visible(t->window, FALSE);
    }
    FrontendDesktopUi *models[] = {cupid_gtk_open(ui, 2, 0), cupid_gtk_open(ui, 2, 1), cupid_gtk_open(ui, 0, 0)};
    CupidGtkTool *tools[3] = {0};
    uint32_t stale = SDL_GetTicks() - 100;
    for (CupidGtkTool *t = ui->gtk->tools; t; t = t->next) {
        for (unsigned i = 0; i < G_N_ELEMENTS(models); ++i) {
            if (&t->ui == models[i]) {
                tools[i] = t;
                t->refreshed = stale;
            }
        }
    }
    bool ok = tools[0] && tools[1] && tools[2];
    ui->gtk->refresh_next = tools[0];
    for (unsigned frame = 1; ok && frame <= 3; ++frame) {
        frontend_desktop_render(ui, 256, 240, "Refresh fixture", "NTSC", "Paused");
        unsigned refreshed = 0;
        for (unsigned i = 0; i < 3; ++i) {
            refreshed += tools[i]->refreshed != stale;
        }
        ok = refreshed == frame;
    }
    for (CupidGtkTool *t = ui->gtk->tools; t; t = t->next) {
        gtk_widget_set_visible(t->window, g_list_find(visible, t) != NULL);
    }
    g_list_free(visible);
    return ok;
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

    /* Selecting a numbered slot must work without opening Settings. */
    unsigned previous_slot = ui->settings->state_slot;
    frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
    menu = gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar));
    char *slot_action = menu_attribute(menu, "Slot 4", G_MENU_ATTRIBUTE_ACTION);
    const char *slot_name = slot_action ? strchr(slot_action, '.') : NULL;
    bool slots = slot_name != NULL;
    if (slot_name) {
        g_action_group_activate_action(G_ACTION_GROUP(ui->gtk->actions), slot_name + 1, NULL);
        slots &= ui->settings->state_slot == 3;
        ui->gtk->refreshed = SDL_GetTicks() - 100;
        frontend_desktop_render(ui, 256, 240, "Workflow fixture", "NTSC", "Paused");
        GVariant *checked = g_action_group_get_action_state(G_ACTION_GROUP(ui->gtk->actions), slot_name + 1);
        slots &= checked && g_variant_get_boolean(checked);
        g_clear_pointer(&checked, g_variant_unref);
    }
    g_free(slot_action);
    ui->settings->state_slot = previous_slot;

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
    bool fair = fair_refresh(ui);
    printf("GTK workflows: reordered=%d replaced=%d shortcuts=%d file extensions=%d fair refresh=%d slots=%d\n", reordered,
           replaced, remapped, filtered, fair, slots);
    return ok && reordered && replaced && remapped && filtered && fair && slots;
}
