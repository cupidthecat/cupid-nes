/*
 * frontend_panels.c - Shared feature panel registry
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "frontend_panels.h"
#include <stdio.h>
#include <string.h>

enum { PANEL_CAPACITY = 48, PANEL_TEXT = 64 };

typedef struct {
    bool used;
    unsigned id;
    char title[PANEL_TEXT];
    char category[PANEL_TEXT];
    unsigned flags;
    bool enabled;
    FrontendPanelSnapshot snapshot;
    FrontendPanelAction action;
    void *userdata;
} PanelEntry;

static PanelEntry panels[PANEL_CAPACITY];
static size_t panel_count;

static PanelEntry *find_panel(unsigned id) {
    for (size_t i = 0; i < PANEL_CAPACITY; ++i)
        if (panels[i].used && panels[i].id == id) return &panels[i];
    return NULL;
}

static bool copy_text(char *dst, size_t size, const char *src) {
    if (!src || strlen(src) >= size) return false;
    strcpy(dst, src);
    return true;
}

static void export_info(const PanelEntry *entry, FrontendPanelInfo *info) {
    info->id = entry->id;
    info->title = entry->title;
    info->category = entry->category;
    info->flags = entry->flags;
    info->enabled = entry->enabled;
}

void frontend_panels_reset(void) {
    memset(panels, 0, sizeof(panels));
    panel_count = 0;
}

bool frontend_panel_register(const FrontendPanelSpec *spec) {
    if (!spec || !spec->id || !spec->title || !spec->snapshot || find_panel(spec->id))
        return false;
    PanelEntry *entry = NULL;
    for (size_t i = 0; i < PANEL_CAPACITY; ++i) {
        if (!panels[i].used) {
            entry = &panels[i];
            break;
        }
    }
    if (!entry || !copy_text(entry->title, sizeof(entry->title), spec->title)
        || !copy_text(entry->category, sizeof(entry->category),
                      spec->category ? spec->category : "Tools")) {
        if (entry) memset(entry, 0, sizeof(*entry));
        return false;
    }
    entry->used = true;
    entry->id = spec->id;
    entry->flags = spec->flags;
    entry->enabled = true;
    entry->snapshot = spec->snapshot;
    entry->action = spec->action;
    entry->userdata = spec->userdata;
    ++panel_count;
    return true;
}

bool frontend_panel_unregister(unsigned id) {
    PanelEntry *entry = find_panel(id);
    if (!entry) return false;
    memset(entry, 0, sizeof(*entry));
    --panel_count;
    return true;
}

bool frontend_panel_set_enabled(unsigned id, bool enabled) {
    PanelEntry *entry = find_panel(id);
    if (!entry) return false;
    entry->enabled = enabled;
    return true;
}

bool frontend_panel_get(unsigned id, FrontendPanelInfo *info) {
    PanelEntry *entry = find_panel(id);
    if (!entry || !info) return false;
    export_info(entry, info);
    return true;
}

size_t frontend_panel_count(void) {
    return panel_count;
}

bool frontend_panel_at(size_t index, FrontendPanelInfo *info) {
    if (!info) return false;
    size_t seen = 0;
    for (size_t i = 0; i < PANEL_CAPACITY; ++i) {
        if (!panels[i].used) continue;
        if (seen++ == index) {
            export_info(&panels[i], info);
            return true;
        }
    }
    return false;
}

bool frontend_panel_snapshot(unsigned id, FrontendPanelModel *model,
                             char *error, size_t error_size) {
    PanelEntry *entry = find_panel(id);
    if (!entry || !entry->enabled || !model || !model->controls) {
        if (error && error_size) snprintf(error, error_size, "Panel is unavailable");
        return false;
    }
    model->count = 0;
    model->status = NULL;
    if (error && error_size) error[0] = '\0';
    return entry->snapshot(entry->userdata, model, error, error_size);
}

bool frontend_panel_action(unsigned id, unsigned control_id,
                           const char *value, int selected,
                           char *error, size_t error_size) {
    PanelEntry *entry = find_panel(id);
    if (!entry || !entry->enabled || !entry->action) {
        if (error && error_size) snprintf(error, error_size, "Panel action is unavailable");
        return false;
    }
    if (error && error_size) error[0] = '\0';
    return entry->action(entry->userdata, control_id, value, selected, error, error_size);
}

bool frontend_panel_add_control(FrontendPanelModel *model,
                                const FrontendPanelControl *control) {
    if (!model || !control || !model->controls || model->count >= model->capacity)
        return false;
    model->controls[model->count++] = *control;
    return true;
}
