/*
 * frontend_panels.h - Shared feature panel registry
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_PANELS_H
#define FRONTEND_PANELS_H

#include <stdbool.h>
#include <stddef.h>

enum { FRONTEND_PANEL_EXTENSION_BASE = 0x1000 };

typedef enum {
    FRONTEND_PANEL_TEXT,
    FRONTEND_PANEL_LIST,
    FRONTEND_PANEL_CHOICE,
    FRONTEND_PANEL_CHECKBOX,
    FRONTEND_PANEL_ACTION
} FrontendPanelControlType;

typedef enum {
    FRONTEND_PANEL_NEEDS_SESSION = 1u << 0,
    FRONTEND_PANEL_MODAL = 1u << 1
} FrontendPanelFlags;

typedef struct {
    unsigned id;
    FrontendPanelControlType type;
    const char *label;
    const char *value;
    const char *const *items;
    size_t item_count;
    int selected;
    bool enabled;
    bool read_only;
} FrontendPanelControl;

typedef struct {
    FrontendPanelControl *controls;
    size_t capacity;
    size_t count;
    const char *status;
} FrontendPanelModel;

typedef bool (*FrontendPanelSnapshot)(void *userdata, FrontendPanelModel *model,
                                     char *error, size_t error_size);
typedef bool (*FrontendPanelAction)(void *userdata, unsigned control_id,
                                   const char *value, int selected,
                                   char *error, size_t error_size);

typedef struct {
    unsigned id;
    const char *title;
    const char *category;
    unsigned flags;
    FrontendPanelSnapshot snapshot;
    FrontendPanelAction action;
    void *userdata;
} FrontendPanelSpec;

typedef struct {
    unsigned id;
    const char *title;
    const char *category;
    unsigned flags;
    bool enabled;
} FrontendPanelInfo;

void frontend_panels_reset(void);
bool frontend_panel_register(const FrontendPanelSpec *spec);
bool frontend_panel_unregister(unsigned id);
bool frontend_panel_set_enabled(unsigned id, bool enabled);
bool frontend_panel_get(unsigned id, FrontendPanelInfo *info);
size_t frontend_panel_count(void);
bool frontend_panel_at(size_t index, FrontendPanelInfo *info);
bool frontend_panel_snapshot(unsigned id, FrontendPanelModel *model,
                             char *error, size_t error_size);
bool frontend_panel_action(unsigned id, unsigned control_id,
                           const char *value, int selected,
                           char *error, size_t error_size);
bool frontend_panel_add_control(FrontendPanelModel *model,
                                const FrontendPanelControl *control);

#endif
