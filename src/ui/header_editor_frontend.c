/*
 * header_editor_frontend.c - Cartridge header editor
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "header_editor_frontend.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "../media/header_editor.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static struct {
    HeaderEditor *editor;
    struct FrontendExecutionRuntime *execution;
    HeaderEditorMetadata draft;
    char fields[HEADER_FIELD_COUNT][32], status[256];
    bool loaded;
} panel;

static void refresh(void) {
    for (unsigned i = 0; i < HEADER_FIELD_COUNT; ++i) {
        snprintf(panel.fields[i], sizeof(panel.fields[i]), "%" PRIu64, panel.draft.value[i]);
    }
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    iNESHeader header;
    bool valid = header_editor_validate(panel.editor, &panel.draft, &header, panel.status, sizeof(panel.status));
    if (valid) {
        strcpy(panel.status, "Valid header and payload. Save a copy, then open it normally to check board support.");
    }
    FrontendPanelControl open = {HEADER_EDITOR_OPEN,
                                 FRONTEND_PANEL_FILE_OPEN,
                                 "Choose cartridge image",
                                 NULL,
                                 NULL,
                                 0,
                                 FRONTEND_OPEN_IMAGE,
                                 true,
                                 false};
    if (!frontend_panel_add_control(model, &open)) {
        return false;
    }
    for (unsigned i = 0; i < HEADER_FIELD_COUNT; ++i) {
        FrontendPanelControl control = {HEADER_EDITOR_FIELD_BASE + i,
                                        FRONTEND_PANEL_TEXT,
                                        header_editor_labels[i],
                                        panel.fields[i],
                                        NULL,
                                        0,
                                        0,
                                        panel.loaded,
                                        false};
        if (!frontend_panel_add_control(model, &control)) {
            return false;
        }
    }
    FrontendPanelControl reset = {
        HEADER_EDITOR_RESET, FRONTEND_PANEL_ACTION, "Revert draft", NULL, NULL, 0, 0, panel.loaded, false};
    FrontendPanelControl save = {HEADER_EDITOR_SAVE,
                                 FRONTEND_PANEL_FILE_SAVE,
                                 "Save edited copy (.nes)",
                                 NULL,
                                 NULL,
                                 0,
                                 FRONTEND_SAVE_MEMORY,
                                 valid,
                                 false};
    model->status = panel.status;
    return frontend_panel_add_control(model, &reset) && frontend_panel_add_control(model, &save);
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)context;
    (void)selected;
    if (id == HEADER_EDITOR_OPEN) {
        if (!header_editor_open(panel.editor, value, error, size)) {
            return false;
        }
        panel.loaded = header_editor_metadata(panel.editor, &panel.draft);
        refresh();
        return panel.loaded;
    }
    if (id == HEADER_EDITOR_RESET && panel.loaded) {
        header_editor_metadata(panel.editor, &panel.draft);
        refresh();
        return true;
    }
    if (id == HEADER_EDITOR_SAVE) {
        return header_editor_save_copy(panel.editor, &panel.draft, value, panel.execution, error, size);
    }
    if (panel.loaded && id >= HEADER_EDITOR_FIELD_BASE && id < HEADER_EDITOR_FIELD_BASE + HEADER_FIELD_COUNT && value &&
        *value) {
        uint64_t number = 0;
        for (const char *p = value; *p; ++p) {
            if (*p < '0' || *p > '9' || number > (UINT64_MAX - (unsigned)(*p - '0')) / 10) {
                if (error && size) {
                    snprintf(error, size, "Enter an unsigned decimal number without overflow");
                }
                return false;
            }
            number = number * 10 + (unsigned)(*p - '0');
        }
        panel.draft.value[id - HEADER_EDITOR_FIELD_BASE] = number;
        refresh();
        return true;
    }
    if (error && size) {
        snprintf(error, size, "Choose an image and a valid field");
    }
    return false;
}

bool header_editor_frontend_register(struct FrontendExecutionRuntime *execution) {
    if (panel.editor) {
        return false;
    }
    panel.editor = header_editor_create();
    panel.execution = execution;
    if (!panel.editor) {
        return false;
    }
    FrontendPanelSpec spec = {HEADER_EDITOR_PANEL, "iNES / NES 2.0 Header Editor", "Tools", 0, snapshot, action, NULL};
    if (frontend_panel_register(&spec)) {
        return true;
    }
    header_editor_destroy(panel.editor);
    memset(&panel, 0, sizeof(panel));
    return false;
}

void header_editor_frontend_unregister(void) {
    frontend_panel_unregister(HEADER_EDITOR_PANEL);
    header_editor_destroy(panel.editor);
    memset(&panel, 0, sizeof(panel));
}
