/*
 * overclock_frontend.c - Optional extra CPU time panel
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "overclock_frontend.h"
#include "frontend_panels.h"
#include "../system/execution_policy.h"
#include <stdio.h>

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendOverclock *frontend = context;
    NesOverclockConfig config = nes_overclock_config();
    NesOverclockConfig active = nes_overclock_active_config();
    if (!model || model->capacity < 5) {
        if (error && size) {
            snprintf(error, size, "Overclock panel needs five controls");
        }
        return false;
    }
    bool editable = nes_execution_allows_host_configuration();
    snprintf(frontend->postrender, sizeof(frontend->postrender), "%u", config.postrender_scanlines);
    snprintf(frontend->vblank, sizeof(frontend->vblank), "%u", config.vblank_scanlines);
    snprintf(frontend->status, sizeof(frontend->status),
             "Changes apply next frame. Active extra lines: %u before NMI, %u after NMI. "
             "This intentionally changes console timing.",
             active.enabled ? active.postrender_scanlines : 0, active.enabled ? active.vblank_scanlines : 0);
    model->controls[0] = (FrontendPanelControl){.id = 0x2c01,
                                                .type = FRONTEND_PANEL_CHECKBOX,
                                                .label = "Enable CPU overclock",
                                                .selected = config.enabled,
                                                .enabled = editable};
    model->controls[1] = (FrontendPanelControl){.id = 0x2c02,
                                                .type = FRONTEND_PANEL_TEXT,
                                                .label = "Extra post-render scanlines (0-1000)",
                                                .value = frontend->postrender,
                                                .enabled = editable};
    model->controls[2] = (FrontendPanelControl){.id = 0x2c03,
                                                .type = FRONTEND_PANEL_TEXT,
                                                .label = "Extra vblank scanlines (0-1000)",
                                                .value = frontend->vblank,
                                                .enabled = editable};
    model->controls[3] = (FrontendPanelControl){.id = 0x2c04,
                                                .type = FRONTEND_PANEL_CHECKBOX,
                                                .label = "Use normal timing after DMC playback or direct sample writes",
                                                .selected = config.dmc_compatibility,
                                                .enabled = editable};
    model->controls[4] = (FrontendPanelControl){
        .id = 0x2c05, .type = FRONTEND_PANEL_ACTION, .label = "Restore normal timing defaults", .enabled = editable};
    model->count = 5;
    model->status = frontend->status;
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool parse_lines(const char *text, uint16_t *lines) {
    if (!text || !*text) {
        return false;
    }
    unsigned count = 0;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9') {
            return false;
        }
        count = count * 10u + (unsigned)(*text - '0');
        if (count > NES_OVERCLOCK_MAX_SCANLINES) {
            return false;
        }
    }
    *lines = (uint16_t)count;
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendOverclock *frontend = context;
    NesOverclockConfig config = nes_overclock_config();
    bool valid = true;
    switch (id) {
    case 0x2c01:
        valid = selected == 0 || selected == 1;
        config.enabled = selected != 0;
        break;
    case 0x2c02:
        valid = parse_lines(value, &config.postrender_scanlines);
        break;
    case 0x2c03:
        valid = parse_lines(value, &config.vblank_scanlines);
        break;
    case 0x2c04:
        valid = selected == 0 || selected == 1;
        config.dmc_compatibility = selected != 0;
        break;
    case 0x2c05:
        config = (NesOverclockConfig){false, 0, 0, true};
        break;
    default:
        valid = false;
        break;
    }
    if (!valid || !nes_set_overclock_config(&config)) {
        if (error && size) {
            snprintf(error, size, "%s",
                     valid ? "Stop the replay or network session before changing timing"
                           : "Enter a whole number from 0 through 1000");
        }
        return false;
    }
    if (frontend->changed) {
        frontend->changed(frontend->userdata, &config);
    }
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool frontend_overclock_register(FrontendOverclock *frontend) {
    if (!frontend) {
        return false;
    }
    FrontendPanelSpec spec = {FRONTEND_OVERCLOCK_PANEL, "CPU Overclock", "Options", 0, snapshot, action, frontend};
    return frontend_panel_register(&spec);
}

void frontend_overclock_unregister(void) {
    (void)frontend_panel_unregister(FRONTEND_OVERCLOCK_PANEL);
}
