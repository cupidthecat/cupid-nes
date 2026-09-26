/*
 * timing_frontend.c - Host frame statistics panel
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "timing_frontend.h"
#include "frontend_panels.h"
#include <stdio.h>
#include <string.h>

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendTiming *frontend = context;
    NesFrameTimingSummary summary;
    if (!model || model->capacity < 2 || !nes_frame_timing_summary(&frontend->timing, &summary)) {
        if (error && size) {
            snprintf(error, size, "Frame timing panel has insufficient storage");
        }
        return false;
    }
    static const char *const names[] = {"Emulation", "Render", "Present / VSync", "Pacing delay", "Frame interval"};
    for (size_t i = 0; i < 5; ++i) {
        const NesHostMetric *m = &summary.metrics[i];
        snprintf(frontend->lines[i], sizeof(frontend->lines[i]),
                 "%s: %.3f ms | avg %.3f | min %.3f | max %.3f | p95 %.3f", names[i], m->recent, m->average, m->minimum,
                 m->maximum, m->p95);
    }
    snprintf(frontend->lines[5], sizeof(frontend->lines[5]), "Effective FPS: %.2f | %zu recent host frames",
             summary.fps, summary.count);
    if (summary.audio_available) {
        snprintf(frontend->lines[6], sizeof(frontend->lines[6]), "Audio queue: %.2f ms | underruns: %llu",
                 summary.audio_queue_ms, (unsigned long long)summary.audio_underruns);
    } else {
        snprintf(frontend->lines[6], sizeof(frontend->lines[6]),
                 "Audio queue / underruns: unavailable for callback output");
    }
    for (size_t i = 0; i < 7; ++i) {
        frontend->items[i] = frontend->lines[i];
    }
    model->controls[0] = (FrontendPanelControl){.id = 0x2721,
                                                .type = FRONTEND_PANEL_LIST,
                                                .label = "Host wall-clock time (milliseconds)",
                                                .items = frontend->items,
                                                .item_count = 7,
                                                .enabled = true,
                                                .read_only = true};
    model->controls[1] = (FrontendPanelControl){
        .id = 0x2722, .type = FRONTEND_PANEL_ACTION, .label = "Reset measurements", .enabled = true};
    model->count = 2;
    model->status = "Latest 240 samples; hardware CPU/PPU cycle timing is unchanged.";
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)value;
    (void)selected;
    if (id != 0x2722) {
        return false;
    }
    nes_frame_timing_reset(&((FrontendTiming *)context)->timing);
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool frontend_timing_register(FrontendTiming *frontend) {
    if (!frontend) {
        return false;
    }
    memset(frontend, 0, sizeof(*frontend));
    FrontendPanelSpec spec = {0x2720, "Frame Timing Statistics", "Tools", 0, snapshot, action, frontend};
    return frontend_panel_register(&spec);
}

void frontend_timing_unregister(void) {
    (void)frontend_panel_unregister(0x2720);
}
