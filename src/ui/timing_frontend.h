/*
 * timing_frontend.h - Host frame statistics panel
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_TIMING_FRONTEND_H
#define CUPID_TIMING_FRONTEND_H
#include "../video/frame_timing.h"

typedef struct {
    NesFrameTiming timing;
    char lines[8][192];
    const char *items[8];
    bool (*draw_stats)(void *context, NesFrameTimingSummary *summary, bool reset);
    void *draw_context;
} FrontendTiming;

bool frontend_timing_register(FrontendTiming *frontend);
void frontend_timing_unregister(void);
void frontend_timing_reset(FrontendTiming *frontend);
#endif
