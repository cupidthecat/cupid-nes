/*
 * overclock_frontend.h - Optional extra CPU time controls
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_OVERCLOCK_FRONTEND_H
#define CUPID_OVERCLOCK_FRONTEND_H
#include "../system/timing.h"

enum { FRONTEND_OVERCLOCK_PANEL = 0x2c00 };

typedef struct {
    char postrender[16], vblank[16], status[192];
    void (*changed)(void *userdata, const NesOverclockConfig *config);
    void *userdata;
} FrontendOverclock;

/* Caller owns the context until unregister. The optional callback persists
 * accepted preferences; state restoration does not invoke it. */
bool frontend_overclock_register(FrontendOverclock *frontend);
void frontend_overclock_unregister(void);
#endif
