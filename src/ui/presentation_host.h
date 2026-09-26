/*
 * presentation_host.h - Application integration for presentation tools
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_PRESENTATION_HOST_H
#define CUPID_PRESENTATION_HOST_H
#include "history_frontend.h"
#include "presentation_tools.h"
#include "timing_frontend.h"

typedef struct {
    FrontendHistory *history;
    FrontendPresentationTools *tools;
    FrontendTiming timing;
    FrontendAudioRuntime *audio;
    FrontendSettings *settings;
    uint64_t previous_end, frequency;
    bool registered;
} FrontendPresentationHost;

bool frontend_presentation_host_init(FrontendPresentationHost *host, FrontendVideoRuntime *video,
                                     FrontendAudioRuntime *audio, FrontendExecutionRuntime *execution,
                                     void (*changed)(void *), void *context);
void frontend_presentation_host_shutdown(FrontendPresentationHost *host);
bool frontend_presentation_host_event(FrontendPresentationHost *host, const SDL_Event *event, char *error, size_t size);
void frontend_presentation_host_tick(FrontendPresentationHost *host);
/* Call before replacing or resetting a session, or clearing/reconfiguring rewind. */
void frontend_presentation_host_reset(FrontendPresentationHost *host);
/* Monotonic performance-counter boundaries for one completed host frame. Idle
 * and paused iterations should instead clear previous_end and reset timing. */
bool frontend_presentation_host_record(FrontendPresentationHost *host, uint64_t emulation_start, uint64_t render_start,
                                       uint64_t present_start, uint64_t present_end, uint64_t pacing_end);
#endif
