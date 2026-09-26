/*
 * presentation_host.c - Application integration for presentation tools
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "presentation_host.h"
#include <string.h>

bool frontend_presentation_host_init(FrontendPresentationHost *host, FrontendVideoRuntime *video,
                                     FrontendAudioRuntime *audio, FrontendExecutionRuntime *execution,
                                     void (*changed)(void *), void *context) {
    if (!host || !video || !audio || !execution) {
        return false;
    }
    memset(host, 0, sizeof(*host));
    host->audio = audio;
    host->settings = video->settings;
    host->frequency = SDL_GetPerformanceFrequency();
    host->history = frontend_history_create(execution);
    host->tools = frontend_presentation_tools_create(video, audio, changed, context);
    host->registered = frontend_timing_register(&host->timing);
    if (!host->history || !host->tools || !host->registered) {
        frontend_presentation_host_shutdown(host);
        return false;
    }
    return true;
}

void frontend_presentation_host_shutdown(FrontendPresentationHost *host) {
    if (!host) {
        return;
    }
    frontend_history_destroy(host->history);
    frontend_presentation_tools_destroy(host->tools);
    if (host->registered) {
        frontend_timing_unregister();
    }
    memset(host, 0, sizeof(*host));
}

bool frontend_presentation_host_event(FrontendPresentationHost *host, const SDL_Event *event, char *error,
                                      size_t size) {
    if (!host || !event) {
        return false;
    }
    if (frontend_history_event(host->history, event) || frontend_presentation_tools_event(host->tools, event)) {
        return true;
    }
    if (event->type == SDL_AUDIODEVICEREMOVED) {
        (void)frontend_audio_runtime_handle_device_event(host->audio, host->settings, &event->adevice, error, size);
        return true;
    }
    return false;
}

void frontend_presentation_host_tick(FrontendPresentationHost *host) {
    if (!host) {
        return;
    }
    frontend_history_tick(host->history);
    frontend_presentation_tools_tick(host->tools);
}

void frontend_presentation_host_reset(FrontendPresentationHost *host) {
    if (!host) {
        return;
    }
    frontend_history_close(host->history);
    nes_frame_timing_reset(&host->timing.timing);
    host->previous_end = 0;
}

bool frontend_presentation_host_record(FrontendPresentationHost *host, uint64_t emulation_start, uint64_t render_start,
                                       uint64_t present_start, uint64_t present_end, uint64_t pacing_end) {
    if (!host || !host->frequency || emulation_start > render_start || render_start > present_start ||
        present_start > present_end || present_end > pacing_end) {
        return false;
    }
    NesHostFrameSample sample = {0};
    uint64_t boundaries[] = {emulation_start, render_start, present_start, present_end, pacing_end};
    for (unsigned i = 0; i < 4; ++i) {
        sample.milliseconds[i] = nes_frame_timing_elapsed(boundaries[i], boundaries[i + 1], host->frequency);
    }
    uint64_t begin = host->previous_end && host->previous_end <= pacing_end ? host->previous_end : emulation_start;
    sample.milliseconds[NES_HOST_INTERVAL] = nes_frame_timing_elapsed(begin, pacing_end, host->frequency);
    host->previous_end = pacing_end;
    return nes_frame_timing_push(&host->timing.timing, &sample);
}
