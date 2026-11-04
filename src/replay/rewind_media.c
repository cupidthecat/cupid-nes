/*
 * rewind_media.c - Bounded input and audio retained with rewind boundaries
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "rewind.h"
#include "../system/execution_policy.h"
#include <stdlib.h>
#include <string.h>

enum { MAX_PENDING = 256, MAX_AUDIO_FRAMES = 8192 };
static NesRewindHistory *tracked;
static NesInputEvent pending[MAX_PENDING];
static size_t pending_count;
static bool overflow;

static void observe(const NesInputEvent *event, void *context) {
    (void)context;
    if (!tracked || !tracked->capacity || nes_execution_policy() != NES_EXECUTION_LIVE) return;
    if (pending_count < MAX_PENDING) pending[pending_count++] = *event;
    else overflow = true;
}

void nes_rewind_track_inputs(NesRewindHistory *history) {
    tracked = history;
    pending_count = 0;
    overflow = false;
    nes_input_event_set_listener(history ? observe : NULL, NULL);
}

void nes_rewind_reset_inputs(NesRewindHistory *history) {
    if (tracked == history) { pending_count = 0; overflow = false; }
}

void nes_rewind_untrack_inputs(NesRewindHistory *history) {
    if (tracked == history) nes_rewind_track_inputs(NULL);
}

void nes_rewind_free_media(NesRewindHistory *history, size_t slot) {
    if (!history || !history->media || slot >= history->capacity) return;
    NesRewindMedia *media = &history->media[slot];
    size_t bytes = media->event_count * sizeof(*media->events) + media->audio_frames * 2u * sizeof(float);
    if (bytes <= history->total_bytes) history->total_bytes -= bytes;
    free(media->events);
    free(media->audio);
    memset(media, 0, sizeof(*media));
}

void nes_rewind_capture_inputs(NesRewindHistory *history, size_t slot) {
    if (!history || !history->media || slot >= history->capacity) return;
    NesRewindMedia *media = &history->media[slot];
    media->inputs_valid = tracked == history && !overflow;
    size_t bytes = pending_count * sizeof(*pending);
    if (media->inputs_valid && bytes) {
        if (bytes > history->max_bytes - history->total_bytes || !(media->events = malloc(bytes))) {
            media->inputs_valid = false;
        } else {
            memcpy(media->events, pending, bytes);
            media->event_count = pending_count;
            history->total_bytes += bytes;
        }
    }
    nes_rewind_reset_inputs(history);
}

const NesRewindMedia *nes_rewind_media(const NesRewindHistory *history, size_t index) {
    if (!history || !history->media || !history->capacity || index >= history->count) return NULL;
    return &history->media[(history->head + index) % history->capacity];
}

void nes_rewind_complete(NesRewindHistory *history, bool completed) {
    if (history && history->count && history->media)
        history->media[(history->head + history->count - 1) % history->capacity].complete = completed;
}

void nes_rewind_append_audio(NesRewindHistory *history, const float *stereo, unsigned frames, unsigned rate) {
    if (!history || !history->count || !history->media || !stereo || !frames || rate < 8000 || rate > 192000) return;
    NesRewindMedia *media = &history->media[(history->head + history->count - 1) % history->capacity];
    if (!media->complete || frames > MAX_AUDIO_FRAMES - media->audio_frames ||
        (media->audio_rate && media->audio_rate != rate)) return;
    size_t added = frames * 2u * sizeof(float);
    if (added > history->max_bytes - history->total_bytes) return;
    float *audio = realloc(media->audio, (media->audio_frames + frames) * 2u * sizeof(float));
    if (!audio) return;
    memcpy(audio + media->audio_frames * 2u, stereo, added);
    media->audio = audio;
    media->audio_frames += frames;
    media->audio_rate = rate;
    history->total_bytes += added;
}

void nes_rewind_truncate(NesRewindHistory *history, size_t count) {
    if (!history || count > history->count || !history->capacity) return;
    while (history->count > count) {
        size_t slot = (history->head + history->count - 1) % history->capacity;
        nes_rewind_free_media(history, slot);
        history->total_bytes -= history->entries[slot].size;
        nes_state_blob_free(&history->entries[slot]);
        --history->count;
    }
    if (!history->count) history->head = 0;
    nes_rewind_reset_inputs(history);
}
