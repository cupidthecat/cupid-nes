/*
 * history_frontend.c - Visual rewind timeline controls
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "history_frontend.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include "../debugger/debugger.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include <math.h>
#include "../util/file_io.h"
#include "../video/history_view.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { HISTORY_SUSPEND = 1u << 8 };

struct FrontendHistory {
    FrontendExecutionRuntime *execution;
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
    size_t index, count, head, bytes, audio_cursor;
    const void *entries, *last;
    uint64_t session, revision;
    double next_tick, fps;
    size_t range_first, range_last, bookmarks[HISTORY_BOOKMARKS];
    unsigned volume;
    SDL_AudioDeviceID audio_device;
    unsigned audio_rate;
    unsigned width;
    bool playing, active, native, audio_failed;
    uint32_t pixels[512 * 240];
    char position[96], status[256];
};

static bool fail(FrontendHistory *history, char *error, size_t size, const char *message) {
    snprintf(history->status, sizeof(history->status), "%s", message);
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

static void stop_audio(FrontendHistory *history) {
    if (history->audio_device) SDL_ClearQueuedAudio(history->audio_device);
    history->audio_cursor = history->index;
}

static void queue_audio(FrontendHistory *history, size_t index) {
    const NesRewindMedia *media = nes_rewind_media(&history->execution->rewind, index);
    if (!media || !media->audio_frames || !history->volume || history->audio_failed) return;
    if (history->audio_device && history->audio_rate != media->audio_rate) {
        SDL_CloseAudioDevice(history->audio_device);
        history->audio_device = 0;
    }
    if (!history->audio_device) {
        SDL_AudioSpec want = {0};
        want.freq = (int)media->audio_rate;
        want.format = AUDIO_F32SYS;
        want.channels = 2;
        want.samples = 256;
        history->audio_device = SDL_OpenAudioDevice(NULL, 0, &want, NULL, 0);
        history->audio_rate = media->audio_rate;
    }
    if (!history->audio_device) { history->audio_failed = true; return; }
    if (SDL_GetQueuedAudioSize(history->audio_device) > media->audio_rate * 2u * sizeof(float) / 4u) return;
    float samples[512];
    for (unsigned i = 0; i < media->audio_frames;) {
        unsigned n = media->audio_frames - i;
        if (n > 256) n = 256;
        for (unsigned j = 0; j < n * 2; ++j) samples[j] = media->audio[i * 2 + j] * history->volume / 100.f;
        if (SDL_QueueAudio(history->audio_device, samples, n * 2u * sizeof(float)) != 0) {
            history->audio_failed = true;
            return;
        }
        i += n;
    }
    SDL_PauseAudioDevice(history->audio_device, 0);
}

static void fill_audio(FrontendHistory *history) {
    /* All samples are already retained. Queue a short lookahead so a slow
     * preview restore or GTK draw cannot starve the history output device. */
    size_t end = history->index + 4;
    if (end > history->count) end = history->count;
    while (history->audio_cursor < end) queue_audio(history, history->audio_cursor++);
}

void frontend_history_close(FrontendHistory *history) {
    if (!history) {
        return;
    }
    if (history->audio_device) SDL_CloseAudioDevice(history->audio_device);
    history->audio_device = 0;
    history->audio_failed = false;
    history->active = false;
    SDL_DestroyTexture(history->texture);
    SDL_DestroyRenderer(history->renderer);
    SDL_DestroyWindow(history->window);
    history->texture = NULL;
    history->renderer = NULL;
    history->window = NULL;
    history->playing = false;
    frontend_execution_suspend(history->execution, HISTORY_SUSPEND, false);
}

static bool current(const FrontendHistory *history) {
    const NesRewindHistory *ring = &history->execution->rewind;
    return history->active && nes_execution_policy() == NES_EXECUTION_LIVE &&
           history->session == debugger_session_revision() && ring->entries == history->entries &&
           ring->count == history->count && ring->head == history->head && ring->total_bytes == history->bytes &&
           ring->count && ring->entries[(ring->head + ring->count - 1) % ring->capacity].data == history->last;
}

static bool seek(FrontendHistory *history, size_t index, char *error, size_t size) {
    if (!current(history) || index >= history->count) {
        return fail(history, error, size, "History changed; reopen the viewer");
    }
    NesStateResult state;
    frontend_execution_begin_machine_change(history->execution);
    NesReplayResult result =
        nes_history_preview(&history->execution->rewind, index, history->pixels, 512u * 240u, &history->width, &state);
    frontend_execution_end_machine_change_preserving_audio(history->execution);
    if (result != NES_REPLAY_OK) {
        return fail(history, error, size,
                    result == NES_REPLAY_STATE_ERROR ? nes_state_result_string(state)
                                                     : nes_replay_result_string(result));
    }
    history->index = index;
    if (!history->native && !history->texture) {
        history->texture = SDL_CreateTexture(history->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                             (int)history->width, 240);
    }
    if (!history->native && (!history->texture ||
        SDL_UpdateTexture(history->texture, NULL, history->pixels, (int)(history->width * sizeof(uint32_t))) != 0)) {
        return fail(history, error, size, SDL_GetError());
    }
    snprintf(history->position, sizeof(history->position), "%zu", index + 1);
    snprintf(history->status, sizeof(history->status), "Frame %zu / %zu%s",
             index + 1, history->count, history->audio_failed ? "; audio output unavailable" : "");
    if (history->window) SDL_SetWindowTitle(history->window, history->status);
    ++history->revision;
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool open_view(FrontendHistory *history, char *error, size_t size) {
    if (current(history)) {
        if (history->window) SDL_RaiseWindow(history->window);
        return true;
    }
    frontend_history_close(history);
    NesRewindHistory *ring = &history->execution->rewind;
    if (nes_execution_policy() != NES_EXECUTION_LIVE || !nes_replay_host_state_supported() || !ring->count) {
        return fail(history, error, size, "History requires live execution and retained rewind frames");
    }
    if (!history->native) history->window = SDL_CreateWindow("Rewind History", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 768, 740,
                                       SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (history->window) {
        history->renderer = SDL_CreateRenderer(history->window, -1, 0);
    }
    if (!history->native && !history->renderer) {
        fail(history, error, size, SDL_GetError());
        frontend_history_close(history);
        return false;
    }
    history->active = true;
    history->fps = nes_timing()->fps;
    frontend_execution_suspend(history->execution, HISTORY_SUSPEND, true);
    history->session = debugger_session_revision();
    history->entries = ring->entries;
    history->count = ring->count;
    history->head = ring->head;
    history->bytes = ring->total_bytes;
    history->last = ring->entries[(ring->head + ring->count - 1) % ring->capacity].data;
    history->range_first = 0;
    history->range_last = ring->count - 1;
    for (unsigned i = 0; i < HISTORY_BOOKMARKS; ++i) history->bookmarks[i] = SIZE_MAX;
    if (!seek(history, 0, error, size)) {
        frontend_history_close(history);
        return false;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    FrontendHistory *history = context;
    if (id == 0x2701) {
        return open_view(history, error, size);
    }
    if (id == 0x2708) {
        frontend_history_close(history);
        return true;
    }
    if (!current(history)) {
        return fail(history, error, size, "Open the history viewer first");
    }
    if (id == HISTORY_VOLUME) {
        if (!history->native) selected *= 25;
        if (selected < 0 || selected > 100) return fail(history, error, size, "Volume must be 0–100");
        history->volume = (unsigned)selected;
        stop_audio(history);
        if (history->playing) fill_audio(history);
        return true;
    }
    if (id == HISTORY_BOOKMARK_SET || id == HISTORY_BOOKMARK_JUMP) {
        if (selected < 0 || selected >= HISTORY_BOOKMARKS) return false;
        if (id == HISTORY_BOOKMARK_SET) {
            history->bookmarks[selected] = history->index;
            ++history->revision;
            return true;
        }
        if (history->bookmarks[selected] == SIZE_MAX) return fail(history, error, size, "Bookmark is empty");
        history->playing = false;
        stop_audio(history);
        return seek(history, history->bookmarks[selected], error, size);
    }
    if (id == HISTORY_EXPORT) {
        if (!value || !*value || !frontend_output_path_allowed(value, history->execution, NULL, 0, error, size)) return false;
        NesMovieResult result = nes_movie_export_history(&history->execution->rewind,
            history->range_first, history->range_last + 1, value);
        if (result != NES_MOVIE_OK) return fail(history, error, size, nes_movie_result_string(result));
        snprintf(history->status, sizeof(history->status), "Movie exported (%zu frames)",
            history->range_last - history->range_first + 1);
        return true;
    }
    if (id == HISTORY_RANGE_FIRST || id == HISTORY_RANGE_LAST) {
        size_t first = id == HISTORY_RANGE_FIRST ? history->index : history->range_first;
        size_t last = id == HISTORY_RANGE_LAST ? history->index : history->range_last;
        if (first > last) return fail(history, error, size, "Range start must precede range end");
        history->range_first = first;
        history->range_last = last;
        ++history->revision;
        return true;
    }
    if (id == HISTORY_POSITION) {
        char *end = NULL;
        errno = 0;
        unsigned long long number = value ? strtoull(value, &end, 10) : 0;
        if (!value || value[0] == '-' || errno || end == value || *end || !number || number > history->count) {
            return fail(history, error, size, "Position must name a retained frame, starting at 1");
        }
        history->playing = false;
        stop_audio(history);
        return seek(history, (size_t)number - 1, error, size);
    }
    if (id == HISTORY_PREVIOUS || id == HISTORY_NEXT || id == HISTORY_FIRST || id == HISTORY_LAST) {
        size_t index = history->index;
        if (id == 0x2703 && index) {
            --index;
        }
        if (id == 0x2704 && index + 1 < history->count) {
            ++index;
        }
        if (id == HISTORY_FIRST) index = 0;
        if (id == HISTORY_LAST) index = history->count - 1;
        history->playing = false;
        stop_audio(history);
        return seek(history, index, error, size);
    }
    if (id == 0x2705) {
        stop_audio(history);
        if (!history->playing && history->index + 1 == history->count && !seek(history, 0, error, size)) return false;
        history->playing = !history->playing;
        if (history->playing) history->audio_failed = false;
        history->next_tick = (double)SDL_GetPerformanceCounter() + (double)SDL_GetPerformanceFrequency() / history->fps;
        if (history->playing) {
            history->audio_cursor = history->index;
            fill_audio(history);
        }
        return true;
    }
    if (id == 0x2706) {
        if (!value || !*value) {
            return fail(history, error, size, "Choose a state path separate from game, firmware, and session files");
        }
        if (!frontend_output_path_allowed(value, history->execution, NULL, 0, error, size)) {
            return false;
        }
        frontend_execution_begin_machine_change(history->execution);
        NesStateResult result = nes_history_save(&history->execution->rewind, history->index, value);
        frontend_execution_end_machine_change_preserving_audio(history->execution);
        return result == NES_STATE_OK || fail(history, error, size, nes_state_result_string(result));
    }
    if (id == 0x2707) {
        if (history->execution->before_machine_change &&
            !history->execution->before_machine_change(history->execution->machine_change_context, error, size)) {
            return false;
        }
        NesStateResult state;
        frontend_execution_begin_machine_change(history->execution);
        NesReplayResult result = nes_history_resume(&history->execution->rewind, history->index, &state);
        frontend_execution_end_machine_change(history->execution);
        if (result != NES_REPLAY_OK) {
            return fail(history, error, size, nes_replay_result_string(result));
        }
        frontend_execution_timeline_restored(history->execution);
        frontend_history_close(history);
        execution_control_set_paused(&history->execution->execution, false);
        frontend_execution_refresh_audio(history->execution);
        return true;
    }
    return fail(history, error, size, "Unknown history action");
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendHistory *history = context;
    if (!model || model->capacity < 14) {
        return fail(history, error, size, "History panel needs fourteen controls");
    }
    bool active = current(history);
    static const char *const labels[] = {"Open video timeline",
                                         "Position (1 = oldest)",
                                         "Previous frame",
                                         "Next frame",
                                         "Play / pause history",
                                         "Save selected state",
                                         "Resume from selection",
                                         "Close viewer",
                                         "Export selected range as movie",
                                         "First frame", "Last frame", "Set range start", "Set range end", "Playback volume"};
    for (unsigned i = 0; i < 14; ++i) {
        model->controls[i] = (FrontendPanelControl){.id = 0x2701 + i,
                                                    .type = FRONTEND_PANEL_ACTION,
                                                    .label = labels[i],
                                                    .enabled = i == 0 || active};
    }
    model->controls[1].type = FRONTEND_PANEL_TEXT;
    model->controls[1].value = history->position;
    model->controls[5].type = FRONTEND_PANEL_FILE_SAVE;
    model->controls[5].selected = FRONTEND_SAVE_STATE;
    model->controls[8].type = FRONTEND_PANEL_FILE_SAVE;
    model->controls[8].selected = FRONTEND_SAVE_MOVIE;
    model->controls[13].type = FRONTEND_PANEL_CHOICE;
    static const char *const volumes[] = {"Muted", "25%", "50%", "75%", "100%"};
    model->controls[13].items = volumes;
    model->controls[13].item_count = 5;
    model->controls[13].selected = (int)history->volume / 25;
    model->count = 14;
    model->status = history->status;
    return true;
}

FrontendHistory *frontend_history_create(FrontendExecutionRuntime *execution) {
    if (!execution || execution->history_viewer) {
        return NULL;
    }
    FrontendHistory *history = calloc(1, sizeof(*history));
    if (!history) {
        return NULL;
    }
    history->execution = execution;
    history->volume = 100;
    execution->history_viewer = history;
    FrontendPanelSpec spec = {0x2700,   "History Viewer", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                              snapshot, action,           history};
    if (!frontend_panel_register(&spec)) {
        execution->history_viewer = NULL;
        free(history);
        return NULL;
    }
    return history;
}

void frontend_history_destroy(FrontendHistory *history) {
    if (!history) {
        return;
    }
    frontend_history_close(history);
    history->execution->history_viewer = NULL;
    (void)frontend_panel_unregister(0x2700);
    free(history);
}

bool frontend_history_event(FrontendHistory *history, const SDL_Event *event) {
    if (!history || !history->window || !event) {
        return false;
    }
    Uint32 window = SDL_GetWindowID(history->window);
    if (event->type == SDL_WINDOWEVENT && event->window.windowID == window) {
        if (event->window.event == SDL_WINDOWEVENT_CLOSE) {
            frontend_history_close(history);
        }
        return true;
    }
    if (event->type == SDL_KEYDOWN && event->key.windowID == window) {
        unsigned id = event->key.keysym.sym == SDLK_LEFT     ? 0x2703
                      : event->key.keysym.sym == SDLK_RIGHT  ? 0x2704
                      : event->key.keysym.sym == SDLK_SPACE  ? 0x2705
                      : event->key.keysym.sym == SDLK_ESCAPE ? 0x2708
                                                             : 0;
        if (id) {
            (void)action(history, id, NULL, 0, NULL, 0);
        }
        return true;
    }
    if ((event->type == SDL_MOUSEBUTTONDOWN && event->button.windowID == window) ||
        (event->type == SDL_MOUSEMOTION && event->motion.windowID == window &&
         (event->motion.state & SDL_BUTTON_LMASK))) {
        int w, h;
        SDL_GetWindowSize(history->window, &w, &h);
        int x = event->type == SDL_MOUSEMOTION ? event->motion.x : event->button.x;
        int y = event->type == SDL_MOUSEMOTION ? event->motion.y : event->button.y;
        if (w > 0 && y >= h - 28 && x >= 0 && x < w && current(history)) {
            history->playing = false;
            stop_audio(history);
            (void)seek(history, (size_t)x * history->count / (size_t)w, NULL, 0);
        }
        return true;
    }
    return false;
}

void frontend_history_tick(FrontendHistory *history) {
    if (!history || !history->active) {
        return;
    }
    if (!current(history)) {
        frontend_history_close(history);
        return;
    }
    double now = (double)SDL_GetPerformanceCounter();
    double period = (double)SDL_GetPerformanceFrequency() / history->fps;
    unsigned catchup = 0;
    while (history->playing && now >= history->next_tick && catchup++ < 4) {
        if (history->index + 1 >= history->count || !seek(history, history->index + 1, NULL, 0)) {
            history->playing = false;
            break;
        }
        fill_audio(history);
        history->next_tick += period;
    }
    if (history->next_tick + period * 4 < now) history->next_tick = now + period;
    if (history->native) return;
    int w, h;
    SDL_GetRendererOutputSize(history->renderer, &w, &h);
    int height = h > 28 ? h - 28 : h;
    double scale = fmin((double)w / history->width, (double)height / 240);
    SDL_Rect video = {(w - (int)(history->width * scale)) / 2, (height - (int)(240 * scale)) / 2,
                      (int)(history->width * scale), (int)(240 * scale)};
    SDL_SetRenderDrawColor(history->renderer, 15, 15, 20, 255);
    SDL_RenderClear(history->renderer);
    SDL_RenderCopy(history->renderer, history->texture, NULL, &video);
    SDL_Rect bar = {0, h - 24, (int)((history->index + 1) * (size_t)w / history->count), 24};
    SDL_SetRenderDrawColor(history->renderer, 65, 150, 220, 255);
    SDL_RenderFillRect(history->renderer, &bar);
    SDL_RenderPresent(history->renderer);
}

void frontend_history_use_native(FrontendHistory *history) {
    if (!history) return;
    frontend_history_close(history);
    history->native = true;
}

bool frontend_history_action(FrontendHistory *history, unsigned id, const char *value,
                              int selected, char *error, size_t size) {
    return history && action(history, id, value, selected, error, size);
}

bool frontend_history_state(FrontendHistory *history, FrontendHistoryState *state) {
    if (!history || !state) return false;
    *state = (FrontendHistoryState){.open = current(history), .playing = history->playing,
        .index = history->index, .count = history->count, .width = history->width, .fps = history->fps,
        .volume = history->volume, .range_first = history->range_first, .range_last = history->range_last,
        .revision = history->revision, .pixels = history->pixels, .status = history->status};
    memcpy(state->bookmarks, history->bookmarks, sizeof(state->bookmarks));
    state->can_export = state->open;
    for (size_t i = history->range_first; state->can_export && i <= history->range_last; ++i) {
        const NesRewindMedia *media = nes_rewind_media(&history->execution->rewind, i);
        state->can_export = media && media->complete && media->inputs_valid;
    }
    return true;
}
