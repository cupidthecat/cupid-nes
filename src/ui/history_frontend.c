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
    size_t index, count, head, bytes;
    const void *entries, *last;
    uint64_t session, next_tick;
    unsigned width;
    bool playing;
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

void frontend_history_close(FrontendHistory *history) {
    if (!history) {
        return;
    }
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
    return history->window && nes_execution_policy() == NES_EXECUTION_LIVE &&
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
    if (!history->texture) {
        history->texture = SDL_CreateTexture(history->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                             (int)history->width, 240);
    }
    if (!history->texture ||
        SDL_UpdateTexture(history->texture, NULL, history->pixels, (int)(history->width * sizeof(uint32_t))) != 0) {
        return fail(history, error, size, SDL_GetError());
    }
    snprintf(history->position, sizeof(history->position), "%zu", index + 1);
    snprintf(history->status, sizeof(history->status), "Frame %zu / %zu; retained snapshots have no input movie stream",
             index + 1, history->count);
    SDL_SetWindowTitle(history->window, history->status);
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

static bool open_view(FrontendHistory *history, char *error, size_t size) {
    if (current(history)) {
        SDL_RaiseWindow(history->window);
        return true;
    }
    frontend_history_close(history);
    NesRewindHistory *ring = &history->execution->rewind;
    if (nes_execution_policy() != NES_EXECUTION_LIVE || !nes_replay_host_state_supported() || !ring->count) {
        return fail(history, error, size, "History requires live execution and retained rewind frames");
    }
    history->window = SDL_CreateWindow("Rewind History", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 768, 740,
                                       SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (history->window) {
        history->renderer = SDL_CreateRenderer(history->window, -1, 0);
    }
    if (!history->renderer) {
        fail(history, error, size, SDL_GetError());
        frontend_history_close(history);
        return false;
    }
    frontend_execution_suspend(history->execution, HISTORY_SUSPEND, true);
    history->session = debugger_session_revision();
    history->entries = ring->entries;
    history->count = ring->count;
    history->head = ring->head;
    history->bytes = ring->total_bytes;
    history->last = ring->entries[(ring->head + ring->count - 1) % ring->capacity].data;
    if (!seek(history, ring->count - 1, error, size)) {
        frontend_history_close(history);
        return false;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    (void)selected;
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
    if (id == 0x2702) {
        char *end = NULL;
        errno = 0;
        unsigned long long number = value ? strtoull(value, &end, 10) : 0;
        if (!value || value[0] == '-' || errno || end == value || *end || !number || number > history->count) {
            return fail(history, error, size, "Position must name a retained frame, starting at 1");
        }
        return seek(history, (size_t)number - 1, error, size);
    }
    if (id == 0x2703 || id == 0x2704) {
        size_t index = history->index;
        if (id == 0x2703 && index) {
            --index;
        }
        if (id == 0x2704 && index + 1 < history->count) {
            ++index;
        }
        return seek(history, index, error, size);
    }
    if (id == 0x2705) {
        history->playing = !history->playing;
        history->next_tick = SDL_GetPerformanceCounter();
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
        if (history->execution->restore_handler) {
            history->execution->restore_handler(history->execution->restore_userdata);
        }
        frontend_history_close(history);
        execution_control_set_paused(&history->execution->execution, false);
        frontend_execution_refresh_audio(history->execution);
        return true;
    }
    return fail(history, error, size, "Unknown history action");
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    FrontendHistory *history = context;
    if (!model || model->capacity < 9) {
        return fail(history, error, size, "History panel needs nine controls");
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
                                         "Movie export unavailable: no retained inputs"};
    for (unsigned i = 0; i < 9; ++i) {
        model->controls[i] = (FrontendPanelControl){.id = 0x2701 + i,
                                                    .type = FRONTEND_PANEL_ACTION,
                                                    .label = labels[i],
                                                    .enabled = i == 0 || (active && i != 8)};
    }
    model->controls[1].type = FRONTEND_PANEL_TEXT;
    model->controls[1].value = history->position;
    model->controls[5].type = FRONTEND_PANEL_FILE_SAVE;
    model->controls[5].selected = FRONTEND_SAVE_STATE;
    model->count = 9;
    model->status = history->status;
    return true;
}

FrontendHistory *frontend_history_create(FrontendExecutionRuntime *execution) {
    if (!execution) {
        return NULL;
    }
    FrontendHistory *history = calloc(1, sizeof(*history));
    if (!history) {
        return NULL;
    }
    history->execution = execution;
    FrontendPanelSpec spec = {0x2700,   "Rewind History", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                              snapshot, action,           history};
    if (!frontend_panel_register(&spec)) {
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
            (void)seek(history, (size_t)x * history->count / (size_t)w, NULL, 0);
        }
        return true;
    }
    return false;
}

void frontend_history_tick(FrontendHistory *history) {
    if (!history || !history->window) {
        return;
    }
    if (!current(history)) {
        frontend_history_close(history);
        return;
    }
    uint64_t now = SDL_GetPerformanceCounter();
    if (history->playing && now >= history->next_tick) {
        if (history->index + 1 >= history->count || !seek(history, history->index + 1, NULL, 0)) {
            history->playing = false;
        }
        history->next_tick = now + (uint64_t)((double)SDL_GetPerformanceFrequency() / nes_timing()->fps);
    }
    int w, h;
    SDL_GetRendererOutputSize(history->renderer, &w, &h);
    SDL_Rect video = {0, 0, w, h > 28 ? h - 28 : h};
    SDL_SetRenderDrawColor(history->renderer, 15, 15, 20, 255);
    SDL_RenderClear(history->renderer);
    SDL_RenderCopy(history->renderer, history->texture, NULL, &video);
    SDL_Rect bar = {0, h - 24, (int)((history->index + 1) * (size_t)w / history->count), 24};
    SDL_SetRenderDrawColor(history->renderer, 65, 150, 220, 255);
    SDL_RenderFillRect(history->renderer, &bar);
    SDL_RenderPresent(history->renderer);
}
