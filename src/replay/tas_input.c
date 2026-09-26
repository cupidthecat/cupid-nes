/*
 * tas_input.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Movie inputs and recording takes. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_session_internal.h"
#include "tas_startup.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/fds.h"
#include "../system/vs_system.h"
#include <string.h>

bool tas_input_observer(const NesInputEvent *event, void *userdata) {
    NesTasSession *session = userdata;
    if (!session || !session->active || !event) {
        return false;
    }
    /* Keep physical controls separately from the emulated pads. In particular,
     * focus-loss releases must reach this buffer while replay owns the machine. */
    if (event->type == NES_INPUT_EVENT_PLAYER_BUTTON && event->a >= 0 && event->a < 4 && event->b >= BTN_A &&
        event->b <= BTN_RIGHT) {
        uint8_t mask = (uint8_t)(1u << event->b);
        if (event->c) {
            session->live_pads[event->a] |= mask;
        } else {
            session->live_pads[event->a] &= (uint8_t)~mask;
        }
    } else if (event->type == NES_INPUT_EVENT_ZAPPER && event->a >= 0 && event->a < 2) {
        NesFm2Zapper *zapper = &session->live_zappers[event->a];
        zapper->x = event->b < 0 || event->b > 255 ? 255 : (uint8_t)event->b;
        zapper->y = event->c < 0 || event->c > 239 ? 255 : (uint8_t)event->c;
        bool offscreen = event->b < 0 || event->b > 255 || event->c < 0 || event->c > 239;
        zapper->button = event->d ? (offscreen ? 2 : 1) : 0;
    } else if (session->recording && !session->read_only && !session->frame_in_progress && !session->seeking) {
        if (event->type == NES_INPUT_EVENT_SOFT_RESET) {
            session->pending_commands |= NES_FM2_COMMAND_RESET;
        } else if (event->type == NES_INPUT_EVENT_POWER_CYCLE) {
            session->pending_commands |= NES_FM2_COMMAND_POWER;
        } else if (event->type == NES_INPUT_EVENT_VS_COIN && event->a >= 0 && event->a < 2 && event->b) {
            session->pending_commands |= event->a ? NES_FM2_COMMAND_VS_COIN_2 : NES_FM2_COMMAND_VS_COIN_1;
        } else if (event->type == NES_INPUT_EVENT_VS_SERVICE && event->b) {
            session->pending_commands |= NES_FM2_COMMAND_VS_SERVICE;
        }
    }
    return false;
}

static bool event_apply(NesInputEventType type, int32_t a, int32_t b, int32_t c, int32_t d) {
    NesInputEvent event = {.type = type, .a = a, .b = b, .c = c, .d = d};
    return nes_input_event_apply(&event);
}

static bool valid_commands(uint8_t commands) {
    if (commands & 0x80u) {
        return false;
    }
    if ((commands & (NES_FM2_COMMAND_FDS_INSERT | NES_FM2_COMMAND_FDS_SELECT)) && !fds_active()) {
        return false;
    }
    if ((commands & (NES_FM2_COMMAND_VS_COIN_1 | NES_FM2_COMMAND_VS_COIN_2 | NES_FM2_COMMAND_VS_SERVICE)) &&
        !vs_enabled()) {
        return false;
    }
    return true;
}

NesMovieResult nes_tas_session_queue_commands(NesTasSession *session, uint8_t commands) {
    if (!session || !session->active || !session->recording || session->read_only || session->frame_in_progress ||
        session->seeking) {
        return tas_result(session, NES_MOVIE_CONFLICT, "Enable TAS recording at a frame boundary to record commands.");
    }
    if (!valid_commands(commands)) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "This command is unavailable on the loaded hardware.");
    }
    session->pending_commands |= commands;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

bool tas_apply_frame(NesTasSession *session, const NesFm2Frame *input) {
    const NesFm2Movie *movie = nes_tas_project_movie(session->project);
    uint8_t commands = input->commands;
    if (!valid_commands(commands)) {
        return false;
    }
    if ((commands & NES_FM2_COMMAND_POWER) && !nes_tas_startup_power(movie)) {
        return false;
    }
    if ((commands & NES_FM2_COMMAND_RESET) &&
        (!event_apply(NES_INPUT_EVENT_SOFT_RESET, 0, 0, 0, 0) || !ppu_begin_tas_timing())) {
        return false;
    }
    if (commands & NES_FM2_COMMAND_FDS_INSERT) {
        if (!fds_active()) {
            return false;
        }
        if (fds_disk_inserted()) {
            if (!event_apply(NES_INPUT_EVENT_FDS_EJECT, 0, 0, 0, 0)) {
                return false;
            }
        } else if (!event_apply(NES_INPUT_EVENT_FDS_INSERT, (int32_t)session->selected_side, 0, 0, 0)) {
            return false;
        }
    }
    if (commands & NES_FM2_COMMAND_FDS_SELECT) {
        if (!fds_active() || !fds_side_count()) {
            return false;
        }
        if (!fds_disk_inserted()) {
            session->selected_side = (session->selected_side + 1) % fds_side_count();
        }
    }
    if (vs_enabled()) {
        if (!event_apply(NES_INPUT_EVENT_VS_COIN, 0, !!(commands & NES_FM2_COMMAND_VS_COIN_1), 0, 0) ||
            !event_apply(NES_INPUT_EVENT_VS_COIN, 1, !!(commands & NES_FM2_COMMAND_VS_COIN_2), 0, 0) ||
            !event_apply(NES_INPUT_EVENT_VS_SERVICE, 0, !!(commands & NES_FM2_COMMAND_VS_SERVICE), 0, 0)) {
            return false;
        }
    } else if (commands & (NES_FM2_COMMAND_VS_COIN_1 | NES_FM2_COMMAND_VS_COIN_2 | NES_FM2_COMMAND_VS_SERVICE)) {
        return false;
    }
    for (unsigned player = 0; player < 4; ++player) {
        uint8_t buttons = input->pads[player];
        if (player == 1 && movie->microphone) {
            buttons &= (uint8_t)~(1u << BTN_START);
        }
        if (player >= 2 && !movie->fourscore) {
            buttons = 0;
        }
        for (unsigned button = 0; button < 8; ++button) {
            if (!event_apply(NES_INPUT_EVENT_PLAYER_BUTTON, (int32_t)player, (int32_t)button,
                             !!(buttons & (1u << button)), 0)) {
                return false;
            }
        }
        if (player < 2 && movie->ports[player] == NES_FM2_PORT_ZAPPER) {
            const NesFm2Zapper *zapper = &input->zappers[player];
            bool offscreen = (zapper->button & 2u) != 0;
            if (!event_apply(NES_INPUT_EVENT_ZAPPER, (int32_t)player, offscreen ? -1 : zapper->x,
                             offscreen ? -1 : zapper->y, zapper->bogo != 0)) {
                return false;
            }
        }
    }
    return event_apply(NES_INPUT_EVENT_MICROPHONE, movie->microphone && (input->pads[1] & (1u << BTN_START)), 0, 0, 0);
}

static bool inputs_equal(const NesFm2Frame *left, const NesFm2Frame *right) {
    if (!left || !right || left->commands != right->commands || memcmp(left->pads, right->pads, 4)) {
        return false;
    }
    for (unsigned port = 0; port < 2; ++port) {
        const NesFm2Zapper *a = &left->zappers[port], *b = &right->zappers[port];
        if (a->x != b->x || a->y != b->y || a->button != b->button || a->bogo != b->bogo || a->zaphit != b->zaphit) {
            return false;
        }
    }
    return true;
}

NesMovieResult tas_record_frame(NesTasSession *session) {
    bool insert = session->record_mode == NES_TAS_RECORD_INSERT;
    const NesFm2Frame *existing = nes_tas_project_frame(session->project, session->frame);
    if (!insert && inputs_equal(existing, &session->pending_input)) {
        return NES_MOVIE_OK;
    }
    if (!session->take_open) {
        NesTasResult begin = nes_tas_edit_begin(session->project);
        if (begin != NES_TAS_OK) {
            return tas_model_result(session, begin);
        }
        session->take_open = true;
        session->take_changed = false;
    }
    bool overwriting = existing != NULL;
    if (!session->take_changed && overwriting) {
        NesTasResult incremented = nes_tas_increment_rerecord(session->project);
        if (incremented != NES_TAS_OK) {
            nes_tas_edit_cancel(session->project);
            session->take_open = false;
            return tas_model_result(session, incremented);
        }
    }
    NesTasResult result = nes_tas_record_frame(session->project, session->frame, &session->pending_input, insert);
    if (result != NES_TAS_OK) {
        if (!session->take_changed) {
            nes_tas_edit_cancel(session->project);
            session->take_open = false;
        }
        return tas_model_result(session, result);
    }
    tas_invalidate_after(session, session->frame);
    session->take_changed = true;
    session->seen_revision = nes_tas_project_revision(session->project);
    return NES_MOVIE_OK;
}

NesMovieResult tas_finish_take(NesTasSession *session) {
    if (session->completed_pending) {
        NesMovieResult completed = nes_tas_session_frame_complete(session, true);
        if (completed != NES_MOVIE_OK) {
            return completed;
        }
    }
    if (session->take_open) {
        NesTasResult result = nes_tas_edit_end(session->project);
        if (result != NES_TAS_OK) {
            return tas_model_result(session, result);
        }
        session->take_open = session->take_changed = false;
        session->seen_revision = nes_tas_project_revision(session->project);
    }
    return NES_MOVIE_OK;
}

NesMovieResult nes_tas_session_set_read_only(NesTasSession *session, bool read_only) {
    if (!session || !session->active || session->frame_in_progress) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (read_only) {
        NesMovieResult finished = tas_finish_take(session);
        if (finished != NES_MOVIE_OK) {
            return finished;
        }
        session->recording = false;
        session->pending_commands = 0;
    }
    session->read_only = read_only;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_set_recording(NesTasSession *session, bool recording, NesTasRecordMode mode,
                                             unsigned players) {
    if (!session || !session->active || (recording && session->read_only) || session->frame_in_progress ||
        session->seeking) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    const NesFm2Movie *movie = nes_tas_project_movie(session->project);
    unsigned mask = movie->fourscore ? 15u : 3u;
    if (!players || (players & ~mask) || (unsigned)mode > NES_TAS_RECORD_INSERT) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    NesMovieResult finished = tas_finish_take(session);
    if (finished != NES_MOVIE_OK) {
        return finished;
    }
    session->recording = recording;
    if (!recording) {
        session->pending_commands = 0;
    }
    session->record_mode = mode;
    session->record_players = players;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_set_hold(NesTasSession *session, unsigned player, uint8_t buttons) {
    if (!session || player >= 4) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    session->hold[player] = buttons;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_set_autofire(NesTasSession *session, unsigned player, uint8_t buttons,
                                            unsigned on_frames, unsigned off_frames) {
    if (!session || player >= 4 || !on_frames || !off_frames || on_frames > 3600 || off_frames > 3600) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    session->autofire[player] = buttons;
    session->fire_on[player] = on_frames;
    session->fire_off[player] = off_frames;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

uint8_t nes_tas_session_hold(const NesTasSession *session, unsigned player) {
    return session && player < 4 ? session->hold[player] : 0;
}

uint8_t nes_tas_session_autofire(const NesTasSession *session, unsigned player) {
    return session && player < 4 ? session->autofire[player] : 0;
}
