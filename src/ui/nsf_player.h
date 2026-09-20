/*
 * nsf_player.h - Music transport, track timing, and automatic progression
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NSF_PLAYER_H
#define NSF_PLAYER_H

#include "execution_control.h"
#include "nsf_frontend.h"
#include "../rom/nsf.h"
#include <stddef.h>
#include <stdint.h>

enum {
    NSF_COMMAND_PLAY_PAUSE = 0x1500,
    NSF_COMMAND_STOP,
    NSF_COMMAND_NEXT,
    NSF_COMMAND_PREVIOUS,
    NSF_COMMAND_RESTART,
    NSF_COMMAND_REPEAT,
    NSF_COMMAND_SHUFFLE,
    NSF_COMMAND_AUTOMATIC,
    NSF_COMMAND_SILENCE,
    NSF_COMMAND_END,
    NSF_PLAYER_PANEL = 0x1500
};

typedef struct {
    bool automatic;
    bool repeat;
    bool shuffle;
    bool detect_silence;
    unsigned silence_ms;
    float silence_threshold;
} NsfPlayerOptions;

typedef struct {
    NsfFrontendAudioGuard lock_audio;
    NsfFrontendAudioGuard unlock_audio;
    void (*set_paused)(void *context, bool paused);
    bool (*audio_active)(void *context);
    void *context;
} NsfPlayerHooks;

typedef struct {
    ExecutionControl *execution;
    NsfPlayerHooks hooks;
    NsfPlayerOptions options;
    uint32_t observer_token;
    uint32_t shuffle_state;
    uint32_t image_crc;
    uint32_t last_policy;
    unsigned last_track;
    double last_elapsed;
    double silent_seconds;
    bool attached;
    bool stopped;
    void *ui;
} NsfPlayer;

typedef struct {
    const NsfMetadata *metadata;
    unsigned track;
    double position_seconds;
    double length_seconds;
    double fade_seconds;
    bool paused;
    bool stopped;
} NsfPlayerInfo;

void nsf_player_options_defaults(NsfPlayerOptions *options);
bool nsf_player_set_options(NsfPlayer *player, const NsfPlayerOptions *options);
bool nsf_player_init(NsfPlayer *player, ExecutionControl *execution,
                      const NsfPlayerHooks *hooks, uint32_t shuffle_seed);
void nsf_player_shutdown(NsfPlayer *player);
void nsf_player_image_changed(NsfPlayer *player);
bool nsf_player_info(const NsfPlayer *player, NsfPlayerInfo *info);
bool nsf_player_play_pause(NsfPlayer *player, char *error, size_t error_size);
bool nsf_player_stop(NsfPlayer *player, char *error, size_t error_size);
bool nsf_player_select(NsfPlayer *player, unsigned track, char *error, size_t error_size);
bool nsf_player_next(NsfPlayer *player, char *error, size_t error_size);
bool nsf_player_previous(NsfPlayer *player, char *error, size_t error_size);
/* Call after each completed emulated frame and after a session/state change.
 * Returns true when an automatic track change occurred. */
bool nsf_player_poll(NsfPlayer *player, char *error, size_t error_size);

bool nsf_player_register_ui(NsfPlayer *player);
void nsf_player_unregister_ui(NsfPlayer *player);
void nsf_player_refresh_ui(NsfPlayer *player);

#endif
