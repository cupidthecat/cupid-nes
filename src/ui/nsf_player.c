/*
 * nsf_player.c - Music transport, track timing, and automatic progression
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "nsf_player.h"
#include "../apu/apu.h"
#include "../audio/audio_observer.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool fail(char *error, size_t capacity, const char *message) {
    if (error && capacity) snprintf(error, capacity, "%s", message);
    return false;
}

void nsf_player_options_defaults(NsfPlayerOptions *options) {
    if (options) *options = (NsfPlayerOptions){true, false, false, true, 3000, 0.0005f};
}

bool nsf_player_set_options(NsfPlayer *player, const NsfPlayerOptions *options) {
    if (!player || !options || options->silence_ms < 10 || options->silence_ms > 600000
        || !isfinite(options->silence_threshold) || options->silence_threshold < 0.0f
        || options->silence_threshold > 0.1f) return false;

    player->options = *options;
    player->silent_seconds = 0.0;
    nsf_player_refresh_ui(player);
    return true;
}

static void observe_music(void *context, unsigned machine, double sample_rate,
                           float left, float right) {
    NsfPlayer *player = context;
    if (machine || !player->execution || player->execution->paused || player->stopped
        || !player->options.detect_silence || !rom_is_nsf()
        || nes_execution_policy() != NES_EXECUTION_LIVE) return;

    if (!isfinite(left) || !isfinite(right)
        || fabsf(left) > player->options.silence_threshold
        || fabsf(right) > player->options.silence_threshold) {
        player->silent_seconds = 0.0;
    } else {
        player->silent_seconds += 1.0 / sample_rate;
    }
}

bool nsf_player_init(NsfPlayer *player, ExecutionControl *execution,
                      const NsfPlayerHooks *hooks, uint32_t shuffle_seed) {
    if (!player || !execution || (hooks && (!!hooks->lock_audio != !!hooks->unlock_audio))
        || (hooks && hooks->audio_active && !hooks->lock_audio)) return false;

    memset(player, 0, sizeof(*player));
    player->execution = execution;
    if (hooks) player->hooks = *hooks;
    player->shuffle_state = shuffle_seed ? shuffle_seed : 0x6D2B79F5u;
    nsf_player_options_defaults(&player->options);
    player->observer_token = nes_audio_observer_add(observe_music, player);
    if (!player->observer_token) {
        memset(player, 0, sizeof(*player));
        return false;
    }

    nsf_player_image_changed(player);
    return true;
}

void nsf_player_shutdown(NsfPlayer *player) {
    if (!player) return;
    nsf_player_unregister_ui(player);
    (void)nes_audio_observer_remove(player->observer_token);
    memset(player, 0, sizeof(*player));
}

void nsf_player_image_changed(NsfPlayer *player) {
    if (!player) return;
    player->attached = rom_is_nsf();
    player->image_crc = rom_file_crc32();
    player->last_track = rom_nsf_current_track();
    player->last_elapsed = rom_nsf_elapsed_seconds();
    player->silent_seconds = 0.0;
    player->stopped = false;
    player->last_policy = nes_execution_policy();
    nsf_player_refresh_ui(player);
}

bool nsf_player_info(const NsfPlayer *player, NsfPlayerInfo *info) {
    const NsfMetadata *music = rom_nsf_metadata();
    if (!player || !info || !music || !music->total_songs) return false;

    memset(info, 0, sizeof(*info));
    info->metadata = music;
    info->track = rom_nsf_current_track();
    if (info->track >= music->total_songs) return false;
    info->position_seconds = rom_nsf_elapsed_seconds();
    int32_t length = music->track_length[info->track];
    int32_t fade = music->track_fade[info->track];
    if (length > 0) {
        info->fade_seconds = fade > 0 ? (double)fade / 1000.0 : 0.0;
        info->length_seconds = (double)length / 1000.0 + info->fade_seconds;
    }
    info->paused = player->execution && player->execution->paused;
    info->stopped = player->stopped;
    return true;
}

static bool require_music(NsfPlayer *player, char *error, size_t size) {
    if (error && size) error[0] = '\0';
    if (!player || !player->execution || !rom_is_nsf())
        return fail(error, size, "Open an NSF or NSFe music image first");
    if (nes_execution_policy() != NES_EXECUTION_LIVE)
        return fail(error, size, "Music transport is unavailable during replay or netplay");
    return true;
}

static void set_paused(NsfPlayer *player, bool paused) {
    if (player->hooks.set_paused) player->hooks.set_paused(player->hooks.context, paused);
    execution_control_set_paused(player->execution, paused);
    nsf_player_refresh_ui(player);
}

static bool select_track(NsfPlayer *player, unsigned track, bool automatic,
                          char *error, size_t error_size) {
    if (!require_music(player, error, error_size)) return false;
    const NsfMetadata *music = rom_nsf_metadata();
    if (!music || track >= music->total_songs)
        return fail(error, error_size, "The selected music track is outside this image");

    bool carry_audio = automatic && player->hooks.audio_active
        && player->hooks.audio_active(player->hooks.context);
    float *pending = carry_audio ? malloc(sizeof(float) * APU_RING_CAP * 2u) : NULL;
    if (carry_audio && !pending)
        return fail(error, error_size, "Could not retain queued audio for the next track");

    if (player->hooks.lock_audio) player->hooks.lock_audio(player->hooks.context);
    uint32_t queued = 0;
    float previous_middle = apu.last_read_sample;
    float previous_side = apu.last_read_side;
    if (carry_audio) {
        uint32_t read = atomic_load_explicit(&apu.ring_r, memory_order_relaxed);
        uint32_t write = atomic_load_explicit(&apu.ring_w, memory_order_acquire);
        queued = (write - read) & (APU_RING_CAP - 1u);
        /* A reset can itself emit a few samples. Wait for the consumer when
         * there is no room to retain both sides of the transition. */
        if (queued > APU_RING_CAP - 32u) {
            player->hooks.unlock_audio(player->hooks.context);
            free(pending);
            return false;
        }
        for (uint32_t i = 0; i < queued; ++i) {
            unsigned index = (read + i) & (APU_RING_CAP - 1u);
            pending[i * 2u] = apu.ring[index];
            pending[i * 2u + 1u] = apu.ring_side[index];
        }
    }

    bool selected = rom_nsf_select_track(track);
    if (selected && carry_audio) {
        uint32_t read = atomic_load_explicit(&apu.ring_r, memory_order_relaxed);
        uint32_t write = atomic_load_explicit(&apu.ring_w, memory_order_acquire);
        uint32_t added = (write - read) & (APU_RING_CAP - 1u);
        for (uint32_t i = 0; i < added; ++i) {
            unsigned index = (read + i) & (APU_RING_CAP - 1u);
            pending[(queued + i) * 2u] = apu.ring[index];
            pending[(queued + i) * 2u + 1u] = apu.ring_side[index];
        }
        queued += added;
        for (uint32_t i = 0; i < queued; ++i) {
            apu.ring[i] = pending[i * 2u];
            apu.ring_side[i] = pending[i * 2u + 1u];
        }
        apu.last_read_sample = previous_middle;
        apu.last_read_side = previous_side;
        atomic_store_explicit(&apu.ring_r, 0, memory_order_relaxed);
        atomic_store_explicit(&apu.ring_w, queued, memory_order_release);
    }
    if (player->hooks.unlock_audio) player->hooks.unlock_audio(player->hooks.context);
    free(pending);
    if (!selected) return fail(error, error_size, "The music track could not be selected");

    nsf_player_image_changed(player);
    return true;
}

bool nsf_player_select(NsfPlayer *player, unsigned track, char *error, size_t error_size) {
    return select_track(player, track, false, error, error_size);
}

bool nsf_player_play_pause(NsfPlayer *player, char *error, size_t error_size) {
    if (!require_music(player, error, error_size)) return false;
    bool paused = player->stopped ? false : !player->execution->paused;
    player->stopped = false;
    set_paused(player, paused);
    return true;
}

bool nsf_player_stop(NsfPlayer *player, char *error, size_t error_size) {
    if (!select_track(player, rom_nsf_current_track(), false, error, error_size)) return false;
    player->stopped = true;
    set_paused(player, true);
    return true;
}

bool nsf_player_next(NsfPlayer *player, char *error, size_t error_size) {
    if (!require_music(player, error, error_size)) return false;
    const NsfMetadata *music = rom_nsf_metadata();
    return select_track(player, (rom_nsf_current_track() + 1u) % music->total_songs,
                         false, error, error_size);
}

bool nsf_player_previous(NsfPlayer *player, char *error, size_t error_size) {
    if (!require_music(player, error, error_size)) return false;
    const NsfMetadata *music = rom_nsf_metadata();
    unsigned track = rom_nsf_current_track();
    if (rom_nsf_elapsed_seconds() < 2.0) track = track ? track - 1u : music->total_songs - 1u;
    return select_track(player, track, false, error, error_size);
}

static uint32_t shuffle_random(NsfPlayer *player) {
    uint32_t value = player->shuffle_state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    player->shuffle_state = value;
    return value;
}

static unsigned automatic_track(NsfPlayer *player, unsigned current, unsigned count) {
    if (player->options.repeat || count < 2) return current;
    if (!player->options.shuffle) return (current + 1u) % count;

    uint32_t range = count - 1u;
    uint32_t minimum = (0u - range) % range;
    uint32_t value;
    do {
        value = shuffle_random(player);
    } while (value < minimum);
    return (current + 1u + value % range) % count;
}

bool nsf_player_poll(NsfPlayer *player, char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!player) return false;
    NsfPlayerInfo info;
    if (!nsf_player_info(player, &info)) {
        if (player->attached) nsf_player_image_changed(player);
        return false;
    }
    if (!player->attached || player->image_crc != rom_file_crc32()
        || player->last_track != info.track || info.position_seconds < player->last_elapsed
        || player->last_policy != nes_execution_policy()) nsf_player_image_changed(player);

    player->last_elapsed = info.position_seconds;
    nsf_player_refresh_ui(player);
    if (info.paused || player->stopped || !player->options.automatic
        || nes_execution_policy() != NES_EXECUTION_LIVE) return false;

    bool expired = info.length_seconds > 0.0 && info.position_seconds >= info.length_seconds;
    bool silent = player->options.detect_silence
        && player->silent_seconds * 1000.0 >= player->options.silence_ms;
    if (!expired && !silent) return false;

    unsigned selected = automatic_track(player, info.track, info.metadata->total_songs);
    return select_track(player, selected, true, error, error_size);
}
