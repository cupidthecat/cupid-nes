/*
 * nsf_player_ui.c - Music commands and track information panel
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "nsf_player.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    MUSIC_TITLE = 1,
    MUSIC_ARTIST,
    MUSIC_COPYRIGHT,
    MUSIC_RIPPER,
    MUSIC_POSITION,
    MUSIC_TRACK,
    MUSIC_SILENCE_DELAY,
    MUSIC_SILENCE_THRESHOLD
};

typedef struct {
    unsigned registered_commands;
    bool registered_panel;
    char fallback_names[NSF_MAX_TRACKS][24];
    const char *track_names[NSF_MAX_TRACKS];
    char position[96];
    char silence_delay[24];
    char silence_threshold[24];
    char status[128];
} MusicUi;

static bool command_play_pause(void *context, char *error, size_t size) {
    return nsf_player_play_pause(context, error, size);
}

static bool command_stop(void *context, char *error, size_t size) {
    return nsf_player_stop(context, error, size);
}

static bool command_next(void *context, char *error, size_t size) {
    return nsf_player_next(context, error, size);
}

static bool command_previous(void *context, char *error, size_t size) {
    return nsf_player_previous(context, error, size);
}

static bool command_restart(void *context, char *error, size_t size) {
    return nsf_player_select(context, rom_nsf_current_track(), error, size);
}

static bool toggle_option(NsfPlayer *player, unsigned command) {
    NsfPlayerOptions options = player->options;
    switch (command) {
        case NSF_COMMAND_REPEAT: options.repeat = !options.repeat; break;
        case NSF_COMMAND_SHUFFLE: options.shuffle = !options.shuffle; break;
        case NSF_COMMAND_AUTOMATIC: options.automatic = !options.automatic; break;
        case NSF_COMMAND_SILENCE: options.detect_silence = !options.detect_silence; break;
        default: return false;
    }
    return nsf_player_set_options(player, &options);
}

static bool command_repeat(void *context, char *error, size_t size) {
    (void)error;
    (void)size;
    return toggle_option(context, NSF_COMMAND_REPEAT);
}

static bool command_shuffle(void *context, char *error, size_t size) {
    (void)error;
    (void)size;
    return toggle_option(context, NSF_COMMAND_SHUFFLE);
}

static bool command_automatic(void *context, char *error, size_t size) {
    (void)error;
    (void)size;
    return toggle_option(context, NSF_COMMAND_AUTOMATIC);
}

static bool command_silence(void *context, char *error, size_t size) {
    (void)error;
    (void)size;
    return toggle_option(context, NSF_COMMAND_SILENCE);
}

static const struct {
    unsigned id;
    const char *label;
    const char *shortcut;
    FrontendCommandHandler handler;
    bool checkbox;
} music_commands[] = {
    {NSF_COMMAND_PLAY_PAUSE, "Music Play / Pause", "Ctrl+Space", command_play_pause, true},
    {NSF_COMMAND_STOP, "Stop Music", "Ctrl+End", command_stop, false},
    {NSF_COMMAND_NEXT, "Next Music Track", "Page Up", command_next, false},
    {NSF_COMMAND_PREVIOUS, "Previous Music Track", "Page Down", command_previous, false},
    {NSF_COMMAND_RESTART, "Restart Music Track", "Ctrl+Home", command_restart, false},
    {NSF_COMMAND_REPEAT, "Repeat Music Track", "", command_repeat, true},
    {NSF_COMMAND_SHUFFLE, "Shuffle Music Tracks", "", command_shuffle, true},
    {NSF_COMMAND_AUTOMATIC, "Advance Music Automatically", "", command_automatic, true},
    {NSF_COMMAND_SILENCE, "Detect Music Silence", "", command_silence, true}
};

static bool add_text(FrontendPanelModel *model, unsigned id, const char *label,
                      const char *text, bool read_only) {
    FrontendPanelControl control = {
        .id = id, .type = FRONTEND_PANEL_TEXT, .label = label,
        .value = text, .enabled = true, .read_only = read_only
    };
    return frontend_panel_add_control(model, &control);
}

static bool music_snapshot(void *context, FrontendPanelModel *model,
                            char *error, size_t error_size) {
    NsfPlayer *player = context;
    MusicUi *ui = player->ui;
    NsfPlayerInfo info;
    if (!ui || !nsf_player_info(player, &info)) {
        if (error && error_size) snprintf(error, error_size, "Open an NSF or NSFe music image first");
        return false;
    }

    const NsfMetadata *music = info.metadata;
    unsigned elapsed = info.position_seconds > UINT_MAX ? UINT_MAX : (unsigned)info.position_seconds;
    if (info.length_seconds > 0.0) {
        unsigned length = (unsigned)info.length_seconds;
        snprintf(ui->position, sizeof(ui->position), "%u:%02u / %u:%02u (fade %.3g s)",
                 elapsed / 60u, elapsed % 60u, length / 60u, length % 60u, info.fade_seconds);
    } else {
        snprintf(ui->position, sizeof(ui->position), "%u:%02u / duration not provided",
                 elapsed / 60u, elapsed % 60u);
    }
    snprintf(ui->status, sizeof(ui->status), "%s | track %u of %u | %s",
             info.stopped ? "Stopped" : info.paused ? "Paused" : "Playing",
             info.track + 1u, (unsigned)music->total_songs, music->nsfe ? "NSFe" : "NSF");
    snprintf(ui->silence_delay, sizeof(ui->silence_delay), "%u", player->options.silence_ms);
    snprintf(ui->silence_threshold, sizeof(ui->silence_threshold), "%.6f",
             (double)player->options.silence_threshold);
    for (unsigned i = 0; i < music->total_songs; ++i) {
        snprintf(ui->fallback_names[i], sizeof(ui->fallback_names[i]), "Track %u", i + 1u);
        ui->track_names[i] = music->track_names[i][0] ? music->track_names[i] : ui->fallback_names[i];
    }

    bool ok = add_text(model, MUSIC_TITLE, "Title", music->title, true)
        && add_text(model, MUSIC_ARTIST, "Artist", music->artist, true)
        && add_text(model, MUSIC_COPYRIGHT, "Copyright", music->copyright, true)
        && add_text(model, MUSIC_RIPPER, "Ripper", music->ripper, true)
        && add_text(model, MUSIC_POSITION, "Position", ui->position, true);
    FrontendPanelControl track = {
        .id = MUSIC_TRACK, .type = FRONTEND_PANEL_CHOICE, .label = "Track",
        .items = ui->track_names, .item_count = music->total_songs,
        .selected = (int)info.track, .enabled = nes_execution_policy() == NES_EXECUTION_LIVE
    };
    ok = ok && frontend_panel_add_control(model, &track);
    for (size_t i = 0; i < sizeof(music_commands) / sizeof(music_commands[0]); ++i) {
        FrontendCommandInfo command;
        if (!frontend_command_get(music_commands[i].id, &command)) return false;
        FrontendPanelControl control = {
            .id = command.id,
            .type = music_commands[i].checkbox ? FRONTEND_PANEL_CHECKBOX : FRONTEND_PANEL_ACTION,
            .label = command.label, .enabled = command.enabled, .selected = command.checked ? 1 : 0
        };
        ok = ok && frontend_panel_add_control(model, &control);
    }
    ok = ok && add_text(model, MUSIC_SILENCE_DELAY, "Silence delay (milliseconds)", ui->silence_delay, false)
        && add_text(model, MUSIC_SILENCE_THRESHOLD, "Silence amplitude (0 to 0.1)", ui->silence_threshold, false);
    model->status = ui->status;
    if (!ok && error && error_size) snprintf(error, error_size, "The music panel needs 17 controls");
    return ok;
}

static bool music_action(void *context, unsigned id, const char *value, int selected,
                          char *error, size_t error_size) {
    NsfPlayer *player = context;
    if (id == MUSIC_TRACK && selected >= 0)
        return nsf_player_select(player, (unsigned)selected, error, error_size);
    if (id >= NSF_COMMAND_PLAY_PAUSE && id < NSF_COMMAND_END)
        return frontend_command_invoke(id, error, error_size);

    NsfPlayerOptions options = player->options;
    char *end = NULL;
    errno = 0;
    if (id == MUSIC_SILENCE_DELAY && value && value[0] >= '0' && value[0] <= '9') {
        unsigned long delay = strtoul(value, &end, 10);
        if (!errno && end && !*end && delay >= 10 && delay <= 600000) {
            options.silence_ms = (unsigned)delay;
            return nsf_player_set_options(player, &options);
        }
    } else if (id == MUSIC_SILENCE_THRESHOLD && value && value[0]) {
        float threshold = strtof(value, &end);
        if (!errno && end && end != value && !*end && isfinite(threshold)
            && threshold >= 0.0f && threshold <= 0.1f) {
            options.silence_threshold = threshold;
            return nsf_player_set_options(player, &options);
        }
    }
    if (error && error_size) snprintf(error, error_size, "Enter a valid music setting");
    return false;
}

bool nsf_player_register_ui(NsfPlayer *player) {
    if (!player || !player->observer_token || player->ui) return false;
    if (rom_is_nsf()) {
        frontend_command_set_session_active(true);
        frontend_panel_set_session_active(true);
    }
    MusicUi *ui = calloc(1, sizeof(*ui));
    if (!ui) return false;
    player->ui = ui;
    for (size_t i = 0; i < sizeof(music_commands) / sizeof(music_commands[0]); ++i) {
        FrontendCommandSpec spec = {
            .id = music_commands[i].id, .label = music_commands[i].label,
            .menu = "Tools", .shortcut = music_commands[i].shortcut,
            .flags = FRONTEND_COMMAND_NEEDS_SESSION
                | (music_commands[i].checkbox ? FRONTEND_COMMAND_CHECKABLE : 0u),
            .handler = music_commands[i].handler, .userdata = player
        };
        if (!frontend_command_register(&spec)) {
            nsf_player_unregister_ui(player);
            return false;
        }
        ++ui->registered_commands;
    }
    FrontendPanelSpec panel = {
        .id = NSF_PLAYER_PANEL, .title = "Music Player", .category = "Tools",
        .flags = FRONTEND_PANEL_NEEDS_SESSION,
        .snapshot = music_snapshot, .action = music_action, .userdata = player
    };
    if (!frontend_panel_register(&panel)) {
        nsf_player_unregister_ui(player);
        return false;
    }
    ui->registered_panel = true;
    nsf_player_refresh_ui(player);
    return true;
}

void nsf_player_unregister_ui(NsfPlayer *player) {
    if (!player || !player->ui) return;
    MusicUi *ui = player->ui;
    for (unsigned i = 0; i < ui->registered_commands; ++i)
        (void)frontend_command_unregister(music_commands[i].id);
    if (ui->registered_panel) (void)frontend_panel_unregister(NSF_PLAYER_PANEL);
    free(ui);
    player->ui = NULL;
}

void nsf_player_refresh_ui(NsfPlayer *player) {
    if (!player || !player->ui) return;
    bool music = rom_is_nsf();
    bool controls = music && nes_execution_policy() == NES_EXECUTION_LIVE;
    for (unsigned id = NSF_COMMAND_PLAY_PAUSE; id < NSF_COMMAND_END; ++id)
        (void)frontend_command_set_enabled(id, controls);
    (void)frontend_panel_set_enabled(NSF_PLAYER_PANEL, music);
    (void)frontend_command_set_checked(NSF_COMMAND_PLAY_PAUSE,
                                       controls && !player->stopped && !player->execution->paused);
    (void)frontend_command_set_checked(NSF_COMMAND_REPEAT, player->options.repeat);
    (void)frontend_command_set_checked(NSF_COMMAND_SHUFFLE, player->options.shuffle);
    (void)frontend_command_set_checked(NSF_COMMAND_AUTOMATIC, player->options.automatic);
    (void)frontend_command_set_checked(NSF_COMMAND_SILENCE, player->options.detect_silence);
}
