/* Desktop network sessions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "netplay_frontend.h"
#include "frontend_execution.h"
#include "host_input.h"
#include "frontend_panels.h"
#include "frontend_commands.h"
#include "../debugger/debugger.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { NETPLAY_PANEL = 0x1900, NET_STATUS, NET_HOST, NET_PORT, NET_LISTEN,
       NET_JOIN, NET_DISCONNECT, NET_PLAYER = 0x1920 };
struct FrontendNetplay {
    FrontendExecutionRuntime *execution;
    char host[256];
    char port_text[8];
    char status[192];
    unsigned owner[6];
    unsigned port;
};

static bool report(FrontendNetplay *ui, NesNetplayResult result, char *error, size_t size) {
    bool ok = result == NES_NETPLAY_OK || result == NES_NETPLAY_PAUSED;
    if (!ok) snprintf(ui->status, sizeof(ui->status), "Netplay: %s", nes_netplay_result_string(result));
    if (error && size) snprintf(error, size, "%s", ok ? "" : ui->status);
    return ok;
}

static void sync_pause(FrontendNetplay *ui) {
    FrontendExecutionRuntime *execution = ui->execution;
    bool paused = nes_netplay_paused(execution->netplay);
    execution_control_set_paused(&execution->execution, paused);
    execution->execution.frame_advance_pending = false;
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, paused);
    if (execution->audio_device && *execution->audio_device)
        SDL_PauseAudioDevice(*execution->audio_device, paused || execution->muted);
}

static void restored(FrontendNetplay *ui) {
    FrontendExecutionRuntime *execution = ui->execution;
    frontend_execution_clear_timeline(execution);
    debugger_reset_session();
    execution->debugger_pause_revision = debugger_pause_revision();
    execution_control_set_paused(&execution->execution, true);
    if (execution->restore_handler) execution->restore_handler(execution->restore_userdata);
    frontend_execution_release_host_input(execution);
    frontend_host_input_release_all();
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, true);
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)error; (void)size;
    FrontendNetplay *ui = context;
    NesNetplayMode mode = nes_netplay_mode(ui->execution->netplay);
    bool idle = mode == NES_NETPLAY_IDLE;
    snprintf(ui->port_text, sizeof(ui->port_text), "%u", ui->port);
    FrontendPanelControl controls[] = {
        {NET_STATUS, FRONTEND_PANEL_TEXT, "Session", frontend_netplay_status(ui), NULL, 0, -1, true, true},
        {NET_HOST, FRONTEND_PANEL_TEXT, "Host address", ui->host, NULL, 0, -1, idle, false},
        {NET_PORT, FRONTEND_PANEL_TEXT, "TCP port", ui->port_text, NULL, 0, -1, idle, false},
        {NET_LISTEN, FRONTEND_PANEL_ACTION, "Host game", NULL, NULL, 0, -1, idle, false},
        {NET_JOIN, FRONTEND_PANEL_ACTION, "Join game", NULL, NULL, 0, -1, idle, false},
        {NET_DISCONNECT, FRONTEND_PANEL_ACTION, "Disconnect / cancel", NULL, NULL, 0, -1, !idle, false}
    };
    for (unsigned i = 0; i < sizeof(controls)/sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    static const char *const owners[] = {"Unused", "Host", "Guest"};
    static const char *const labels[] = {"Player 1", "Player 2", "Player 3", "Player 4", "Player 5", "Player 6"};
    for (unsigned i = 0; i < 6; ++i) {
        FrontendPanelControl control = {NET_PLAYER+i, FRONTEND_PANEL_CHOICE, labels[i], NULL,
            owners, 3, (int)ui->owner[i], idle, false};
        if (!frontend_panel_add_control(model, &control)) return false;
    }
    model->status = "Host assigns slots before listening. Both peers must open the same image.";
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected,
                   char *error, size_t size) {
    FrontendNetplay *ui = context;
    FrontendExecutionRuntime *execution = ui->execution;
    NesNetplayMode mode = nes_netplay_mode(execution->netplay);
    if (id == NET_DISCONNECT) {
        frontend_execution_begin_machine_change(execution);
        NesNetplayResult result = nes_netplay_disconnect(execution->netplay);
        restored(ui);
        frontend_execution_end_machine_change_preserving_audio(execution);
        if (report(ui, result, error, size)) {
            snprintf(ui->status, sizeof(ui->status), "Disconnected. Previous session restored and paused.");
            return true;
        }
        return false;
    }
    if (mode != NES_NETPLAY_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE)
        return report(ui, NES_NETPLAY_CONFLICT, error, size);
    if (id == NET_HOST) {
        if (!value || !*value || strlen(value) >= sizeof(ui->host)) return false;
        strcpy(ui->host, value);
        return true;
    }
    if (id == NET_PORT) {
        char *end;
        unsigned long port = value ? strtoul(value, &end, 10) : 0;
        if (!port || port > 65535 || *end) return report(ui, NES_NETPLAY_INVALID_ARGUMENT, error, size);
        ui->port = (unsigned)port;
        return true;
    }
    if (id >= NET_PLAYER && id < NET_PLAYER+6 && selected >= 0 && selected < 3) {
        ui->owner[id-NET_PLAYER] = (unsigned)selected;
        return true;
    }
    if (id != NET_LISTEN && id != NET_JOIN) return false;
    if (execution->before_machine_change && !execution->before_machine_change(
            execution->machine_change_context, error, size)) return false;
    unsigned host = 0, guest = 0;
    for (unsigned i = 0; i < 6; ++i) {
        if (ui->owner[i] == 1) host |= 1u << i;
        if (ui->owner[i] == 2) guest |= 1u << i;
    }
    frontend_execution_begin_machine_change(execution);
    NesNetplayResult result = id == NET_LISTEN
        ? nes_netplay_host_listen(execution->netplay, (uint16_t)ui->port, host, guest)
        : nes_netplay_join(execution->netplay, ui->host, (uint16_t)ui->port, 3000);
    if (result == NES_NETPLAY_OK) {
        frontend_execution_clear_timeline(execution);
        debugger_reset_session();
        execution->debugger_pause_revision = debugger_pause_revision();
        if (id == NET_JOIN) sync_pause(ui);
    }
    frontend_execution_end_machine_change_preserving_audio(execution);
    return report(ui, result, error, size);
}

FrontendNetplay *frontend_netplay_create(FrontendExecutionRuntime *execution) {
    FrontendNetplay *ui = calloc(1, sizeof(*ui));
    if (!ui) return NULL;
    ui->execution = execution;
    strcpy(ui->host, "127.0.0.1");
    ui->port = 8964;
    ui->owner[0] = 1; ui->owner[1] = 2;
    return ui;
}

bool frontend_netplay_register(FrontendNetplay *ui) {
    if (!ui || !ui->execution->netplay) return false;
    FrontendPanelSpec panel = {NETPLAY_PANEL, "Netplay", "Tools", FRONTEND_PANEL_NEEDS_SESSION,
                              snapshot, action, ui};
    return frontend_panel_register(&panel);
}

void frontend_netplay_destroy(FrontendNetplay *ui) {
    if (!ui) return;
    frontend_panel_unregister(NETPLAY_PANEL);
    free(ui);
}

const char *frontend_netplay_status(FrontendNetplay *ui) {
    if (!ui) return "Offline";
    NesNetplayProgress progress;
    nes_netplay_progress(ui->execution->netplay, &progress);
    if (progress.mode == NES_NETPLAY_LISTENING)
        snprintf(ui->status, sizeof(ui->status), "Listening on TCP port %u", progress.port);
    else if (progress.mode == NES_NETPLAY_CONNECTED)
        snprintf(ui->status, sizeof(ui->status), "%s, frame %llu, local slots 0x%X%s",
            progress.role == NES_NETPLAY_ROLE_HOST ? "Host" : "Guest",
            (unsigned long long)progress.frame, progress.local_player_mask, progress.paused ? ", paused" : "");
    else if (!ui->status[0]) snprintf(ui->status, sizeof(ui->status), "Offline");
    return ui->status;
}

bool frontend_netplay_pause(FrontendNetplay *ui, bool paused, char *error, size_t size) {
    if (!ui) return false;
    NesNetplayResult result = nes_netplay_set_paused(ui->execution->netplay, paused);
    if (report(ui, result, error, size)) { sync_pause(ui); return true; }
    return false;
}

bool frontend_netplay_before_frame(FrontendNetplay *ui) {
    if (!ui) return true;
    FrontendExecutionRuntime *execution = ui->execution;
    NesNetplayMode before = nes_netplay_mode(execution->netplay);
    if (before == NES_NETPLAY_IDLE || before == NES_NETPLAY_FAILED) return true;
    frontend_execution_begin_machine_change(execution);
    NesNetplayResult result;
    if (before == NES_NETPLAY_LISTENING) {
        result = nes_netplay_host_accept(execution->netplay, 0);
        if (result == NES_NETPLAY_OK) sync_pause(ui);
        frontend_execution_end_machine_change_preserving_audio(execution);
        if (result != NES_NETPLAY_TIMEOUT) (void)report(ui, result, NULL, 0);
        return false;
    }
    result = nes_netplay_frame_begin(execution->netplay, 3000);
    if (nes_netplay_mode(execution->netplay) != NES_NETPLAY_CONNECTED) restored(ui);
    else sync_pause(ui);
    frontend_execution_end_machine_change_preserving_audio(execution);
    (void)report(ui, result, NULL, 0);
    return result == NES_NETPLAY_OK && nes_netplay_mode(execution->netplay) == NES_NETPLAY_CONNECTED;
}

void frontend_netplay_after_frame(FrontendNetplay *ui, bool completed) {
    if (!ui || nes_netplay_mode(ui->execution->netplay) != NES_NETPLAY_CONNECTED) return;
    frontend_execution_begin_machine_change(ui->execution);
    NesNetplayResult result = nes_netplay_frame_complete(ui->execution->netplay, completed);
    if (nes_netplay_mode(ui->execution->netplay) != NES_NETPLAY_CONNECTED) restored(ui);
    frontend_execution_end_machine_change_preserving_audio(ui->execution);
    (void)report(ui, result, NULL, 0);
}
