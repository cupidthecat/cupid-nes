/*
 * netplay.h - Deterministic lockstep network multiplayer
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REPLAY_NETPLAY_H
#define CUPID_REPLAY_NETPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct NesNetplaySession NesNetplaySession;

typedef enum {
    NES_NETPLAY_IDLE = 0,
    NES_NETPLAY_LISTENING,
    NES_NETPLAY_CONNECTED,
    NES_NETPLAY_FAILED
} NesNetplayMode;

typedef enum {
    NES_NETPLAY_ROLE_NONE = 0,
    NES_NETPLAY_ROLE_HOST,
    NES_NETPLAY_ROLE_CLIENT
} NesNetplayRole;

typedef enum {
    NES_NETPLAY_OK = 0,
    NES_NETPLAY_PAUSED,
    NES_NETPLAY_INVALID_ARGUMENT,
    NES_NETPLAY_CONFLICT,
    NES_NETPLAY_NO_IMAGE,
    NES_NETPLAY_UNSUPPORTED_HOST_STATE,
    NES_NETPLAY_OUT_OF_MEMORY,
    NES_NETPLAY_NETWORK_ERROR,
    NES_NETPLAY_TIMEOUT,
    NES_NETPLAY_PROTOCOL_ERROR,
    NES_NETPLAY_VERSION_ERROR,
    NES_NETPLAY_INCOMPATIBLE,
    NES_NETPLAY_STATE_ERROR,
    NES_NETPLAY_INPUT_ERROR,
    NES_NETPLAY_LIMIT_REACHED,
    NES_NETPLAY_DESYNC
} NesNetplayResult;

typedef struct {
    NesNetplayMode mode;
    NesNetplayRole role;
    bool paused;
    uint16_t port;
    uint32_t local_player_mask;
    uint32_t remote_player_mask;
    uint64_t frame;
    size_t queued_local_events;
    NesNetplayResult last_result;
} NesNetplayProgress;

enum {
    NES_NETPLAY_PLAYER_MASK = (1u << 6) - 1u,
    NES_NETPLAY_DEFAULT_TIMEOUT_MS = 3000
};

NesNetplaySession *nes_netplay_create(void);
void nes_netplay_destroy(NesNetplaySession *session);

/* Host setup is two-stage so a desktop frontend can open a listening session
 * and accept a peer separately. Port zero requests an ephemeral port. */
NesNetplayResult nes_netplay_host_listen(NesNetplaySession *session, uint16_t port,
                                         uint32_t host_player_mask,
                                         uint32_t client_player_mask);
NesNetplayResult nes_netplay_host_accept(NesNetplaySession *session,
                                         unsigned timeout_ms);
NesNetplayResult nes_netplay_join(NesNetplaySession *session, const char *host,
                                  uint16_t port, unsigned timeout_ms);

/* Call immediately before and after each authoritative emulation frame.
 * Frame begin exchanges the complete bounded input batch, validates it, applies
 * remote input atomically, and compares canonical hardware-only checkpoints. */
NesNetplayResult nes_netplay_frame_begin(NesNetplaySession *session,
                                         unsigned timeout_ms);
NesNetplayResult nes_netplay_frame_complete(NesNetplaySession *session,
                                            bool completed);

/* Pause is host-owned session control. A client observes pause/resume through
 * frame_begin or poll, including while emulation is stopped. */
NesNetplayResult nes_netplay_set_paused(NesNetplaySession *session, bool paused);
NesNetplayResult nes_netplay_poll(NesNetplaySession *session, unsigned timeout_ms);
bool nes_netplay_paused(const NesNetplaySession *session);

/* Netplay is an isolated timeline. Disconnect restores the exact pre-session
 * live machine and releases every remote-held input with that restore. */
NesNetplayResult nes_netplay_disconnect(NesNetplaySession *session);

NesNetplayMode nes_netplay_mode(const NesNetplaySession *session);
NesNetplayRole nes_netplay_role(const NesNetplaySession *session);
uint16_t nes_netplay_port(const NesNetplaySession *session);
uint32_t nes_netplay_local_player_mask(const NesNetplaySession *session);
uint32_t nes_netplay_remote_player_mask(const NesNetplaySession *session);
void nes_netplay_progress(const NesNetplaySession *session, NesNetplayProgress *progress);
const char *nes_netplay_result_string(NesNetplayResult result);

#ifdef __cplusplus
}
#endif

#endif
