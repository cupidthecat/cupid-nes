/*
 * netplay.c - Versioned deterministic lockstep multiplayer
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "netplay.h"
#include "input_event.h"
#include "netplay_hash.h"
#include "netplay_transport.h"
#include "rewind.h"
#include "../cheats/cheats.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../joypad/joypad.h"
#include "../rom/rom.h"
#include "../state/state.h"
#include "../state/state_io.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"

#include <stdlib.h>
#include <string.h>

#define NETPLAY_MAGIC UINT32_C(0x31504E43) /* CNP1, little endian */

enum {
    NETPLAY_PROTOCOL_VERSION = 1,
    NETPLAY_HEADER_SIZE = 12,
    NETPLAY_MAX_FRAME_EVENTS = 1024
};

typedef enum {
    NETPLAY_PACKET_HELLO = 1,
    NETPLAY_PACKET_WELCOME,
    NETPLAY_PACKET_STATE,
    NETPLAY_PACKET_READY,
    NETPLAY_PACKET_READY_ACK,
    NETPLAY_PACKET_FRAME,
    NETPLAY_PACKET_FRAME_DONE,
    NETPLAY_PACKET_PAUSE,
    NETPLAY_PACKET_DISCONNECT,
    NETPLAY_PACKET_ERROR
} NetplayPacketType;

typedef struct {
    uint16_t type;
    uint8_t *data;
    size_t size;
} NetplayPacket;

typedef struct {
    uint32_t file_crc;
    uint32_t prg_crc;
    uint32_t prg_chr_crc;
    uint32_t cheats;
    uint32_t profile;
    uint32_t region;
    uint32_t adapter;
    uint32_t port0;
    uint32_t port1;
    uint32_t expansion;
} NetplayCompatibility;

struct NesNetplaySession {
    NesNetplayMode mode;
    NesNetplayRole role;
    NesNetplayResult last_result;
    NesNetplaySocket listener;
    NesNetplaySocket peer;
    uint16_t port;
    uint32_t local_player_mask;
    uint32_t remote_player_mask;
    uint64_t frame;
    bool paused;
    bool frame_in_progress;
    bool outbound_frame_sent;
    bool completion_sent;
    bool timeline_active;
    uint32_t previous_policy;
    NesStateRestore *resume_restore;
    NesInputEvent *local_events;
    size_t local_event_count;
    size_t sent_event_count;
    size_t local_event_capacity;
    NesNetplayResult observer_error;
    NetplayPacket pending_packet;
};

static NesNetplayResult netplay_set_result(NesNetplaySession *session,
                                           NesNetplayResult result) {
    if (session) session->last_result = result;
    return result;
}

static void packet_destroy(NetplayPacket *packet) {
    if (!packet) return;
    free(packet->data);
    memset(packet, 0, sizeof(*packet));
}

static NesNetplayResult io_result(NesNetplayIoResult result) {
    switch (result) {
        case NES_NETPLAY_IO_OK: return NES_NETPLAY_OK;
        case NES_NETPLAY_IO_TIMEOUT: return NES_NETPLAY_TIMEOUT;
        case NES_NETPLAY_IO_CLOSED:
        case NES_NETPLAY_IO_ERROR: return NES_NETPLAY_NETWORK_ERROR;
        default: return NES_NETPLAY_NETWORK_ERROR;
    }
}

static NesNetplayResult send_packet(NesNetplaySocket socket, uint16_t type,
                                    const void *payload, size_t payload_size,
                                    unsigned timeout_ms) {
    if (socket == NES_NETPLAY_INVALID_SOCKET || payload_size > UINT32_MAX
        || payload_size > NES_STATE_MAX_SIZE || (payload_size && !payload))
        return NES_NETPLAY_INVALID_ARGUMENT;
    NesStateWriter header;
    nes_state_writer_init(&header, NETPLAY_HEADER_SIZE);
    bool ok = nes_state_write_u32(&header, NETPLAY_MAGIC)
           && nes_state_write_u16(&header, NETPLAY_PROTOCOL_VERSION)
           && nes_state_write_u16(&header, type)
           && nes_state_write_u32(&header, (uint32_t)payload_size)
           && header.size == NETPLAY_HEADER_SIZE;
    if (!ok) {
        nes_state_writer_destroy(&header);
        return NES_NETPLAY_OUT_OF_MEMORY;
    }
    NesNetplayResult result =
        io_result(nes_netplay_socket_send(socket, header.data, header.size, timeout_ms));
    nes_state_writer_destroy(&header);
    if (result != NES_NETPLAY_OK || !payload_size) return result;
    return io_result(nes_netplay_socket_send(socket, payload, payload_size, timeout_ms));
}

static NesNetplayResult receive_packet(NesNetplaySocket socket, unsigned timeout_ms,
                                       NetplayPacket *packet) {
    if (!packet) return NES_NETPLAY_INVALID_ARGUMENT;
    packet_destroy(packet);
    uint8_t header_bytes[NETPLAY_HEADER_SIZE];
    NesNetplayResult result = io_result(nes_netplay_socket_receive(
        socket, header_bytes, sizeof(header_bytes), timeout_ms));
    if (result != NES_NETPLAY_OK) return result;
    NesStateReader header;
    uint32_t magic, payload_size;
    uint16_t version, type;
    nes_state_reader_init(&header, header_bytes, sizeof(header_bytes));
    if (!nes_state_read_u32(&header, &magic)
        || !nes_state_read_u16(&header, &version)
        || !nes_state_read_u16(&header, &type)
        || !nes_state_read_u32(&header, &payload_size)
        || nes_state_reader_remaining(&header) != 0
        || magic != NETPLAY_MAGIC) return NES_NETPLAY_PROTOCOL_ERROR;
    if (version != NETPLAY_PROTOCOL_VERSION) return NES_NETPLAY_VERSION_ERROR;
    if (type < NETPLAY_PACKET_HELLO || type > NETPLAY_PACKET_ERROR
        || payload_size > NES_STATE_MAX_SIZE) return NES_NETPLAY_PROTOCOL_ERROR;
    if (payload_size) {
        packet->data = (uint8_t *)malloc(payload_size);
        if (!packet->data) return NES_NETPLAY_OUT_OF_MEMORY;
        result = io_result(nes_netplay_socket_receive(socket, packet->data,
                                                       payload_size, timeout_ms));
        if (result != NES_NETPLAY_OK) {
            packet_destroy(packet);
            return result;
        }
    }
    packet->type = type;
    packet->size = payload_size;
    return NES_NETPLAY_OK;
}

static bool write_hash(NesStateWriter *writer, const NesNetplayHardwareHash *hash) {
    return hash && nes_state_write_u64(writer, hash->first)
        && nes_state_write_u64(writer, hash->second);
}

static bool read_hash(NesStateReader *reader, NesNetplayHardwareHash *hash) {
    return hash && nes_state_read_u64(reader, &hash->first)
        && nes_state_read_u64(reader, &hash->second);
}

static uint32_t hardware_profile_hash(void) {
    CpuStartupAlignment alignment = cpu_get_configured_startup_alignment();
    const uint32_t values[] = {
        nes_console_model(), nes_ram_power_on_state(), nes_randomize_vblank_enabled(),
        apu_get_cpu_revision(), apu_noise_mode_disabled(), apu_swap_duty_cycles_enabled(),
        ppu_revision(), ppu_oam_row_corruption_worst_case(), ppu_startup_write_restriction_enabled(),
        ppu_oam_decay_enabled(), ppu_oamdata_read_disabled(), ppu_palette_readback_disabled(),
        ppu_reset_suppression_enabled(), ppu_sprite_eval_wrap_bug_enabled(),
        cpu_test_mode_enabled(), cpu_get_startup_alignment_mode(),
        alignment.cpu_offset, alignment.ppu_phase, cpu_get_startup_alignment_seed(),
        cart_dip_switches(), vs_dip_switches()
    };
    uint32_t hash = UINT32_C(2166136261);
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
        for (unsigned shift = 0; shift < 32; shift += 8)
            hash = (hash ^ ((values[i] >> shift) & 255u)) * UINT32_C(16777619);
    const char *revision = cart_mmc3_revision_name();
    while (revision && *revision) hash = (hash ^ (uint8_t)*revision++) * UINT32_C(16777619);
    return hash;
}

static NetplayCompatibility current_compatibility(void) {
    const NesTiming *timing = nes_timing();
    NetplayCompatibility result = {
        rom_file_crc32(), rom_prg_crc32(), rom_prg_chr_crc32(),
        cheats_compatibility_hash(), hardware_profile_hash(),
        timing ? (uint32_t)timing->region : UINT32_MAX,
        (uint32_t)joypad_adapter(),
        (uint32_t)joypad_port_device(0),
        (uint32_t)joypad_port_device(1),
        (uint32_t)joypad_expansion_device()
    };
    return result;
}

static bool compatibility_equal(const NetplayCompatibility *left,
                                const NetplayCompatibility *right) {
    return left && right && memcmp(left, right, sizeof(*left)) == 0;
}

static bool write_compatibility(NesStateWriter *writer,
                                const NetplayCompatibility *compatibility) {
    return writer && compatibility
        && nes_state_write_u32(writer, compatibility->file_crc)
        && nes_state_write_u32(writer, compatibility->prg_crc)
        && nes_state_write_u32(writer, compatibility->prg_chr_crc)
        && nes_state_write_u32(writer, compatibility->cheats)
        && nes_state_write_u32(writer, compatibility->profile)
        && nes_state_write_u32(writer, compatibility->region)
        && nes_state_write_u32(writer, compatibility->adapter)
        && nes_state_write_u32(writer, compatibility->port0)
        && nes_state_write_u32(writer, compatibility->port1)
        && nes_state_write_u32(writer, compatibility->expansion);
}

static bool read_compatibility(NesStateReader *reader,
                               NetplayCompatibility *compatibility) {
    return reader && compatibility
        && nes_state_read_u32(reader, &compatibility->file_crc)
        && nes_state_read_u32(reader, &compatibility->prg_crc)
        && nes_state_read_u32(reader, &compatibility->prg_chr_crc)
        && nes_state_read_u32(reader, &compatibility->cheats)
        && nes_state_read_u32(reader, &compatibility->profile)
        && nes_state_read_u32(reader, &compatibility->region)
        && nes_state_read_u32(reader, &compatibility->adapter)
        && nes_state_read_u32(reader, &compatibility->port0)
        && nes_state_read_u32(reader, &compatibility->port1)
        && nes_state_read_u32(reader, &compatibility->expansion);
}

static bool write_event(NesStateWriter *writer, const NesInputEvent *event) {
    return nes_input_event_validate(event)
        && nes_state_write_u32(writer, (uint32_t)event->type)
        && nes_state_write_u32(writer, (uint32_t)event->a)
        && nes_state_write_u32(writer, (uint32_t)event->b)
        && nes_state_write_u32(writer, (uint32_t)event->c)
        && nes_state_write_u32(writer, (uint32_t)event->d)
        && nes_state_write_bytes(writer, event->text, sizeof(event->text));
}

static bool read_event(NesStateReader *reader, NesInputEvent *event) {
    uint32_t type, a, b, c, d;
    memset(event, 0, sizeof(*event));
    if (!nes_state_read_u32(reader, &type)
        || !nes_state_read_u32(reader, &a)
        || !nes_state_read_u32(reader, &b)
        || !nes_state_read_u32(reader, &c)
        || !nes_state_read_u32(reader, &d)
        || !nes_state_read_bytes(reader, event->text, sizeof(event->text)))
        return false;
    event->type = (NesInputEventType)type;
    event->a = (int32_t)a;
    event->b = (int32_t)b;
    event->c = (int32_t)c;
    event->d = (int32_t)d;
    return nes_input_event_validate(event);
}

static bool event_owned_by_mask(const NesInputEvent *event, uint32_t player_mask,
                                bool global_owner) {
    if (!event || !nes_input_event_validate(event)) return false;
    if (event->type == NES_INPUT_EVENT_PLAYER_BUTTON) {
        unsigned player = (unsigned)event->a;
        return player < NES_INPUT_PLAYERS && (player_mask & (1u << player)) != 0;
    }
    return global_owner;
}

static bool reserve_local_events(NesNetplaySession *session, size_t additional) {
    if (!session || additional > NETPLAY_MAX_FRAME_EVENTS - session->local_event_count)
        return false;
    size_t needed = session->local_event_count + additional;
    if (needed <= session->local_event_capacity) return true;
    size_t grown = session->local_event_capacity ? session->local_event_capacity : 32;
    while (grown < needed) {
        if (grown >= NETPLAY_MAX_FRAME_EVENTS / 2) {
            grown = NETPLAY_MAX_FRAME_EVENTS;
            break;
        }
        grown *= 2;
    }
    if (grown > SIZE_MAX / sizeof(*session->local_events)) return false;
    NesInputEvent *replacement =
        (NesInputEvent *)realloc(session->local_events,
                                 grown * sizeof(*session->local_events));
    if (!replacement) return false;
    session->local_events = replacement;
    session->local_event_capacity = grown;
    return true;
}

static bool netplay_input_observer(const NesInputEvent *event, void *userdata) {
    NesNetplaySession *session = (NesNetplaySession *)userdata;
    if (!session || session->mode != NES_NETPLAY_CONNECTED
        || session->observer_error != NES_NETPLAY_OK
        || !event_owned_by_mask(event, session->local_player_mask,
                                session->role == NES_NETPLAY_ROLE_HOST))
        return false;
    if (session->local_event_count >= NETPLAY_MAX_FRAME_EVENTS
        || !reserve_local_events(session, 1)) {
        session->observer_error = session->local_event_count >= NETPLAY_MAX_FRAME_EVENTS
                                ? NES_NETPLAY_LIMIT_REACHED
                                : NES_NETPLAY_OUT_OF_MEMORY;
        session->last_result = session->observer_error;
        return false;
    }
    session->local_events[session->local_event_count++] = *event;
    /* Lockstep applies both peers' complete event batches at the frame boundary. */
    return false;
}

static NesNetplayResult state_result(NesStateResult result) {
    switch (result) {
        case NES_STATE_OK: return NES_NETPLAY_OK;
        case NES_STATE_ERROR_NO_IMAGE: return NES_NETPLAY_NO_IMAGE;
        case NES_STATE_ERROR_OUT_OF_MEMORY: return NES_NETPLAY_OUT_OF_MEMORY;
        case NES_STATE_ERROR_VERSION: return NES_NETPLAY_VERSION_ERROR;
        case NES_STATE_ERROR_INCOMPATIBLE: return NES_NETPLAY_INCOMPATIBLE;
        default: return NES_NETPLAY_STATE_ERROR;
    }
}

static NesNetplayResult prepare_resume(NesNetplaySession *session,
                                       NesStateBlob *captured) {
    NesStateResult state = nes_state_capture(captured);
    NesNetplayResult result = state_result(state);
    if (result != NES_NETPLAY_OK) return result;
    state = nes_state_prepare_restore(captured->data, captured->size,
                                      &session->resume_restore);
    return state_result(state);
}

static void discard_resume(NesNetplaySession *session) {
    if (!session) return;
    nes_state_restore_free(session->resume_restore);
    session->resume_restore = NULL;
}

static void clear_pending_packet(NesNetplaySession *session) {
    if (session) packet_destroy(&session->pending_packet);
}

static NesNetplayResult restore_live_timeline(NesNetplaySession *session) {
    if (!session || !session->timeline_active) return NES_NETPLAY_OK;
    nes_input_event_clear_observer();
    NesNetplayResult result = NES_NETPLAY_OK;
    if (!session->resume_restore
        || nes_state_apply_prepared(session->resume_restore) != NES_STATE_OK)
        result = NES_NETPLAY_STATE_ERROR;
    discard_resume(session);
    if (!nes_execution_set_policy(session->previous_policy)
        && result == NES_NETPLAY_OK) result = NES_NETPLAY_CONFLICT;
    session->timeline_active = false;
    return result;
}

static void reset_frame_state(NesNetplaySession *session) {
    session->frame = 0;
    session->paused = false;
    session->frame_in_progress = false;
    session->outbound_frame_sent = false;
    session->completion_sent = false;
    session->local_event_count = 0;
    session->observer_error = NES_NETPLAY_OK;
    clear_pending_packet(session);
}

static NesNetplayResult fail_connected(NesNetplaySession *session,
                                       NesNetplayResult reason) {
    if (!session) return reason;
    nes_netplay_socket_close(&session->peer);
    nes_netplay_socket_close(&session->listener);
    NesNetplayResult restore = restore_live_timeline(session);
    reset_frame_state(session);
    session->mode = NES_NETPLAY_FAILED;
    if (restore != NES_NETPLAY_OK) reason = restore;
    return netplay_set_result(session, reason);
}

static NesNetplayResult clean_remote_disconnect(NesNetplaySession *session) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    nes_netplay_socket_close(&session->peer);
    nes_netplay_socket_close(&session->listener);
    NesNetplayResult restore = restore_live_timeline(session);
    reset_frame_state(session);
    session->mode = NES_NETPLAY_IDLE;
    session->role = NES_NETPLAY_ROLE_NONE;
    session->port = 0;
    session->local_player_mask = 0;
    session->remote_player_mask = 0;
    return netplay_set_result(session, restore);
}

static NesNetplayResult send_error(NesNetplaySocket socket, NesNetplayResult error,
                                   unsigned timeout_ms) {
    NesStateWriter writer;
    nes_state_writer_init(&writer, 4);
    if (!nes_state_write_u32(&writer, (uint32_t)error)) {
        nes_state_writer_destroy(&writer);
        return NES_NETPLAY_OUT_OF_MEMORY;
    }
    NesNetplayResult result = send_packet(socket, NETPLAY_PACKET_ERROR,
                                          writer.data, writer.size, timeout_ms);
    nes_state_writer_destroy(&writer);
    return result;
}

static bool read_remote_error(const NetplayPacket *packet, NesNetplayResult *error) {
    if (!packet || packet->type != NETPLAY_PACKET_ERROR || !error) return false;
    NesStateReader reader;
    uint32_t value;
    nes_state_reader_init(&reader, packet->data, packet->size);
    if (!nes_state_read_u32(&reader, &value)
        || nes_state_reader_remaining(&reader) != 0
        || value > NES_NETPLAY_DESYNC) return false;
    *error = (NesNetplayResult)value;
    return true;
}

static NesNetplayResult host_handshake_failure(NesNetplaySession *session,
                                               NesNetplayResult result) {
    nes_netplay_socket_close(&session->peer);
    discard_resume(session);
    session->last_result = result;
    return result;
}

static NesNetplayResult client_handshake_failure(NesNetplaySession *session,
                                                 NesNetplayResult result) {
    nes_netplay_socket_close(&session->peer);
    NesNetplayResult restore = restore_live_timeline(session);
    discard_resume(session);
    session->mode = NES_NETPLAY_FAILED;
    if (restore != NES_NETPLAY_OK) result = restore;
    return netplay_set_result(session, result);
}

static bool valid_assignment(uint32_t host_mask, uint32_t client_mask) {
    return host_mask && client_mask
        && !(host_mask & ~NES_NETPLAY_PLAYER_MASK)
        && !(client_mask & ~NES_NETPLAY_PLAYER_MASK)
        && !(host_mask & client_mask);
}

static NesNetplayResult activate_policy(NesNetplaySession *session) {
    session->previous_policy = nes_execution_policy();
    if (session->previous_policy != NES_EXECUTION_LIVE
        || !nes_execution_set_policy(session->previous_policy | NES_EXECUTION_NETPLAY))
        return NES_NETPLAY_CONFLICT;
    session->timeline_active = true;
    return NES_NETPLAY_OK;
}

NesNetplaySession *nes_netplay_create(void) {
    if (!nes_netplay_socket_system_init()) return NULL;
    NesNetplaySession *session = (NesNetplaySession *)calloc(1, sizeof(*session));
    if (!session) {
        nes_netplay_socket_system_shutdown();
        return NULL;
    }
    session->listener = NES_NETPLAY_INVALID_SOCKET;
    session->peer = NES_NETPLAY_INVALID_SOCKET;
    session->last_result = NES_NETPLAY_OK;
    session->observer_error = NES_NETPLAY_OK;
    return session;
}

void nes_netplay_destroy(NesNetplaySession *session) {
    if (!session) return;
    if (session->mode != NES_NETPLAY_IDLE) (void)nes_netplay_disconnect(session);
    discard_resume(session);
    packet_destroy(&session->pending_packet);
    free(session->local_events);
    nes_netplay_socket_close(&session->peer);
    nes_netplay_socket_close(&session->listener);
    free(session);
    nes_netplay_socket_system_shutdown();
}

NesNetplayResult nes_netplay_host_listen(NesNetplaySession *session, uint16_t port,
                                         uint32_t host_player_mask,
                                         uint32_t client_player_mask) {
    if (!session || !valid_assignment(host_player_mask, client_player_mask))
        return netplay_set_result(session, NES_NETPLAY_INVALID_ARGUMENT);
    if (session->mode != NES_NETPLAY_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (!nes_replay_host_state_supported())
        return netplay_set_result(session, NES_NETPLAY_UNSUPPORTED_HOST_STATE);
    if (rom_metadata_source() == ROM_METADATA_NONE)
        return netplay_set_result(session, NES_NETPLAY_NO_IMAGE);
    uint16_t bound_port = 0;
    session->listener = nes_netplay_socket_listen(port, &bound_port);
    if (session->listener == NES_NETPLAY_INVALID_SOCKET)
        return netplay_set_result(session, NES_NETPLAY_NETWORK_ERROR);
    session->mode = NES_NETPLAY_LISTENING;
    session->role = NES_NETPLAY_ROLE_HOST;
    session->port = bound_port;
    session->local_player_mask = host_player_mask;
    session->remote_player_mask = client_player_mask;
    reset_frame_state(session);
    return netplay_set_result(session, NES_NETPLAY_OK);
}

NesNetplayResult nes_netplay_host_accept(NesNetplaySession *session,
                                         unsigned timeout_ms) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode != NES_NETPLAY_LISTENING || session->role != NES_NETPLAY_ROLE_HOST)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (!nes_replay_host_state_supported())
        return netplay_set_result(session, NES_NETPLAY_UNSUPPORTED_HOST_STATE);
    session->peer = nes_netplay_socket_accept(session->listener, timeout_ms);
    if (session->peer == NES_NETPLAY_INVALID_SOCKET)
        return netplay_set_result(session, NES_NETPLAY_TIMEOUT);
    unsigned handshake_timeout = timeout_ms ? timeout_ms : NES_NETPLAY_DEFAULT_TIMEOUT_MS;

    NetplayPacket packet = {0};
    NesNetplayResult result = receive_packet(session->peer, handshake_timeout, &packet);
    if (result != NES_NETPLAY_OK) return host_handshake_failure(session, result);
    NetplayCompatibility remote;
    NesStateReader reader;
    nes_state_reader_init(&reader, packet.data, packet.size);
    if (packet.type != NETPLAY_PACKET_HELLO
        || !read_compatibility(&reader, &remote)
        || nes_state_reader_remaining(&reader) != 0) {
        packet_destroy(&packet);
        (void)send_error(session->peer, NES_NETPLAY_PROTOCOL_ERROR, handshake_timeout);
        return host_handshake_failure(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    packet_destroy(&packet);
    NetplayCompatibility local = current_compatibility();
    if (!compatibility_equal(&local, &remote)) {
        (void)send_error(session->peer, NES_NETPLAY_INCOMPATIBLE, handshake_timeout);
        return host_handshake_failure(session, NES_NETPLAY_INCOMPATIBLE);
    }

    NesStateWriter welcome;
    nes_state_writer_init(&welcome, 64);
    bool welcome_ok = write_compatibility(&welcome, &local)
        && nes_state_write_u32(&welcome, session->local_player_mask)
        && nes_state_write_u32(&welcome, session->remote_player_mask);
    if (!welcome_ok) {
        nes_state_writer_destroy(&welcome);
        return host_handshake_failure(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    result = send_packet(session->peer, NETPLAY_PACKET_WELCOME,
                         welcome.data, welcome.size, handshake_timeout);
    nes_state_writer_destroy(&welcome);
    if (result != NES_NETPLAY_OK) return host_handshake_failure(session, result);

    NesStateBlob start = {0};
    result = prepare_resume(session, &start);
    if (result != NES_NETPLAY_OK) {
        (void)send_error(session->peer, result, handshake_timeout);
        nes_state_blob_free(&start);
        return host_handshake_failure(session, result);
    }
    result = send_packet(session->peer, NETPLAY_PACKET_STATE,
                         start.data, start.size, handshake_timeout);
    nes_state_blob_free(&start);
    if (result != NES_NETPLAY_OK) return host_handshake_failure(session, result);

    result = receive_packet(session->peer, handshake_timeout, &packet);
    if (result != NES_NETPLAY_OK) return host_handshake_failure(session, result);
    NesNetplayResult remote_error;
    if (read_remote_error(&packet, &remote_error)) {
        packet_destroy(&packet);
        return host_handshake_failure(session, remote_error);
    }
    NesNetplayHardwareHash remote_hash, local_hash;
    nes_state_reader_init(&reader, packet.data, packet.size);
    if (packet.type != NETPLAY_PACKET_READY || !read_hash(&reader, &remote_hash)
        || nes_state_reader_remaining(&reader) != 0) {
        packet_destroy(&packet);
        (void)send_error(session->peer, NES_NETPLAY_PROTOCOL_ERROR, handshake_timeout);
        return host_handshake_failure(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    packet_destroy(&packet);
    if (!nes_netplay_hardware_hash(&local_hash))
        return host_handshake_failure(session, NES_NETPLAY_STATE_ERROR);
    if (!nes_netplay_hardware_hash_equal(&local_hash, &remote_hash)) {
        (void)send_error(session->peer, NES_NETPLAY_DESYNC, handshake_timeout);
        return host_handshake_failure(session, NES_NETPLAY_DESYNC);
    }
    result = activate_policy(session);
    if (result != NES_NETPLAY_OK)
        return host_handshake_failure(session, result);

    NesStateWriter ready;
    nes_state_writer_init(&ready, 16);
    if (!write_hash(&ready, &local_hash)) {
        nes_state_writer_destroy(&ready);
        return fail_connected(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    result = send_packet(session->peer, NETPLAY_PACKET_READY_ACK,
                         ready.data, ready.size, handshake_timeout);
    nes_state_writer_destroy(&ready);
    if (result != NES_NETPLAY_OK) return fail_connected(session, result);
    nes_netplay_socket_close(&session->listener);
    session->mode = NES_NETPLAY_CONNECTED;
    session->last_result = NES_NETPLAY_OK;
    nes_input_event_set_observer(netplay_input_observer, session);
    return NES_NETPLAY_OK;
}

NesNetplayResult nes_netplay_join(NesNetplaySession *session, const char *host,
                                  uint16_t port, unsigned timeout_ms) {
    if (!session || !host || !*host || !port)
        return netplay_set_result(session, NES_NETPLAY_INVALID_ARGUMENT);
    if (session->mode != NES_NETPLAY_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (!nes_replay_host_state_supported())
        return netplay_set_result(session, NES_NETPLAY_UNSUPPORTED_HOST_STATE);
    if (rom_metadata_source() == ROM_METADATA_NONE)
        return netplay_set_result(session, NES_NETPLAY_NO_IMAGE);

    NesStateBlob live = {0};
    NesNetplayResult result = prepare_resume(session, &live);
    nes_state_blob_free(&live);
    if (result != NES_NETPLAY_OK) return netplay_set_result(session, result);
    session->peer = nes_netplay_socket_connect(host, port, timeout_ms);
    if (session->peer == NES_NETPLAY_INVALID_SOCKET) {
        discard_resume(session);
        return netplay_set_result(session, NES_NETPLAY_NETWORK_ERROR);
    }
    session->role = NES_NETPLAY_ROLE_CLIENT;
    session->port = port;

    NetplayCompatibility local = current_compatibility();
    NesStateWriter hello;
    nes_state_writer_init(&hello, 48);
    if (!write_compatibility(&hello, &local)) {
        nes_state_writer_destroy(&hello);
        return client_handshake_failure(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    result = send_packet(session->peer, NETPLAY_PACKET_HELLO,
                         hello.data, hello.size, timeout_ms);
    nes_state_writer_destroy(&hello);
    if (result != NES_NETPLAY_OK) return client_handshake_failure(session, result);

    NetplayPacket packet = {0};
    result = receive_packet(session->peer, timeout_ms, &packet);
    if (result != NES_NETPLAY_OK) return client_handshake_failure(session, result);
    NesNetplayResult remote_error;
    if (read_remote_error(&packet, &remote_error)) {
        packet_destroy(&packet);
        return client_handshake_failure(session, remote_error);
    }
    NesStateReader reader;
    NetplayCompatibility remote;
    uint32_t host_mask, client_mask;
    nes_state_reader_init(&reader, packet.data, packet.size);
    if (packet.type != NETPLAY_PACKET_WELCOME
        || !read_compatibility(&reader, &remote)
        || !nes_state_read_u32(&reader, &host_mask)
        || !nes_state_read_u32(&reader, &client_mask)
        || nes_state_reader_remaining(&reader) != 0
        || !valid_assignment(host_mask, client_mask)) {
        packet_destroy(&packet);
        (void)send_error(session->peer, NES_NETPLAY_PROTOCOL_ERROR, timeout_ms);
        return client_handshake_failure(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    packet_destroy(&packet);
    if (!compatibility_equal(&local, &remote)) {
        (void)send_error(session->peer, NES_NETPLAY_INCOMPATIBLE, timeout_ms);
        return client_handshake_failure(session, NES_NETPLAY_INCOMPATIBLE);
    }
    session->local_player_mask = client_mask;
    session->remote_player_mask = host_mask;

    result = receive_packet(session->peer, timeout_ms, &packet);
    if (result != NES_NETPLAY_OK) return client_handshake_failure(session, result);
    if (read_remote_error(&packet, &remote_error)) {
        packet_destroy(&packet);
        return client_handshake_failure(session, remote_error);
    }
    if (packet.type != NETPLAY_PACKET_STATE || !packet.size) {
        packet_destroy(&packet);
        (void)send_error(session->peer, NES_NETPLAY_PROTOCOL_ERROR, timeout_ms);
        return client_handshake_failure(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    NesStateRestore *host_restore = NULL;
    NesStateResult state = nes_state_prepare_restore(packet.data, packet.size, &host_restore);
    packet_destroy(&packet);
    result = state_result(state);
    if (result != NES_NETPLAY_OK) {
        (void)send_error(session->peer, result, timeout_ms);
        nes_state_restore_free(host_restore);
        return client_handshake_failure(session, result);
    }
    result = activate_policy(session);
    if (result != NES_NETPLAY_OK) {
        nes_state_restore_free(host_restore);
        return client_handshake_failure(session, result);
    }
    state = nes_state_apply_prepared(host_restore);
    nes_state_restore_free(host_restore);
    if (state != NES_STATE_OK) {
        (void)send_error(session->peer, NES_NETPLAY_STATE_ERROR, timeout_ms);
        return client_handshake_failure(session, NES_NETPLAY_STATE_ERROR);
    }

    NesNetplayHardwareHash local_hash;
    if (!nes_netplay_hardware_hash(&local_hash)) {
        (void)send_error(session->peer, NES_NETPLAY_STATE_ERROR, timeout_ms);
        return client_handshake_failure(session, NES_NETPLAY_STATE_ERROR);
    }
    NesStateWriter ready;
    nes_state_writer_init(&ready, 16);
    if (!write_hash(&ready, &local_hash)) {
        nes_state_writer_destroy(&ready);
        return client_handshake_failure(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    result = send_packet(session->peer, NETPLAY_PACKET_READY,
                         ready.data, ready.size, timeout_ms);
    nes_state_writer_destroy(&ready);
    if (result != NES_NETPLAY_OK) return client_handshake_failure(session, result);

    result = receive_packet(session->peer, timeout_ms, &packet);
    if (result != NES_NETPLAY_OK) return client_handshake_failure(session, result);
    if (read_remote_error(&packet, &remote_error)) {
        packet_destroy(&packet);
        return client_handshake_failure(session, remote_error);
    }
    NesNetplayHardwareHash host_hash;
    nes_state_reader_init(&reader, packet.data, packet.size);
    if (packet.type != NETPLAY_PACKET_READY_ACK || !read_hash(&reader, &host_hash)
        || nes_state_reader_remaining(&reader) != 0) {
        packet_destroy(&packet);
        return client_handshake_failure(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    packet_destroy(&packet);
    if (!nes_netplay_hardware_hash_equal(&local_hash, &host_hash))
        return client_handshake_failure(session, NES_NETPLAY_DESYNC);
    session->mode = NES_NETPLAY_CONNECTED;
    session->last_result = NES_NETPLAY_OK;
    reset_frame_state(session);
    nes_input_event_set_observer(netplay_input_observer, session);
    return NES_NETPLAY_OK;
}

static NesNetplayResult store_pending_packet(NesNetplaySession *session,
                                             NetplayPacket *packet) {
    if (session->pending_packet.type) return NES_NETPLAY_PROTOCOL_ERROR;
    session->pending_packet = *packet;
    memset(packet, 0, sizeof(*packet));
    return NES_NETPLAY_OK;
}

static NesNetplayResult next_packet(NesNetplaySession *session, unsigned timeout_ms,
                                    NetplayPacket *packet) {
    if (session->pending_packet.type) {
        *packet = session->pending_packet;
        memset(&session->pending_packet, 0, sizeof(session->pending_packet));
        return NES_NETPLAY_OK;
    }
    return receive_packet(session->peer, timeout_ms, packet);
}

static NesNetplayResult handle_control_packet(NesNetplaySession *session,
                                              NetplayPacket *packet,
                                              bool *handled) {
    *handled = false;
    if (packet->type == NETPLAY_PACKET_PAUSE) {
        *handled = true;
        if (session->role != NES_NETPLAY_ROLE_CLIENT) return NES_NETPLAY_PROTOCOL_ERROR;
        NesStateReader reader;
        bool paused;
        nes_state_reader_init(&reader, packet->data, packet->size);
        if (!nes_state_read_bool(&reader, &paused)
            || nes_state_reader_remaining(&reader) != 0)
            return NES_NETPLAY_PROTOCOL_ERROR;
        session->paused = paused;
        return NES_NETPLAY_OK;
    }
    if (packet->type == NETPLAY_PACKET_DISCONNECT) {
        *handled = true;
        if (packet->size) return NES_NETPLAY_PROTOCOL_ERROR;
        return clean_remote_disconnect(session);
    }
    if (packet->type == NETPLAY_PACKET_ERROR) {
        *handled = true;
        NesNetplayResult remote_error;
        if (!read_remote_error(packet, &remote_error)) return NES_NETPLAY_PROTOCOL_ERROR;
        return remote_error;
    }
    return NES_NETPLAY_OK;
}

NesNetplayResult nes_netplay_poll(NesNetplaySession *session, unsigned timeout_ms) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode != NES_NETPLAY_CONNECTED)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    NesNetplayIoResult available = nes_netplay_socket_wait_readable(session->peer, timeout_ms);
    if (available == NES_NETPLAY_IO_TIMEOUT) return netplay_set_result(session, NES_NETPLAY_OK);
    if (available != NES_NETPLAY_IO_OK)
        return fail_connected(session, NES_NETPLAY_NETWORK_ERROR);
    NetplayPacket packet = {0};
    NesNetplayResult result = receive_packet(
        session->peer, timeout_ms ? timeout_ms : NES_NETPLAY_DEFAULT_TIMEOUT_MS, &packet);
    if (result != NES_NETPLAY_OK) return fail_connected(session, result);
    bool handled;
    result = handle_control_packet(session, &packet, &handled);
    if (!handled && result == NES_NETPLAY_OK)
        result = store_pending_packet(session, &packet);
    packet_destroy(&packet);
    if (result != NES_NETPLAY_OK) return fail_connected(session, result);
    return netplay_set_result(session, session->paused ? NES_NETPLAY_PAUSED
                                                       : NES_NETPLAY_OK);
}

NesNetplayResult nes_netplay_set_paused(NesNetplaySession *session, bool paused) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode != NES_NETPLAY_CONNECTED
        || session->role != NES_NETPLAY_ROLE_HOST
        || session->frame_in_progress)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (session->paused == paused) return netplay_set_result(
        session, paused ? NES_NETPLAY_PAUSED : NES_NETPLAY_OK);
    NesStateWriter writer;
    nes_state_writer_init(&writer, 1);
    if (!nes_state_write_bool(&writer, paused)) {
        nes_state_writer_destroy(&writer);
        return fail_connected(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    NesNetplayResult result = send_packet(session->peer, NETPLAY_PACKET_PAUSE,
                                          writer.data, writer.size,
                                          NES_NETPLAY_DEFAULT_TIMEOUT_MS);
    nes_state_writer_destroy(&writer);
    if (result != NES_NETPLAY_OK) return fail_connected(session, result);
    session->paused = paused;
    return netplay_set_result(session, paused ? NES_NETPLAY_PAUSED : NES_NETPLAY_OK);
}

static NesNetplayResult send_frame(NesNetplaySession *session, unsigned timeout_ms) {
    if (session->observer_error != NES_NETPLAY_OK)
        return fail_connected(session, session->observer_error);
    NesNetplayHardwareHash hash;
    if (!nes_netplay_hardware_hash(&hash))
        return fail_connected(session, NES_NETPLAY_STATE_ERROR);
    NesStateWriter writer;
    nes_state_writer_init(&writer, NES_STATE_MAX_SIZE);
    bool ok = nes_state_write_u64(&writer, session->frame)
        && write_hash(&writer, &hash)
        && session->local_event_count <= UINT32_MAX
        && nes_state_write_u32(&writer, (uint32_t)session->local_event_count);
    for (size_t i = 0; ok && i < session->local_event_count; ++i)
        ok = write_event(&writer, &session->local_events[i]);
    if (!ok) {
        nes_state_writer_destroy(&writer);
        return fail_connected(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    NesNetplayResult result = send_packet(session->peer, NETPLAY_PACKET_FRAME,
                                          writer.data, writer.size, timeout_ms);
    nes_state_writer_destroy(&writer);
    if (result != NES_NETPLAY_OK) return fail_connected(session, result);
    session->sent_event_count = session->local_event_count;
    session->outbound_frame_sent = true;
    return NES_NETPLAY_OK;
}

static NesNetplayResult receive_expected(NesNetplaySession *session, uint16_t type,
                                         unsigned timeout_ms, NetplayPacket *packet) {
    for (;;) {
        NesNetplayResult result = next_packet(session, timeout_ms, packet);
        if (result != NES_NETPLAY_OK) return fail_connected(session, result);
        bool handled;
        result = handle_control_packet(session, packet, &handled);
        if (handled) {
            packet_destroy(packet);
            if (result != NES_NETPLAY_OK) return fail_connected(session, result);
            if (session->mode != NES_NETPLAY_CONNECTED)
                return netplay_set_result(session, NES_NETPLAY_NETWORK_ERROR);
            if (session->paused) return netplay_set_result(session, NES_NETPLAY_PAUSED);
            continue;
        }
        if (packet->type != type) {
            packet_destroy(packet);
            return fail_connected(session, NES_NETPLAY_PROTOCOL_ERROR);
        }
        return NES_NETPLAY_OK;
    }
}

static bool apply_events(const NesInputEvent *events, size_t count) {
    for (size_t i = 0; i < count; ++i)
        if (!nes_input_event_apply(&events[i])) return false;
    return true;
}

NesNetplayResult nes_netplay_frame_begin(NesNetplaySession *session,
                                         unsigned timeout_ms) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode != NES_NETPLAY_CONNECTED)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (session->frame_in_progress) return netplay_set_result(session, NES_NETPLAY_OK);

    if (session->role == NES_NETPLAY_ROLE_CLIENT || session->paused) {
        NesNetplayResult poll = nes_netplay_poll(session, 0);
        if (poll != NES_NETPLAY_OK) return poll;
    }
    if (session->paused) return netplay_set_result(session, NES_NETPLAY_PAUSED);
    if (!session->outbound_frame_sent) {
        NesNetplayResult sent = send_frame(session, timeout_ms);
        if (sent != NES_NETPLAY_OK) return sent;
    }

    NetplayPacket packet = {0};
    NesNetplayResult result =
        receive_expected(session, NETPLAY_PACKET_FRAME, timeout_ms, &packet);
    if (result != NES_NETPLAY_OK) return result;
    NesStateReader reader;
    uint64_t frame;
    uint32_t event_count;
    NesNetplayHardwareHash remote_hash, local_hash;
    nes_state_reader_init(&reader, packet.data, packet.size);
    if (!nes_state_read_u64(&reader, &frame) || frame != session->frame
        || !read_hash(&reader, &remote_hash)
        || !nes_state_read_u32(&reader, &event_count)
        || event_count > NETPLAY_MAX_FRAME_EVENTS) {
        packet_destroy(&packet);
        return fail_connected(session, NES_NETPLAY_PROTOCOL_ERROR);
    }
    NesInputEvent *remote_events = event_count
        ? (NesInputEvent *)malloc((size_t)event_count * sizeof(*remote_events)) : NULL;
    if (event_count && !remote_events) {
        packet_destroy(&packet);
        return fail_connected(session, NES_NETPLAY_OUT_OF_MEMORY);
    }
    bool events_ok = true;
    for (uint32_t i = 0; i < event_count; ++i) {
        if (!read_event(&reader, &remote_events[i])
            || !event_owned_by_mask(&remote_events[i], session->remote_player_mask,
                                    session->role == NES_NETPLAY_ROLE_CLIENT)) {
            events_ok = false;
            break;
        }
    }
    if (!events_ok || nes_state_reader_remaining(&reader) != 0) {
        free(remote_events);
        packet_destroy(&packet);
        return fail_connected(session, NES_NETPLAY_INPUT_ERROR);
    }
    packet_destroy(&packet);
    if (!nes_netplay_hardware_hash(&local_hash)) {
        free(remote_events);
        return fail_connected(session, NES_NETPLAY_STATE_ERROR);
    }
    if (!nes_netplay_hardware_hash_equal(&local_hash, &remote_hash)) {
        free(remote_events);
        (void)send_error(session->peer, NES_NETPLAY_DESYNC, timeout_ms);
        return fail_connected(session, NES_NETPLAY_DESYNC);
    }

    bool applied;
    if (session->role == NES_NETPLAY_ROLE_HOST)
        applied = apply_events(session->local_events, session->sent_event_count)
               && apply_events(remote_events, event_count);
    else
        applied = apply_events(remote_events, event_count)
               && apply_events(session->local_events, session->sent_event_count);
    free(remote_events);
    if (!applied) return fail_connected(session, NES_NETPLAY_INPUT_ERROR);
    session->local_event_count -= session->sent_event_count;
    if (session->local_event_count)
        memmove(session->local_events, session->local_events + session->sent_event_count,
                session->local_event_count * sizeof(*session->local_events));
    session->sent_event_count = 0;
    session->outbound_frame_sent = false;
    session->frame_in_progress = true;
    return netplay_set_result(session, NES_NETPLAY_OK);
}

NesNetplayResult nes_netplay_frame_complete(NesNetplaySession *session,
                                            bool completed) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode != NES_NETPLAY_CONNECTED)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (!session->frame_in_progress || !completed)
        return netplay_set_result(session, NES_NETPLAY_OK);
    unsigned timeout_ms = NES_NETPLAY_DEFAULT_TIMEOUT_MS;
    if (!session->completion_sent) {
        NesNetplayHardwareHash hash;
        if (!nes_netplay_hardware_hash(&hash))
            return fail_connected(session, NES_NETPLAY_STATE_ERROR);
        NesStateWriter writer;
        nes_state_writer_init(&writer, 24);
        if (!nes_state_write_u64(&writer, session->frame) || !write_hash(&writer, &hash)) {
            nes_state_writer_destroy(&writer);
            return fail_connected(session, NES_NETPLAY_OUT_OF_MEMORY);
        }
        NesNetplayResult result = send_packet(session->peer, NETPLAY_PACKET_FRAME_DONE,
                                              writer.data, writer.size, timeout_ms);
        nes_state_writer_destroy(&writer);
        if (result != NES_NETPLAY_OK) return fail_connected(session, result);
        session->completion_sent = true;
    }
    NetplayPacket packet = {0};
    NesNetplayResult result =
        receive_expected(session, NETPLAY_PACKET_FRAME_DONE, timeout_ms, &packet);
    if (result != NES_NETPLAY_OK) return result;
    NesStateReader reader;
    uint64_t frame;
    NesNetplayHardwareHash remote_hash, local_hash;
    nes_state_reader_init(&reader, packet.data, packet.size);
    bool valid = nes_state_read_u64(&reader, &frame) && frame == session->frame
        && read_hash(&reader, &remote_hash)
        && nes_state_reader_remaining(&reader) == 0;
    packet_destroy(&packet);
    if (!valid) return fail_connected(session, NES_NETPLAY_PROTOCOL_ERROR);
    if (!nes_netplay_hardware_hash(&local_hash))
        return fail_connected(session, NES_NETPLAY_STATE_ERROR);
    if (!nes_netplay_hardware_hash_equal(&local_hash, &remote_hash)) {
        (void)send_error(session->peer, NES_NETPLAY_DESYNC, timeout_ms);
        return fail_connected(session, NES_NETPLAY_DESYNC);
    }
    session->frame++;
    session->frame_in_progress = false;
    session->completion_sent = false;
    return netplay_set_result(session, NES_NETPLAY_OK);
}

NesNetplayResult nes_netplay_disconnect(NesNetplaySession *session) {
    if (!session) return NES_NETPLAY_INVALID_ARGUMENT;
    if (session->mode == NES_NETPLAY_IDLE)
        return netplay_set_result(session, NES_NETPLAY_CONFLICT);
    if (session->mode == NES_NETPLAY_CONNECTED) {
        (void)send_packet(session->peer, NETPLAY_PACKET_DISCONNECT, NULL, 0, 100);
        return clean_remote_disconnect(session);
    }
    nes_netplay_socket_close(&session->peer);
    nes_netplay_socket_close(&session->listener);
    NesNetplayResult restore = restore_live_timeline(session);
    discard_resume(session);
    reset_frame_state(session);
    session->mode = NES_NETPLAY_IDLE;
    session->role = NES_NETPLAY_ROLE_NONE;
    session->port = 0;
    session->local_player_mask = 0;
    session->remote_player_mask = 0;
    return netplay_set_result(session, restore);
}

NesNetplayMode nes_netplay_mode(const NesNetplaySession *session) {
    return session ? session->mode : NES_NETPLAY_IDLE;
}

NesNetplayRole nes_netplay_role(const NesNetplaySession *session) {
    return session ? session->role : NES_NETPLAY_ROLE_NONE;
}

uint16_t nes_netplay_port(const NesNetplaySession *session) {
    return session ? session->port : 0;
}

uint32_t nes_netplay_local_player_mask(const NesNetplaySession *session) {
    return session ? session->local_player_mask : 0;
}

uint32_t nes_netplay_remote_player_mask(const NesNetplaySession *session) {
    return session ? session->remote_player_mask : 0;
}

bool nes_netplay_paused(const NesNetplaySession *session) {
    return session && session->paused;
}

void nes_netplay_progress(const NesNetplaySession *session, NesNetplayProgress *progress) {
    if (!progress) return;
    memset(progress, 0, sizeof(*progress));
    if (!session) {
        progress->last_result = NES_NETPLAY_INVALID_ARGUMENT;
        return;
    }
    progress->mode = session->mode;
    progress->role = session->role;
    progress->paused = session->paused;
    progress->port = session->port;
    progress->local_player_mask = session->local_player_mask;
    progress->remote_player_mask = session->remote_player_mask;
    progress->frame = session->frame;
    progress->queued_local_events = session->local_event_count;
    progress->last_result = session->last_result;
}

const char *nes_netplay_result_string(NesNetplayResult result) {
    switch (result) {
        case NES_NETPLAY_OK: return "ok";
        case NES_NETPLAY_PAUSED: return "paused";
        case NES_NETPLAY_INVALID_ARGUMENT: return "invalid argument";
        case NES_NETPLAY_CONFLICT: return "conflicting deterministic session";
        case NES_NETPLAY_NO_IMAGE: return "no loaded image";
        case NES_NETPLAY_UNSUPPORTED_HOST_STATE: return "unsupported host state";
        case NES_NETPLAY_OUT_OF_MEMORY: return "out of memory";
        case NES_NETPLAY_NETWORK_ERROR: return "network error";
        case NES_NETPLAY_TIMEOUT: return "network timeout";
        case NES_NETPLAY_PROTOCOL_ERROR: return "invalid netplay protocol";
        case NES_NETPLAY_VERSION_ERROR: return "incompatible netplay version";
        case NES_NETPLAY_INCOMPATIBLE: return "incompatible machine";
        case NES_NETPLAY_STATE_ERROR: return "save-state operation failed";
        case NES_NETPLAY_INPUT_ERROR: return "invalid or unauthorized input";
        case NES_NETPLAY_LIMIT_REACHED: return "frame input limit reached";
        case NES_NETPLAY_DESYNC: return "netplay desynchronization";
        default: return "unknown netplay error";
    }
}
