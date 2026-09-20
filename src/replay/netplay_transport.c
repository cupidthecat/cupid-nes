/*
 * netplay_transport.c - Small blocking-with-deadline TCP transport
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include "netplay_transport.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <SDL2/SDL.h>
#include <stdio.h>
#include <string.h>

static unsigned socket_users;

#ifdef _WIN32
typedef int NesSockLen;
#define NATIVE_SOCKET(value) ((SOCKET)(value))
#else
typedef socklen_t NesSockLen;
#define NATIVE_SOCKET(value) (value)
#endif

static int socket_error(void) {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

static bool socket_would_block(int error) {
#ifdef _WIN32
    return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS;
#else
    return error == EWOULDBLOCK || error == EAGAIN || error == EINPROGRESS;
#endif
}

static bool set_nonblocking(NesNetplaySocket socket, bool enabled) {
#ifdef _WIN32
    u_long mode = enabled ? 1ul : 0ul;
    return ioctlsocket(NATIVE_SOCKET(socket), FIONBIO, &mode) == 0;
#else
    int flags = fcntl(socket, F_GETFL, 0);
    if (flags < 0) return false;
    if (enabled) flags |= O_NONBLOCK;
    else flags &= ~O_NONBLOCK;
    return fcntl(socket, F_SETFL, flags) == 0;
#endif
}

static int wait_socket(NesNetplaySocket socket, bool writing, unsigned timeout_ms) {
#ifndef _WIN32
    if (socket < 0 || socket >= FD_SETSIZE) return -1;
#endif
    fd_set read_set, write_set;
    FD_ZERO(&read_set);
    FD_ZERO(&write_set);
    if (writing) FD_SET(NATIVE_SOCKET(socket), &write_set);
    else FD_SET(NATIVE_SOCKET(socket), &read_set);
    struct timeval timeout = {
        (long)(timeout_ms / 1000u), (long)((timeout_ms % 1000u) * 1000u)
    };
#ifdef _WIN32
    int result = select(0, writing ? NULL : &read_set,
                        writing ? &write_set : NULL, NULL, &timeout);
#else
    int result = select(socket + 1, writing ? NULL : &read_set,
                        writing ? &write_set : NULL, NULL, &timeout);
#endif
    return result;
}

bool nes_netplay_socket_system_init(void) {
    if (socket_users++) return true;
#ifdef _WIN32
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        socket_users = 0;
        return false;
    }
#endif
    return true;
}

void nes_netplay_socket_system_shutdown(void) {
    if (!socket_users || --socket_users) return;
#ifdef _WIN32
    WSACleanup();
#endif
}

void nes_netplay_socket_close(NesNetplaySocket *socket) {
    if (!socket || *socket == NES_NETPLAY_INVALID_SOCKET) return;
#ifdef _WIN32
    closesocket(NATIVE_SOCKET(*socket));
#else
    close(*socket);
#endif
    *socket = NES_NETPLAY_INVALID_SOCKET;
}

NesNetplaySocket nes_netplay_socket_listen(uint16_t port, uint16_t *bound_port) {
    NesNetplaySocket listener = (NesNetplaySocket)
        socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == NES_NETPLAY_INVALID_SOCKET) return listener;
    int reuse = 1;
    (void)setsockopt(NATIVE_SOCKET(listener), SOL_SOCKET, SO_REUSEADDR,
                     (const char *)&reuse, (NesSockLen)sizeof(reuse));
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (bind(NATIVE_SOCKET(listener), (struct sockaddr *)&address, sizeof(address)) != 0
        || listen(NATIVE_SOCKET(listener), 1) != 0) {
        nes_netplay_socket_close(&listener);
        return NES_NETPLAY_INVALID_SOCKET;
    }
    if (bound_port) {
        NesSockLen length = (NesSockLen)sizeof(address);
        if (getsockname(NATIVE_SOCKET(listener), (struct sockaddr *)&address, &length) != 0) {
            nes_netplay_socket_close(&listener);
            return NES_NETPLAY_INVALID_SOCKET;
        }
        *bound_port = ntohs(address.sin_port);
    }
    return listener;
}

NesNetplaySocket nes_netplay_socket_accept(NesNetplaySocket listener, unsigned timeout_ms) {
    if (listener == NES_NETPLAY_INVALID_SOCKET || wait_socket(listener, false, timeout_ms) <= 0)
        return NES_NETPLAY_INVALID_SOCKET;
    NesNetplaySocket peer = (NesNetplaySocket)accept(NATIVE_SOCKET(listener), NULL, NULL);
    if (peer != NES_NETPLAY_INVALID_SOCKET) {
        int enabled = 1;
        (void)setsockopt(NATIVE_SOCKET(peer), IPPROTO_TCP, TCP_NODELAY,
                        (const char *)&enabled, (NesSockLen)sizeof(enabled));
        if (!set_nonblocking(peer, true)) nes_netplay_socket_close(&peer);
    }
    return peer;
}

NesNetplaySocket nes_netplay_socket_connect(const char *host, uint16_t port,
                                            unsigned timeout_ms) {
    if (!host || !*host) return NES_NETPLAY_INVALID_SOCKET;
    char service[6];
    if (snprintf(service, sizeof(service), "%u", (unsigned)port) <= 0)
        return NES_NETPLAY_INVALID_SOCKET;
    struct addrinfo hints, *addresses = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (getaddrinfo(host, service, &hints, &addresses) != 0) return NES_NETPLAY_INVALID_SOCKET;

    NesNetplaySocket connected = NES_NETPLAY_INVALID_SOCKET;
    for (struct addrinfo *address = addresses; address; address = address->ai_next) {
        NesNetplaySocket candidate = (NesNetplaySocket)
            socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (candidate == NES_NETPLAY_INVALID_SOCKET || !set_nonblocking(candidate, true)) {
            nes_netplay_socket_close(&candidate);
            continue;
        }
        int result = connect(NATIVE_SOCKET(candidate), address->ai_addr,
                             (NesSockLen)address->ai_addrlen);
        if (result != 0 && !socket_would_block(socket_error())) {
            nes_netplay_socket_close(&candidate);
            continue;
        }
        if (result != 0 && wait_socket(candidate, true, timeout_ms) <= 0) {
            nes_netplay_socket_close(&candidate);
            continue;
        }
        int error = 0;
        NesSockLen length = (NesSockLen)sizeof(error);
        if (getsockopt(NATIVE_SOCKET(candidate), SOL_SOCKET, SO_ERROR,
                       (char *)&error, &length) != 0 || error != 0) {
            nes_netplay_socket_close(&candidate);
            continue;
        }
        int enabled = 1;
        (void)setsockopt(NATIVE_SOCKET(candidate), IPPROTO_TCP, TCP_NODELAY,
                        (const char *)&enabled, (NesSockLen)sizeof(enabled));
        connected = candidate;
        break;
    }
    freeaddrinfo(addresses);
    return connected;
}

NesNetplayIoResult nes_netplay_socket_wait_readable(NesNetplaySocket socket,
                                                    unsigned timeout_ms) {
    if (socket == NES_NETPLAY_INVALID_SOCKET) return NES_NETPLAY_IO_ERROR;
    int ready = wait_socket(socket, false, timeout_ms);
    if (ready > 0) return NES_NETPLAY_IO_OK;
    return ready == 0 ? NES_NETPLAY_IO_TIMEOUT : NES_NETPLAY_IO_ERROR;
}

NesNetplayIoResult nes_netplay_socket_send(NesNetplaySocket socket, const void *data,
                                           size_t size, unsigned timeout_ms) {
    if (socket == NES_NETPLAY_INVALID_SOCKET || (size && !data)) return NES_NETPLAY_IO_ERROR;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t offset = 0;
    uint64_t deadline = SDL_GetTicks64() + timeout_ms;
    while (offset < size) {
        uint64_t now = SDL_GetTicks64();
        unsigned wait = now >= deadline ? 0 : (unsigned)(deadline - now);
        int ready = wait_socket(socket, true, wait);
        if (ready == 0) return NES_NETPLAY_IO_TIMEOUT;
        if (ready < 0) return NES_NETPLAY_IO_ERROR;
        size_t remaining = size - offset;
        int chunk = remaining > 0x40000000u ? 0x40000000 : (int)remaining;
#ifdef MSG_NOSIGNAL
        int send_flags = MSG_NOSIGNAL;
#else
        int send_flags = 0;
#endif
        int sent = send(NATIVE_SOCKET(socket), (const char *)bytes + offset, chunk, send_flags);
        if (sent == 0) return NES_NETPLAY_IO_CLOSED;
        if (sent < 0) {
            if (socket_would_block(socket_error()) && SDL_GetTicks64() < deadline) continue;
            return NES_NETPLAY_IO_ERROR;
        }
        offset += (size_t)sent;
    }
    return NES_NETPLAY_IO_OK;
}

NesNetplayIoResult nes_netplay_socket_receive(NesNetplaySocket socket, void *data,
                                              size_t size, unsigned timeout_ms) {
    if (socket == NES_NETPLAY_INVALID_SOCKET || (size && !data)) return NES_NETPLAY_IO_ERROR;
    uint8_t *bytes = (uint8_t *)data;
    size_t offset = 0;
    uint64_t deadline = SDL_GetTicks64() + timeout_ms;
    while (offset < size) {
        uint64_t now = SDL_GetTicks64();
        unsigned wait = now >= deadline ? 0 : (unsigned)(deadline - now);
        int ready = wait_socket(socket, false, wait);
        if (ready == 0) return NES_NETPLAY_IO_TIMEOUT;
        if (ready < 0) return NES_NETPLAY_IO_ERROR;
        size_t remaining = size - offset;
        int chunk = remaining > 0x40000000u ? 0x40000000 : (int)remaining;
        int received = recv(NATIVE_SOCKET(socket), (char *)bytes + offset, chunk, 0);
        if (received == 0) return NES_NETPLAY_IO_CLOSED;
        if (received < 0) {
            if (socket_would_block(socket_error()) && SDL_GetTicks64() < deadline) continue;
            return NES_NETPLAY_IO_ERROR;
        }
        offset += (size_t)received;
    }
    return NES_NETPLAY_IO_OK;
}
