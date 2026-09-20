/*
 * netplay_transport.h - Small blocking-with-deadline TCP transport
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_NETPLAY_TRANSPORT_H
#define CUPID_NETPLAY_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#include <BaseTsd.h>
typedef UINT_PTR NesNetplaySocket;
#else
typedef int NesNetplaySocket;
#endif

#ifdef _WIN32
#define NES_NETPLAY_INVALID_SOCKET ((NesNetplaySocket)(~(UINT_PTR)0))
#else
#define NES_NETPLAY_INVALID_SOCKET (-1)
#endif

typedef enum {
    NES_NETPLAY_IO_OK = 0,
    NES_NETPLAY_IO_TIMEOUT,
    NES_NETPLAY_IO_CLOSED,
    NES_NETPLAY_IO_ERROR
} NesNetplayIoResult;

bool nes_netplay_socket_system_init(void);
void nes_netplay_socket_system_shutdown(void);
NesNetplaySocket nes_netplay_socket_listen(uint16_t port, uint16_t *bound_port);
NesNetplaySocket nes_netplay_socket_accept(NesNetplaySocket listener, unsigned timeout_ms);
NesNetplaySocket nes_netplay_socket_connect(const char *host, uint16_t port,
                                            unsigned timeout_ms);
NesNetplayIoResult nes_netplay_socket_wait_readable(NesNetplaySocket socket,
                                                    unsigned timeout_ms);
NesNetplayIoResult nes_netplay_socket_send(NesNetplaySocket socket, const void *data,
                                           size_t size, unsigned timeout_ms);
NesNetplayIoResult nes_netplay_socket_receive(NesNetplaySocket socket, void *data,
                                              size_t size, unsigned timeout_ms);
void nes_netplay_socket_close(NesNetplaySocket *socket);

#endif
