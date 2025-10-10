// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h> // atoi

#include "rtp_protocol.h"

static int _is_multicast(const xtcp_ipaddr_t &addr) {
    return (addr[0] & 0xF0) == 0xE0;
}

static int port_string_to_port_number(const char _port[]) {
    int port_number = atoi(_port);

    if (port_number < 0 || port_number > 65535)
        return -1;

    return port_number;
}

aes67_status_t
aes67_socket_open_recv(client xtcp_if xtcp,
                       aes67_socket_t &sock,
                       const char address[],
                       const char _port[]) {
    int port_number = port_string_to_port_number(_port);
    xtcp_error_code_t err;

    if (port_number < 0)
        return AES67_STATUS_SOCKET_ERROR;

    err = xtcp.listen(sock.fd, port_number, sock.dest_addr);
    if (err)
        return err;

    if (_is_multicast(sock.dest_addr))
        xtcp.join_multicast_group(sock.dest_addr);

    return AES67_STATUS_OK;
}

aes67_status_t
aes67_socket_open_send(client xtcp_if xtcp,
                       aes67_socket_t &sock,
                       const char address[],
                       const char _port[]) {
    int port_number = port_string_to_port_number(_port);

    if (port_number < 0)
        return AES67_STATUS_SOCKET_ERROR;

#if 0
    err = xtcp.connect(sock.fd, port_number, sock.dest_addr);
    if (err)
        return err;
#else
    sock.dest_port = port_number;
#endif

    if (_is_multicast(sock.dest_addr))
        xtcp.join_multicast_group(sock.dest_addr);

    return AES67_STATUS_OK;
}

int aes67_socket_recv(client xtcp_if xtcp,
                      const aes67_socket_t &sock,
                      uint8_t data[len],
                      size_t len) {
    uint16_t port_number = 0;

    return xtcp.recvfrom(sock.fd, data, len, sock.src_addr, port_number);
}

int aes67_socket_send(client xtcp_if xtcp,
                      const aes67_socket_t &sock,
                      uint8_t data[len],
                      size_t len) {
    // we want to use sendto here so the socket does not need fto be connected
    return xtcp.sendto(sock.fd, data, len, sock.dest_addr, sock.dest_port);
}

void aes67_socket_close(client xtcp_if xtcp, aes67_socket_t &sock) {
    if (sock.joined_group) {
        xtcp.leave_multicast_group(sock.dest_addr);
        sock.joined_group = 0;
    }

    xtcp.close(sock.fd);
}
