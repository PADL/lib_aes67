// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h> // atoi
#include <xassert.h> // assert
#include <string.h> // memcpy
#include <xtcp.h>

#include "rtp_protocol.h"

aes67_status_t
aes67_socket_recv(client xtcp_if xtcp,
                  const aes67_socket_t &sock,
                  uint8_t buffer[buffer_size],
                  size_t buffer_size,
                  size_t &received_len) {
    uint16_t port_number = 0;
    int nread;

    nread = xtcp.recvfrom(sock.fd, buffer, buffer_size, sock.src_addr, port_number);
    if (nread <= 0)
        return AES67_STATUS_SOCKET_ERROR;

    received_len = nread;

    return AES67_STATUS_OK;
}

aes67_status_t
aes67_socket_send(client xtcp_if xtcp,
                  const aes67_socket_t &sock,
                  const uint8_t buffer[len],
                  size_t len) {
    int nsent;

    nsent = xtcp.sendto(sock.fd, buffer, len, sock.dest_addr, sock.dest_port);
    if (nsent != (int)len)
        return AES67_STATUS_SOCKET_ERROR;

    return AES67_STATUS_OK;
}

void aes67_socket_close(client xtcp_if xtcp, aes67_socket_t &sock) {
    if (sock.joined_group) {
        xtcp.leave_multicast_group(sock.dest_addr);
        sock.joined_group = 0;
    }

    xtcp.close(sock.fd);
    sock.fd = -1;
}

void __aes67_rtp_send_hp_packet(streaming_chanend_t c_tx_hp, uint8_t packet[n], size_t n) {
    ethernet_send_hp_packet(c_tx_hp, packet, n, ETHERNET_ALL_INTERFACES);
}
