// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h>
#include <xassert.h>

#include "bytestoint.h"
#include "rtp_protocol.h"
#include "sap/sap.h"

aes67_status_t aes67_rtp_parse(aes67_rtp_packet_t *packet) {
    uint32_t rtp_length = aes67_rtp_packet_length_rtp(packet);
    uint8_t *rtp_data = aes67_rtp_packet_start_rtp(packet);
    aes67_rtp_header_t *rtp_header = &packet->rtp_header;
    size_t header_len = RTP_HEADER_LENGTH;

    if (packet->rtp_length < header_len)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Convert multi-byte fields from network to host byte order
    packet->rtp_header.sequence = ntohs(packet->rtp_header.sequence);
    packet->rtp_header.timestamp = ntohl(packet->rtp_header.timestamp);
    packet->rtp_header.ssrc = ntohl(packet->rtp_header.ssrc);

    // Validate RTP version using macro
    if (RTP_VERSION_GET(rtp_header) != RTP_VERSION)
        return AES67_STATUS_UNSUPPORTED_RTP_VERSION;

    // Calculate the size of the payload including CSRC list
    header_len += RTP_CSRC_COUNT_GET(rtp_header) * 4;

    if (packet->rtp_length < header_len)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    // Handle extension header if present
    if (RTP_EXTENSION_GET(rtp_header)) {
        uint16_t ext_header_len;
        uint8_t *ext_ptr = &rtp_data[header_len];

        if (rtp_length < header_len + 4)
            return AES67_STATUS_BAD_PACKET_LENGTH;

        ext_header_len = RTP_EXT_LENGTH_GET(ext_ptr);
        header_len += 4 + (ext_header_len * 4);

        if (rtp_length < header_len)
            return AES67_STATUS_BAD_PACKET_LENGTH;
    }

    // Calculate payload length after removing header
    uint32_t rtp_payload_len = rtp_length - header_len;

    // Handle padding if present using macro
    if (RTP_PADDING_GET(rtp_header)) {
        uint8_t padding_len;

        if (rtp_payload_len < 1)
            return AES67_STATUS_BAD_PACKET_LENGTH;

        padding_len = rtp_data[rtp_length - 1];
        if (padding_len < 1 || padding_len > rtp_payload_len)
            return AES67_STATUS_BAD_PACKET_LENGTH;

        rtp_payload_len -= padding_len;
    }

    // rtp_length should include header + payload (excluding padding)
    packet->rtp_length = header_len + rtp_payload_len;

    // Success
    return AES67_STATUS_OK;
}

#if AES67_XMOS
aes67_status_t aes67_rtp_recv(unsigned xtcp,
                              aes67_socket_t *socket,
                              aes67_rtp_packet_t *packet)
#else
aes67_status_t aes67_rtp_recv(aes67_socket_t *socket,
                              aes67_rtp_packet_t *packet)
#endif
{
#if AES67_XMOS
    aes67_status_t status = aes67_socket_recv_rtp(xtcp, socket, packet);
#else
    aes67_status_t status = aes67_socket_recv_rtp(socket, packet);
#endif
    if (status != AES67_STATUS_OK)
        return status;

    status = aes67_rtp_parse(packet);
    if (status != AES67_STATUS_OK)
        return status;

    return AES67_STATUS_OK;
}

aes67_status_t aes67_socket_recv_rtp(unsigned xtcp, const aes67_socket_t *sock, aes67_rtp_packet_t *packet) {
    size_t rtp_buffer_size = sizeof(packet->rtp_header) + sizeof(packet->payload);
    uint8_t *rtp_data = aes67_rtp_packet_start_rtp(packet);
    size_t received_len;
    aes67_status_t status;

#if AES67_XMOS
    status = aes67_socket_recv(xtcp, sock, rtp_data, rtp_buffer_size, &received_len);
#else
    status = aes67_socket_recv(sock, rtp_data, rtp_buffer_size, &received_len);
#endif
    if (status != AES67_STATUS_OK)
        return status;

    assert(received_len <= 0xffff);
    packet->rtp_length = received_len;

    return AES67_STATUS_OK;
}

aes67_status_t aes67_socket_send_rtp(unsigned xtcp, const aes67_socket_t *sock, const aes67_rtp_packet_t *packet) {
    const uint8_t *rtp_data = aes67_const_rtp_packet_start_rtp(packet);
    size_t rtp_length = aes67_rtp_packet_length_rtp(packet);

#if AES67_XMOS
    return aes67_socket_send(xtcp, sock, rtp_data, rtp_length);
#else
    return aes67_socket_send(sock, rtp_data, rtp_length);
#endif
}
