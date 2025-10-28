// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h>
#include <xassert.h>

#include "rtp_protocol.h"
#include "sap.h"

aes67_status_t aes67_rtp_parse(aes67_rtp_packet_t *packet) {
    uint32_t rtp_length = aes67_rtp_packet_length_rtp(packet);
    uint8_t *rtp_data = aes67_rtp_packet_start_rtp(packet);
    aes67_rtp_header_t *rtp_header = &packet->rtp_header;
    size_t header_len = RTP_HEADER_LENGTH;

    if (packet->rtp_length < header_len)
        return AES67_STATUS_BAD_PACKET_LENGTH;

    packet->rtp_header.sequence = ntohs(packet->rtp_header.sequence);
    packet->rtp_header.timestamp = ntohl(packet->rtp_header.timestamp);
    packet->rtp_header.ssrc = ntohl(packet->rtp_header.ssrc);

    if (RTP_VERSION_GET(rtp_header) != RTP_VERSION)
        return AES67_STATUS_UNSUPPORTED_RTP_VERSION;

    header_len += RTP_CSRC_COUNT_GET(rtp_header) * 4;

    if (packet->rtp_length < header_len)
        return AES67_STATUS_BAD_PACKET_LENGTH;

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

    uint32_t rtp_payload_len = rtp_length - header_len;

    if (RTP_PADDING_GET(rtp_header)) {
        uint8_t padding_len;

        if (rtp_payload_len < 1)
            return AES67_STATUS_BAD_PACKET_LENGTH;

        padding_len = rtp_data[rtp_length - 1];
        if (padding_len < 1 || padding_len > rtp_payload_len)
            return AES67_STATUS_BAD_PACKET_LENGTH;

        rtp_payload_len -= padding_len;
    }

    // rtp_length includes header and payload, without padding
    packet->rtp_length = header_len + rtp_payload_len;

    return AES67_STATUS_OK;
}

#if AES67_XMOS
aes67_status_t aes67_rtp_recv(chanend xtcp,
                              aes67_socket_t *socket,
                              aes67_rtp_packet_t *packet)
#else
aes67_status_t aes67_rtp_recv(aes67_socket_t *socket,
                              aes67_rtp_packet_t *packet)
#endif
{
    aes67_status_t status;

#if AES67_XMOS
    status = aes67_socket_recv_rtp(xtcp, socket, packet);
#else
    status = aes67_socket_recv_rtp(socket, packet);
#endif
    if (status != AES67_STATUS_OK)
        return status;

    status = aes67_rtp_parse(packet);
    if (status != AES67_STATUS_OK)
        return status;

    return AES67_STATUS_OK;
}

#if AES67_XMOS
aes67_status_t aes67_socket_recv_rtp(chanend xtcp, const aes67_socket_t *sock, aes67_rtp_packet_t *packet) {
#else
aes67_status_t aes67_socket_recv_rtp(const aes67_socket_t *sock, aes67_rtp_packet_t *packet) {
#endif
    const size_t rtp_buffer_size = RTP_HEADER_LENGTH + RTP_MAX_PAYLOAD;
    uint8_t *rtp_data = aes67_rtp_packet_start_rtp(packet);
    aes67_status_t status;
    size_t received_len;

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

#if AES67_XMOS
aes67_status_t aes67_socket_send_rtp(chanend xtcp, const aes67_socket_t *sock, const aes67_rtp_packet_t *packet) {
#else
aes67_status_t aes67_socket_send_rtp(const aes67_socket_t *sock, const aes67_rtp_packet_t *packet) {
#endif
    const uint8_t *rtp_data = aes67_const_rtp_packet_start_rtp(packet);
    size_t rtp_length = aes67_rtp_packet_length_rtp(packet);

#if AES67_XMOS
    return aes67_socket_send(xtcp, sock, rtp_data, rtp_length);
#else
    return aes67_socket_send(sock, rtp_data, rtp_length);
#endif
}
