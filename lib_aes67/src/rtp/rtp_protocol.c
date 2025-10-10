// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include <stdlib.h>

#include "bytestoint.h"
#include "rtp_protocol.h"
#include "sap/sap.h"

#define bitMask(byte, mask, shift) ((byte & (mask << shift)) >> shift)

int aes67_rtp_parse(aes67_rtp_packet_t *packet) {
    size_t header_len = RTP_HEADER_LENGTH;

    // Byte 1
    packet->version = bitMask(packet->buffer[0], 0x02, 6);
    packet->padding = bitMask(packet->buffer[0], 0x01, 5);
    packet->extension = bitMask(packet->buffer[0], 0x01, 4);
    packet->csrc_count = bitMask(packet->buffer[0], 0x0F, 0);

    // Byte 2
    packet->marker = bitMask(packet->buffer[1], 0x01, 7);
    packet->payload_type = bitMask(packet->buffer[1], 0x7F, 0);

    // Bytes 3 and 4
    packet->sequence = bytesToUInt16(&packet->buffer[2]);

    // Bytes 5-8
    packet->timestamp = bytesToUInt32(&packet->buffer[4]);

    // Bytes 9-12
    packet->ssrc = bytesToUInt32(&packet->buffer[8]);

    // Calculate the size of the payload
    header_len += (packet->csrc_count * 4);

    if (packet->extension) {
        uint16_t ext_header_len;

        if (packet->length < header_len + 4)
            return -1;

        /* ignore extension header ID at buffer + header_len */
        ext_header_len = bytesToUInt16(&packet->buffer[header_len + 2]);
        header_len += 4 + (ext_header_len * 4);
    }

    if (packet->length < header_len)
        return -1;

    packet->payload_length = packet->length - header_len;
    packet->payload = packet->buffer + header_len;

    if (packet->padding) {
        uint8_t padding_len;

        if (packet->payload_length < 1)
            return -1;

        padding_len = packet->payload[packet->payload_length - 1];
        if (padding_len < 1 || padding_len > packet->payload_length)
            return -1;

        packet->payload_length -= padding_len;
    }

    // Success
    return 0;
}

#if AES67_XMOS
int aes67_rtp_recv(unsigned xtcp,
                   aes67_socket_t *socket,
                   aes67_rtp_packet_t *packet)
#else
int aes67_rtp_recv(aes67_socket_t *socket, aes67_rtp_packet_t *packet)
#endif
{
#if AES67_XMOS
    int len =
        aes67_socket_recv(xtcp, socket, packet->buffer, sizeof(packet->buffer));
#else
    int len = aes67_socket_recv(socket, packet->buffer, sizeof(packet->buffer));
#endif

    // Failure or too short to be an RTP packet?
    if (len <= RTP_HEADER_LENGTH)
        return -1;

    packet->length = len;

    return aes67_rtp_parse(packet);
}
