// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include "rtp_protocol.h"
#include "sap.h"
#include "aes67_utils.h"
#include <string.h>

#if AES67_XMOS
#include <stdint.h>
#else
#include <arpa/inet.h>
#include <netinet/ip.h>
#endif // AES67_XMOS

aes67_status_t
aes67_sap_parse(const uint8_t *data, size_t data_len, aes67_sap_t *sap) {
    int offset = 4;
    memset(sap, 0, sizeof(aes67_sap_t));

    if (data_len < 12) {
        // FIXME: pick a better minimum packet length
        aes67_debug("Error: received SAP packet is too short");
        return AES67_STATUS_INVALID_SAP_PACKET;
    }

    if (((data[0] & 0x20) >> 5) != AES67_SAP_VERSION_1) {
        aes67_debug("Error: received SAP Version is not 1");
        return AES67_STATUS_INVALID_SAP_PACKET;
    }

#if AES67_XMOS
    if (((data[0] & 0x10) >> 4) == 0) {
        ip4_addr_t addr;

        memcpy(&addr.addr, data, 4);
        if (!ip4addr_ntoa_r(&addr, sap->message_source,
                            sizeof(sap->message_source))) {
            return AES67_STATUS_INVALID_SAP_PACKET;
        }
        offset += 4;
    } else {
        aes67_debug("IPv6 not supported");
        return AES67_STATUS_INVALID_SAP_PACKET;
    }
#else
    if (((data[0] & 0x10) >> 4) == 0) {
        inet_ntop(AF_INET, &data[4], sap->message_source,
                  sizeof(sap->message_source));
        offset += 4;
    } else {
        inet_ntop(AF_INET6, &data[4], sap->message_source,
                  sizeof(sap->message_source));
        offset += 16;
    }
#endif // AES67_XMOS

    // Store the message type (announce or delete)
    sap->message_type = ((data[0] & 0x04) >> 2);

    if (((data[0] & 0x02) >> 1) == 1) {
        aes67_debug("Error: received SAP packet is encrypted");
        return AES67_STATUS_INVALID_SAP_PACKET;
    }

    // FIXME: add support for SAP packet compression (using zlib)
    if (((data[0] & 0x01) >> 0) == 1) {
        aes67_debug("Error: received SAP packet is compressed");
        return AES67_STATUS_INVALID_SAP_PACKET;
    }

    // Add on the authentication data length
    offset += (data[1] * 4);

    // Store the Message ID Hash
    sap->message_id_hash = (((uint16_t)data[2] << 8) | (uint16_t)data[3]);

    // Check the MIME type
    const char *mime_type = (char *)&data[offset];
    if (mime_type[0] == '\0') {
        offset += 1;
    } else if (strcmp(mime_type, AES67_SDP_MIME_TYPE) == 0) {
        offset += sizeof(AES67_SDP_MIME_TYPE);
    } else if (mime_type[0] == 'v' && mime_type[1] == '=' &&
               mime_type[2] == '0') {
        // No MIME Type field
    } else {
        aes67_debug("Error: unsupported MIME type in SAP packet: %s",
                    mime_type);
        return AES67_STATUS_INVALID_SAP_PACKET;
    }

    // Copy over the SDP data
    memcpy(sap->sdp, &data[offset], data_len - offset);

    // Ensure that the SDP is null terminated
    // FIXME: do we need to worry about the buffer size?
    sap->sdp[data_len - offset] = '\0';

    return AES67_STATUS_OK;
}

static uint16_t _crc16(const uint8_t *data, size_t data_len) {
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (data_len--) {
        x = crc >> 8 ^ *data++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^
              ((uint16_t)x);
    }

    return crc;
}

int aes67_sap_generate(const aes67_socket_t *sock,
                       const char *sdp,
                       uint8_t message_type,
                       uint8_t *buffer,
                       size_t buffer_len) {
    size_t sdp_len = strlen(sdp);
    uint16_t message_hash = _crc16((const uint8_t *)sdp, sdp_len);
    int pos = 0;

    if (sdp_len + 1 + AES67_SAP_MAX_HEADER > buffer_len) {
        // Buffer isn't big enough
        return -1;
    }

    buffer[pos++] = (AES67_SAP_VERSION_1 << 5); // SAP Version 1
    if (message_type == AES67_SAP_MESSAGE_DELETE) {
        // SAP Flag: T=1
        buffer[0] |= (0x1 << 2);
    }

    buffer[pos++] = 0; // Authentication Len
    buffer[pos++] = (message_hash & 0xFF00) >> 8;
    buffer[pos++] = (message_hash & 0x00FF);

#if AES67_XMOS
    memcpy(&buffer[pos], sock->src_addr, sizeof(sock->src_addr));
    pos += sizeof(sock->src_addr);
#else
    if (sock->src_addr.ss_family == AF_INET) {
        struct sockaddr_in *sin = (struct sockaddr_in *)&sock->src_addr;
        memcpy(&buffer[pos], &sin->sin_addr, sizeof(sin->sin_addr));
        pos += sizeof(sin->sin_addr);
    } else if (sock->src_addr.ss_family == AF_INET6) {
        struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)&sock->src_addr;
        memcpy(&buffer[pos], &sin6->sin6_addr, sizeof(sin6->sin6_addr));
        pos += sizeof(sin6->sin6_addr);
        buffer[0] |= (0x1 << 4); // SAP Flag: A=1
    } else {
        return -1;
    }
#endif // AES67_XMOS

    // Add the MIME type (this is NUL terminated, so add one to AES67_SDP_MIME_TYPE_LEN)
    memcpy(&buffer[pos], AES67_SDP_MIME_TYPE, AES67_SDP_MIME_TYPE_LEN + 1);
    pos += AES67_SDP_MIME_TYPE_LEN + 1;

    // Finally the SDP payload
    memcpy(&buffer[pos], sdp, sdp_len);
    pos += sdp_len;

    buffer[pos] = '\0';

    return pos;
}

#if AES67_XMOS
int aes67_sap_send_sdp_string(unsigned xtcp,
                              const aes67_socket_t *sock,
                              const char sdp[],
                              uint8_t message_type)
#else
int aes67_sap_send_sdp_string(const aes67_socket_t *sock,
                              const char *sdp,
                              uint8_t message_type)
#endif
{
    uint8_t packet[AES67_SAP_MAX_LEN];
    int len;

    len = aes67_sap_generate(sock, sdp, message_type, packet, sizeof(packet));
    if (len > 0) {
#if AES67_XMOS
        return aes67_socket_send(xtcp, sock, packet, len);
#else
        return aes67_socket_send(sock, packet, len);
#endif
    } else {
        return len;
    }
}
