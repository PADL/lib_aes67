// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#include "sap.h"
#include "rtp_protocol.h"
#include "aes67_internal.h"
#include "aes67_utils.h"
#include "ptp_internal.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

/*
 * RFC 4556 5.2: sess-id is a numeric string
 */
static int is_numeric_string_p(const char *s) {
    size_t len = strlen(s);
    size_t i;

    if (len == 0)
        return 0;

    for (i = 0; i < len; i++) {
        if (!isdigit(s[i]))
            return 0;
    }

    return 1;
}

static void sdp_origin_parse(aes67_sdp_t *sdp, char *line, int line_num) {
    char *session_id, *nettype, *addrtype, *addr;

    strsep(&line, " ");              // 1: Username
    session_id = strsep(&line, " "); // 2: Session Id
    strsep(&line, " ");              // 3: Session Version
    nettype = strsep(&line, " ");    // 4: Network Type
    addrtype = strsep(&line, " ");   // 5: Address Type
    addr = strsep(&line, " ");       // 6: Address

    if (session_id && is_numeric_string_p(session_id)) {
        strncpy(sdp->session_id, session_id, sizeof(sdp->session_id) - 1);
    } else {
        aes67_error("Failed to parse session id on line %d", line_num);
    }

    if (nettype == NULL || strcmp(nettype, "IN") != 0) {
        aes67_error("SDP origin net type is not 'IN': %s", nettype);
    }

    if (addrtype == NULL ||
        (strcmp(addrtype, "IP4") != 0 && strcmp(addrtype, "IP6") != 0)) {
        aes67_error("SDP Origin address type is not IP4/IP6: %s", nettype);
    }

    if (addr) {
        strncpy(sdp->session_origin, addr, sizeof(sdp->session_origin) - 1);
    } else {
        aes67_error("Failed to parse origin address on line %d", line_num);
    }
}

static void sdp_connection_parse(aes67_sdp_t *sdp, char *line, int line_num) {
    char *nettype = strsep(&line, " ");
    char *addrtype = strsep(&line, " ");
    char *addr = strsep(&line, " /");

    if (nettype == NULL || strcmp(nettype, "IN") != 0) {
        aes67_error("SDP net type is not 'IN': %s", nettype);
        return;
    }

    if (addrtype == NULL ||
        (strcmp(addrtype, "IP4") != 0 && strcmp(addrtype, "IP6") != 0)) {
        aes67_warn("SDP Address type is not IP4/IP6: %s", nettype);
        return;
    }

    if (addr) {
        aes67_sdp_set_address(sdp, addr);
    } else {
        aes67_error("Failed to parse connection address on line %d", line_num);
    }
}

static void sdp_media_parse(aes67_sdp_t *sdp, char *line, int line_num) {
    char *media = strsep(&line, " ");
    char *port = strsep(&line, " ");
    char *proto = strsep(&line, " ");
    char *fmt = strsep(&line, " ");
    char *pEnd = NULL;

    if (media == NULL || strcmp(media, "audio") != 0) {
        aes67_error("SDP media type is not audio: %s", media);
    }

    if (port == NULL || strlen(port) > 2) {
        aes67_sdp_set_port(sdp, port);
    } else {
        aes67_error("Invalid connection port: %s", port);
    }

    if (proto == NULL || strcmp(proto, "RTP/AVP") != 0) {
        aes67_error("SDP transport protocol is not RTP/AVP: %s", proto);
    }

    if (fmt == NULL || strtoul(fmt, &pEnd, 10) > 127 || *pEnd) {
        aes67_error("SDP media format is not valid: %s", fmt);
    } else {
        aes67_sdp_set_payload_type(sdp, atoi(fmt));
    }
}

static void sdp_attribute_parse(aes67_sdp_t *sdp, char *line, int line_num) {
    char *attr = strsep(&line, ":");
    if (attr == NULL)
        return;

    if (strcmp(attr, "rtpmap") == 0) {
        char *pt = strsep(&line, " ");

        if (pt && atoi(pt) == sdp->payload_type) {
            char *encoding = strsep(&line, "/");
            char *sample_rate = strsep(&line, "/");
            char *channel_count = strsep(&line, "/");

            if (encoding)
                aes67_sdp_set_encoding_name(sdp, encoding);
            if (sample_rate)
                sdp->sample_rate = atoi(sample_rate);
            if (channel_count)
                sdp->channel_count = atoi(channel_count);
        }
    } else if (strcmp(attr, "ptime") == 0) {
        sdp->packet_duration = atof(line);
    } else if (strcmp(attr, "ts-refclk") == 0) {
        char *clksrc_type = strsep(&line, "=");
        char *ptp_version = strsep(&line, ":");
        char *ptp_gmid = strsep(&line, ":");
        char *ptp_domain = strsep(&line, ":");

        if (clksrc_type && strcmp(clksrc_type, "ptp") == 0) {
            if (ptp_version && strcmp(ptp_version, "IEEE1588-2008") != 0) {
                aes67_warn("PTP version is not IEEE1588-2008: %s", ptp_version);
            }

            if (ptp_gmid) {
                strncpy(sdp->ptp_gmid, ptp_gmid, sizeof(sdp->ptp_gmid) - 1);
            }

            if (ptp_domain) {
                sdp->ptp_domain = atoi(ptp_domain);
                if (sdp->ptp_domain)
                    aes67_warn("PTP domain is not 0: %s", ptp_version);
            }
        } else {
            aes67_warn("SDP Clock Source is not PTP");
        }
    } else if (strcmp(attr, "mediaclk") == 0) {
        char *mediaclk_type = strsep(&line, "=");
        char *clock_offset = strsep(&line, " ");

        if (mediaclk_type && strcmp(mediaclk_type, "direct") == 0) {
            if (clock_offset) {
                sdp->clock_offset = atoll(clock_offset);
            }
        } else {
            aes67_warn("SDP Media Clock is not set to direct: %s",
                       mediaclk_type);
        }
    }
}

static int sdp_parse_line(char *line, aes67_sdp_t *sdp, int line_num) {
    // Remove whitespace from the end of the line
    for (int i = strlen(line) - 1; i > 0; i--) {
        if (isspace(line[i])) {
            line[i] = '\0';
        } else {
            break;
        }
    }

    if (line_num == 1 && strcmp("v=0", line) != 0) {
        aes67_warn("First line of SDP is not v=0", line_num);
        return 1;
    }

    if (!islower(line[0])) {
        aes67_warn("Line %d of SDP file does not start with a lowercase letter",
                   line_num);
        return 1;
    }

    if (line[1] != '=') {
        aes67_warn("Line %d of SDP file is not an equals sign");
        return 1;
    }

    if (strlen(line) < 3) {
        aes67_warn("Line %d of SDP file is too short", line_num);
        return 1;
    }

    switch (line[0]) {
    case 'o':
        sdp_origin_parse(sdp, &line[2], line_num);
        break;

    case 's':
        strncpy(sdp->session_name, &line[2], sizeof(sdp->session_name) - 1);
        break;

    case 'i':
        strncpy(sdp->information, &line[2], sizeof(sdp->information) - 1);
        break;

    case 'c':
        sdp_connection_parse(sdp, &line[2], line_num);
        break;

    case 'm':
        sdp_media_parse(sdp, &line[2], line_num);
        break;

    case 'a':
        sdp_attribute_parse(sdp, &line[2], line_num);
        break;
    }

    return 0;
}

aes67_status_t aes67_sdp_parse_string(const char *str, aes67_sdp_t *sdp) {
    char line_buffer[255];
    size_t start = 0;
    size_t end = 0;
    size_t str_len = strlen(str);
    int line_num = 1;

    memset(sdp, 0, sizeof(aes67_sdp_t));
    sdp->encoding = -1;

    while (start < str_len) {
        // Find the end of the line
        end = 0;
        for (int i = start; i < str_len; i++) {
            if (str[i] == '\n') {
                end = i;
                break;
            }
        }

        if (end == 0) {
            aes67_warn("Failed to find the end of line %d", line_num);
            break;
        } else {
            int line_len = end - start;
            int result;

            if (line_len > sizeof(line_buffer) - 1) {
                aes67_warn("Ingoring line %d because it is too long", line_num);
                break;
            } else {
                memcpy(line_buffer, &str[start], line_len);
                line_buffer[line_len] = '\0';

                result = sdp_parse_line(line_buffer, sdp, line_num);
                if (result)
                    return result;
            }

            start = end + 1;
            line_num++;
        }
    }

    if (!aes67_sdp_is_valid(sdp))
        return AES67_STATUS_INVALID_SESSION_DESCRIPTION;

    return AES67_STATUS_OK;
}

void aes67_sdp_set_defaults(aes67_sdp_t *sdp) {
    memset(sdp, 0, sizeof(aes67_sdp_t));

    aes67_sdp_set_port(sdp, AES67_DEFAULT_PORT_STR);

    sdp->session_name[0] = '\0';
    sdp->payload_type = -1;
    aes67_sdp_set_encoding(sdp, AES67_DEFAULT_ENCODING);
    sdp->sample_rate = AES67_DEFAULT_SAMPLE_RATE;
    sdp->channel_count = AES67_MAX_CHANNELS_PER_SENDER;
}

int aes67_sdp_is_valid(const aes67_sdp_t *sdp) {
    if (sdp->address[0] == 0 || sdp->port[0] == 0 || sdp->encoding == -1 ||
        sdp->sample_size == 0 || sdp->sample_rate == 0 ||
        sdp->channel_count == 0) {
        return FALSE;
    } else {
        return TRUE;
    }
}

void aes67_sdp_set_address(aes67_sdp_t *sdp, const char *address) {
    if (address && strlen(address) > 1) {
        strncpy(sdp->address, address, sizeof(sdp->address) - 1);
    } else {
        sdp->address[0] = '\0';
    }
}

void aes67_sdp_set_ipv4_address(aes67_sdp_t *sdp, const uint8_t ip_addr[4]) {
    snprintf(sdp->address, sizeof(sdp->address), "%u.%u.%u.%u", ip_addr[0],
             ip_addr[1], ip_addr[2], ip_addr[3]);
}

void aes67_sdp_set_ipv4_session_origin(aes67_sdp_t *sdp,
                                       const uint8_t ip_addr[4]) {
    snprintf(sdp->session_origin, sizeof(sdp->session_origin), "%u.%u.%u.%u",
             ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
}

void aes67_sdp_set_ipv4_port(aes67_sdp_t *sdp, uint16_t __port) {
    snprintf(sdp->port, sizeof(sdp->port), "%u", __port);
}

void aes67_sdp_set_ptp_gmid(aes67_sdp_t *sdp, const uint8_t ptp_gmid[8]) {
    snprintf(sdp->ptp_gmid, sizeof(sdp->ptp_gmid),
             "%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x", ptp_gmid[0],
             ptp_gmid[1], ptp_gmid[2], ptp_gmid[3], ptp_gmid[4], ptp_gmid[5],
             ptp_gmid[6], ptp_gmid[7]);
}

void aes67_sdp_set_ptp_domain(aes67_sdp_t *sdp, int domain) {
    sdp->ptp_domain = domain;
}

void aes67_sdp_set_port(aes67_sdp_t *sdp, const char *port) {
    if (port && strlen(port) > 1) {
        strncpy(sdp->port, port, sizeof(sdp->port) - 1);
    } else {
        sdp->port[0] = '\0';
    }
}

void aes67_sdp_set_payload_type(aes67_sdp_t *sdp, int payload_type) {
    sdp->payload_type = payload_type;

    if (payload_type == 10) {
        aes67_sdp_set_encoding(sdp, AES67_ENCODING_L16);
        sdp->sample_rate = 44100;
        sdp->channel_count = 2;
    } else if (payload_type == 11) {
        aes67_sdp_set_encoding(sdp, AES67_ENCODING_L16);
        sdp->sample_rate = 44100;
        sdp->channel_count = 1;
    } else if (payload_type < 96) {
        aes67_error("Unsupported static payload type: %d", payload_type);
    }
}

void aes67_sdp_set_encoding(aes67_sdp_t *sdp, int encoding) {
    sdp->encoding = encoding;

    switch (encoding) {
    case AES67_ENCODING_L8:
        sdp->sample_size = 8;
        break;
    case AES67_ENCODING_L16:
    case AES67_ENCODING_PCMU:
    case AES67_ENCODING_PCMA:
    case AES67_ENCODING_G722:
    case AES67_ENCODING_GSM:
        sdp->sample_size = 16;
        break;
    case AES67_ENCODING_L24:
        sdp->sample_size = 24;
        break;
    case AES67_ENCODING_L32:
    case AES67_ENCODING_AM824:
        sdp->sample_size = 32;
        break;
    default:
        sdp->sample_size = 0;
        sdp->encoding = -1;
        break;
    }
}

void aes67_sdp_set_encoding_name(aes67_sdp_t *sdp, const char *encoding_name) {
    int encoding = aes67_encoding_lookup(encoding_name);
    aes67_sdp_set_encoding(sdp, encoding);
    if (encoding < 0) {
        aes67_error("Unsupported encoding: %s", encoding_name);
    }
}

aes67_status_t
aes67_sdp_to_string(const aes67_sdp_t *sdp, char *buffer, size_t buflen) {
    int res;
    int ip6Origin = strchr(sdp->session_origin, ':') != NULL;
    int ip6Address = strchr(sdp->address, ':') != NULL;
    char domainSuffix[64] = "";

    if (sdp->ptp_domain != 0)
        snprintf(domainSuffix, sizeof(domainSuffix), ":%u", sdp->ptp_domain);

    if (sdp->payload_type == 0) {
        res = snprintf(buffer, buflen,
                       "v=0\r\n"
                       "o=- %s %s IN %s %s\r\n",
                       sdp->session_id, sdp->session_id,
                       ip6Origin ? "IP6" : "IP4", sdp->session_origin);
    } else {
        res = snprintf(buffer, buflen,
                       "v=0\r\n"
                       "o=- %s %s IN %s %s\r\n"
                       "s=%s\r\n"
                       "c=IN %s %s\r\n"
                       "t=0 0\r\n"
                       "a=keywds:XMOS\r\n"
                       "m=audio %s RTP/AVP %d\r\n"
                       "i=%s\r\n"
                       "a=recvonly\r\n"
                       "a=rtpmap:%d L%d/%d/%d\r\n"
                       "a=ptime:%f\r\n"
                       "a=ts-refclk:ptp=IEEE1588-2008:%s%s\r\n"
                       "a=mediaclk:direct=%llu\r\n",
                       /* o= */ sdp->session_id, sdp->session_id,
                       ip6Origin ? "IP6" : "IP4", sdp->session_origin,
                       /* s= */ sdp->session_name,
                       /* c= */ ip6Address ? "IP6" : "IP4", sdp->address,
                       /* m= */ sdp->port, sdp->payload_type,
                       /* i= */ sdp->information,
                       /* a=rtpmap */ sdp->payload_type, sdp->sample_size,
                       sdp->sample_rate, sdp->channel_count,
                       /* a=ptime */ sdp->packet_duration,
                       /* a=ts-refclk */ sdp->ptp_gmid, domainSuffix,
                       /* a=mediaclk */ sdp->clock_offset);
    }

    if (res >= buflen)
        return AES67_STATUS_OUT_OF_BUFFER_SPACE;

    return AES67_STATUS_OK;
}

// Return the duration of a packet in microseconds
uint32_t aes67_rtp_packet_duration(const aes67_rtp_packet_t *packet,
                                   const aes67_sdp_t *sdp) {
    uint32_t payload_length = aes67_rtp_packet_length_rtp(packet) - RTP_HEADER_LENGTH;
    uint32_t frames = ((payload_length / (sdp->sample_size / 8)) / sdp->channel_count);
    return (frames * 1000000) / sdp->sample_rate;
}

void aes67_sdp_set_session_id(aes67_sdp_t *sdp, uint64_t session_id) {
    snprintf(sdp->session_id, sizeof(sdp->session_id), "%llu", session_id);
}

aes67_status_t aes67_sdp_get_ipv4_address(const aes67_sdp_t *sdp, uint8_t ip_addr[4]) {
    ip4_addr_t addr;

    if (ip4addr_aton(sdp->address, &addr) != 1)
        return AES67_STATUS_INVALID_IP_DEST_ADDR;

    uint32_to_xtcp_ipaddr(ip_addr, addr.addr);
    return AES67_STATUS_OK;
}

aes67_status_t aes67_sdp_get_ipv4_session_origin(const aes67_sdp_t *sdp, uint8_t ip_addr[4]) {
    ip4_addr_t addr;

    if (ip4addr_aton(sdp->session_origin, &addr) != 1)
        return AES67_STATUS_INVALID_IP_SRC_ADDR;

    uint32_to_xtcp_ipaddr(ip_addr, addr.addr);
    return AES67_STATUS_OK;
}

aes67_status_t aes67_sdp_get_ptp_gmid(const aes67_sdp_t *sdp, uint8_t ptp_gmid[8]) {
    n64_t parsed_gmid = parse_ptp_gmid(sdp->ptp_gmid);

    for (int i = 0; i < 8; i++) {
        ptp_gmid[i] = parsed_gmid.data[i];
    }

    return AES67_STATUS_OK;
}

int aes67_sdp_get_ptp_domain(const aes67_sdp_t *sdp) {
    return sdp->ptp_domain;
}

aes67_status_t aes67_sdp_get_ipv4_port(const aes67_sdp_t *sdp, uint16_t *__port) {
    int parsed_port = atoi(sdp->port);

    if (parsed_port < 0 || parsed_port > 65535)
        return AES67_STATUS_INVALID_UDP_DEST_PORT;

    *__port = (uint16_t)parsed_port;

    return AES67_STATUS_OK;
}
