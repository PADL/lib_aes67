// SPDX-License-Identifier: MIT
// Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include "../util/nettypes.h"
#include <string.h>
#include <stdlib.h>

n64_t parse_ptp_gmid(const char *gmid_str) {
    uint64_t result = 0;
    uint8_t bytes[8];
    int byte_idx = 0;
    static const n64_t zero_n64;

    if (!gmid_str ||
        strlen(gmid_str) != 23) { // Format: AA-BB-CC-DD-EE-FF-GG-HH (23 chars)
        return zero_n64;
    }

    // Parse each two-character hex byte separated by dashes
    for (int i = 0; i < 23 && byte_idx < 8; i += 3) {
        if (i > 0 && gmid_str[i - 1] != '-') {
            return zero_n64; // Invalid format
        }

        // Parse two hex digits
        char hex_byte[3];
        hex_byte[0] = gmid_str[i];
        hex_byte[1] = gmid_str[i + 1];
        hex_byte[2] = '\0';

        char *endptr;
        unsigned long val = strtoul(hex_byte, &endptr, 16);
        if (*endptr != '\0' || val > 255) {
            return zero_n64; // Invalid hex byte
        }

        bytes[byte_idx++] = (uint8_t)val;
    }

    if (byte_idx != 8) {
        return zero_n64; // Didn't parse exactly 8 bytes
    }

    // Convert to 64-bit integer in network byte order
    for (int i = 0; i < 8; i++) {
        result = (result << 8) | bytes[i];
    }

    return hton64(result);
}