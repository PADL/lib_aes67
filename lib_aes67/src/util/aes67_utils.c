// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2025 PADL Software Pty Ltd. All rights reserved.

#include "aes67_internal.h"
#include "aes67_utils.h"

#if __has_include(<xs1.h>)

#include <xs1.h>

#else

#include <signal.h>
#include <stdio.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>

#endif

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#if !__has_include(<debug_print.h>)

void aes67_log(aes67_log_level level, const char *fmt, ...) {
    time_t t = time(NULL);
    char *time_str;
    va_list args;

    // Display the message level
    switch (level) {
    case aes67_LOG_DEBUG:
        if (!verbose)
            return;
        fprintf(stderr, "[DEBUG]   ");
        break;
    case aes67_LOG_INFO:
        if (quiet)
            return;
        fprintf(stderr, "[INFO]    ");
        break;
    case aes67_LOG_WARN:
        fprintf(stderr, "[WARNING] ");
        break;
    case aes67_LOG_ERROR:
        fprintf(stderr, "[ERROR]   ");
        break;
    default:
        fprintf(stderr, "[UNKNOWN] ");
        break;
    }

    // Display timestamp
    time_str = ctime(&t);
    time_str[strlen(time_str) - 1] = 0; // remove \n
    fprintf(stderr, "%s  ", time_str);

    // Display the error message
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);

    // If an erron then stop
    if (level == aes67_LOG_ERROR) {
        // Exit with a non-zero exit code if there was a fatal error
        exit_code++;
        if (running) {
            // Quit gracefully
            running = 0;
        } else {
            fprintf(stderr,
                    "Fatal error while quiting; exiting immediately.\n");
            exit(-1);
        }
    }
}

#endif

const char *aes67_encoding_names[AES67_ENCODING_MAX] = {
    [AES67_ENCODING_L8] = "L8",      [AES67_ENCODING_L16] = "L16",
    [AES67_ENCODING_L24] = "L24",    [AES67_ENCODING_L32] = "L32",
    [AES67_ENCODING_PCMU] = "PCMU",  [AES67_ENCODING_PCMA] = "PCMA",
    [AES67_ENCODING_G722] = "G722",  [AES67_ENCODING_GSM] = "GSM",
    [AES67_ENCODING_AM824] = "AM824"};

const char *aes67_encoding_name(int encoding) {
    if (encoding > 0 && encoding < AES67_ENCODING_MAX) {
        return aes67_encoding_names[encoding];
    } else {
        return NULL;
    }
}

int aes67_encoding_lookup(const char *name) {
    for (size_t i = 0; i < AES67_ENCODING_MAX; i++) {
        if (strcmp(aes67_encoding_names[i], name) == 0)
            return i;
    }

    return -1;
}

const char *aes67_status_to_string(aes67_status_t status) {
    switch (status) {
    case AES67_STATUS_OK:
        return "OK";
    case AES67_STATUS_INVALID_STREAM_ID:
        return "Invalid stream ID";
    case AES67_STATUS_ALREADY_SUBSCRIBED:
        return "Already subscribed";
    case AES67_STATUS_NOT_SUBSCRIBED:
        return "Not subscribed";
    case AES67_STATUS_NOT_IMPLEMENTED:
        return "Not implemented";
    case AES67_STATUS_STREAM_NAME_MISMATCH:
        return "Stream name mismatch";
    case AES67_STATUS_SOCKET_ERROR:
        return "Socket error";
    case AES67_STATUS_UNSUPPORTED_RTP_VERSION:
        return "Unsupported RTP version";
    case AES67_STATUS_UNKNOWN_RTP_ENCODING:
        return "Unknown RTP encoding";
    case AES67_STATUS_UNEXPECTED_RTP_PAYLOAD_TYPE:
        return "Unexpected RTP payload type";
    case AES67_STATUS_BAD_PACKET_LENGTH:
        return "Bad packet length";
    case AES67_STATUS_RTP_PACKET_OUT_OF_SEQUENCE:
        return "RTP packet out of sequence";
    case AES67_STATUS_INVALID_SAMPLE_SIZE:
        return "Invalid sample size";
    case AES67_STATUS_INVALID_CHANNEL_COUNT:
        return "Invalid channel count";
    case AES67_STATUS_INVALID_SAMPLE_RATE:
        return "Invalid sample rate";
    case AES67_STATUS_INVALID_SDP_ADDRESS:
        return "Invalid SDP address";
    case AES67_STATUS_INVALID_SAP_PACKET:
        return "Invalid SAP packet";
    case AES67_STATUS_INVALID_SESSION_DESCRIPTION:
        return "Invalid session description";
    case AES67_STATUS_OUT_OF_BUFFER_SPACE:
        return "Out of buffer space";
    case AES67_STREAM_ALREADY_ADVERTISING:
        return "Stream already advertising";
    case AES67_STREAM_NOT_ADVERTISED:
        return "Stream not advertised";
    case AES67_STREAM_NOT_STREAMING:
        return "Stream not streaming";
    case AES67_STREAM_INVALID_FRAME_COUNT:
        return "Stream invalid frame count";
    case AES67_STATUS_UNKNOWN_ERROR:
        /* fallthrough */
    default:
        return "Unknown error";
    }
}
