// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Nicholas J. Humfrey
// Portions Copyright (c) 2020-2025 PADL Software Pty Ltd

#pragma once

#include <stdint.h>
#include <debug_print.h>

#include "aes67_internal.h"

// ------- Encoding Types ---------

typedef enum {
    AES67_ENCODING_L8,
    AES67_ENCODING_L16,
    AES67_ENCODING_L24,
    AES67_ENCODING_L32,
    AES67_ENCODING_PCMU, // G.711 - uLaw
    AES67_ENCODING_PCMA, // G.711 - aLaw
    AES67_ENCODING_G722, // SB-ADPCM
    AES67_ENCODING_GSM,
    AES67_ENCODING_AM824,
    AES67_ENCODING_MAX
} aes67_encodings;

// ------- Logging ---------

typedef enum {
    aes67_LOG_DEBUG,
    aes67_LOG_INFO,
    aes67_LOG_WARN,
    aes67_LOG_ERROR
} aes67_log_level;

#if AES67_XMOS
#define aes67_log(level, fmt, ...) debug_printf(fmt, ##__VA_ARGS__)
#else
void aes67_log(aes67_log_level level, const char *fmt, ...);
#endif

// Only display debug if verbose
#define aes67_debug(...) aes67_log(aes67_LOG_DEBUG, __VA_ARGS__)

// Don't show info when quiet
#define aes67_info(...) aes67_log(aes67_LOG_INFO, __VA_ARGS__)

#define aes67_warn(...) aes67_log(aes67_LOG_WARN, __VA_ARGS__)

// All errors are fatal
#define aes67_error(...) aes67_log(aes67_LOG_ERROR, __VA_ARGS__)

// ------- Encoding Utilities ---------

#ifdef __XC__
const char *unsafe aes67_encoding_name(int encoding);
#else
const char *aes67_encoding_name(int encoding);
#endif

int aes67_encoding_lookup(const char *name);
