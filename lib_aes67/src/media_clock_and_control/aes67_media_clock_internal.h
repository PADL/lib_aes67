// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved

#pragma once

#include <xccompat.h>
#include "aes67_internal.h"

#ifndef MAX_CLK_CTL_CLIENTS
#define MAX_CLK_CTL_CLIENTS 8
#endif

#ifndef PLL_TO_WORD_MULTIPLIER
#define PLL_TO_WORD_MULTIPLIER 100
#endif

/** A description of a media clock */
typedef struct aes67_media_clock_t {
    aes67_media_clock_info_t info;
    unsigned int wordLength;
    unsigned int baseLengthRemainder;
    unsigned int wordTime;
    unsigned int baseLength;
    unsigned int lowBits;
    int count;
    unsigned int next_event;
    unsigned int bit;
} aes67_media_clock_t;

typedef struct buf_info_t {
    int lock_count;
    int prev_diff;
    int stability_count;
    int adjust;
    uintptr_t fifo;
} buf_info_t;

extern buf_info_t buf_info[AES67_NUM_MEDIA_OUTPUTS];
extern uint32_t fifo_uninit_count;

extern aes67_media_clock_t ptp_media_clock;

void aes67_media_clock_init_buffers(void);
int aes67_media_clock_get_buf_info(uintptr_t fifo);

#define WC_FRACTIONAL_BITS 16

// The number of ticks between period clock recovery checks
#define CLOCK_RECOVERY_PERIOD (1 << 21)

void aes67_init_media_clock_recovery(NULLABLE_RESOURCE(chanend, ptp_svr),
                                     uint32_t clk_time,
                                     uint32_t rate);

uint32_t aes67_update_media_clock(
    NULLABLE_RESOURCE(chanend, ptp_svr),
    REFERENCE_PARAM(const aes67_media_clock_t, mclock),
    uint32_t t2,
    int32_t period,
    REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t,
                    pid_coefficients));
