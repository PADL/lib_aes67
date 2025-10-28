// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved

#pragma once

#include <stdint.h>

#define CLK_CTL_SET_RATE 0x1
#define CLK_CTL_STOP 0x2

/** Commands that can be sent to the buffer */
#define BUF_CTL_GET_INCOMING_COUNT 1
#define BUF_CTL_GET_OUTGOING_COUNT 2
#define BUF_CTL_GET_COUNTS 3
#define BUF_CTL_RESET_COUNTS 4
#define BUF_CTL_RESET_LOCKED 5
#define BUF_CTL_RESET_UNLOCKED 6
#define BUF_CTL_RESET_COUNTS_NO_NOTIFY 7
#define BUF_CTL_GET_INCOMING_TIMESTAMP 8
#define BUF_CTL_GET_OUTGOING_TIMESTAMP 9
#define BUF_CTL_SYNC 10
#define BUF_CTL_FILL_LEVEL 11
#define BUF_CTL_REQUEST_INFO 12
#define BUF_CTL_ADJUST_FILL 13
#define BUF_CTL_ACK 14
#define BUF_CTL_GOT_INFO 15
#define BUF_CTL_RESET 16
#define BUF_CTL_NEW_STREAM 17
#define BUF_CTL_NOTIFY_STREAM_INFO 18

void aes67_notify_buf_ctl_stream_info(chanend buf_ctl, uintptr_t stream_num);

void aes67_buf_ctl_ack(chanend buf_ctl);
int aes67_get_buf_ctl_adjust(chanend buf_ctl);
int aes67_get_buf_ctl_cmd(chanend buf_ctl);
void aes67_send_buf_ctl_info(chanend buf_ctl,
                             int active,
                             unsigned int ptp_ts,
                             unsigned int local_ts,
                             unsigned int rdptr,
                             unsigned int wrptr,
                             timer tmr);
