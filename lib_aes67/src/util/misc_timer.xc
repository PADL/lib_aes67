// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
#include <xs1.h>
#include "misc_timer.h"

#define TICKS_PER_CENTISECOND (XS1_TIMER_KHZ * 10)
#define timeafter(A, B) ((int)((B) - (A)) < 0)

unsigned get_local_time(void) {
    unsigned t;
    timer tmr;

    tmr :> t;
    return t;
}

void waitfor(unsigned t) {
    timer tmr;

    tmr when timerafter(t) :> void;
}
