#pragma once

#include "pico/time.h"


class LoopRate
{
public:
    explicit LoopRate(int64_t period_ms);

    int64_t wait();

private:
    const int64_t m_period;
    absolute_time_t m_prev_ts;
};
