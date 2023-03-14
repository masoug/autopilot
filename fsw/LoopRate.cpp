#include "LoopRate.hpp"


LoopRate::LoopRate(const int64_t period_ms)
    : m_period(period_ms*1000)
    , m_prev_ts(get_absolute_time())
{
}

int64_t LoopRate::wait()
{
    const auto now = get_absolute_time();
    const auto delta_t = absolute_time_diff_us(m_prev_ts, now);
    const auto slack = m_period - delta_t;
    if (slack > 0)
    {
        // sleep the difference
        sleep_us(slack);
    }

    m_prev_ts = get_absolute_time();
    return slack;
}
