/**
 * @file timer.cpp
 * @brief Timer HAL implementation.
 */

#include "hal/timer/timer.hpp"

#include "utils/assert/assert.hpp"

namespace hal
{
    void PicoTimerBackend::init()
    {
        // Timer is already initialized by pico_stdlib
    }

    uint64_t PicoTimerBackend::get_time_us()
    {
        return time_us_64();
    }

    uint64_t PicoTimerBackend::get_elapsed_us(uint64_t start_time)
    {
        return time_us_64() - start_time;
    }

    void PicoTimerBackend::delay_us(uint32_t us)
    {
        sleep_us(us);
    }

    void PicoTimerBackend::delay_ms(uint32_t ms)
    {
        sleep_ms(ms);
    }

    bool PicoTimerBackend::is_timeout(uint64_t start_time, uint64_t timeout_us)
    {
        return (time_us_64() - start_time) >= timeout_us;
    }

    Timer::Timer(ITimerBackend& backend) : m_backend(backend)
    {
        m_backend.init();
    }

    uint64_t Timer::now() const
    {
        return m_backend.get_time_us();
    }

    uint64_t Timer::now_ms() const
    {
        return m_backend.get_time_us() / 1000ULL;
    }

    void Timer::delay_us(uint32_t us) const
    {
        m_backend.delay_us(us);
    }

    void Timer::delay_ms(uint32_t ms) const
    {
        m_backend.delay_ms(ms);
    }

    uint64_t Timer::elapsed_us(uint64_t start_time) const
    {
        return m_backend.get_elapsed_us(start_time);
    }

    uint64_t Timer::elapsed_ms(uint64_t start_time) const
    {
        return m_backend.get_elapsed_us(start_time) / 1000ULL;
    }

    bool Timer::is_timeout_us(uint64_t start_time, uint64_t timeout_us) const
    {
        return m_backend.is_timeout(start_time, timeout_us);
    }

    bool Timer::is_timeout_ms(uint64_t start_time, uint64_t timeout_ms) const
    {
        return m_backend.is_timeout(start_time, timeout_ms * 1000ULL);
    }

    uint64_t Timer::stopwatch_start() const
    {
        return m_backend.get_time_us();
    }

    uint64_t Timer::stopwatch_elapsed_us(uint64_t start_time) const
    {
        return m_backend.get_elapsed_us(start_time);
    }

    uint64_t Timer::stopwatch_elapsed_ms(uint64_t start_time) const
    {
        return m_backend.get_elapsed_us(start_time) / 1000ULL;
    }

    bool Timer::rate_limit_ms(uint64_t& last_time, uint32_t min_interval_ms) const
    {
        const uint64_t current_time = m_backend.get_time_us();
        const uint64_t min_interval_us = static_cast<uint64_t>(min_interval_ms) * 1000ULL;

        if ((current_time - last_time) >= min_interval_us)
        {
            last_time = current_time;
            return true;
        }

        return false;
    }

    bool Timer::rate_limit_us(uint64_t& last_time, uint32_t min_interval_us) const
    {
        const uint64_t current_time = m_backend.get_time_us();

        if ((current_time - last_time) >= min_interval_us)
        {
            last_time = current_time;
            return true;
        }

        return false;
    }
}
