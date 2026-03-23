/**
 * @file test_timer.cpp
 * @brief Unit tests for Timer driver.
 */

#include "hal/timer/timer.hpp"
#include <cstdint>

// Mock backend for testing
class MockTimerBackend : public hal::ITimerBackend
{
public:
    void init() override
    {
        m_initialized = true;
    }

    uint64_t get_time_us() override
    {
        return m_current_time_us;
    }

    uint64_t get_elapsed_us(uint64_t start_time) override
    {
        return m_current_time_us - start_time;
    }

    void delay_us(uint32_t us) override
    {
        m_last_delay_us = us;
        m_delay_called = true;
    }

    void delay_ms(uint32_t ms) override
    {
        m_last_delay_ms = ms;
        m_delay_called = true;
    }

    bool is_timeout(uint64_t start_time, uint64_t timeout_us) override
    {
        return (m_current_time_us - start_time) >= timeout_us;
    }

    // Test accessors
    void set_current_time(uint64_t time_us) { m_current_time_us = time_us; }
    bool is_initialized() const { return m_initialized; }
    bool delay_was_called() const { return m_delay_called; }
    uint32_t get_last_delay_us() const { return m_last_delay_us; }
    uint32_t get_last_delay_ms() const { return m_last_delay_ms; }

private:
    uint64_t m_current_time_us = 0;
    bool m_initialized = false;
    bool m_delay_called = false;
    uint32_t m_last_delay_us = 0;
    uint32_t m_last_delay_ms = 0;
};

// Test initialization
bool test_timer_initialization()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    return backend.is_initialized();
}

// Test now_us
bool test_timer_now_us()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(12345);
    return timer.now() == 12345;
}

// Test now_ms
bool test_timer_now_ms()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(5000000);  // 5000 ms
    return timer.now_ms() == 5000;
}

// Test delay_ms
bool test_timer_delay_ms()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    timer.delay_ms(500);
    return backend.delay_was_called() && backend.get_last_delay_ms() == 500;
}

// Test delay_us
bool test_timer_delay_us()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    timer.delay_us(1000);
    return backend.delay_was_called() && backend.get_last_delay_us() == 1000;
}

// Test elapsed_us
bool test_timer_elapsed_us()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000);
    uint64_t start = timer.now();

    backend.set_current_time(6000);
    uint64_t elapsed = timer.elapsed_us(start);

    return elapsed == 5000;
}

// Test elapsed_ms
bool test_timer_elapsed_ms()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000000);
    uint64_t start = timer.now();

    backend.set_current_time(6000000);
    uint64_t elapsed = timer.elapsed_ms(start);

    return elapsed == 5000;
}

// Test timeout_us - not timed out
bool test_timer_timeout_us_not_expired()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000);
    uint64_t start = timer.now();

    backend.set_current_time(3000);  // Only 2000 us elapsed
    return !timer.is_timeout_us(start, 5000);
}

// Test timeout_us - timed out
bool test_timer_timeout_us_expired()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000);
    uint64_t start = timer.now();

    backend.set_current_time(8000);  // 7000 us elapsed
    return timer.is_timeout_us(start, 5000);
}

// Test timeout_ms
bool test_timer_timeout_ms()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000000);
    uint64_t start = timer.now_ms();

    backend.set_current_time(3500000);  // 2500 ms elapsed
    return timer.is_timeout_ms(start, 2000);
}

// Test stopwatch
bool test_timer_stopwatch()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000);
    uint64_t sw_start = timer.stopwatch_start();

    backend.set_current_time(11000);
    uint64_t sw_elapsed = timer.stopwatch_elapsed_ms(sw_start);

    return sw_elapsed == 10;
}

// Test rate_limit_ms - not enough time passed
bool test_timer_rate_limit_ms_not_ready()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000000);
    uint64_t last_time = timer.now_ms();

    backend.set_current_time(1300000);  // Only 300 ms passed
    return !timer.rate_limit_ms(last_time, 500);
}

// Test rate_limit_ms - enough time passed
bool test_timer_rate_limit_ms_ready()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000000);
    uint64_t last_time = timer.now_ms();

    backend.set_current_time(1700000);  // 700 ms passed
    return timer.rate_limit_ms(last_time, 500);
}

// Test rate_limit_us
bool test_timer_rate_limit_us()
{
    MockTimerBackend backend;
    hal::Timer timer(backend);

    backend.set_current_time(1000);
    uint64_t last_time = timer.now();

    backend.set_current_time(6000);  // 5000 us passed
    return timer.rate_limit_us(last_time, 3000);
}
