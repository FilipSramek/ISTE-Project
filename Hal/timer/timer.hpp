/**
 * @file timer.hpp
 * @brief Timer HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/ITimerBackend.hpp"

namespace hal
{
    /**
     * @brief Pico-specific Timer backend.
     */
    class PicoTimerBackend final : public ITimerBackend
    {
    public:
        void init() override;
        uint64_t get_time_us() override;
        uint64_t get_elapsed_us(uint64_t start_time) override;
        void delay_us(uint32_t us) override;
        void delay_ms(uint32_t ms) override;
        bool is_timeout(uint64_t start_time, uint64_t timeout_us) override;
    };

    /**
     * @brief Timer driver with advanced timing utilities.
     */
    class Timer final
    {
    public:
        /**
         * @brief Construct Timer driver with backend.
         * @param backend Reference to backend implementation
         */
        explicit Timer(ITimerBackend& backend);

        /**
         * @brief Get current time in microseconds.
         * @return Current time in microseconds
         */
        uint64_t now() const;

        /**
         * @brief Get current time in milliseconds.
         * @return Current time in milliseconds
         */
        uint64_t now_ms() const;

        /**
         * @brief Delay execution for specified microseconds.
         * @param us Microseconds to delay
         */
        void delay_us(uint32_t us) const;

        /**
         * @brief Delay execution for specified milliseconds.
         * @param ms Milliseconds to delay
         */
        void delay_ms(uint32_t ms) const;

        /**
         * @brief Get elapsed time in microseconds since a reference point.
         * @param start_time Reference start time in microseconds
         * @return Elapsed time in microseconds
         */
        uint64_t elapsed_us(uint64_t start_time) const;

        /**
         * @brief Get elapsed time in milliseconds since a reference point.
         * @param start_time Reference start time in milliseconds
         * @return Elapsed time in milliseconds
         */
        uint64_t elapsed_ms(uint64_t start_time) const;

        /**
         * @brief Check if a timeout has occurred.
         * @param start_time Reference start time in microseconds
         * @param timeout_us Timeout duration in microseconds
         * @return True if timeout has occurred, false otherwise
         */
        bool is_timeout_us(uint64_t start_time, uint64_t timeout_us) const;

        /**
         * @brief Check if a timeout has occurred (millisecond version).
         * @param start_time Reference start time in milliseconds
         * @param timeout_ms Timeout duration in milliseconds
         * @return True if timeout has occurred, false otherwise
         */
        bool is_timeout_ms(uint64_t start_time, uint64_t timeout_ms) const;

        /**
         * @brief Stopwatch functionality - start timing.
         * @return Stopwatch start time in microseconds
         */
        uint64_t stopwatch_start() const;

        /**
         * @brief Get stopwatch elapsed time in microseconds.
         * @param start_time Stopwatch start time
         * @return Elapsed time in microseconds
         */
        uint64_t stopwatch_elapsed_us(uint64_t start_time) const;

        /**
         * @brief Get stopwatch elapsed time in milliseconds.
         * @param start_time Stopwatch start time
         * @return Elapsed time in milliseconds
         */
        uint64_t stopwatch_elapsed_ms(uint64_t start_time) const;

        /**
         * @brief Rate limiter - check if enough time has passed.
         * @param last_time Reference time of last execution
         * @param min_interval_ms Minimum interval in milliseconds
         * @return True if min_interval_ms has passed, false otherwise
         */
        bool rate_limit_ms(uint64_t& last_time, uint32_t min_interval_ms) const;

        /**
         * @brief Rate limiter - check if enough time has passed (microsecond version).
         * @param last_time Reference time of last execution
         * @param min_interval_us Minimum interval in microseconds
         * @return True if min_interval_us has passed, false otherwise
         */
        bool rate_limit_us(uint64_t& last_time, uint32_t min_interval_us) const;

    private:
        ITimerBackend& m_backend;
    };
}
