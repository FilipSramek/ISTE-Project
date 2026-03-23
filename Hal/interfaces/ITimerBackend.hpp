/**
 * @file ITimerBackend.hpp
 * @brief Backend interface for Timer access.
 */

#pragma once

#include "pico/stdlib.h"

namespace hal
{
    /**
     * @brief Abstract interface for Timer backend implementation.
     */
    class ITimerBackend
    {
    public:
        virtual ~ITimerBackend() = default;

        /**
         * @brief Initialize the timer.
         */
        virtual void init() = 0;

        /**
         * @brief Get current timer count in microseconds.
         * @return Current time in microseconds
         */
        virtual uint64_t get_time_us() = 0;

        /**
         * @brief Get elapsed time in microseconds since a reference point.
         * @param start_time Reference start time in microseconds
         * @return Elapsed time in microseconds
         */
        virtual uint64_t get_elapsed_us(uint64_t start_time) = 0;

        /**
         * @brief Delay for specified microseconds.
         * @param us Microseconds to delay
         */
        virtual void delay_us(uint32_t us) = 0;

        /**
         * @brief Delay for specified milliseconds.
         * @param ms Milliseconds to delay
         */
        virtual void delay_ms(uint32_t ms) = 0;

        /**
         * @brief Check if a timeout has occurred.
         * @param start_time Reference start time in microseconds
         * @param timeout_us Timeout duration in microseconds
         * @return True if timeout has occurred, false otherwise
         */
        virtual bool is_timeout(uint64_t start_time, uint64_t timeout_us) = 0;
    };
}
