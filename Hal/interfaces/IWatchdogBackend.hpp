/**
 * @file IWatchdogBackend.hpp
 * @brief Backend interface for watchdog timer access.
 */

#pragma once

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

namespace hal
{
    /**
     * @brief Abstract interface for watchdog backend implementation.
     */
    class IWatchdogBackend
    {
    public:
        virtual ~IWatchdogBackend() = default;

        /**
         * @brief Enable watchdog timer.
         * @param timeout_ms Timeout in milliseconds
         * @param pause_on_debug Pause watchdog when debugging
         */
        virtual void enable(uint32_t timeout_ms, bool pause_on_debug) = 0;

        /**
         * @brief Feed (kick) the watchdog.
         */
        virtual void update() = 0;

        /**
         * @brief Check if last reboot was caused by watchdog.
         * @return true if watchdog caused last reboot
         */
        virtual bool caused_reboot() = 0;

        /**
         * @brief Reboot the system after a delay.
         * @param delay_ms Delay before reboot in milliseconds
         */
        virtual void reboot(uint32_t delay_ms) = 0;
    };
}
