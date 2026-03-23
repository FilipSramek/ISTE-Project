/**
 * @file IGPIOBackend.hpp
 * @brief Backend interface for GPIO access.
 */

#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace hal
{
    /**
     * @brief Abstract interface for GPIO backend implementation.
     */
    class IGPIOBackend
    {
    public:
        virtual ~IGPIOBackend() = default;

        /**
         * @brief Initialize GPIO pin.
         * @param pin GPIO pin number
         */
        virtual void init(uint8_t pin) = 0;

        /**
         * @brief Set GPIO direction.
         * @param pin GPIO pin number
         * @param is_output true for output, false for input
         */
        virtual void set_dir(uint8_t pin, bool is_output) = 0;

        /**
         * @brief Configure GPIO pull-up/pull-down.
         * @param pin GPIO pin number
         * @param pull_up Enable pull-up
         * @param pull_down Enable pull-down
         */
        virtual void set_pull(uint8_t pin, bool pull_up, bool pull_down) = 0;

        /**
         * @brief Write GPIO output level.
         * @param pin GPIO pin number
         * @param value Output level
         */
        virtual void write(uint8_t pin, bool value) = 0;

        /**
         * @brief Read GPIO input level.
         * @param pin GPIO pin number
         * @return true if pin is high
         */
        virtual bool read(uint8_t pin) = 0;

        /**
         * @brief Toggle GPIO output level.
         * @param pin GPIO pin number
         */
        virtual void toggle(uint8_t pin) = 0;
    };
}
