/**
 * @file IInputCaptureBackend.hpp
 * @brief Backend interface for input capture (PWM timing) access.
 */

#pragma once

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace hal
{
    /**
     * @brief Abstract interface for input capture backend implementation.
     */
    class IInputCaptureBackend
    {
    public:
        using CaptureCallback = void (*)(uint8_t pin, uint32_t events, void* user_data);

        virtual ~IInputCaptureBackend() = default;

        /**
         * @brief Initialize input capture on a GPIO pin.
         * @param pin GPIO pin number
         * @param pull_up Enable pull-up
         * @param pull_down Enable pull-down
         */
        virtual void init(uint8_t pin, bool pull_up, bool pull_down) = 0;

        /**
         * @brief Set callback for capture events.
         * @param callback Callback function
         * @param user_data User pointer passed to callback
         */
        virtual void set_callback(CaptureCallback callback, void* user_data) = 0;

        /**
         * @brief Enable edge interrupts for capture.
         * @param pin GPIO pin number
         * @param events_mask GPIO_IRQ_EDGE_RISE and/or GPIO_IRQ_EDGE_FALL
         */
        virtual void enable_irq(uint8_t pin, uint32_t events_mask) = 0;

        /**
         * @brief Disable edge interrupts for capture.
         * @param pin GPIO pin number
         */
        virtual void disable_irq(uint8_t pin) = 0;
    };
}
