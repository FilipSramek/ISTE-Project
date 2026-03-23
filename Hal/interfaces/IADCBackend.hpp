/**
 * @file IADCBackend.hpp
 * @brief Backend interface for ADC access.
 */

#pragma once

#include "pico/stdlib.h"

namespace hal
{
    /**
     * @brief Abstract interface for ADC backend implementation.
     */
    class IADCBackend
    {
    public:
        virtual ~IADCBackend() = default;

        /**
         * @brief Initialize ADC channel.
         * @param channel ADC channel number (0-4 for RP2040)
         */
        virtual void init(uint8_t channel) = 0;

        /**
         * @brief Read ADC value from channel.
         * @param channel ADC channel number
         * @return 12-bit ADC value (0-4095)
         */
        virtual uint16_t read(uint8_t channel) = 0;

        /**
         * @brief Set ADC sampling rate / clock divider.
         * @param channel ADC channel number
         * @param clock_div Clock divider value
         */
        virtual void set_clock_div(uint8_t channel, float clock_div) = 0;
    };
}
