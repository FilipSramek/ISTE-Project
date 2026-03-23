/**
 * @file adc.hpp
 * @brief ADC HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/IADCBackend.hpp"
#include "hardware/adc.h"

namespace hal
{
    /**
     * @brief Pico-specific ADC backend.
     */
    class PicoADCBackend final : public IADCBackend
    {
    public:
        void init(uint8_t channel) override;
        uint16_t read(uint8_t channel) override;
        void set_clock_div(uint8_t channel, float clock_div) override;
    };

    /**
     * @brief ADC driver with configuration management.
     */
    class ADC final
    {
    public:
        /**
         * @brief ADC configuration structure.
         */
        struct Config
        {
            uint8_t channel;    ///< ADC channel (0-4 for RP2040)
            float clock_div;    ///< Clock divider for sampling rate (default 0.0)
        };

        /**
         * @brief Construct ADC driver with configuration and backend.
         * @param config ADC configuration
         * @param backend Reference to backend implementation
         */
        ADC(const Config& config, IADCBackend& backend);

        /**
         * @brief Read ADC value.
         * @return 12-bit ADC value (0-4095)
         */
        uint16_t read() const;

        /**
         * @brief Set clock divider for sampling rate.
         * @param clock_div Clock divider value
         */
        void set_clock_div(float clock_div);

        /**
         * @brief Get configured channel.
         * @return ADC channel number
         */
        uint8_t get_channel() const;

    private:
        const Config m_config;
        IADCBackend& m_backend;
    };
}
