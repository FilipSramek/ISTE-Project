/**
 * @file adc.cpp
 * @brief ADC HAL implementation.
 */

#include "hal/adc/adc.hpp"

#include "utils/assert/assert.hpp"

namespace hal
{
    void PicoADCBackend::init(uint8_t channel)
    {
        ASSERT(channel <= 4U);

        adc_init();
        adc_gpio_init(26 + channel);  // ADC channels 0-4 are on GPIO 26-30
    }

    uint16_t PicoADCBackend::read(uint8_t channel)
    {
        ASSERT(channel <= 4U);

        adc_select_input(channel);
        return adc_read();
    }

    void PicoADCBackend::set_clock_div(uint8_t channel, float clock_div)
    {
        ASSERT(channel <= 4U);
        ASSERT(clock_div >= 0.0f);

        adc_set_clkdiv(clock_div);
    }

    ADC::ADC(const Config& config, IADCBackend& backend)
        : m_config(config), m_backend(backend)
    {
        ASSERT(config.channel <= 4U);

        m_backend.init(m_config.channel);
        if (m_config.clock_div > 0.0f)
        {
            m_backend.set_clock_div(m_config.channel, m_config.clock_div);
        }
    }

    uint16_t ADC::read() const
    {
        return m_backend.read(m_config.channel);
    }

    void ADC::set_clock_div(float clock_div)
    {
        ASSERT(clock_div >= 0.0f);

        m_backend.set_clock_div(m_config.channel, clock_div);
    }

    uint8_t ADC::get_channel() const
    {
        return m_config.channel;
    }
}
