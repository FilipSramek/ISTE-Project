/**
 * @file test_adc.cpp
 * @brief Unit tests for ADC driver.
 */

#include "hal/adc/adc.hpp"
#include <cstdint>

// Mock backend for testing
class MockADCBackend : public hal::IADCBackend
{
public:
    void init(uint8_t channel) override
    {
        m_last_init_channel = channel;
    }

    uint16_t read(uint8_t channel) override
    {
        m_last_read_channel = channel;
        return m_mock_value;
    }

    void set_clock_div(uint8_t channel, float clock_div) override
    {
        m_last_clock_div_channel = channel;
        m_last_clock_div = clock_div;
    }

    // Test accessors
    uint8_t get_last_init_channel() const { return m_last_init_channel; }
    uint8_t get_last_read_channel() const { return m_last_read_channel; }
    uint8_t get_last_clock_div_channel() const { return m_last_clock_div_channel; }
    float get_last_clock_div() const { return m_last_clock_div; }
    void set_mock_value(uint16_t value) { m_mock_value = value; }

private:
    uint8_t m_last_init_channel = 0xFF;
    uint8_t m_last_read_channel = 0xFF;
    uint8_t m_last_clock_div_channel = 0xFF;
    float m_last_clock_div = 0.0f;
    uint16_t m_mock_value = 0;
};

// Basic test function
bool test_adc_initialization()
{
    MockADCBackend backend;
    hal::ADC::Config config{.channel = 2, .clock_div = 0.0f};
    hal::ADC adc(config, backend);

    return backend.get_last_init_channel() == 2;
}

// Test reading
bool test_adc_read()
{
    MockADCBackend backend;
    hal::ADC::Config config{.channel = 1, .clock_div = 0.0f};
    hal::ADC adc(config, backend);

    backend.set_mock_value(2048);
    uint16_t value = adc.read();

    return value == 2048 && backend.get_last_read_channel() == 1;
}

// Test clock divider
bool test_adc_clock_div()
{
    MockADCBackend backend;
    hal::ADC::Config config{.channel = 0, .clock_div = 2500.0f};
    hal::ADC adc(config, backend);

    return backend.get_last_clock_div() == 2500.0f &&
           backend.get_last_clock_div_channel() == 0;
}

// Test get_channel
bool test_adc_get_channel()
{
    MockADCBackend backend;
    hal::ADC::Config config{.channel = 4, .clock_div = 0.0f};
    hal::ADC adc(config, backend);

    return adc.get_channel() == 4;
}
