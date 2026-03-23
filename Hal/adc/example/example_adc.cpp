/**
 * @file example_adc.cpp
 * @brief Example usage of ADC driver.
 */

#include "hal/adc/adc.hpp"
#include "pico/stdlib.h"
#include <cstdio>

int main()
{
    stdio_init_all();

    // Create ADC configuration
    hal::ADC::Config adc_config{
        .channel = 0,        // Use ADC channel 0 (GPIO 26)
        .clock_div = 0.0f,   // Use default clock divider
    };

    // Create backend and ADC driver
    hal::PicoADCBackend adc_backend;
    hal::ADC adc(adc_config, adc_backend);

    printf("ADC Example - Reading from channel %u\n", adc.get_channel());

    // Read ADC values
    for (int i = 0; i < 10; ++i)
    {
        uint16_t value = adc.read();
        printf("ADC reading %d: %u\n", i, value);
        sleep_ms(100);
    }

    return 0;
}
