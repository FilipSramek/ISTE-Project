/**
 * @file example_gpio.cpp
 * @brief Example usage of the GPIO module.
 */

#include "hal/gpio/gpio.hpp"

namespace example
{
    /**
     * @brief Example GPIO usage.
     */
    void example_gpio()
    {
        hal::PicoGPIOBackend backend;

        const hal::GPIO::Config config = {
            25U,   // pin (onboard LED)
            true,  // output
            false, // initial low
            false, // pull-up
            false  // pull-down
        };

        hal::GPIO gpio(config, backend);
        bool ok = gpio.init();
        if (!ok)
        {
            return;
        }

        ok = gpio.write(true);
        if (!ok)
        {
            return;
        }

        bool level = false;
        ok = gpio.read(level);
        if (!ok)
        {
            return;
        }

        ok = gpio.toggle();
        if (!ok)
        {
            return;
        }
    }
}
