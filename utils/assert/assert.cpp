#include "utils/assert/assert.hpp"

#include <cstdio>
#include "pico/stdlib.h"

namespace utils
{
    void assert_failed(const char* file, uint32_t line)
    {
        (void)file;
        (void)line;

        // Intentional fail-stop: stay here forever for debugging.
        while (true)
        {
            tight_loop_contents();
            
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
            sleep_ms(250);

            printf("%s at line %lu\n", file, line);
        }
    }
} // namespace utils
