#include "utils/assert/assert.hpp"

#include <cstdio>
#include "pico/stdlib.h"

namespace utils
{
    void assert_failed(const char* file, uint32_t line, const char* expression)
    {
        stdio_flush();
        printf("\n[ASSERT] %s\n", expression != nullptr ? expression : "<null expression>");
        printf("[ASSERT] file: %s\n", file != nullptr ? file : "<null file>");
        printf("[ASSERT] line: %lu\n", static_cast<unsigned long>(line));
        stdio_flush();

        // Intentional fail-stop: stay here forever for debugging.
        while (true)
        {
            tight_loop_contents();
            
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
            sleep_ms(250);
        }
    }
} // namespace utils
