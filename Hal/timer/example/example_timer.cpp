/**
 * @file example_timer.cpp
 * @brief Example usage of Timer driver.
 */

#include "hal/timer/timer.hpp"
#include "pico/stdlib.h"
#include <cstdio>

int main()
{
    stdio_init_all();

    // Create backend and timer driver
    hal::PicoTimerBackend timer_backend;
    hal::Timer timer(timer_backend);

    printf("Timer Example\n");
    printf("=============\n\n");

    // Example 1: Basic timing
    printf("Example 1: Current time\n");
    printf("Current time: %llu us\n", timer.now());
    printf("Current time: %llu ms\n\n", timer.now_ms());

    // Example 2: Delay
    printf("Example 2: Delay for 1 second\n");
    timer.delay_ms(1000);
    printf("Done!\n\n");

    // Example 3: Elapsed time measurement
    printf("Example 3: Measure elapsed time\n");
    uint64_t start = timer.stopwatch_start();
    timer.delay_ms(500);
    uint64_t elapsed = timer.stopwatch_elapsed_ms(start);
    printf("Elapsed time: %llu ms\n\n", elapsed);

    // Example 4: Timeout checking
    printf("Example 4: Timeout checking\n");
    uint64_t timeout_start = timer.now_ms();
    uint32_t timeout_counter = 0;
    while (!timer.is_timeout_ms(timeout_start, 2000))
    {
        timeout_counter++;
        if (timeout_counter % 1000000 == 0)
        {
            printf("Still running... (%llu ms elapsed)\n", timer.elapsed_ms(timeout_start));
        }
    }
    printf("Timeout reached!\n\n");

    // Example 5: Rate limiting
    printf("Example 5: Rate limiting (print every 500ms)\n");
    uint64_t last_print = timer.now_ms();
    uint32_t print_count = 0;
    while (print_count < 5)
    {
        if (timer.rate_limit_ms(last_print, 500))
        {
            printf("Print #%u at %llu ms\n", ++print_count, timer.now_ms());
        }
    }

    printf("\nTimer example complete!\n");
    return 0;
}
