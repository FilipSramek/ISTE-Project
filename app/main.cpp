#include <stdio.h>

#include "app/app_init.hpp"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"


int main()
{
    stdio_init_all();

    printf("Starting application...\n");
    while (true)
    {
        app::AppContext app_context;
        if (!app::init(app_context))
        {
            printf("Application init failed: %s\n", app::last_init_error());
            sleep_ms(1000);
            continue;
        }

        printf("Application init successful\n");

        while (true)
        {
            if (!app_context.bmp280.update())
            {
                printf("[RUNTIME] bmp280.update() failed\n");
            }

            if (!app_context.ina219.update())
            {
                printf("[RUNTIME] ina219.update() failed\n");
            }

            if (!app_context.voltage_divider_1.update())
            {
                printf("[RUNTIME] voltage_divider_1.update() failed\n");
            }

            if (!app_context.voltage_divider_2.update())
            {
                printf("[RUNTIME] voltage_divider_2.update() failed\n");
            }

            if (!app_context.sx1278.update())
            {
                printf("[RUNTIME] sx1278.update() failed\n");
            }

            printf("System initialized and running\n");
            sleep_ms(1000);
        }
    }
}
