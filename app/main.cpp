#include <stdio.h>

#include "app/app_init.hpp"
#include "pico/stdlib.h"

int main()
{
    stdio_init_all();

    app::AppContext app_context;
    if (!app::init(app_context))
    {
        while (true)
        {
            printf("Application init failed\n");
            sleep_ms(1000);
        }
    }

    while (true)
    {
        (void)app_context.bmp280.update();
        (void)app_context.ina219.update();
        (void)app_context.voltage_divider_1.update();
        (void)app_context.voltage_divider_2.update();
        (void)app_context.sx1278.update();

        printf("System initialized and running\n");
        sleep_ms(1000);
    }
}
