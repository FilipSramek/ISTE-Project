#pragma once

#include "drivers/bmp280/bmp280.hpp"
#include "drivers/ina219/ina219.hpp"
#include "drivers/sx1278/sx1278.hpp"
#include "drivers/voltage_divider/voltage_divider.hpp"
#include "drivers/soil_moisture/soil_moisture.hpp"
#include "drivers/sd/sd.hpp"
#include "hal/adc/adc.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"
#include "app_data.hpp"

namespace app
{
    struct AppContext
    {
        AppContext();

        hal::PicoI2CBackend i2c_backend;
        hal::I2C i2c_bmp280;
        hal::I2C i2c_ina219;

        hal::PicoADCBackend adc_backend;
        drivers::VoltageDivider voltage_divider_1;

        hal::PicoSPIBackend spi_backend;
        hal::SPI spi;

        hal::PicoGPIOBackend gpio_nss_backend;
        hal::PicoGPIOBackend gpio_reset_backend;
        hal::PicoGPIOBackend gpio_dio0_backend;
        hal::GPIO gpio_nss;
        hal::GPIO gpio_reset;
        hal::GPIO gpio_dio0;

        hal::PicoTimerBackend timer_backend;
        hal::Timer timer;

        // SD Card
        hal::PicoGPIOBackend gpio_sd_cs_backend;
        hal::GPIO gpio_sd_cs;
        hal::PicoSDCardBackend sd_backend;
        hal::SDCard sd_card;

        drivers::BMP280 bmp280;
        drivers::INA219 ina219;
        drivers::SX1278 sx1278;
        soil_moisture::SoilMoistureSensor soil_sensor;

        // Napájení humidity senzoru
        hal::PicoGPIOBackend gpio_humidity_power_backend;
        hal::GPIO gpio_humidity_power;

        // Stavova LED po inicializaci
        hal::PicoGPIOBackend gpio_status_led_backend;
        hal::GPIO gpio_status_led;
    };

    bool init(AppContext& context);
    const char* last_init_error();
}

namespace app {
    struct AppContext;
}
extern app::AppContext* g_app_context_ptr;
