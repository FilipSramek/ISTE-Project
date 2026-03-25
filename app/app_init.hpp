#pragma once

#include "drivers/bmp280/bmp280.hpp"
#include "drivers/ina219/ina219.hpp"
#include "drivers/sx1278/sx1278.hpp"
#include "drivers/voltage_divider/voltage_divider.hpp"
#include "hal/adc/adc.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"

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
        drivers::VoltageDivider voltage_divider_2;

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

        drivers::BMP280 bmp280;
        drivers::INA219 ina219;
        drivers::SX1278 sx1278;
    };

    bool init(AppContext& context);
}
