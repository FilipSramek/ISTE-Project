#include "app/app_init.hpp"

namespace
{
    hal::I2C::Config make_i2c_config(uint8_t address)
    {
        return hal::I2C::Config{
            .instance = i2c0,
            .sda_pin = 4U,
            .scl_pin = 5U,
            .baudrate = 400000U,
            .address = address
        };
    }

    drivers::BMP280::Config make_bmp280_config()
    {
        return drivers::BMP280::Config{
            .temp_oversampling = drivers::BMP280::Oversampling::X2,
            .press_oversampling = drivers::BMP280::Oversampling::X16,
            .mode = drivers::BMP280::Mode::NORMAL,
            .filter = drivers::BMP280::Filter::X16,
            .standby = drivers::BMP280::Standby::MS_500
        };
    }

    drivers::INA219::Config make_ina219_config()
    {
        return drivers::INA219::Config{
            .bus_range = drivers::INA219::BusRange::RANGE_32V,
            .pga_gain = drivers::INA219::PGAGain::GAIN_8_320MV,
            .bus_adc = drivers::INA219::ADCResolution::BIT_12,
            .shunt_adc = drivers::INA219::ADCResolution::BIT_12,
            .mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
            .shunt_resistance = 0.1f,
            .max_expected_current = 3.2f
        };
    }

    drivers::VoltageDivider::Config make_divider1_config()
    {
        return drivers::VoltageDivider::Config{
            .channel = 0U,
            .adc_reference_voltage = 3.3f,
            .r_top_ohm = 30000.0f,
            .r_bottom_ohm = 10000.0f,
            .max_input_voltage = 16.0f,
            .adc_max_count = 4095U,
            .clock_div = 0.0f
        };
    }

    drivers::VoltageDivider::Config make_divider2_config()
    {
        return drivers::VoltageDivider::Config{
            .channel = 1U,
            .adc_reference_voltage = 3.3f,
            .r_top_ohm = 47000.0f,
            .r_bottom_ohm = 10000.0f,
            .max_input_voltage = 20.0f,
            .adc_max_count = 4095U,
            .clock_div = 0.0f
        };
    }

    hal::SPI::Config make_spi_config()
    {
        return hal::SPI::Config{
            .instance = spi0,
            .clk_pin = 18U,
            .mosi_pin = 19U,
            .miso_pin = 16U,
            .cs_pin = 17U,
            .baudrate = 1000000U
        };
    }

    hal::GPIO::Config make_nss_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 17U,
            .is_output = true,
            .initial_high = true,
            .pull_up = false,
            .pull_down = false
        };
    }

    hal::GPIO::Config make_reset_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 20U,
            .is_output = true,
            .initial_high = true,
            .pull_up = false,
            .pull_down = false
        };
    }

    hal::GPIO::Config make_dio0_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 21U,
            .is_output = false,
            .initial_high = false,
            .pull_up = false,
            .pull_down = false
        };
    }

    drivers::SX1278::Config make_sx1278_config()
    {
        return drivers::SX1278::Config{
            .frequency_hz = 433000000U,
            .bandwidth = drivers::SX1278::Bandwidth::BW_125_KHZ,
            .coding_rate = drivers::SX1278::CodingRate::CR_4_5,
            .spreading_factor = drivers::SX1278::SpreadingFactor::SF7,
            .preamble_length = 8U,
            .sync_word = 0x12U,
            .tx_power_dbm = 17,
            .crc_enabled = true,
            .rx_timeout_ms = 100U
        };
    }
}

namespace app
{
    AppContext::AppContext()
        : i2c_backend(),
          i2c_bmp280(make_i2c_config(0x76U), i2c_backend),
          i2c_ina219(make_i2c_config(0x40U), i2c_backend),
          adc_backend(),
          voltage_divider_1(adc_backend, make_divider1_config()),
          voltage_divider_2(adc_backend, make_divider2_config()),
          spi_backend(),
          spi(make_spi_config(), spi_backend),
          gpio_nss_backend(),
          gpio_reset_backend(),
          gpio_dio0_backend(),
          gpio_nss(make_nss_gpio_config(), gpio_nss_backend),
          gpio_reset(make_reset_gpio_config(), gpio_reset_backend),
          gpio_dio0(make_dio0_gpio_config(), gpio_dio0_backend),
          timer_backend(),
          timer(timer_backend),
          bmp280(i2c_bmp280, make_bmp280_config()),
          ina219(i2c_ina219, make_ina219_config()),
          sx1278(spi, gpio_nss, gpio_reset, gpio_dio0, timer, make_sx1278_config())
    {
    }

    bool init(AppContext& context)
    {
        if (!context.i2c_bmp280.init())
        {
            return false;
        }

        if (!context.i2c_ina219.init())
        {
            return false;
        }

        if (!context.spi.init())
        {
            return false;
        }

        if (!context.gpio_nss.init() || !context.gpio_reset.init() || !context.gpio_dio0.init())
        {
            return false;
        }

        if (!context.bmp280.init())
        {
            return false;
        }

        if (!context.ina219.init())
        {
            return false;
        }

        if (!context.voltage_divider_1.init())
        {
            return false;
        }

        if (!context.voltage_divider_2.init())
        {
            return false;
        }

        if (!context.sx1278.init())
        {
            return false;
        }

        return true;
    }
}
