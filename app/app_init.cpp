#include "app/app_init.hpp"

#include <stdio.h>

namespace
{
    char g_last_init_error[160] = "init not started";
    constexpr uint8_t INA219_REG_CONFIG = 0x00U;
    constexpr bool ENABLE_I2C_DEBUG_SCANNER = true;
    constexpr uint8_t I2C_SCAN_START_ADDRESS = 0x08U;
    constexpr uint8_t I2C_SCAN_END_ADDRESS = 0x77U;
    constexpr uint32_t I2C_SCAN_TIMEOUT_US = 2000U;

    void set_last_init_error(const char* message)
    {
        if (message == nullptr)
        {
            (void)snprintf(g_last_init_error, sizeof(g_last_init_error), "%s", "unknown init error");
            return;
        }

        (void)snprintf(g_last_init_error, sizeof(g_last_init_error), "%s", message);
    }

    void set_last_init_error_with_detail(const char* prefix, const char* detail)
    {
        const char* safe_prefix = (prefix != nullptr) ? prefix : "init failed";
        const char* safe_detail = (detail != nullptr) ? detail : "unknown reason";
        (void)snprintf(g_last_init_error, sizeof(g_last_init_error), "%s: %s", safe_prefix, safe_detail);
    }

    void log_init_checkpoint(const char* message)
    {
        printf("[INIT] %s\n", message);
    }

    void debug_scan_i2c_bus(i2c_inst_t* instance)
    {
        if (!ENABLE_I2C_DEBUG_SCANNER || (instance == nullptr))
        {
            return;
        }

        printf("[I2C-SCAN] Scanning bus...\n");
        bool found_any = false;
        uint8_t probe_byte = 0U;

        for (uint8_t address = I2C_SCAN_START_ADDRESS; address <= I2C_SCAN_END_ADDRESS; ++address)
        {
            // Some devices ACK write probe but NACK direct read probe.
            const int write_probe = i2c_write_timeout_us(instance,
                                                         address,
                                                         &probe_byte,
                                                         0U,
                                                         false,
                                                         I2C_SCAN_TIMEOUT_US);

            const int read_probe = i2c_read_timeout_us(instance,
                                                       address,
                                                       &probe_byte,
                                                       1U,
                                                       false,
                                                       I2C_SCAN_TIMEOUT_US);

            if ((write_probe >= 0) || (read_probe >= 0))
            {
                found_any = true;
                if ((write_probe >= 0) && (read_probe >= 0))
                {
                    printf("[I2C-SCAN] ACK at 0x%02X (R/W)\n", address);
                }
                else if (write_probe >= 0)
                {
                    printf("[I2C-SCAN] ACK at 0x%02X (W only)\n", address);
                }
                else
                {
                    printf("[I2C-SCAN] ACK at 0x%02X (R only)\n", address);
                }
            }
        }

        if (!found_any)
        {
            printf("[I2C-SCAN] No devices responded\n");
        }
    }

    bool probe_ina219_config_register(hal::I2C& i2c, uint8_t address, uint16_t& value)
    {
        if (!i2c.set_address(address))
        {
            return false;
        }

        uint8_t reg = INA219_REG_CONFIG;
        if (!i2c.write(&reg, 1U))
        {
            return false;
        }

        uint8_t buffer[2] = {0U, 0U};
        uint32_t length = 2U;
        if (!i2c.receive(buffer, length) || (length != 2U))
        {
            return false;
        }

        value = static_cast<uint16_t>((static_cast<uint16_t>(buffer[0]) << 8U) |
                                      static_cast<uint16_t>(buffer[1]));
        return true;
    }


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
            .channel = 1U,
            .adc_reference_voltage = 3.3f,
            .r_top_ohm = 100000.0f,
            .r_bottom_ohm = 100000.0f,
            .max_input_voltage = 16.0f,
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
            .cs_pin = 17U, // Hlavní CS pro SX1278
            .baudrate = 1000000U
        };
    }

    hal::GPIO::Config make_sd_cs_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 22U, // Nastav správný pin dle zapojení SD karty
            .is_output = true,
            .initial_high = true,
            .pull_up = false,
            .pull_down = false
        };
    }

    hal::SDCard::Config make_sd_config()
    {
        return hal::SDCard::Config{
            .block_size = 512U,
            .max_blocks = 32768U, // 16 MB karta, uprav dle potřeby
            .cmd_timeout_ms = 100U,
            .data_timeout_ms = 500U
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
            .pull_up = true,
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
            .rx_timeout_ms = 300U
        };
    }

    hal::GPIO::Config make_humidity_power_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 15U, // Nastav správný pin dle zapojení MOSFETu
            .is_output = true,
            .initial_high = false, // Po startu vypnuto
            .pull_up = false,
            .pull_down = false
        };
    }

    hal::GPIO::Config make_status_led_gpio_config()
    {
        return hal::GPIO::Config{
            .pin = 25U, // Stavova LED na Pico
            .is_output = true,
            .initial_high = false, // Rozsviti se az po uspesne inicializaci
            .pull_up = false,
            .pull_down = false
        };
    }
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
          gpio_sd_cs_backend(),
          gpio_sd_cs(make_sd_cs_gpio_config(), gpio_sd_cs_backend),
          sd_backend(spi, gpio_sd_cs, timer),
          sd_card(make_sd_config(), sd_backend),
          bmp280(i2c_bmp280, make_bmp280_config()),
          ina219(i2c_ina219, make_ina219_config()),
          sx1278(spi, gpio_nss, gpio_reset, gpio_dio0, timer, make_sx1278_config()),
          soil_sensor(0),
          gpio_humidity_power_backend(),
            gpio_humidity_power(make_humidity_power_gpio_config(), gpio_humidity_power_backend),
            gpio_status_led_backend(),
            gpio_status_led(make_status_led_gpio_config(), gpio_status_led_backend)
    {
    }

    bool init(AppContext& context)
    {
        if (!context.gpio_humidity_power.init()) {
            set_last_init_error("gpio_humidity_power.init() failed (pin 23)");
            return false;
        }
        log_init_checkpoint("GPIO humidity power initialized");

        log_init_checkpoint("Initialization started");
        set_last_init_error("unknown init error");

        if (!context.i2c_bmp280.init())
        {
            set_last_init_error("i2c_bmp280.init() failed (addr 0x76)");
            return false;
        }
        log_init_checkpoint("I2C BMP280 bus initialized");

        if (!context.i2c_ina219.init())
        {
            set_last_init_error("i2c_ina219.init() failed (addr 0x40)");
            return false;
        }
        log_init_checkpoint("I2C INA219 bus initialized");

        // Debug-only scanner to quickly see what is present on the I2C bus.
        debug_scan_i2c_bus(i2c0);

        if (!context.spi.init())
        {
            set_last_init_error("spi.init() failed");
            return false;
        }
        log_init_checkpoint("SPI initialized");

        if (!context.gpio_nss.init())
        {
            set_last_init_error("gpio_nss.init() failed (pin 17)");
            return false;
        }
        log_init_checkpoint("GPIO NSS initialized");

        if (!context.gpio_reset.init())
        {
            set_last_init_error("gpio_reset.init() failed (pin 20)");
            return false;
        }
        log_init_checkpoint("GPIO RESET initialized");

        if (!context.gpio_dio0.init())
        {
            set_last_init_error("gpio_dio0.init() failed (pin 21)");
            return false;
        }
        log_init_checkpoint("GPIO DIO0 initialized");

        if (!context.bmp280.init())
        {
            set_last_init_error_with_detail("bmp280.init() failed", context.bmp280.last_error());
            return false;
        }
        log_init_checkpoint("BMP280 sensor initialized");

        constexpr uint8_t ina219_first_address = 0x40U;
        constexpr uint8_t ina219_last_address = 0x4FU;
        bool ina219_initialized = false;
        bool ina219_any_probe = false;

        for (uint8_t address = ina219_first_address; address <= ina219_last_address; ++address)
        {
            uint16_t config_reg = 0U;
            if (probe_ina219_config_register(context.i2c_ina219, address, config_reg))
            {
                ina219_any_probe = true;
                printf("[INIT] INA219 probe ACK on 0x%02X, CONFIG=0x%04X\n", address, config_reg);
            }

            if (!context.i2c_ina219.set_address(address))
            {
                continue;
            }

            if (context.ina219.init())
            {
                printf("[INIT] INA219 sensor initialized on 0x%02X\n", address);
                ina219_initialized = true;
                break;
            }

            printf("[INIT] INA219 init failed on 0x%02X: %s\n", address, context.ina219.last_error());
        }

        if (!ina219_initialized)
        {
            if (!ina219_any_probe)
            {
                set_last_init_error("ina219 not detected on 0x40-0x4F (no I2C ACK)");
                return false;
            }

            set_last_init_error_with_detail("ina219.init() failed", context.ina219.last_error());
            return false;
        }

        if (!context.voltage_divider_1.init())
        {
            set_last_init_error("voltage_divider_1.init() failed");
            return false;
        }
        log_init_checkpoint("Voltage divider 1 initialized");


        context.soil_sensor.init();
        log_init_checkpoint("Soil moisture sensor initialized");

        // Inicializace SD karty
        if (!context.gpio_sd_cs.init()) {
            set_last_init_error("gpio_sd_cs.init() failed (pin 22)");
            return false;
        }
        log_init_checkpoint("GPIO SD CS initialized");

        if (!context.sd_card.init()) {
            set_last_init_error("sd_card.init() failed");
            return false;
        }
        log_init_checkpoint("SD card initialized");


        if (!context.sx1278.init())
        {
            log_init_checkpoint("SX1278 init failed! Přecházím do testovacího režimu.");
            set_last_init_error("sx1278.init() failed");
        }
        else
        {
            log_init_checkpoint("SX1278 initialized");
        }

        if (!context.gpio_status_led.init())
        {
            set_last_init_error("gpio_status_led.init() failed (pin 25)");
            return false;
        }

        if (!context.gpio_status_led.write(true))
        {
            set_last_init_error("gpio_status_led.write() failed (pin 25)");
            return false;
        }
        log_init_checkpoint("Status LED initialized");

        set_last_init_error("none");
        log_init_checkpoint("Initialization finished successfully");

        return true;
    }

    const char* last_init_error()
    {
        return g_last_init_error;
    }
}

app::AppContext* g_app_context_ptr = nullptr;
