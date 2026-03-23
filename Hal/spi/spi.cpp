/**
 * @file spi.cpp
 * @brief SPI HAL implementation.
 */

#include "hal/spi/spi.hpp"

#include "hardware/gpio.h"
#include "utils/assert/assert.hpp"


namespace hal
{
    void PicoSPIBackend::init(spi_inst_t* instance, uint32_t baudrate)
    {
        ASSERT(instance != nullptr);
        ASSERT(baudrate != 0U);

        if ((instance == nullptr) || (baudrate == 0U))
        {
            return;
        }

        (void)spi_init(instance, baudrate);
    }

    void PicoSPIBackend::set_pins(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin)
    {
        ASSERT(clk_pin != mosi_pin);
        ASSERT(clk_pin != miso_pin);
        ASSERT(clk_pin != cs_pin);
        ASSERT(mosi_pin != miso_pin);
        ASSERT(mosi_pin != cs_pin);
        ASSERT(miso_pin != cs_pin);

        gpio_set_function(clk_pin, GPIO_FUNC_SPI);
        gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(miso_pin, GPIO_FUNC_SPI);
        gpio_set_function(cs_pin, GPIO_FUNC_SIO);
        gpio_set_dir(cs_pin, GPIO_OUT);
        gpio_put(cs_pin, 1);
    }

    int PicoSPIBackend::write(spi_inst_t* instance,
                               const uint8_t* data,
                               uint32_t length)
    {
        ASSERT(instance != nullptr);
        ASSERT(data != nullptr);
        ASSERT(length != 0U);

        return spi_write_blocking(instance, data, length);
    }
    
    int PicoSPIBackend::read(spi_inst_t* instance,
                                uint8_t* buffer,
                                uint32_t length)
    {
        ASSERT(instance != nullptr);
        ASSERT(buffer != nullptr);
        ASSERT(length != 0U);

        return spi_read_blocking(instance, 0xFFU, buffer, length);
    }

    int PicoSPIBackend::transfer(spi_inst_t* instance,
                                 const uint8_t* tx_data,
                                 uint8_t* rx_buffer,
                                 uint32_t length)
    {
        ASSERT(instance != nullptr);
        ASSERT(tx_data != nullptr);
        ASSERT(rx_buffer != nullptr);
        ASSERT(length != 0U);

        return spi_write_read_blocking(instance, tx_data, rx_buffer, length);
    }

    SPI::SPI(const Config& config, ISPIBackend& backend)
        : config_(config),
          backend_(backend),
          initialized_(false)
    {
    }

    bool SPI::init()
    {
        ASSERT(config_.instance != nullptr);
        ASSERT(config_.baudrate != 0U);
        ASSERT(config_.clk_pin != config_.mosi_pin);
        ASSERT(config_.clk_pin != config_.miso_pin);

        if (!is_valid_config_())
        {
            return false;
        }

        backend_.set_pins(config_.clk_pin, config_.mosi_pin, config_.miso_pin, config_.cs_pin);
        backend_.init(config_.instance, config_.baudrate);
        initialized_ = true;
        return true;
    }

    bool SPI::write(const uint8_t* data, uint32_t length)
    {
        ASSERT(initialized_);
        ASSERT(data != nullptr);
        ASSERT(length != 0U);

        if ((!initialized_) || (data == nullptr) || (length == 0U))
        {
            return false;
        }

        const int result = backend_.write(config_.instance, data, length);
        if (result < 0)
        {
            return false;
        }

        return (static_cast<uint32_t>(result) == length);
    }

    bool SPI::receive(uint8_t* buffer, uint32_t& length)
    {
        ASSERT(initialized_);
        ASSERT(buffer != nullptr);
        ASSERT(length != 0U);

        if ((!initialized_) || (buffer == nullptr) || (length == 0U))
        {
            length = 0U;
            return false;
        }

        const uint32_t requested = length;
        const int result = backend_.read(config_.instance, buffer, requested);
        if (result < 0)
        {
            length = 0U;
            return false;
        }

        length = static_cast<uint32_t>(result);
        return (length == requested);
    }

    bool SPI::transfer(const uint8_t* tx_data, uint8_t* rx_buffer, uint32_t& length)
    {
        ASSERT(initialized_);
        ASSERT(tx_data != nullptr);
        ASSERT(rx_buffer != nullptr);
        ASSERT(length != 0U);

        if ((!initialized_) || (tx_data == nullptr) || (rx_buffer == nullptr) || (length == 0U))
        {
            length = 0U;
            return false;
        }

        const uint32_t requested = length;
        const int result = backend_.transfer(config_.instance, tx_data, rx_buffer, requested);
        if (result < 0)
        {
            length = 0U;
            return false;
        }

        length = static_cast<uint32_t>(result);
        return (length == requested);
    }

    bool SPI::is_valid_config_() const
    {
        if (config_.instance == nullptr)
        {
            return false;
        }

        if (config_.baudrate == 0U)
        {
            return false;
        }

        if (config_.clk_pin == config_.mosi_pin)
        {
            return false;
        }

        if (config_.clk_pin == config_.miso_pin)
        {
            return false;
        }

        if (config_.clk_pin == config_.cs_pin)
        {
            return false;
        }

        if (config_.mosi_pin == config_.miso_pin)
        {
            return false;
        }

        if (config_.mosi_pin == config_.cs_pin)
        {
            return false;
        }

        if (config_.miso_pin == config_.cs_pin)
        {
            return false;
        }

        return true;
    }
} // namespace hal