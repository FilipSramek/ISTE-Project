/**
 * @file uart.cpp
 * @brief UART HAL implementation.
 */

#include "hal/uart/uart.hpp"

#include "hardware/gpio.h"
#include "utils/assert/assert.hpp"

namespace hal
{
    void PicoUARTBackend::init(uart_inst_t* instance, uint32_t baudrate)
    {
        ASSERT(instance != nullptr);
        ASSERT(baudrate != 0U);

        if ((instance == nullptr) || (baudrate == 0U))
        {
            return;
        }

        (void)uart_init(instance, static_cast<uint32_t>(baudrate));
    }

    void PicoUARTBackend::set_pins(uint8_t tx_pin, uint8_t rx_pin)
    {
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
        gpio_set_function(rx_pin, GPIO_FUNC_UART);
    }

    void PicoUARTBackend::set_format(uart_inst_t* instance,
                                     uint8_t data_bits,
                                     uint8_t stop_bits,
                                     uart_parity_t parity)
    {
        ASSERT(instance != nullptr);
        ASSERT((data_bits >= 5U) && (data_bits <= 8U));
        ASSERT((stop_bits >= 1U) && (stop_bits <= 2U));

        if (instance == nullptr)
        {
            return;
        }

        uart_set_format(instance, data_bits, stop_bits, parity);
    }

    int PicoUARTBackend::write(uart_inst_t* instance,
                               const uint8_t* data,
                               uint32_t length)
    {
        ASSERT(instance != nullptr);
        ASSERT(data != nullptr);
        ASSERT(length != 0U);

        if ((instance == nullptr) || (data == nullptr) || (length == 0U))
        {
            return -1;
        }

        uart_write_blocking(instance, data, static_cast<size_t>(length));
        return static_cast<int>(length);
    }

    int PicoUARTBackend::read(uart_inst_t* instance,
                              uint8_t* buffer,
                              uint32_t length)
    {
        ASSERT(instance != nullptr);
        ASSERT(buffer != nullptr);
        ASSERT(length != 0U);

        if ((instance == nullptr) || (buffer == nullptr) || (length == 0U))
        {
            return -1;
        }

        uint32_t read_count = 0U;
        for (uint32_t i = 0U; i < length; ++i)
        {
            if (uart_is_readable(instance))
            {
                buffer[i] = uart_getc(instance);
                read_count++;
            }
            else
            {
                break;
            }
        }

        return static_cast<int>(read_count);
    }

    bool PicoUARTBackend::is_readable(uart_inst_t* instance)
    {
        ASSERT(instance != nullptr);

        if (instance == nullptr)
        {
            return false;
        }

        return uart_is_readable(instance);
    }

    bool PicoUARTBackend::is_writable(uart_inst_t* instance)
    {
        ASSERT(instance != nullptr);

        if (instance == nullptr)
        {
            return false;
        }

        return uart_is_writable(instance);
    }

    UART::UART(const Config& config, IUARTBackend& backend)
        : config_(config),
          backend_(backend),
          initialized_(false)
    {
    }

    bool UART::init()
    {
        ASSERT(config_.instance != nullptr);
        ASSERT(config_.baudrate != 0U);
        ASSERT(config_.tx_pin != config_.rx_pin);
        ASSERT((config_.data_bits >= 5U) && (config_.data_bits <= 8U));
        ASSERT((config_.stop_bits >= 1U) && (config_.stop_bits <= 2U));

        if (!is_valid_config_())
        {
            return false;
        }

        backend_.init(config_.instance, config_.baudrate);
        backend_.set_pins(config_.tx_pin, config_.rx_pin);
        backend_.set_format(config_.instance,
                            config_.data_bits,
                            config_.stop_bits,
                            config_.parity);
        initialized_ = true;
        return true;
    }

    bool UART::write(const uint8_t* data, uint32_t length)
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

    bool UART::receive(uint8_t* buffer, uint32_t& length)
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
        return (length > 0U);
    }

    bool UART::is_valid_config_() const
    {
        if (config_.instance == nullptr)
        {
            return false;
        }

        if (config_.baudrate == 0U)
        {
            return false;
        }

        if (config_.tx_pin == config_.rx_pin)
        {
            return false;
        }

        if ((config_.data_bits < 5U) || (config_.data_bits > 8U))
        {
            return false;
        }

        if ((config_.stop_bits < 1U) || (config_.stop_bits > 2U))
        {
            return false;
        }

        return true;
    }
} // namespace hal