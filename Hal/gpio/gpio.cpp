/**
 * @file gpio.cpp
 * @brief GPIO HAL implementation.
 */

#include "hal/gpio/gpio.hpp"

#include "utils/assert/assert.hpp"

namespace hal
{
    void PicoGPIOBackend::init(uint8_t pin)
    {
        ASSERT(pin <= 29U);

        gpio_init(pin);
    }

    void PicoGPIOBackend::set_dir(uint8_t pin, bool is_output)
    {
        ASSERT(pin <= 29U);

        gpio_set_dir(pin, is_output ? GPIO_OUT : GPIO_IN);
    }

    void PicoGPIOBackend::set_pull(uint8_t pin, bool pull_up, bool pull_down)
    {
        ASSERT(pin <= 29U);
        ASSERT(!(pull_up && pull_down));

        gpio_set_pulls(pin, pull_up, pull_down);
    }

    void PicoGPIOBackend::write(uint8_t pin, bool value)
    {
        ASSERT(pin <= 29U);

        gpio_put(pin, value ? 1 : 0);
    }

    bool PicoGPIOBackend::read(uint8_t pin)
    {
        ASSERT(pin <= 29U);

        return gpio_get(pin) != 0U;
    }

    void PicoGPIOBackend::toggle(uint8_t pin)
    {
        ASSERT(pin <= 29U);

        gpio_xor_mask(1u << pin);
    }

    GPIO::GPIO(const Config& config, IGPIOBackend& backend)
        : config_(config), backend_(backend), initialized_(false)
    {
    }

    bool GPIO::init()
    {
        ASSERT(config_.pin <= 29U);
        ASSERT(!(config_.pull_up && config_.pull_down));

        if (!is_valid_config_())
        {
            return false;
        }

        backend_.init(config_.pin);
        backend_.set_dir(config_.pin, config_.is_output);
        backend_.set_pull(config_.pin, config_.pull_up, config_.pull_down);

        if (config_.is_output)
        {
            backend_.write(config_.pin, config_.initial_high);
        }

        initialized_ = true;
        return true;
    }

    bool GPIO::write(bool value)
    {
        ASSERT(initialized_);

        if (!initialized_)
        {
            return false;
        }

        if (!config_.is_output)
        {
            return false;
        }

        backend_.write(config_.pin, value);
        return true;
    }

    bool GPIO::read(bool& value)
    {
        ASSERT(initialized_);

        if (!initialized_)
        {
            return false;
        }

        value = backend_.read(config_.pin);
        return true;
    }

    bool GPIO::toggle()
    {
        ASSERT(initialized_);

        if (!initialized_)
        {
            return false;
        }

        if (!config_.is_output)
        {
            return false;
        }

        backend_.toggle(config_.pin);
        return true;
    }

    bool GPIO::is_valid_config_() const
    {
        if (config_.pin > 29U)
        {
            return false;
        }

        if (config_.pull_up && config_.pull_down)
        {
            return false;
        }

        return true;
    }
}
