/**
 * @file test_gpio.cpp
 * @brief Unit tests for the GPIO module.
 */

#include "hal/gpio/gpio.hpp"

namespace test
{
    /**
     * @brief Fake GPIO backend for testing.
     */
    class FakeGPIOBackend final : public hal::IGPIOBackend
    {
    public:
        FakeGPIOBackend()
            : init_called(false),
              set_dir_called(false),
              set_pull_called(false),
              write_called(false),
              toggle_called(false),
              last_pin(0U),
              last_is_output(false),
              last_pull_up(false),
              last_pull_down(false),
              last_write_value(false),
              read_value(false)
        {
        }

        void init(uint8_t pin) override
        {
            init_called = true;
            last_pin = pin;
        }

        void set_dir(uint8_t pin, bool is_output) override
        {
            set_dir_called = true;
            last_pin = pin;
            last_is_output = is_output;
        }

        void set_pull(uint8_t pin, bool pull_up, bool pull_down) override
        {
            set_pull_called = true;
            last_pin = pin;
            last_pull_up = pull_up;
            last_pull_down = pull_down;
        }

        void write(uint8_t pin, bool value) override
        {
            write_called = true;
            last_pin = pin;
            last_write_value = value;
        }

        bool read(uint8_t pin) override
        {
            last_pin = pin;
            return read_value;
        }

        void toggle(uint8_t pin) override
        {
            toggle_called = true;
            last_pin = pin;
        }

        bool init_called;
        bool set_dir_called;
        bool set_pull_called;
        bool write_called;
        bool toggle_called;
        uint8_t last_pin;
        bool last_is_output;
        bool last_pull_up;
        bool last_pull_down;
        bool last_write_value;
        bool read_value;
    };

    /**
     * @brief Basic GPIO driver test.
     * @return true if all checks pass
     */
    bool test_gpio_basic()
    {
        FakeGPIOBackend backend;

        const hal::GPIO::Config config = {
            2U,
            true,
            false,
            false,
            false
        };

        hal::GPIO gpio(config, backend);
        bool ok = gpio.init();
        if (!ok)
        {
            return false;
        }

        if (!backend.init_called || !backend.set_dir_called || !backend.set_pull_called)
        {
            return false;
        }

        ok = gpio.write(true);
        if (!ok || !backend.write_called || backend.last_write_value != true)
        {
            return false;
        }

        ok = gpio.toggle();
        if (!ok || !backend.toggle_called)
        {
            return false;
        }

        bool level = false;
        backend.read_value = true;
        ok = gpio.read(level);
        if (!ok || level != true)
        {
            return false;
        }

        return true;
    }
}
