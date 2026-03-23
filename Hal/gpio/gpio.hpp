/**
 * @file gpio.hpp
 * @brief GPIO HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/IGPIOBackend.hpp"
#include "hardware/gpio.h"

namespace hal
{
    /**
     * @brief Pico-specific GPIO backend.
     */
    class PicoGPIOBackend final : public IGPIOBackend
    {
    public:
        void init(uint8_t pin) override;
        void set_dir(uint8_t pin, bool is_output) override;
        void set_pull(uint8_t pin, bool pull_up, bool pull_down) override;
        void write(uint8_t pin, bool value) override;
        bool read(uint8_t pin) override;
        void toggle(uint8_t pin) override;
    };

    /**
     * @brief GPIO driver with configuration management.
     */
    class GPIO final
    {
    public:
        /**
         * @brief GPIO configuration structure.
         */
        struct Config
        {
            uint8_t pin;       ///< GPIO pin number (0-29 for RP2040)
            bool is_output;    ///< Direction: true for output
            bool initial_high; ///< Initial output level if output
            bool pull_up;      ///< Enable pull-up
            bool pull_down;    ///< Enable pull-down
        };

        /**
         * @brief Construct GPIO driver with configuration and backend.
         * @param config GPIO configuration
         * @param backend Reference to backend implementation
         */
        GPIO(const Config& config, IGPIOBackend& backend);

        /**
         * @brief Initialize GPIO with configured settings.
         * @return true if initialization succeeded
         */
        bool init();

        /**
         * @brief Write output level.
         * @param value Output level
         * @return true if write succeeded
         */
        bool write(bool value);

        /**
         * @brief Read input level.
         * @param value Output parameter for pin level
         * @return true if read succeeded
         */
        bool read(bool& value);

        /**
         * @brief Toggle output level.
         * @return true if toggle succeeded
         */
        bool toggle();

    private:
        bool is_valid_config_() const;

        Config config_;
        IGPIOBackend& backend_;
        bool initialized_;
    };
}
