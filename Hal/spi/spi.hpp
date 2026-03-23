/**
 * @file spi.hpp
 * @brief SPI HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/ICominication.hpp"
#include "hal/interfaces/ISPIBackend.hpp"
#include "hardware/spi.h"

namespace hal
{
    /**
     * @brief Pico-specific SPI backend.
     */
    class PicoSPIBackend final : public ISPIBackend
    {
    public:
        /**
         * @brief Initialize SPI peripheral.
         * @param instance SPI instance pointer
         * @param baudrate Bus speed in Hz
         */
        void init(spi_inst_t* instance, uint32_t baudrate) override;

        /**
         * @brief Configure SPI GPIO pins.
         * @param clk_pin Clock pin
         * @param mosi_pin MOSI pin
         * @param miso_pin MISO pin
         * @param cs_pin Chip-select pin
         */
        void set_pins(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin) override;

        /**
         * @brief Write data over SPI.
         * @param instance SPI instance pointer
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return Number of bytes written or error code
         */
        int write(spi_inst_t* instance,
                  const uint8_t* data,
                  uint32_t length) override;

        /**
         * @brief Read data over SPI.
         * @param instance SPI instance pointer
         * @param buffer Receive buffer
         * @param length Number of bytes to read
         * @return Number of bytes read or error code
         */
        int read(spi_inst_t* instance,
                 uint8_t* buffer,
                 uint32_t length) override;

        /**
         * @brief Simultaneously write and read data over SPI.
         * @param instance SPI instance pointer
         * @param tx_data Transmit buffer
         * @param rx_buffer Receive buffer
         * @param length Number of bytes to transfer
         * @return Number of bytes transferred or error code
         */
        int transfer(spi_inst_t* instance,
                     const uint8_t* tx_data,
                     uint8_t* rx_buffer,
                     uint32_t length) override;
    };

    /**
     * @brief SPI communication driver.
     */
    class SPI final : public ICommunication
    {
    public:
        /**
         * @brief SPI configuration structure.
         */
        struct Config
        {
            spi_inst_t* instance; ///< SPI instance pointer
            uint8_t clk_pin;      ///< Clock pin
            uint8_t mosi_pin;     ///< MOSI pin
            uint8_t miso_pin;     ///< MISO pin
            uint8_t cs_pin;       ///< Chip-select pin
            uint32_t baudrate;    ///< Bus speed in Hz
        };

        /**
         * @brief Construct SPI driver with configuration and backend.
         * @param config SPI configuration
         * @param backend Reference to backend implementation
         */
        SPI(const Config& config, ISPIBackend& backend);

        /**
         * @brief Initialize the SPI driver.
         * @return true if initialization succeeded
         */
        bool init();

        /**
         * @brief Write data over SPI.
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return true on success
         */
        bool write(const uint8_t* data, uint32_t length) override;

        /**
         * @brief Receive data over SPI.
         * @param buffer Receive buffer
         * @param length In: buffer size, Out: bytes received
         * @return true on success
         */
        bool receive(uint8_t* buffer, uint32_t& length) override;

        /**
         * @brief Transfer data over SPI.
         * @param tx_data Transmit buffer
         * @param rx_buffer Receive buffer
         * @param length In: number of bytes to transfer, Out: bytes transferred
         * @return true on success
         */
        bool transfer(const uint8_t* tx_data, uint8_t* rx_buffer, uint32_t& length);

    private:
        /**
         * @brief Validate the current configuration.
         * @return true if configuration is valid
         */
        bool is_valid_config_() const;

        Config config_;
        ISPIBackend& backend_;
        bool initialized_;
    };
} // namespace hal