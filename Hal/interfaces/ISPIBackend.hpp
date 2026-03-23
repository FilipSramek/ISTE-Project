/**
 * @file ISPIBackend.hpp
 * @brief Backend interface for SPI hardware access.
 */

#pragma ones

#include "pico/stdlib.h"
#include "hal/interfaces/ICominication.hpp"
#include "hardware/spi.h"

namespace hal
{
    /**
     * @brief Abstract interface for SPI backend implementation.
     */
    class ISPIBackend
    {
    public:
        virtual ~ISPIBackend() = default;

        /**
         * @brief Initialize SPI peripheral.
         * @param instance SPI instance pointer
         * @param baudrate Bus speed in Hz
         */
        virtual void init(spi_inst_t* instance, uint32_t baudrate) = 0;

        /**
         * @brief Configure SPI GPIO pins.
         * @param clk_pin Clock pin
         * @param mosi_pin MOSI pin
         * @param miso_pin MISO pin
         * @param cs_pin Chip-select pin
         */
        virtual void set_pins(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin) = 0;

        /**
         * @brief Write data over SPI.
         * @param instance SPI instance pointer
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return Number of bytes written or error code
         */
        virtual int write(spi_inst_t* instance,
                          const uint8_t* data,
                          uint32_t length) = 0;

        /**
         * @brief Read data over SPI.
         * @param instance SPI instance pointer
         * @param buffer Receive buffer
         * @param length Number of bytes to read
         * @return Number of bytes read or error code
         */
        virtual int read(spi_inst_t* instance,
                         uint8_t* buffer,
                         uint32_t length) = 0;

        /**
         * @brief Simultaneously write and read data over SPI.
         * @param instance SPI instance pointer
         * @param tx_data Transmit buffer
         * @param rx_buffer Receive buffer
         * @param length Number of bytes to transfer
         * @return Number of bytes transferred or error code
         */
        virtual int transfer(spi_inst_t* instance,
                             const uint8_t* tx_data,
                             uint8_t* rx_buffer,
                             uint32_t length) = 0;
    };
}