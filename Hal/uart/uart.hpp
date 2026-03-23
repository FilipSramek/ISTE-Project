/**
 * @file uart.hpp
 * @brief UART HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/ICominication.hpp"
#include "hal/interfaces/IUARTBackend.hpp"
#include "hardware/uart.h"

namespace hal
{
    /**
     * @brief Pico-specific UART backend.
     */
    class PicoUARTBackend final : public IUARTBackend
    {
    public:
        /**
         * @brief Initialize UART peripheral.
         * @param instance UART instance pointer
         * @param baudrate Baud rate in bps
         */
        void init(uart_inst_t* instance, uint32_t baudrate) override;

        /**
         * @brief Configure UART GPIO pins.
         * @param tx_pin TX pin number
         * @param rx_pin RX pin number
         */
        void set_pins(uint8_t tx_pin, uint8_t rx_pin) override;

        /**
         * @brief Configure UART format.
         * @param instance UART instance pointer
         * @param data_bits Number of data bits
         * @param stop_bits Number of stop bits
         * @param parity Parity configuration
         */
        void set_format(uart_inst_t* instance,
                        uint8_t data_bits,
                        uint8_t stop_bits,
                        uart_parity_t parity) override;

        /**
         * @brief Write data to UART.
         * @param instance UART instance pointer
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return Number of bytes written or error code
         */
        int write(uart_inst_t* instance,
                  const uint8_t* data,
                  uint32_t length) override;

        /**
         * @brief Read data from UART.
         * @param instance UART instance pointer
         * @param buffer Receive buffer
         * @param length Number of bytes to read
         * @return Number of bytes read or error code
         */
        int read(uart_inst_t* instance,
                 uint8_t* buffer,
                 uint32_t length) override;

        /**
         * @brief Check if UART is readable.
         * @param instance UART instance pointer
         * @return true if data is available to read
         */
        bool is_readable(uart_inst_t* instance) override;

        /**
         * @brief Check if UART is writable.
         * @param instance UART instance pointer
         * @return true if UART can accept more data
         */
        bool is_writable(uart_inst_t* instance) override;
    };

    /**
     * @brief UART communication driver.
     */
    class UART final : public ICommunication
    {
    public:
        /**
         * @brief UART configuration structure.
         */
        struct Config
        {
            uart_inst_t* instance; ///< UART instance pointer
            uint8_t tx_pin;        ///< TX pin number
            uint8_t rx_pin;        ///< RX pin number
            uint32_t baudrate;     ///< Baud rate in bps
            uint8_t data_bits;     ///< Number of data bits
            uint8_t stop_bits;     ///< Number of stop bits
            uart_parity_t parity;  ///< Parity configuration
        };

        /**
         * @brief Construct UART driver with configuration and backend.
         * @param config UART configuration
         * @param backend Reference to backend implementation
         */
        UART(const Config& config, IUARTBackend& backend);

        /**
         * @brief Initialize the UART driver.
         * @return true if initialization succeeded
         */
        bool init();

        /**
         * @brief Write data to UART.
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return true on success
         */
        bool write(const uint8_t* data, uint32_t length) override;

        /**
         * @brief Receive data from UART.
         * @param buffer Receive buffer
         * @param length In: buffer size, Out: bytes received
         * @return true on success
         */
        bool receive(uint8_t* buffer, uint32_t& length) override;

    private:
        /**
         * @brief Validate the current configuration.
         * @return true if configuration is valid
         */
        bool is_valid_config_() const;

        Config config_;
        IUARTBackend& backend_;
        bool initialized_;
    };
} // namespace hal