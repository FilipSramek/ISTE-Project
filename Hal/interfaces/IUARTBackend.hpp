/**
 * @file IUARTBackend.hpp
 * @brief Backend interface for UART hardware access.
 */

#pragma ones

#include "pico/stdlib.h"
#include "hardware/uart.h"

namespace hal
{
    /**
     * @brief Abstract interface for UART backend implementation.
     */
    class IUARTBackend
    {
    public:
        virtual ~IUARTBackend() = default;

        /**
         * @brief Initialize UART peripheral.
         * @param instance UART instance pointer
         * @param baudrate Baud rate in bps
         */
        virtual void init(uart_inst_t* instance, uint32_t baudrate) = 0;

        /**
         * @brief Configure UART GPIO pins.
         * @param tx_pin TX pin number
         * @param rx_pin RX pin number
         */
        virtual void set_pins(uint8_t tx_pin, uint8_t rx_pin) = 0;

        /**
         * @brief Configure UART format.
         * @param instance UART instance pointer
         * @param data_bits Number of data bits
         * @param stop_bits Number of stop bits
         * @param parity Parity configuration
         */
        virtual void set_format(uart_inst_t* instance,
                                uint8_t data_bits,
                                uint8_t stop_bits,
                                uart_parity_t parity) = 0;

        /**
         * @brief Write data to UART.
         * @param instance UART instance pointer
         * @param data Data buffer
         * @param length Number of bytes to write
         * @return Number of bytes written or error code
         */
        virtual int write(uart_inst_t* instance,
                          const uint8_t* data,
                          uint32_t length) = 0;

        /**
         * @brief Read data from UART.
         * @param instance UART instance pointer
         * @param buffer Receive buffer
         * @param length Number of bytes to read
         * @return Number of bytes read or error code
         */
        virtual int read(uart_inst_t* instance,
                         uint8_t* buffer,
                         uint32_t length) = 0;

        /**
         * @brief Check if UART is readable.
         * @param instance UART instance pointer
         * @return true if data is available to read
         */
        virtual bool is_readable(uart_inst_t* instance) = 0;

        /**
         * @brief Check if UART is writable.
         * @param instance UART instance pointer
         * @return true if UART can accept more data
         */
        virtual bool is_writable(uart_inst_t* instance) = 0;
    };
}