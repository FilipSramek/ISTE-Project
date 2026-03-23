/**
 * @file example_uart.cpp
 * @brief Example usage of the UART module.
 */

#include "hal/uart/uart.hpp"

namespace example
{
    /**
     * @brief Example UART usage.
     */
    void example_uart()
    {
        hal::PicoUARTBackend backend;

        const hal::UART::Config config = {
            uart0,
            0U,
            1U,
            115200U,
            8U,
            1U,
            UART_PARITY_NONE
        };

        hal::UART uart(config, backend);
        bool ok = uart.init();
        if (!ok)
        {
            return;
        }

        const uint8_t tx_data[13] = "Hello World!";
        ok = uart.write(tx_data, 12U);
        if (!ok)
        {
            return;
        }

        uint8_t rx_data[16] = { 0U };
        uint32_t rx_length = 16U;
        ok = uart.receive(rx_data, rx_length);
        if (!ok)
        {
            return;
        }
    }
} // namespace example