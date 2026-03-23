/**
 * @file example_spi.cpp
 * @brief Example usage of the SPI module.
 */

#include "hal/spi/spi.hpp"

namespace example
{
    /**
     * @brief Example SPI usage.
     */
    void example_spi()
    {
        hal::PicoSPIBackend backend;

        const hal::SPI::Config config = {
            spi0,
            18U,
            19U,
            16U,
            17U,
            1000000U
        };

        hal::SPI spi(config, backend);
        bool ok = spi.init();
        if (!ok)
        {
            return;
        }

        const uint8_t tx_data[2] = { 0x80U, 0x00U };
        ok = spi.write(tx_data, 2U);
        if (!ok)
        {
            return;
        }

        uint8_t rx_data[2] = { 0U, 0U };
        uint32_t rx_length = 2U;
        ok = spi.receive(rx_data, rx_length);
        if (!ok)
        {
            return;
        }

        uint8_t tx_buffer[2] = { 0xA0U, 0x00U };
        uint8_t rx_buffer[2] = { 0U, 0U };
        uint32_t transfer_length = 2U;
        ok = spi.transfer(tx_buffer, rx_buffer, transfer_length);
        if (!ok)
        {
            return;
        }
    }
} // namespace example