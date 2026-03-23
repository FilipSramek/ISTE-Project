/**
 * @file test_spi.cpp
 * @brief Unit tests for the SPI module.
 */

#include "hal/spi/spi.hpp"

namespace test
{
    /**
     * @brief Fake SPI backend for testing.
     */
    class FakeSPIBackend final : public hal::ISPIBackend
    {
    public:
        FakeSPIBackend()
            : init_called(false),
              set_pins_called(false),
              last_instance(nullptr),
              last_baudrate(0U),
              last_clk(0U),
              last_mosi(0U),
              last_miso(0U),
              last_cs(0U),
              last_write_len(0U),
              last_read_len(0U),
              last_transfer_len(0U)
        {
            stored_tx[0] = 0U;
            stored_tx[1] = 0U;
        }

        void init(spi_inst_t* instance, uint32_t baudrate) override
        {
            init_called = true;
            last_instance = instance;
            last_baudrate = baudrate;
        }

        void set_pins(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin) override
        {
            set_pins_called = true;
            last_clk = clk_pin;
            last_mosi = mosi_pin;
            last_miso = miso_pin;
            last_cs = cs_pin;
        }

        int write(spi_inst_t* instance,
                  const uint8_t* data,
                  uint32_t length) override
        {
            (void)instance;

            last_write_len = length;

            if ((data == nullptr) || (length == 0U))
            {
                return -1;
            }

            if (length >= 2U)
            {
                stored_tx[0] = data[0];
                stored_tx[1] = data[1];
            }
            else
            {
                stored_tx[0] = data[0];
                stored_tx[1] = 0U;
            }

            return static_cast<int>(length);
        }

        int read(spi_inst_t* instance,
                 uint8_t* buffer,
                 uint32_t length) override
        {
            (void)instance;

            last_read_len = length;

            if ((buffer == nullptr) || (length == 0U))
            {
                return -1;
            }

            if (length >= 2U)
            {
                buffer[0] = 0xCCU;
                buffer[1] = 0xDDU;
            }
            else
            {
                buffer[0] = 0xCCU;
            }

            return static_cast<int>(length);
        }

        int transfer(spi_inst_t* instance,
                     const uint8_t* tx_data,
                     uint8_t* rx_buffer,
                     uint32_t length) override
        {
            (void)instance;

            last_transfer_len = length;

            if ((tx_data == nullptr) || (rx_buffer == nullptr) || (length == 0U))
            {
                return -1;
            }

            if (length >= 2U)
            {
                stored_tx[0] = tx_data[0];
                stored_tx[1] = tx_data[1];
                rx_buffer[0] = 0xEEU;
                rx_buffer[1] = 0xFFU;
            }
            else
            {
                stored_tx[0] = tx_data[0];
                rx_buffer[0] = 0xEEU;
            }

            return static_cast<int>(length);
        }

        bool init_called;
        bool set_pins_called;
        spi_inst_t* last_instance;
        uint32_t last_baudrate;
        uint8_t last_clk;
        uint8_t last_mosi;
        uint8_t last_miso;
        uint8_t last_cs;
        uint32_t last_write_len;
        uint32_t last_read_len;
        uint32_t last_transfer_len;
        uint8_t stored_tx[2];
    };

    /**
     * @brief Basic SPI initialization and write test.
     * @return true if all checks pass
     */
    bool test_spi_basic()
    {
        FakeSPIBackend backend;

        const hal::SPI::Config config = {
            reinterpret_cast<spi_inst_t*>(0x1),
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
            return false;
        }

        if ((!backend.init_called) || (!backend.set_pins_called))
        {
            return false;
        }

        const uint8_t tx_data[2] = { 0x80U, 0x01U };
        ok = spi.write(tx_data, 2U);
        if (!ok)
        {
            return false;
        }

        if (backend.last_write_len != 2U)
        {
            return false;
        }

        return true;
    }

    /**
     * @brief SPI read test.
     * @return true if all checks pass
     */
    bool test_spi_read()
    {
        FakeSPIBackend backend;

        const hal::SPI::Config config = {
            reinterpret_cast<spi_inst_t*>(0x1),
            18U,
            19U,
            16U,
            17U,
            1000000U
        };

        hal::SPI spi(config, backend);
        spi.init();

        uint8_t rx_data[2] = { 0U, 0U };
        uint32_t rx_length = 2U;
        bool ok = spi.receive(rx_data, rx_length);
        if (!ok)
        {
            return false;
        }

        if (rx_length != 2U)
        {
            return false;
        }

        if ((rx_data[0] != 0xCCU) || (rx_data[1] != 0xDDU))
        {
            return false;
        }

        return true;
    }

    /**
     * @brief SPI transfer test.
     * @return true if all checks pass
     */
    bool test_spi_transfer()
    {
        FakeSPIBackend backend;

        const hal::SPI::Config config = {
            reinterpret_cast<spi_inst_t*>(0x1),
            18U,
            19U,
            16U,
            17U,
            1000000U
        };

        hal::SPI spi(config, backend);
        spi.init();

        uint8_t tx_buffer[2] = { 0xA0U, 0xB0U };
        uint8_t rx_buffer[2] = { 0U, 0U };
        uint32_t transfer_length = 2U;
        bool ok = spi.transfer(tx_buffer, rx_buffer, transfer_length);
        if (!ok)
        {
            return false;
        }

        if (transfer_length != 2U)
        {
            return false;
        }

        if ((rx_buffer[0] != 0xEEU) || (rx_buffer[1] != 0xFFU))
        {
            return false;
        }

        return true;
    }

    /**
     * @brief SPI invalid configuration test.
     * @return true if all checks pass
     */
    bool test_spi_invalid_config()
    {
        FakeSPIBackend backend;

        const hal::SPI::Config config = {
            nullptr,
            18U,
            19U,
            16U,
            17U,
            1000000U
        };

        hal::SPI spi(config, backend);
        bool ok = spi.init();

        return (!ok);
    }

    /**
     * @brief SPI same-pins validation test.
     * @return true if all checks pass
     */
    bool test_spi_same_pins()
    {
        FakeSPIBackend backend;

        const hal::SPI::Config config = {
            reinterpret_cast<spi_inst_t*>(0x1),
            18U,
            18U,
            16U,
            17U,
            1000000U
        };

        hal::SPI spi(config, backend);
        bool ok = spi.init();

        return (!ok);
    }
} // namespace test