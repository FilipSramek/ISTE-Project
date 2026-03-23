/**
 * @file test_uart.cpp
 * @brief Unit tests for the UART module.
 */

#include "hal/uart/uart.hpp"

namespace test
{
    /**
     * @brief Fake UART backend for testing.
     */
    class FakeUARTBackend final : public hal::IUARTBackend
    {
    public:
        FakeUARTBackend()
            : init_called(false),
              set_pins_called(false),
              set_format_called(false),
              last_instance(nullptr),
              last_baudrate(0U),
              last_tx(0U),
              last_rx(0U),
              last_data_bits(0U),
              last_stop_bits(0U),
              last_parity(UART_PARITY_NONE),
              last_write_len(0U),
              last_read_len(0U),
              readable(true),
              writable(true)
        {
            stored_tx[0] = 0U;
            stored_tx[1] = 0U;
        }

        void init(uart_inst_t* instance, uint32_t baudrate) override
        {
            init_called = true;
            last_instance = instance;
            last_baudrate = baudrate;
        }

        void set_pins(uint8_t tx_pin, uint8_t rx_pin) override
        {
            set_pins_called = true;
            last_tx = tx_pin;
            last_rx = rx_pin;
        }

        void set_format(uart_inst_t* instance,
                        uint8_t data_bits,
                        uint8_t stop_bits,
                        uart_parity_t parity) override
        {
            (void)instance;
            set_format_called = true;
            last_data_bits = data_bits;
            last_stop_bits = stop_bits;
            last_parity = parity;
        }

        int write(uart_inst_t* instance,
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

        int read(uart_inst_t* instance,
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

        bool is_readable(uart_inst_t* instance) override
        {
            (void)instance;
            return readable;
        }

        bool is_writable(uart_inst_t* instance) override
        {
            (void)instance;
            return writable;
        }

        bool init_called;
        bool set_pins_called;
        bool set_format_called;
        uart_inst_t* last_instance;
        uint32_t last_baudrate;
        uint8_t last_tx;
        uint8_t last_rx;
        uint8_t last_data_bits;
        uint8_t last_stop_bits;
        uart_parity_t last_parity;
        uint32_t last_write_len;
        uint32_t last_read_len;
        bool readable;
        bool writable;
        uint8_t stored_tx[2];
    };

    /**
     * @brief Basic UART initialization and I/O test.
     * @return true if all checks pass
     */
    bool test_uart_basic()
    {
        FakeUARTBackend backend;

        const hal::UART::Config config = {
            reinterpret_cast<uart_inst_t*>(0x1),
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
            return false;
        }

        if ((!backend.init_called) || (!backend.set_pins_called) || (!backend.set_format_called))
        {
            return false;
        }

        const uint8_t tx_data[2] = { 0x30U, 0x40U };
        ok = uart.write(tx_data, 2U);
        if (!ok)
        {
            return false;
        }

        if (backend.last_write_len != 2U)
        {
            return false;
        }

        uint8_t rx_data[2] = { 0U, 0U };
        uint32_t rx_length = 2U;
        ok = uart.receive(rx_data, rx_length);
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
} // namespace test