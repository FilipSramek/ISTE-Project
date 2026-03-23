/**
 * @file test_i2c.cpp
 * @brief Unit tests for the I2C module.
 */

#include "hal/i2c/i2c.hpp"

namespace test
{
	/**
	 * @brief Fake I2C backend for testing.
	 */
	class FakeI2CBackend final : public hal::II2CBackend
	{
	public:
		FakeI2CBackend()
			: init_called(false),
			  set_pins_called(false),
			  last_instance(nullptr),
			  last_baudrate(0U),
			  last_sda(0U),
			  last_scl(0U),
			  last_address(0U),
			  last_write_len(0U),
			  last_read_len(0U)
		{
			stored_tx[0] = 0U;
			stored_tx[1] = 0U;
		}

		void init(i2c_inst_t* instance, uint32_t baudrate) override
		{
			init_called = true;
			last_instance = instance;
			last_baudrate = baudrate;
		}

		void set_pins(uint8_t sda_pin, uint8_t scl_pin) override
		{
			set_pins_called = true;
			last_sda = sda_pin;
			last_scl = scl_pin;
		}

		int write(i2c_inst_t* instance,
				  uint8_t address,
				  const uint8_t* data,
				  uint32_t length,
				  bool no_stop) override
		{
			(void)instance;
			(void)no_stop;

			last_address = address;
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

		int read(i2c_inst_t* instance,
				 uint8_t address,
				 uint8_t* buffer,
				 uint32_t length,
				 bool no_stop) override
		{
			(void)instance;
			(void)no_stop;

			last_address = address;
			last_read_len = length;

			if ((buffer == nullptr) || (length == 0U))
			{
				return -1;
			}

			if (length >= 2U)
			{
				buffer[0] = 0xAAU;
				buffer[1] = 0xBBU;
			}
			else
			{
				buffer[0] = 0xAAU;
			}

			return static_cast<int>(length);
		}

		bool init_called;
		bool set_pins_called;
		i2c_inst_t* last_instance;
		uint32_t last_baudrate;
		uint8_t last_sda;
		uint8_t last_scl;
		uint8_t last_address;
		uint32_t last_write_len;
		uint32_t last_read_len;
		uint8_t stored_tx[2];
	};

	/**
	 * @brief Basic I2C driver test.
	 * @return true if all checks pass
	 */
	bool test_i2c_basic()
	{
		FakeI2CBackend backend;

		const hal::I2C::Config config = {
			reinterpret_cast<i2c_inst_t*>(0x1),
			4U,
			5U,
			100000U,
			0x50U
		};

		hal::I2C i2c(config, backend);

		bool ok = i2c.init();
		if (!ok)
		{
			return false;
		}

		if ((!backend.init_called) || (!backend.set_pins_called))
		{
			return false;
		}

		const uint8_t tx_data[2] = { 0x10U, 0x20U };
		ok = i2c.write(tx_data, 2U);
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
		ok = i2c.receive(rx_data, rx_length);
		if (!ok)
		{
			return false;
		}

		if (rx_length != 2U)
		{
			return false;
		}

		if ((rx_data[0] != 0xAAU) || (rx_data[1] != 0xBBU))
		{
			return false;
		}

		return true;
	}
} // namespace test

