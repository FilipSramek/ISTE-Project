/**
 * @file example_i2c.cpp
 * @brief Example usage of the I2C module.
 */

#include "hal/i2c/i2c.hpp"

namespace example
{
	/**
	 * @brief Example I2C usage.
	 */
	void example_i2c()
	{
		hal::PicoI2CBackend backend;

		const hal::I2C::Config config = {
			i2c0,
			4U,
			5U,
			100000U,
			0x50U
		};

		hal::I2C i2c(config, backend);
		bool ok = i2c.init();
		if (!ok)
		{
			return;
		}

		const uint8_t tx_data[2] = { 0x00U, 0xABU };
		ok = i2c.write(tx_data, 2U);
		if (!ok)
		{
			return;
		}

		uint8_t rx_data[2] = { 0U, 0U };
		uint32_t rx_length = 2U;
		ok = i2c.receive(rx_data, rx_length);
		if (!ok)
		{
			return;
		}
	}
} // namespace example
