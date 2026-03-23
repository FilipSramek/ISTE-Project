/**
 * @file test_ina219.cpp
 * @brief Unit tests for INA219 current and voltage sensor driver.
 */

#include "drivers/ina219/ina219.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/interfaces/II2CBackend.hpp"
#include "pico/stdlib.h"

#include <cstdio>

/**
 * @brief Mock I2C backend for testing.
 */
class MockI2CBackendINA219 : public hal::II2CBackend
{
public:
	MockI2CBackendINA219()
		: write_count_(0),
		  read_count_(0),
		  last_register_(0)
	{
	}

	void init(i2c_inst_t* instance, uint32_t baudrate) override
	{
		(void)instance;
		(void)baudrate;
	}

	void set_pins(uint8_t sda_pin, uint8_t scl_pin) override
	{
		(void)sda_pin;
		(void)scl_pin;
	}

	int write(i2c_inst_t* instance,
			  uint8_t address,
			  const uint8_t* data,
			  uint32_t length,
			  bool no_stop) override
	{
		(void)instance;
		(void)address;
		(void)no_stop;

		write_count_++;
		if (length > 0)
		{
			last_register_ = data[0];
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
		(void)address;
		(void)no_stop;

		read_count_++;

		if (length >= 2)
		{
			// Provide deterministic raw data per register
			if (last_register_ == 0x01)
			{
				buffer[0] = 0x00;
				buffer[1] = 0x64; // 100 * 10uV = 0.001V
			}
			else if (last_register_ == 0x02)
			{
				buffer[0] = 0x1A;
				buffer[1] = 0xC0; // bus voltage value
			}
			else if (last_register_ == 0x03)
			{
				buffer[0] = 0x00;
				buffer[1] = 0x10; // power raw
			}
			else if (last_register_ == 0x04)
			{
				buffer[0] = 0x00;
				buffer[1] = 0x20; // current raw
			}
			else
			{
				buffer[0] = 0x00;
				buffer[1] = 0x00;
			}
		}

		return static_cast<int>(length);
	}

	uint32_t write_count_;
	uint32_t read_count_;
	uint8_t last_register_;
};

/**
 * @brief Test INA219 initialization.
 */
bool test_ina219_init()
{
	MockI2CBackendINA219 mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::INA219::Config ina_config = {
		.bus_range = drivers::INA219::BusRange::RANGE_32V,
		.pga_gain = drivers::INA219::PGAGain::GAIN_8_320MV,
		.bus_adc = drivers::INA219::ADCResolution::BIT_12,
		.shunt_adc = drivers::INA219::ADCResolution::BIT_12,
		.mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
		.shunt_resistance = 0.1f,
		.max_expected_current = 3.2f
	};

	drivers::INA219 ina219(i2c, ina_config);

	bool result = ina219.init();

	return result && ina219.is_initialized();
}

/**
 * @brief Test INA219 data reading.
 */
bool test_ina219_read_data()
{
	MockI2CBackendINA219 mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::INA219::Config ina_config = {
		.bus_range = drivers::INA219::BusRange::RANGE_32V,
		.pga_gain = drivers::INA219::PGAGain::GAIN_8_320MV,
		.bus_adc = drivers::INA219::ADCResolution::BIT_12,
		.shunt_adc = drivers::INA219::ADCResolution::BIT_12,
		.mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
		.shunt_resistance = 0.1f,
		.max_expected_current = 3.2f
	};

	drivers::INA219 ina219(i2c, ina_config);

	if (!ina219.init())
	{
		return false;
	}

	drivers::INA219Data data;
	bool result = ina219.read(data);

	return result && data.valid;
}

/**
 * @brief Test INA219 update and get data.
 */
bool test_ina219_update_and_get()
{
	MockI2CBackendINA219 mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::INA219::Config ina_config = {
		.bus_range = drivers::INA219::BusRange::RANGE_32V,
		.pga_gain = drivers::INA219::PGAGain::GAIN_8_320MV,
		.bus_adc = drivers::INA219::ADCResolution::BIT_12,
		.shunt_adc = drivers::INA219::ADCResolution::BIT_12,
		.mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
		.shunt_resistance = 0.1f,
		.max_expected_current = 3.2f
	};

	drivers::INA219 ina219(i2c, ina_config);

	if (!ina219.init())
	{
		return false;
	}

	if (!ina219.update())
	{
		return false;
	}

	drivers::INA219Data data;
	bool result = ina219.get_data(data);

	return result && data.valid;
}

/**
 * @brief Test INA219 read before initialization.
 */
bool test_ina219_read_before_init()
{
	MockI2CBackendINA219 mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::INA219::Config ina_config = {
		.bus_range = drivers::INA219::BusRange::RANGE_32V,
		.pga_gain = drivers::INA219::PGAGain::GAIN_8_320MV,
		.bus_adc = drivers::INA219::ADCResolution::BIT_12,
		.shunt_adc = drivers::INA219::ADCResolution::BIT_12,
		.mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
		.shunt_resistance = 0.1f,
		.max_expected_current = 3.2f
	};

	drivers::INA219 ina219(i2c, ina_config);

	drivers::INA219Data data;
	bool result = ina219.read(data);

	return !result && !data.valid;
}

/**
 * @brief Run all INA219 tests.
 */
int run_ina219_tests()
{
	bool all_passed = true;

	all_passed = all_passed && test_ina219_init();
	all_passed = all_passed && test_ina219_read_data();
	all_passed = all_passed && test_ina219_update_and_get();
	all_passed = all_passed && test_ina219_read_before_init();

	return all_passed ? 0 : 1;
}

/**
 * @brief Main function for test executable.
 */
int main()
{
	stdio_init_all();
	sleep_ms(2000);

	const int result = run_ina219_tests();
	printf("TEST INA219: %s\n", (result == 0) ? "PASS" : "FAIL");
	fflush(stdout);
	sleep_ms(100);

	return result;
}
