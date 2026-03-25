/**
 * @file test_bmp280.cpp
 * @brief Unit tests for BMP280 temperature and pressure sensor driver.
 */

#include "drivers/bmp280/bmp280.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/interfaces/II2CBackend.hpp"
#include "pico/stdlib.h"

#include <cstdio>

/**
 * @brief Mock I2C backend for testing.
 */
class MockI2CBackend : public hal::II2CBackend
{
public:
	MockI2CBackend()
		: init_called_(false),
		  set_pins_called_(false),
		  write_count_(0),
		  read_count_(0),
		  simulate_chip_id_(true),
		  simulate_calib_data_(true),
		  simulate_sensor_data_(true)
	{
	}

	void init(i2c_inst_t* instance, uint32_t baudrate) override
	{
		init_called_ = true;
		instance_ = instance;
		baudrate_ = baudrate;
	}

	void set_pins(uint8_t sda_pin, uint8_t scl_pin) override
	{
		set_pins_called_ = true;
		sda_pin_ = sda_pin;
		scl_pin_ = scl_pin;
	}

	int write(i2c_inst_t* instance,
			  uint8_t address,
			  const uint8_t* data,
			  uint32_t length,
			  bool no_stop) override
	{
		write_count_++;
		last_write_addr_ = address;
		
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
		read_count_++;

		// Simulate chip ID response
		if (last_register_ == 0xD0 && simulate_chip_id_)
		{
			buffer[0] = 0x58;
			return 1;
		}

		// Simulate calibration data
		if (last_register_ == 0x88 && simulate_calib_data_)
		{
			for (uint32_t i = 0; i < length && i < 24; ++i)
			{
				buffer[i] = static_cast<uint8_t>(i);
			}
			return static_cast<int>(length);
		}

		// Simulate sensor data
		if (last_register_ == 0xF7 && simulate_sensor_data_)
		{
			// Pressure MSB, LSB, XLSB
			buffer[0] = 0x50;
			buffer[1] = 0x00;
			buffer[2] = 0x00;
			// Temperature MSB, LSB, XLSB
			buffer[3] = 0x80;
			buffer[4] = 0x00;
			buffer[5] = 0x00;
			return 6;
		}

		return static_cast<int>(length);
	}

	void set_simulate_chip_id(bool value) { simulate_chip_id_ = value; }
	void set_simulate_calib_data(bool value) { simulate_calib_data_ = value; }
	void set_simulate_sensor_data(bool value) { simulate_sensor_data_ = value; }

	bool init_called_;
	bool set_pins_called_;
	uint32_t write_count_;
	uint32_t read_count_;
	uint8_t last_write_addr_;
	uint8_t last_register_;
	i2c_inst_t* instance_;
	uint32_t baudrate_;
	uint8_t sda_pin_;
	uint8_t scl_pin_;

private:
	bool simulate_chip_id_;
	bool simulate_calib_data_;
	bool simulate_sensor_data_;
};

/**
 * @brief Test BMP280 initialization.
 */
bool test_bmp280_init()
{
	MockI2CBackend mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X2,
		.press_oversampling = drivers::BMP280::Oversampling::X16,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::X16,
		.standby = drivers::BMP280::Standby::MS_500
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	bool result = bmp280.init();

	return result && bmp280.is_initialized();
}

/**
 * @brief Test BMP280 chip ID verification failure.
 */
bool test_bmp280_invalid_chip_id()
{
	MockI2CBackend mock_backend;
	mock_backend.set_simulate_chip_id(false);

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X1,
		.press_oversampling = drivers::BMP280::Oversampling::X1,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::OFF,
		.standby = drivers::BMP280::Standby::MS_0_5
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	bool result = bmp280.init();

	return !result && !bmp280.is_initialized();
}

/**
 * @brief Test BMP280 data reading.
 */
bool test_bmp280_read_data()
{
	MockI2CBackend mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X2,
		.press_oversampling = drivers::BMP280::Oversampling::X16,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::X16,
		.standby = drivers::BMP280::Standby::MS_500
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	if (!bmp280.init())
	{
		return false;
	}

	drivers::BMP280::Data data;
	bool result = bmp280.read(data);

	return result && data.valid;
}

/**
 * @brief Test BMP280 update and get data.
 */
bool test_bmp280_update_and_get()
{
	MockI2CBackend mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X1,
		.press_oversampling = drivers::BMP280::Oversampling::X1,
		.mode = drivers::BMP280::Mode::FORCED,
		.filter = drivers::BMP280::Filter::OFF,
		.standby = drivers::BMP280::Standby::MS_0_5
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	if (!bmp280.init())
	{
		return false;
	}

	if (!bmp280.update())
	{
		return false;
	}

	drivers::BMP280::Data data;
	bool result = bmp280.get_data(data);

	return result && data.valid;
}

/**
 * @brief Test BMP280 read before initialization.
 */
bool test_bmp280_read_before_init()
{
	MockI2CBackend mock_backend;

	hal::I2C::Config i2c_config = {
		.instance = nullptr,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, mock_backend);
	i2c.init();

	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X1,
		.press_oversampling = drivers::BMP280::Oversampling::X1,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::OFF,
		.standby = drivers::BMP280::Standby::MS_0_5
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	drivers::BMP280::Data data;
	bool result = bmp280.read(data);

	return !result && !data.valid;
}

/**
 * @brief Run all BMP280 tests.
 */
int run_bmp280_tests()
{
	bool all_passed = true;

	all_passed = all_passed && test_bmp280_init();
	all_passed = all_passed && test_bmp280_invalid_chip_id();
	all_passed = all_passed && test_bmp280_read_data();
	all_passed = all_passed && test_bmp280_update_and_get();
	all_passed = all_passed && test_bmp280_read_before_init();

	return all_passed ? 0 : 1;
}

/**
 * @brief Main function for test executable.
 */
int main()
{
	stdio_init_all();
	sleep_ms(2000);

	const int result = run_bmp280_tests();
	printf("TEST BMP280: %s\n", (result == 0) ? "PASS" : "FAIL");
	fflush(stdout);
	sleep_ms(100);

	return result;
}
