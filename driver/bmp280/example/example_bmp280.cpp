/**
 * @file example_bmp280.cpp
 * @brief Example usage of the BMP280 temperature and pressure sensor driver.
 */

#include "drivers/bmp280/bmp280.hpp"
#include "drivers/interfaces/ISensor.hpp"
#include "hal/i2c/i2c.hpp"
#include "pico/stdlib.h"

/**
 * @brief Helper function to process sensor via interface.
 * @param sensor Reference to generic typed sensor interface
 */
void process_environmental_sensor(drivers::ITypedSensor<drivers::BMP280Data>& sensor)
{
	if (!sensor.is_initialized())
	{
		return;
	}

	drivers::BMP280Data data;
	if (sensor.read(data) && data.valid)
	{
		// Process temperature and pressure
		int32_t temp_celsius = data.temperature / 100;
		uint32_t pressure_pa = data.pressure;
	}
}

/**
 * @brief Example demonstrating basic BMP280 usage.
 * 
 * This example shows how to:
 * 1. Initialize I2C communication
 * 2. Configure and initialize the BMP280 sensor
 * 3. Read temperature and pressure data
 * 4. Update and retrieve sensor state
 */
void example_bmp280_basic()
{
	// Create I2C backend
	hal::PicoI2CBackend i2c_backend;

	// Configure I2C
	hal::I2C::Config i2c_config = {
		.instance = i2c0,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76  // BMP280 default address (0x77 alternative)
	};

	// Create I2C instance
	hal::I2C i2c(i2c_config, i2c_backend);

	// Initialize I2C
	if (!i2c.init())
	{
		return;
	}

	// Configure BMP280
	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X2,
		.press_oversampling = drivers::BMP280::Oversampling::X16,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::X16,
		.standby = drivers::BMP280::Standby::MS_500
	};

	// Create BMP280 instance
	drivers::BMP280 bmp280(i2c, bmp_config);

	// Initialize sensor
	if (!bmp280.init())
	{
		return;
	}

	// Read sensor data directly
	drivers::BMP280::Data sensor_data;
	
	if (bmp280.read(sensor_data))
	{
		if (sensor_data.valid)
		{
			// Temperature in 0.01°C
			int32_t temp_celsius = sensor_data.temperature / 100;
			// Pressure in Pa
			uint32_t pressure_pa = sensor_data.pressure;
		}
	}

	// Update global state and retrieve data
	if (bmp280.update())
	{
		drivers::BMP280::Data current_data;
		if (bmp280.get_data(current_data))
		{
			// Process current data
		}
	}

	// Use sensor through interface
	process_environmental_sensor(bmp280);
}

/**
 * @brief Example demonstrating periodic sensor reading.
 */
void example_bmp280_periodic()
{
	hal::PicoI2CBackend i2c_backend;

	hal::I2C::Config i2c_config = {
		.instance = i2c0,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 100000,
		.address = 0x76
	};

	hal::I2C i2c(i2c_config, i2c_backend);

	if (!i2c.init())
	{
		return;
	}

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
		return;
	}

	// Periodic reading loop
	constexpr uint32_t MAX_ITERATIONS = 10;
	for (uint32_t i = 0; i < MAX_ITERATIONS; ++i)
	{
		drivers::BMP280::Data data;
		
		if (bmp280.read(data) && data.valid)
		{
			// Process sensor data
			int32_t temp = data.temperature;
			uint32_t press = data.pressure;
		}

		sleep_ms(1000);
	}
}

/**
 * @brief Example demonstrating high precision configuration.
 */
void example_bmp280_high_precision()
{
	hal::PicoI2CBackend i2c_backend;

	hal::I2C::Config i2c_config = {
		.instance = i2c1,
		.sda_pin = 14,
		.scl_pin = 15,
		.baudrate = 400000,  // Fast mode
		.address = 0x77      // Alternative I2C address
	};

	hal::I2C i2c(i2c_config, i2c_backend);

	if (!i2c.init())
	{
		return;
	}

	// High precision configuration
	drivers::BMP280::Config bmp_config = {
		.temp_oversampling = drivers::BMP280::Oversampling::X16,
		.press_oversampling = drivers::BMP280::Oversampling::X16,
		.mode = drivers::BMP280::Mode::NORMAL,
		.filter = drivers::BMP280::Filter::X16,
		.standby = drivers::BMP280::Standby::MS_125
	};

	drivers::BMP280 bmp280(i2c, bmp_config);

	if (!bmp280.init())
	{
		return;
	}

	// Check initialization status
	if (bmp280.is_initialized())
	{
		// Sensor is ready to use
		bmp280.update();
		
		drivers::BMP280::Data data;
		if (bmp280.get_data(data))
		{
			// High precision data available
		}
	}
}
