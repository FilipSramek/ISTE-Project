/**
 * @file example_ina219.cpp
 * @brief Example usage of the INA219 current and voltage sensor driver.
 */

#include "drivers/ina219/ina219.hpp"
#include "drivers/interfaces/ISensor.hpp"
#include "hal/i2c/i2c.hpp"
#include "pico/stdlib.h"

/**
 * @brief Helper function to process sensor via interface.
 * @param sensor Reference to generic typed sensor interface
 */
void process_power_sensor(drivers::ITypedSensor<drivers::INA219Data>& sensor)
{
	if (!sensor.is_initialized())
	{
		return;
	}

	drivers::INA219Data data;
	if (sensor.read(data) && data.valid)
	{
		float bus_v = data.bus_voltage;
		float shunt_v = data.shunt_voltage;
		float current_a = data.current;
		float power_w = data.power;

		(void)bus_v;
		(void)shunt_v;
		(void)current_a;
		(void)power_w;
	}
}

/**
 * @brief Example demonstrating basic INA219 usage.
 */
void example_ina219_basic()
{
	hal::PicoI2CBackend i2c_backend;

	hal::I2C::Config i2c_config = {
		.instance = i2c0,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, i2c_backend);

	if (!i2c.init())
	{
		return;
	}

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
		return;
	}

	drivers::INA219Data data;
	if (ina219.read(data))
	{
		if (data.valid)
		{
			float bus_v = data.bus_voltage;
			float shunt_v = data.shunt_voltage;
			float current_a = data.current;
			float power_w = data.power;

			(void)bus_v;
			(void)shunt_v;
			(void)current_a;
			(void)power_w;
		}
	}

	if (ina219.update())
	{
		drivers::INA219Data current_data;
		if (ina219.get_data(current_data))
		{
			// Process current data
		}
	}

	process_power_sensor(ina219);
}

/**
 * @brief Example demonstrating periodic sensor reading.
 */
void example_ina219_periodic()
{
	hal::PicoI2CBackend i2c_backend;

	hal::I2C::Config i2c_config = {
		.instance = i2c0,
		.sda_pin = 4,
		.scl_pin = 5,
		.baudrate = 400000,
		.address = 0x40
	};

	hal::I2C i2c(i2c_config, i2c_backend);

	if (!i2c.init())
	{
		return;
	}

	drivers::INA219::Config ina_config = {
		.bus_range = drivers::INA219::BusRange::RANGE_16V,
		.pga_gain = drivers::INA219::PGAGain::GAIN_4_160MV,
		.bus_adc = drivers::INA219::ADCResolution::BIT_12,
		.shunt_adc = drivers::INA219::ADCResolution::BIT_12,
		.mode = drivers::INA219::Mode::SHUNT_BUS_CONTINUOUS,
		.shunt_resistance = 0.1f,
		.max_expected_current = 2.0f
	};

	drivers::INA219 ina219(i2c, ina_config);

	if (!ina219.init())
	{
		return;
	}

	constexpr uint32_t MAX_ITERATIONS = 10;
	for (uint32_t i = 0; i < MAX_ITERATIONS; ++i)
	{
		drivers::INA219Data data;
		if (ina219.read(data) && data.valid)
		{
			float bus_v = data.bus_voltage;
			float current_a = data.current;
			float power_w = data.power;

			(void)bus_v;
			(void)current_a;
			(void)power_w;
		}

		sleep_ms(1000);
	}
}
