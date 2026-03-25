/**
 * @file example_voltage_divider.cpp
 * @brief Example usage for voltage divider ADC driver.
 */

#include "drivers/voltage_divider/voltage_divider.hpp"
#include "drivers/interfaces/ISensor.hpp"
#include "hal/adc/adc.hpp"
#include "pico/stdlib.h"
#include <stdint.h>

void process_voltage_sensor(drivers::ITypedSensor<drivers::VoltageDividerData>& sensor)
{
	if (!sensor.is_initialized())
	{
		return;
	}

	drivers::VoltageDividerData data;
	if (sensor.read(data) && data.valid)
	{
		const float vin = data.input_voltage;
		const float v_adc = data.adc_voltage;
		const uint16_t raw = data.raw_adc;

		(void)vin;
		(void)v_adc;
		(void)raw;
	}
}

void example_voltage_divider_basic()
{
	hal::PicoADCBackend adc_backend;

	drivers::VoltageDivider::Config config = {
		0U,
		3.3f,
		30000.0f,
		10000.0f,
		16.0f,
		4095U,
		0.0f
	};

	drivers::VoltageDivider sensor(adc_backend, config);

	if (!sensor.init())
	{
		return;
	}

	drivers::VoltageDividerData data;
	if (sensor.read(data) && data.valid)
	{
		const float vin = data.input_voltage;
		const float v_adc = data.adc_voltage;
		const uint16_t raw = data.raw_adc;

		(void)vin;
		(void)v_adc;
		(void)raw;
	}

	process_voltage_sensor(sensor);
}

void example_voltage_divider_periodic()
{
	hal::PicoADCBackend adc_backend;

	drivers::VoltageDivider::Config config = {
		0U,
		3.3f,
		30000.0f,
		10000.0f,
		16.0f,
		4095U,
		0.0f
	};

	drivers::VoltageDivider sensor(adc_backend, config);
	if (!sensor.init())
	{
		return;
	}

	constexpr uint32_t MAX_ITERATIONS = 10U;
	for (uint32_t i = 0U; i < MAX_ITERATIONS; ++i)
	{
		drivers::VoltageDividerData data;
		if (sensor.read(data) && data.valid)
		{
			const float vin = data.input_voltage;
			const float v_adc = data.adc_voltage;
			const uint16_t raw = data.raw_adc;

			(void)vin;
			(void)v_adc;
			(void)raw;
		}

		sleep_ms(1000);
	}
}

void example_voltage_divider_two_sensors()
{
	hal::PicoADCBackend adc_backend;

	drivers::VoltageDivider::Config battery_config = {
		0U,
		3.3f,
		30000.0f,
		10000.0f,
		16.0f,
		4095U,
		0.0f
	};

	drivers::VoltageDivider::Config pyro_bus_config = {
		1U,
		3.3f,
		47000.0f,
		10000.0f,
		20.0f,
		4095U,
		0.0f
	};

	drivers::VoltageDivider battery_sensor(adc_backend, battery_config);
	drivers::VoltageDivider pyro_bus_sensor(adc_backend, pyro_bus_config);

	if (!battery_sensor.init())
	{
		return;
	}

	if (!pyro_bus_sensor.init())
	{
		return;
	}

	constexpr uint32_t MAX_ITERATIONS = 10U;
	for (uint32_t i = 0U; i < MAX_ITERATIONS; ++i)
	{
		drivers::VoltageDividerData battery_data;
		drivers::VoltageDividerData pyro_data;

		const bool battery_ok = battery_sensor.read(battery_data);
		const bool pyro_ok = pyro_bus_sensor.read(pyro_data);

		if (battery_ok && battery_data.valid && pyro_ok && pyro_data.valid)
		{
			const float battery_v = battery_data.input_voltage;
			const float pyro_v = pyro_data.input_voltage;

			(void)battery_v;
			(void)pyro_v;
		}

		sleep_ms(20);
	}
}
