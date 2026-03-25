/**
 * @file test_voltage_divider.cpp
 * @brief Unit tests for voltage divider ADC driver.
 */

#include "drivers/voltage_divider/voltage_divider.hpp"
#include "hal/interfaces/IADCBackend.hpp"
#include "pico/stdlib.h"

#include <cstdio>

class MockADCBackendVoltageDivider final : public hal::IADCBackend
{
public:
	MockADCBackendVoltageDivider()
		: init_channel_(0xFFU),
		  read_channel_(0xFFU),
		  clock_div_channel_(0xFFU),
		  clock_div_value_(0.0f),
		  sample_(0U)
	{
	}

	void init(uint8_t channel) override
	{
		init_channel_ = channel;
	}

	uint16_t read(uint8_t channel) override
	{
		read_channel_ = channel;
		return sample_;
	}

	void set_clock_div(uint8_t channel, float clock_div) override
	{
		clock_div_channel_ = channel;
		clock_div_value_ = clock_div;
	}

	uint8_t init_channel_;
	uint8_t read_channel_;
	uint8_t clock_div_channel_;
	float clock_div_value_;
	uint16_t sample_;
};

static bool almost_equal(float a, float b, float tolerance)
{
	float diff = a - b;
	if (diff < 0.0f)
	{
		diff = -diff;
	}

	return diff <= tolerance;
}

static drivers::VoltageDivider::Config make_config()
{
	drivers::VoltageDivider::Config config = {
		1U,
		3.3f,
		30000.0f,
		10000.0f,
		12.0f,
		4095U,
		0.0f
	};

	return config;
}

bool test_voltage_divider_init_success()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider sensor(backend, make_config());

	const bool ok = sensor.init();
	return ok && sensor.is_initialized() && (backend.init_channel_ == 1U);
}

bool test_voltage_divider_init_invalid_config()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider::Config config = make_config();
	config.r_bottom_ohm = 0.0f;
	drivers::VoltageDivider sensor(backend, config);

	const bool ok = sensor.init();
	return !ok && !sensor.is_initialized();
}

bool test_voltage_divider_read_before_init()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider sensor(backend, make_config());
	drivers::VoltageDividerData data = {0.0f, 0.0f, 0U, true};

	const bool ok = sensor.read(data);
	return !ok && !data.valid;
}

bool test_voltage_divider_read_nominal()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider sensor(backend, make_config());

	if (!sensor.init())
	{
		return false;
	}

	backend.sample_ = 2048U;
	drivers::VoltageDividerData data;

	if (!sensor.read(data))
	{
		return false;
	}

	const bool raw_ok = (data.raw_adc == 2048U) && (backend.read_channel_ == 1U);
	const bool adc_ok = almost_equal(data.adc_voltage, 1.6504f, 0.01f);
	const bool vin_ok = almost_equal(data.input_voltage, 6.6016f, 0.03f);

	return data.valid && raw_ok && adc_ok && vin_ok;
}

bool test_voltage_divider_saturation()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider::Config config = make_config();
	config.max_input_voltage = 9.0f;
	drivers::VoltageDivider sensor(backend, config);

	if (!sensor.init())
	{
		return false;
	}

	backend.sample_ = 4095U;
	drivers::VoltageDividerData data;
	if (!sensor.read(data))
	{
		return false;
	}

	return data.valid && almost_equal(data.input_voltage, 9.0f, 0.001f);
}

bool test_voltage_divider_update_and_get()
{
	MockADCBackendVoltageDivider backend;
	drivers::VoltageDivider sensor(backend, make_config());

	if (!sensor.init())
	{
		return false;
	}

	backend.sample_ = 1024U;
	if (!sensor.update())
	{
		return false;
	}

	drivers::VoltageDividerData data;
	const bool ok = sensor.get_data(data);

	return ok && data.valid && (data.raw_adc == 1024U);
}

int run_voltage_divider_tests()
{
	bool all_passed = true;

	all_passed = all_passed && test_voltage_divider_init_success();
	all_passed = all_passed && test_voltage_divider_init_invalid_config();
	all_passed = all_passed && test_voltage_divider_read_before_init();
	all_passed = all_passed && test_voltage_divider_read_nominal();
	all_passed = all_passed && test_voltage_divider_saturation();
	all_passed = all_passed && test_voltage_divider_update_and_get();

	return all_passed ? 0 : 1;
}

int main()
{
	stdio_init_all();
	sleep_ms(2000);

	const int result = run_voltage_divider_tests();
	printf("TEST VOLTAGE DIVIDER: %s\n", (result == 0) ? "PASS" : "FAIL");
	fflush(stdout);
	sleep_ms(100);

	return result;
}
