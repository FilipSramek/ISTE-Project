/**
 * @file voltage_divider.cpp
 * @brief Voltage divider sensor driver implementation.
 */

#include "drivers/voltage_divider/voltage_divider.hpp"
#include "utils/assert/assert.hpp"

namespace drivers
{
	VoltageDivider::VoltageDivider(hal::IADCBackend& backend, const Config& config)
		: backend_(backend),
		  config_(config),
		  current_data_{0.0f, 0.0f, 0U, false},
		  initialized_(false)
	{
	}

	bool VoltageDivider::init()
	{
		ASSERT(!initialized_);
		ASSERT(config_.adc_max_count > 0U);

		if (initialized_)
		{
			return false;
		}

		if (!is_valid_config_())
		{
			return false;
		}

		backend_.init(config_.channel);

		if (config_.clock_div > 0.0f)
		{
			backend_.set_clock_div(config_.channel, config_.clock_div);
		}

		initialized_ = true;
		current_data_.valid = false;
		return true;
	}

	bool VoltageDivider::is_initialized() const
	{
		return initialized_;
	}

	bool VoltageDivider::read(Data& data)
	{
		ASSERT(initialized_);
		ASSERT(config_.adc_max_count > 0U);

		if (!initialized_)
		{
			data.input_voltage = 0.0f;
			data.adc_voltage = 0.0f;
			data.raw_adc = 0U;
			data.valid = false;
			return false;
		}

		uint16_t raw_adc = backend_.read(config_.channel);
		if (raw_adc > config_.adc_max_count)
		{
			raw_adc = config_.adc_max_count;
		}

		data.raw_adc = raw_adc;
		data.adc_voltage = compute_adc_voltage_(raw_adc);

		const float computed_input_voltage = compute_input_voltage_(data.adc_voltage);
		data.input_voltage = saturate_input_voltage_(computed_input_voltage);
		data.valid = true;
		return true;
	}

	bool VoltageDivider::update()
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		return read(current_data_);
	}

	bool VoltageDivider::get_data(Data& data) const
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		data = current_data_;
		return current_data_.valid;
	}

	bool VoltageDivider::is_valid_config_() const
	{
		ASSERT(config_.r_bottom_ohm > 0.0f);

		if (config_.adc_reference_voltage <= 0.0f)
		{
			return false;
		}

		if (config_.r_top_ohm <= 0.0f)
		{
			return false;
		}

		if (config_.r_bottom_ohm <= 0.0f)
		{
			return false;
		}

		if (config_.max_input_voltage <= 0.0f)
		{
			return false;
		}

		if (config_.adc_max_count == 0U)
		{
			return false;
		}

		if (config_.clock_div < 0.0f)
		{
			return false;
		}

		return true;
	}

	float VoltageDivider::compute_adc_voltage_(uint16_t raw_adc) const
	{
		ASSERT(config_.adc_max_count > 0U);

		const float raw_f = static_cast<float>(raw_adc);
		const float max_f = static_cast<float>(config_.adc_max_count);
		return (raw_f / max_f) * config_.adc_reference_voltage;
	}

	float VoltageDivider::compute_input_voltage_(float adc_voltage) const
	{
		ASSERT(config_.r_bottom_ohm > 0.0f);
		ASSERT(adc_voltage >= 0.0f);

		const float divider_ratio = (config_.r_top_ohm + config_.r_bottom_ohm) / config_.r_bottom_ohm;
		return adc_voltage * divider_ratio;
	}

	float VoltageDivider::saturate_input_voltage_(float input_voltage) const
	{
		ASSERT(config_.max_input_voltage > 0.0f);

		if (input_voltage < 0.0f)
		{
			return 0.0f;
		}

		if (input_voltage > config_.max_input_voltage)
		{
			return config_.max_input_voltage;
		}

		return input_voltage;
	}
} // namespace drivers
