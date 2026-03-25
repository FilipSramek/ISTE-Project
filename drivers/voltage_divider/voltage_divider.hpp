/**
 * @file voltage_divider.hpp
 * @brief Voltage divider sensor driver using ADC backend.
 */

#pragma once

#include <stdint.h>
#include "drivers/interfaces/ISensor.hpp"
#include "hal/interfaces/IADCBackend.hpp"

namespace drivers
{
	/**
	 * @brief Voltage divider measurement data.
	 */
	struct VoltageDividerData
	{
		float input_voltage;  ///< Estimated voltage before divider in volts
		float adc_voltage;    ///< Voltage measured at ADC pin in volts
		uint16_t raw_adc;     ///< Raw ADC sample
		bool valid;           ///< Data validity flag
	};

	/**
	 * @brief Voltage sensor driver for resistor divider + ADC.
	 */
	class VoltageDivider final : public ITypedSensor<VoltageDividerData>
	{
	public:
		using Data = VoltageDividerData;

		/**
		 * @brief Driver configuration.
		 */
		struct Config
		{
			uint8_t channel;               ///< ADC channel used for measurement
			float adc_reference_voltage;   ///< ADC reference voltage in volts
			float r_top_ohm;               ///< Top resistor (Vin -> ADC pin) in ohms
			float r_bottom_ohm;            ///< Bottom resistor (ADC pin -> GND) in ohms
			float max_input_voltage;       ///< Maximum allowed measured input voltage
			uint16_t adc_max_count;        ///< Maximum ADC count (for 12-bit use 4095)
			float clock_div;               ///< Optional ADC clock divider (0.0f to keep backend default)
		};

		/**
		 * @brief Construct voltage divider driver.
		 * @param backend ADC backend abstraction
		 * @param config Driver configuration
		 */
		VoltageDivider(hal::IADCBackend& backend, const Config& config);

		bool init() override;
		bool is_initialized() const override;
		bool read(Data& data) override;
		bool update() override;
		bool get_data(Data& data) const override;

	private:
		bool is_valid_config_() const;
		float compute_adc_voltage_(uint16_t raw_adc) const;
		float compute_input_voltage_(float adc_voltage) const;
		float saturate_input_voltage_(float input_voltage) const;

		hal::IADCBackend& backend_;
		Config config_;
		Data current_data_;
		bool initialized_;
	};
} // namespace drivers
