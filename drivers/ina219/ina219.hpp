/**
 * @file ina219.hpp
 * @brief INA219 current and voltage sensor driver.
 */

#pragma once

#include <stdint.h>
#include "hal/i2c/i2c.hpp"
#include "drivers/interfaces/ISensor.hpp"

namespace drivers
{
	/**
	 * @brief INA219 sensor data structure.
	 */
	struct INA219Data
	{
		float bus_voltage;    ///< Bus voltage in volts
		float shunt_voltage;  ///< Shunt voltage in volts
		float current;        ///< Current in amperes
		float power;          ///< Power in watts
		bool valid;           ///< Data validity flag
	};

	/**
	 * @brief INA219 current and voltage sensor driver.
	 */
	class INA219 final : public ITypedSensor<INA219Data>
	{
	public:
		/**
		 * @brief Sensor measurement data type alias.
		 */
		using Data = INA219Data;

		/**
		 * @brief Bus voltage range.
		 */
		enum class BusRange : uint8_t
		{
			RANGE_16V = 0x00,
			RANGE_32V = 0x01
		};

		/**
		 * @brief PGA gain and shunt voltage range.
		 */
		enum class PGAGain : uint8_t
		{
			GAIN_1_40MV  = 0x00,
			GAIN_2_80MV  = 0x01,
			GAIN_4_160MV = 0x02,
			GAIN_8_320MV = 0x03
		};

		/**
		 * @brief ADC resolution/averaging settings.
		 */
		enum class ADCResolution : uint8_t
		{
			BIT_9  = 0x00,
			BIT_10 = 0x01,
			BIT_11 = 0x02,
			BIT_12 = 0x03
		};

		/**
		 * @brief Operating mode.
		 */
		enum class Mode : uint8_t
		{
			POWER_DOWN           = 0x00,
			SHUNT_TRIGGERED      = 0x01,
			BUS_TRIGGERED        = 0x02,
			SHUNT_BUS_TRIGGERED  = 0x03,
			ADC_OFF              = 0x04,
			SHUNT_CONTINUOUS     = 0x05,
			BUS_CONTINUOUS       = 0x06,
			SHUNT_BUS_CONTINUOUS = 0x07
		};

		/**
		 * @brief Configuration structure.
		 */
		struct Config
		{
			BusRange bus_range;           ///< Bus voltage range
			PGAGain pga_gain;             ///< Shunt voltage gain
			ADCResolution bus_adc;        ///< Bus ADC resolution
			ADCResolution shunt_adc;      ///< Shunt ADC resolution
			Mode mode;                    ///< Operating mode
			float shunt_resistance;       ///< Shunt resistor value in ohms
			float max_expected_current;   ///< Max expected current in amperes
		};

		/**
		 * @brief Construct INA219 driver.
		 * @param i2c Reference to I2C communication interface
		 * @param config Sensor configuration
		 */
		INA219(hal::I2C& i2c, const Config& config);

		/**
		 * @brief Initialize the INA219 sensor.
		 * @return true if initialization succeeded
		 */
		bool init() override;

		/**
		 * @brief Check if sensor is initialized.
		 * @return true if initialized
		 */
		bool is_initialized() const override;

		/**
		 * @brief Get the last driver error message.
		 * @return Pointer to internal null-terminated text
		 */
		const char* last_error() const;

		/**
		 * @brief Read sensor data directly.
		 * @param data Reference to data structure to fill
		 * @return true if read succeeded
		 */
		bool read(Data& data) override;

		/**
		 * @brief Update global state with new measurement.
		 * @return true if update succeeded
		 */
		bool update() override;

		/**
		 * @brief Get the latest sensor data.
		 * @param data Reference to data structure to fill
		 * @return true if data is valid
		 */
		bool get_data(Data& data) const override;

	private:
		/**
		 * @brief INA219 register addresses.
		 */
		enum class Register : uint8_t
		{
			CONFIG       = 0x00,
			SHUNT_VOLT   = 0x01,
			BUS_VOLT     = 0x02,
			POWER        = 0x03,
			CURRENT      = 0x04,
			CALIBRATION  = 0x05
		};

		/**
		 * @brief Write a register.
		 * @param reg Register address
		 * @param value Value to write
		 * @return true on success
		 */
		bool write_register_(Register reg, uint16_t value);

		/**
		 * @brief Read a register.
		 * @param reg Register address
		 * @param value Reference to store read value
		 * @return true on success
		 */
		bool read_register_(Register reg, uint16_t& value);

		/**
		 * @brief Configure sensor settings.
		 * @return true on success
		 */
		bool configure_sensor_();

		/**
		 * @brief Compute calibration values.
		 * @return true on success
		 */
		bool compute_calibration_();

		/**
		 * @brief Convert raw shunt voltage to volts.
		 * @param raw Raw shunt voltage register value
		 * @return Shunt voltage in volts
		 */
		float convert_shunt_voltage_(int16_t raw) const;

		/**
		 * @brief Convert raw bus voltage to volts.
		 * @param raw Raw bus voltage register value
		 * @return Bus voltage in volts
		 */
		float convert_bus_voltage_(uint16_t raw) const;

		/**
		 * @brief Convert raw current to amperes.
		 * @param raw Raw current register value
		 * @return Current in amperes
		 */
		float convert_current_(int16_t raw) const;

		/**
		 * @brief Convert raw power to watts.
		 * @param raw Raw power register value
		 * @return Power in watts
		 */
		float convert_power_(uint16_t raw) const;

		/**
		 * @brief Save description of the last error.
		 * @param message Null-terminated text
		 */
		void set_last_error_(const char* message);

		hal::I2C& i2c_;
		Config config_;
		Data current_data_;
		char last_error_[96];
		bool initialized_;

		uint16_t calibration_value_;
		float current_lsb_;
		float power_lsb_;

		static constexpr float CALIBRATION_FACTOR = 0.04096f;
	};
} // namespace drivers
