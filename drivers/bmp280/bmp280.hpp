/**
 * @file bmp280.hpp
 * @brief BMP280 temperature and pressure sensor driver.
 */

#pragma once

#include <stdint.h>
#include "hal/i2c/i2c.hpp"
#include "drivers/interfaces/ISensor.hpp"

namespace drivers
{
	// Forward declaration for BMP280 data type
	struct BMP280Data
	{
		int32_t temperature;  ///< Temperature in 0.01 degrees Celsius
		uint32_t pressure;    ///< Pressure in Pa
		bool valid;           ///< Data validity flag
	};

	/**
	 * @brief BMP280 sensor driver class.
	 */
	class BMP280 final : public ITypedSensor<BMP280Data>
	{
	public:
		/**
		 * @brief Sensor measurement data structure.
		 */
		using Data = BMP280Data;

		/**
		 * @brief Oversampling settings.
		 */
		enum class Oversampling : uint8_t
		{
			SKIP = 0x00,
			X1   = 0x01,
			X2   = 0x02,
			X4   = 0x03,
			X8   = 0x04,
			X16  = 0x05
		};

		/**
		 * @brief Operating mode.
		 */
		enum class Mode : uint8_t
		{
			SLEEP  = 0x00,
			FORCED = 0x01,
			NORMAL = 0x03
		};

		/**
		 * @brief IIR filter coefficient.
		 */
		enum class Filter : uint8_t
		{
			OFF = 0x00,
			X2  = 0x01,
			X4  = 0x02,
			X8  = 0x03,
			X16 = 0x04
		};

		/**
		 * @brief Standby time in normal mode.
		 */
		enum class Standby : uint8_t
		{
			MS_0_5  = 0x00,
			MS_62_5 = 0x01,
			MS_125  = 0x02,
			MS_250  = 0x03,
			MS_500  = 0x04,
			MS_1000 = 0x05,
			MS_2000 = 0x06,
			MS_4000 = 0x07
		};

		/**
		 * @brief Configuration structure.
		 */
		struct Config
		{
			Oversampling temp_oversampling;  ///< Temperature oversampling
			Oversampling press_oversampling; ///< Pressure oversampling
			Mode mode;                        ///< Operating mode
			Filter filter;                    ///< IIR filter setting
			Standby standby;                  ///< Standby time
		};

		/**
		 * @brief Construct BMP280 driver.
		 * @param i2c Reference to I2C communication interface
		 * @param config Sensor configuration
		 */
		BMP280(hal::I2C& i2c, const Config& config);

		/**
		 * @brief Initialize the BMP280 sensor.
		 * @return true if initialization succeeded
		 */
		bool init() override;

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

		/**
		 * @brief Check if sensor is initialized.
		 * @return true if initialized
		 */
		bool is_initialized() const override;

		/**
		 * @brief Get the last initialization/read error message.
		 * @return Pointer to internal null-terminated string
		 */
		const char* last_error() const;

	private:
		/**
		 * @brief BMP280 register addresses.
		 */
		enum class Register : uint8_t
		{
			ID             = 0xD0,
			RESET          = 0xE0,
			STATUS         = 0xF3,
			CTRL_MEAS      = 0xF4,
			CONFIG         = 0xF5,
			PRESS_MSB      = 0xF7,
			PRESS_LSB      = 0xF8,
			PRESS_XLSB     = 0xF9,
			TEMP_MSB       = 0xFA,
			TEMP_LSB       = 0xFB,
			TEMP_XLSB      = 0xFC,
			CALIB_START    = 0x88,
			CALIB_END      = 0xA1
		};

		/**
		 * @brief Calibration data structure.
		 */
		struct CalibData
		{
			uint16_t dig_T1;
			int16_t dig_T2;
			int16_t dig_T3;
			uint16_t dig_P1;
			int16_t dig_P2;
			int16_t dig_P3;
			int16_t dig_P4;
			int16_t dig_P5;
			int16_t dig_P6;
			int16_t dig_P7;
			int16_t dig_P8;
			int16_t dig_P9;
		};

		/**
		 * @brief Write a register.
		 * @param reg Register address
		 * @param value Value to write
		 * @return true on success
		 */
		bool write_register_(Register reg, uint8_t value);

		/**
		 * @brief Read a register.
		 * @param reg Register address
		 * @param value Reference to store read value
		 * @return true on success
		 */
		bool read_register_(Register reg, uint8_t& value);

		/**
		 * @brief Read multiple registers.
		 * @param reg Starting register address
		 * @param buffer Buffer to store read data
		 * @param length Number of bytes to read
		 * @return true on success
		 */
		bool read_registers_(Register reg, uint8_t* buffer, uint32_t length);

		/**
		 * @brief Read calibration data from sensor.
		 * @return true on success
		 */
		bool read_calibration_();

		/**
		 * @brief Read raw sensor data.
		 * @param raw_temp Reference to store raw temperature
		 * @param raw_press Reference to store raw pressure
		 * @return true on success
		 */
		bool read_raw_(int32_t& raw_temp, int32_t& raw_press);

		/**
		 * @brief Compensate temperature.
		 * @param raw_temp Raw temperature value
		 * @param t_fine Reference to store fine temperature
		 * @return Compensated temperature in 0.01 degrees Celsius
		 */
		int32_t compensate_temperature_(int32_t raw_temp, int32_t& t_fine);

		/**
		 * @brief Compensate pressure.
		 * @param raw_press Raw pressure value
		 * @param t_fine Fine temperature value
		 * @return Compensated pressure in Pa
		 */
		uint32_t compensate_pressure_(int32_t raw_press, int32_t t_fine);

		/**
		 * @brief Verify sensor chip ID.
		 * @return true if chip ID is correct
		 */
		bool verify_chip_id_();

		/**
		 * @brief Configure sensor settings.
		 * @return true on success
		 */
		bool configure_sensor_();

		/**
		 * @brief Store text describing the last driver error.
		 * @param message Null-terminated text message
		 */
		void set_last_error_(const char* message);

		hal::I2C& i2c_;
		Config config_;
		CalibData calib_;
		Data current_data_;
		char last_error_[96];
		bool initialized_;

		static constexpr uint8_t CHIP_ID = 0x58;
		static constexpr uint8_t RESET_VALUE = 0xB6;
		static constexpr uint32_t CALIB_DATA_SIZE = 24;
	};
} // namespace drivers
