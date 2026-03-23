/**
 * @file bmp280.cpp
 * @brief BMP280 temperature and pressure sensor driver implementation.
 */

#include "drivers/bmp280/bmp280.hpp"
#include "utils/assert/assert.hpp"

namespace drivers
{
	BMP280::BMP280(hal::I2C& i2c, const Config& config)
		: i2c_(i2c),
		  config_(config),
		  calib_{},
		  current_data_{0, 0, false},
		  initialized_(false)
	{
	}

	bool BMP280::init()
	{
		ASSERT(!initialized_);

		if (initialized_)
		{
			return false;
		}

		if (!verify_chip_id_())
		{
			return false;
		}

		if (!read_calibration_())
		{
			return false;
		}

		if (!configure_sensor_())
		{
			return false;
		}

		initialized_ = true;
		current_data_.valid = false;
		return true;
	}

	bool BMP280::read(Data& data)
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			data.valid = false;
			return false;
		}

		int32_t raw_temp = 0;
		int32_t raw_press = 0;

		if (!read_raw_(raw_temp, raw_press))
		{
			data.valid = false;
			return false;
		}

		int32_t t_fine = 0;
		data.temperature = compensate_temperature_(raw_temp, t_fine);
		data.pressure = compensate_pressure_(raw_press, t_fine);
		data.valid = true;

		return true;
	}

	bool BMP280::update()
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		return read(current_data_);
	}

	bool BMP280::get_data(Data& data) const
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		data = current_data_;
		return current_data_.valid;
	}

	bool BMP280::is_initialized() const
	{
		return initialized_;
	}

	bool BMP280::write_register_(Register reg, uint8_t value)
	{
		ASSERT(initialized_ || (reg == Register::RESET));

		uint8_t buffer[2] = {static_cast<uint8_t>(reg), value};
		return i2c_.write(buffer, 2);
	}

	bool BMP280::read_register_(Register reg, uint8_t& value)
	{
		ASSERT(initialized_ || (reg == Register::ID));

		uint8_t reg_addr = static_cast<uint8_t>(reg);
		
		if (!i2c_.write(&reg_addr, 1))
		{
			return false;
		}

		uint32_t length = 1;
		return i2c_.receive(&value, length);
	}

	bool BMP280::read_registers_(Register reg, uint8_t* buffer, uint32_t length)
	{
		ASSERT(buffer != nullptr);
		ASSERT(length != 0U);
		ASSERT(length <= CALIB_DATA_SIZE);

		if ((buffer == nullptr) || (length == 0U))
		{
			return false;
		}

		uint8_t reg_addr = static_cast<uint8_t>(reg);
		
		if (!i2c_.write(&reg_addr, 1))
		{
			return false;
		}

		return i2c_.receive(buffer, length);
	}

	bool BMP280::read_calibration_()
	{
		uint8_t calib_data[CALIB_DATA_SIZE] = {0};
		uint32_t calib_size = CALIB_DATA_SIZE;

		if (!read_registers_(Register::CALIB_START, calib_data, calib_size))
		{
			return false;
		}

		calib_.dig_T1 = static_cast<uint16_t>(calib_data[0] | (calib_data[1] << 8));
		calib_.dig_T2 = static_cast<int16_t>(calib_data[2] | (calib_data[3] << 8));
		calib_.dig_T3 = static_cast<int16_t>(calib_data[4] | (calib_data[5] << 8));
		
		calib_.dig_P1 = static_cast<uint16_t>(calib_data[6] | (calib_data[7] << 8));
		calib_.dig_P2 = static_cast<int16_t>(calib_data[8] | (calib_data[9] << 8));
		calib_.dig_P3 = static_cast<int16_t>(calib_data[10] | (calib_data[11] << 8));
		calib_.dig_P4 = static_cast<int16_t>(calib_data[12] | (calib_data[13] << 8));
		calib_.dig_P5 = static_cast<int16_t>(calib_data[14] | (calib_data[15] << 8));
		calib_.dig_P6 = static_cast<int16_t>(calib_data[16] | (calib_data[17] << 8));
		calib_.dig_P7 = static_cast<int16_t>(calib_data[18] | (calib_data[19] << 8));
		calib_.dig_P8 = static_cast<int16_t>(calib_data[20] | (calib_data[21] << 8));
		calib_.dig_P9 = static_cast<int16_t>(calib_data[22] | (calib_data[23] << 8));

		return true;
	}

	bool BMP280::read_raw_(int32_t& raw_temp, int32_t& raw_press)
	{
		uint8_t data[6] = {0};
		uint32_t data_length = 6;

		if (!read_registers_(Register::PRESS_MSB, data, data_length))
		{
			return false;
		}

		raw_press = static_cast<int32_t>(
			(static_cast<uint32_t>(data[0]) << 12) |
			(static_cast<uint32_t>(data[1]) << 4) |
			(static_cast<uint32_t>(data[2]) >> 4)
		);

		raw_temp = static_cast<int32_t>(
			(static_cast<uint32_t>(data[3]) << 12) |
			(static_cast<uint32_t>(data[4]) << 4) |
			(static_cast<uint32_t>(data[5]) >> 4)
		);

		return true;
	}

	int32_t BMP280::compensate_temperature_(int32_t raw_temp, int32_t& t_fine)
	{
		int32_t var1 = 0;
		int32_t var2 = 0;

		var1 = ((((raw_temp >> 3) - (static_cast<int32_t>(calib_.dig_T1) << 1))) *
				static_cast<int32_t>(calib_.dig_T2)) >> 11;

		var2 = (((((raw_temp >> 4) - static_cast<int32_t>(calib_.dig_T1)) *
				  ((raw_temp >> 4) - static_cast<int32_t>(calib_.dig_T1))) >> 12) *
				static_cast<int32_t>(calib_.dig_T3)) >> 14;

		t_fine = var1 + var2;
		
		int32_t temperature = (t_fine * 5 + 128) >> 8;
		return temperature;
	}

	uint32_t BMP280::compensate_pressure_(int32_t raw_press, int32_t t_fine)
	{
		int64_t var1 = 0;
		int64_t var2 = 0;
		int64_t pressure = 0;

		var1 = static_cast<int64_t>(t_fine) - 128000;
		var2 = var1 * var1 * static_cast<int64_t>(calib_.dig_P6);
		var2 = var2 + ((var1 * static_cast<int64_t>(calib_.dig_P5)) << 17);
		var2 = var2 + ((static_cast<int64_t>(calib_.dig_P4)) << 35);
		var1 = ((var1 * var1 * static_cast<int64_t>(calib_.dig_P3)) >> 8) +
			   ((var1 * static_cast<int64_t>(calib_.dig_P2)) << 12);
		var1 = ((((static_cast<int64_t>(1)) << 47) + var1)) *
			   (static_cast<int64_t>(calib_.dig_P1)) >> 33;

		if (var1 == 0)
		{
			return 0;
		}

		pressure = 1048576 - raw_press;
		pressure = (((pressure << 31) - var2) * 3125) / var1;
		var1 = ((static_cast<int64_t>(calib_.dig_P9)) * (pressure >> 13) *
				(pressure >> 13)) >> 25;
		var2 = ((static_cast<int64_t>(calib_.dig_P8)) * pressure) >> 19;

		pressure = ((pressure + var1 + var2) >> 8) +
				   ((static_cast<int64_t>(calib_.dig_P7)) << 4);

		return static_cast<uint32_t>(pressure / 256);
	}

	bool BMP280::verify_chip_id_()
	{
		uint8_t chip_id = 0;
		
		if (!read_register_(Register::ID, chip_id))
		{
			return false;
		}

		return (chip_id == CHIP_ID);
	}

	bool BMP280::configure_sensor_()
	{
		ASSERT(static_cast<uint8_t>(config_.temp_oversampling) <= 0x05U);
		ASSERT(static_cast<uint8_t>(config_.press_oversampling) <= 0x05U);
		ASSERT(static_cast<uint8_t>(config_.filter) <= 0x04U);
		ASSERT(static_cast<uint8_t>(config_.standby) <= 0x07U);

		uint8_t config_reg = static_cast<uint8_t>(
			(static_cast<uint8_t>(config_.standby) << 5) |
			(static_cast<uint8_t>(config_.filter) << 2)
		);

		if (!write_register_(Register::CONFIG, config_reg))
		{
			return false;
		}

		uint8_t ctrl_meas = static_cast<uint8_t>(
			(static_cast<uint8_t>(config_.temp_oversampling) << 5) |
			(static_cast<uint8_t>(config_.press_oversampling) << 2) |
			static_cast<uint8_t>(config_.mode)
		);

		if (!write_register_(Register::CTRL_MEAS, ctrl_meas))
		{
			return false;
		}

		return true;
	}
} // namespace drivers
