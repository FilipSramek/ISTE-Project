/**
 * @file ina219.cpp
 * @brief INA219 current and voltage sensor driver implementation.
 */

#include "drivers/ina219/ina219.hpp"
#include "utils/assert/assert.hpp"

namespace drivers
{
	INA219::INA219(hal::I2C& i2c, const Config& config)
		: i2c_(i2c),
		  config_(config),
		  current_data_{0.0f, 0.0f, 0.0f, 0.0f, false},
		  initialized_(false),
		  calibration_value_(0),
		  current_lsb_(0.0f),
		  power_lsb_(0.0f)
	{
	}

	bool INA219::init()
	{
		ASSERT(!initialized_);
		ASSERT(config_.shunt_resistance > 0.0f);
		ASSERT(config_.max_expected_current > 0.0f);

		if (initialized_)
		{
			return false;
		}

		if (!compute_calibration_())
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

	bool INA219::is_initialized() const
	{
		return initialized_;
	}

	bool INA219::read(Data& data)
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			data.valid = false;
			return false;
		}

		uint16_t raw_shunt = 0;
		uint16_t raw_bus = 0;
		uint16_t raw_power = 0;
		uint16_t raw_current = 0;

		if (!read_register_(Register::SHUNT_VOLT, raw_shunt))
		{
			data.valid = false;
			return false;
		}

		if (!read_register_(Register::BUS_VOLT, raw_bus))
		{
			data.valid = false;
			return false;
		}

		if (!read_register_(Register::POWER, raw_power))
		{
			data.valid = false;
			return false;
		}

		if (!read_register_(Register::CURRENT, raw_current))
		{
			data.valid = false;
			return false;
		}

		data.shunt_voltage = convert_shunt_voltage_(static_cast<int16_t>(raw_shunt));
		data.bus_voltage = convert_bus_voltage_(raw_bus);
		data.current = convert_current_(static_cast<int16_t>(raw_current));
		data.power = convert_power_(raw_power);
		data.valid = true;

		return true;
	}

	bool INA219::update()
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		return read(current_data_);
	}

	bool INA219::get_data(Data& data) const
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		data = current_data_;
		return current_data_.valid;
	}

	bool INA219::write_register_(Register reg, uint16_t value)
	{
		ASSERT(initialized_ || (reg == Register::CONFIG) || (reg == Register::CALIBRATION));

		uint8_t buffer[3] = {static_cast<uint8_t>(reg),
						 static_cast<uint8_t>((value >> 8) & 0xFFU),
						 static_cast<uint8_t>(value & 0xFFU)};

		return i2c_.write(buffer, 3);
	}

	bool INA219::read_register_(Register reg, uint16_t& value)
	{
		ASSERT(initialized_ || (reg == Register::BUS_VOLT) || (reg == Register::SHUNT_VOLT));

		uint8_t reg_addr = static_cast<uint8_t>(reg);

		if (!i2c_.write(&reg_addr, 1))
		{
			return false;
		}

		uint8_t buffer[2] = {0, 0};
		uint32_t length = 2;

		if (!i2c_.receive(buffer, length))
		{
			return false;
		}

		value = static_cast<uint16_t>((static_cast<uint16_t>(buffer[0]) << 8) |
										   static_cast<uint16_t>(buffer[1]));
		return true;
	}

	bool INA219::configure_sensor_()
	{
		ASSERT(static_cast<uint8_t>(config_.bus_range) <= 0x01U);
		ASSERT(static_cast<uint8_t>(config_.pga_gain) <= 0x03U);
		ASSERT(static_cast<uint8_t>(config_.bus_adc) <= 0x03U);
		ASSERT(static_cast<uint8_t>(config_.shunt_adc) <= 0x03U);
		ASSERT(static_cast<uint8_t>(config_.mode) <= 0x07U);

		uint16_t config_reg = 0;

		config_reg |= static_cast<uint16_t>(static_cast<uint8_t>(config_.bus_range) << 13);
		config_reg |= static_cast<uint16_t>(static_cast<uint8_t>(config_.pga_gain) << 11);
		config_reg |= static_cast<uint16_t>(static_cast<uint8_t>(config_.bus_adc) << 7);
		config_reg |= static_cast<uint16_t>(static_cast<uint8_t>(config_.shunt_adc) << 3);
		config_reg |= static_cast<uint16_t>(static_cast<uint8_t>(config_.mode));

		if (!write_register_(Register::CALIBRATION, calibration_value_))
		{
			return false;
		}

		return write_register_(Register::CONFIG, config_reg);
	}

	bool INA219::compute_calibration_()
	{
		ASSERT(config_.shunt_resistance > 0.0f);
		ASSERT(config_.max_expected_current > 0.0f);

		if ((config_.shunt_resistance <= 0.0f) || (config_.max_expected_current <= 0.0f))
		{
			return false;
		}

		current_lsb_ = config_.max_expected_current / 32768.0f;
		power_lsb_ = current_lsb_ * 20.0f;

		float calibration = CALIBRATION_FACTOR / (current_lsb_ * config_.shunt_resistance);
		if (calibration < 1.0f)
		{
			return false;
		}

		calibration_value_ = static_cast<uint16_t>(calibration);
		return true;
	}

	float INA219::convert_shunt_voltage_(int16_t raw) const
	{
		constexpr float SHUNT_LSB = 0.00001f; // 10uV
		return static_cast<float>(raw) * SHUNT_LSB;
	}

	float INA219::convert_bus_voltage_(uint16_t raw) const
	{
		uint16_t value = static_cast<uint16_t>(raw >> 3);
		constexpr float BUS_LSB = 0.004f; // 4mV
		return static_cast<float>(value) * BUS_LSB;
	}

	float INA219::convert_current_(int16_t raw) const
	{
		return static_cast<float>(raw) * current_lsb_;
	}

	float INA219::convert_power_(uint16_t raw) const
	{
		return static_cast<float>(raw) * power_lsb_;
	}
} // namespace drivers
