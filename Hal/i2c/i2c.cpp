/**
 * @file i2c.cpp
 * @brief I2C HAL implementation.
 */

#include "hal/i2c/i2c.hpp"

#include "hardware/gpio.h"
#include "utils/assert/assert.hpp"

namespace hal
{
	namespace
	{
		constexpr uint32_t I2C_TRANSFER_TIMEOUT_US = 5000U;
	}

	void PicoI2CBackend::init(i2c_inst_t* instance, uint32_t baudrate)
	{
		ASSERT(instance != nullptr);
		ASSERT(baudrate != 0U);

		if ((instance == nullptr) || (baudrate == 0U))
		{
			return;
		}

		(void)i2c_init(instance, static_cast<uint32_t>(baudrate));
	}

	void PicoI2CBackend::set_pins(uint8_t sda_pin, uint8_t scl_pin)
	{
		gpio_set_function(sda_pin, GPIO_FUNC_I2C);
		gpio_set_function(scl_pin, GPIO_FUNC_I2C);
		gpio_pull_up(sda_pin);
		gpio_pull_up(scl_pin);
	}

	int PicoI2CBackend::write(i2c_inst_t* instance,
							  uint8_t address,
							  const uint8_t* data,
							  uint32_t length,
							  bool no_stop)
	{
		ASSERT(instance != nullptr);
		ASSERT(data != nullptr);
		ASSERT(length != 0U);
		ASSERT(address < 0x80U);

		if ((instance == nullptr) || (data == nullptr) || (length == 0U) || (address >= 0x80U))
		{
			return PICO_ERROR_GENERIC;
		}

		return i2c_write_timeout_us(instance,
									address,
									data,
									static_cast<size_t>(length),
									no_stop,
									I2C_TRANSFER_TIMEOUT_US);
	}

	int PicoI2CBackend::read(i2c_inst_t* instance,
							 uint8_t address,
							 uint8_t* buffer,
							 uint32_t length,
							 bool no_stop)
	{
		ASSERT(instance != nullptr);
		ASSERT(buffer != nullptr);
		ASSERT(length != 0U);
		ASSERT(address < 0x80U);

		if ((instance == nullptr) || (buffer == nullptr) || (length == 0U) || (address >= 0x80U))
		{
			return PICO_ERROR_GENERIC;
		}

		return i2c_read_timeout_us(instance,
								   address,
								   buffer,
								   static_cast<size_t>(length),
								   no_stop,
								   I2C_TRANSFER_TIMEOUT_US);
	}

	I2C::I2C(const Config& config, II2CBackend& backend)
		: config_(config),
		  backend_(backend),
		  initialized_(false)
	{
	}

	bool I2C::init()
	{
		ASSERT(config_.instance != nullptr);
		ASSERT(config_.baudrate != 0U);
		ASSERT(config_.sda_pin != config_.scl_pin);
		ASSERT(config_.address < 0x80U);

		if (!is_valid_config_())
		{
			return false;
		}

		backend_.set_pins(config_.sda_pin, config_.scl_pin);
		backend_.init(config_.instance, config_.baudrate);
		initialized_ = true;
		return true;
	}

	bool I2C::write(const uint8_t* data, uint32_t length)
	{
		ASSERT(initialized_);
		ASSERT(data != nullptr);
		ASSERT(length != 0U);

		if ((!initialized_) || (data == nullptr) || (length == 0U))
		{
			return false;
		}

		const int result = backend_.write(config_.instance,
										  config_.address,
										  data,
										  length,
										  false);
		if (result < 0)
		{
			return false;
		}

		return (static_cast<uint32_t>(result) == length);
	}

	bool I2C::receive(uint8_t* buffer, uint32_t& length)
	{
		ASSERT(initialized_);
		ASSERT(buffer != nullptr);
		ASSERT(length != 0U);

		if ((!initialized_) || (buffer == nullptr) || (length == 0U))
		{
			length = 0U;
			return false;
		}

		const uint32_t requested = length;
		const int result = backend_.read(config_.instance,
										 config_.address,
										 buffer,
										 requested,
										 false);
		if (result < 0)
		{
			length = 0U;
			return false;
		}

		length = static_cast<uint32_t>(result);
		return (length == requested);
	}

	bool I2C::set_address(uint8_t address)
	{
		if (address >= 0x80U)
		{
			return false;
		}

		config_.address = address;
		return true;
	}

	uint8_t I2C::get_address() const
	{
		return config_.address;
	}

	bool I2C::is_valid_config_() const
	{
		if (config_.instance == nullptr)
		{
			return false;
		}

		if (config_.baudrate == 0U)
		{
			return false;
		}

		if (config_.sda_pin == config_.scl_pin)
		{
			return false;
		}

		if (config_.address >= 0x80U)
		{
			return false;
		}

		return true;
	}
} // namespace hal
