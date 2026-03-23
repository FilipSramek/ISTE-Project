/**
 * @file i2c.hpp
 * @brief I2C HAL interface and Pico backend implementation.
 */

#pragma once

#include "pico/stdlib.h"
#include "hal/interfaces/ICominication.hpp"
#include "hardware/i2c.h"
#include "hal/interfaces/II2CBackend.hpp"

namespace hal
{
	/**
	 * @brief Pico-specific I2C backend.
	 */
	class PicoI2CBackend final : public II2CBackend
	{
	public:
		/**
		 * @brief Initialize I2C peripheral.
		 * @param instance I2C instance pointer
		 * @param baudrate Bus speed in Hz
		 */
		void init(i2c_inst_t* instance, uint32_t baudrate) override;

		/**
		 * @brief Configure I2C GPIO pins.
		 * @param sda_pin SDA pin number
		 * @param scl_pin SCL pin number
		 */
		void set_pins(uint8_t sda_pin, uint8_t scl_pin) override;

		/**
		 * @brief Write data to an I2C device.
		 * @param instance I2C instance pointer
		 * @param address 7-bit device address
		 * @param data Data buffer to write
		 * @param length Number of bytes to write
		 * @param no_stop Whether to omit STOP condition
		 * @return Number of bytes written or error code
		 */
		int write(i2c_inst_t* instance,
				  uint8_t address,
				  const uint8_t* data,
				  uint32_t length,
				  bool no_stop) override;

		/**
		 * @brief Read data from an I2C device.
		 * @param instance I2C instance pointer
		 * @param address 7-bit device address
		 * @param buffer Receive buffer
		 * @param length Number of bytes to read
		 * @param no_stop Whether to omit STOP condition
		 * @return Number of bytes read or error code
		 */
		int read(i2c_inst_t* instance,
			 uint8_t address,
			 uint8_t* buffer,
			 uint32_t length,
			 bool no_stop) override;
	};

	/**
	 * @brief I2C communication driver.
	 */
	class I2C final : public ICommunication
	{
	public:
		/**
		 * @brief I2C configuration structure.
		 */
		struct Config
		{
			i2c_inst_t* instance; ///< I2C instance pointer
			uint8_t sda_pin;      ///< SDA pin number
			uint8_t scl_pin;      ///< SCL pin number
			uint32_t baudrate;    ///< Bus speed in Hz
			uint8_t address;      ///< 7-bit device address
		};

		/**
		 * @brief Construct I2C driver with configuration and backend.
		 * @param config I2C configuration
		 * @param backend Reference to backend implementation
		 */
		I2C(const Config& config, II2CBackend& backend);

		/**
		 * @brief Initialize the I2C driver.
		 * @return true if initialization succeeded
		 */
		bool init();

		/**
		 * @brief Write data to the configured I2C device.
		 * @param data Data buffer to write
		 * @param length Number of bytes to write
		 * @return true on success
		 */
		bool write(const uint8_t* data, uint32_t length) override;

		/**
		 * @brief Receive data from the configured I2C device.
		 * @param buffer Receive buffer
		 * @param length In: buffer size, Out: bytes received
		 * @return true on success
		 */
		bool receive(uint8_t* buffer, uint32_t& length) override;

	private:
		/**
		 * @brief Validate the current configuration.
		 * @return true if configuration is valid
		 */
		bool is_valid_config_() const;

		Config config_;
		II2CBackend& backend_;
		bool initialized_;
	};
} // namespace hal
