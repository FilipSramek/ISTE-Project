/**
 * @file II2CBackend.hpp
 * @brief Backend interface for I2C hardware access.
 */

#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

namespace hal
{
    /**
     * @brief Abstract interface for I2C backend implementation.
     */
    class II2CBackend
	{
	public:
		virtual ~II2CBackend() = default;

		/**
		 * @brief Initialize I2C peripheral.
		 * @param instance I2C instance pointer
		 * @param baudrate Bus speed in Hz
		 */
		virtual void init(i2c_inst_t* instance, uint32_t baudrate) = 0;

		/**
		 * @brief Configure I2C GPIO pins.
		 * @param sda_pin SDA pin number
		 * @param scl_pin SCL pin number
		 */
		virtual void set_pins(uint8_t sda_pin, uint8_t scl_pin) = 0;

		/**
		 * @brief Write data to an I2C device.
		 * @param instance I2C instance pointer
		 * @param address 7-bit device address
		 * @param data Data buffer to write
		 * @param length Number of bytes to write
		 * @param no_stop Whether to omit STOP condition
		 * @return Number of bytes written or error code
		 */
		virtual int write(i2c_inst_t* instance,
					  uint8_t address,
					  const uint8_t* data,
					  uint32_t length,
					  bool no_stop) = 0;

		/**
		 * @brief Read data from an I2C device.
		 * @param instance I2C instance pointer
		 * @param address 7-bit device address
		 * @param buffer Receive buffer
		 * @param length Number of bytes to read
		 * @param no_stop Whether to omit STOP condition
		 * @return Number of bytes read or error code
		 */
		virtual int read(i2c_inst_t* instance,
					 uint8_t address,
					 uint8_t* buffer,
					 uint32_t length,
					 bool no_stop) = 0;
	};
} // namespace hal
