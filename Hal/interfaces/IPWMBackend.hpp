#pragma once

#include "pico/stdlib.h"
#include "hardware/pwm.h"

namespace hal
{
	/**
	 * @brief Abstract interface for PWM backend implementation.
	 * 
	 * Defines the contract for PWM hardware abstraction, allowing different
	 * platform-specific implementations to be used interchangeably.
	 */
	class IPWMBackend
	{
	public:
		virtual ~IPWMBackend() = default;

		/**
		 * @brief Initialize PWM on the specified pin.
		 * @param pin GPIO pin number (0-29 for RP2040)
		 */
		virtual void init(uint8_t pin) = 0;

		/**
		 * @brief Set PWM frequency in Hz.
		 * @param pin GPIO pin number
		 * @param frequency Frequency in Hz (1 Hz to 1 MHz recommended)
		 */
		virtual void set_frequency(uint8_t pin, uint32_t frequency) = 0;

		/**
		 * @brief Set PWM duty cycle.
		 * @param pin GPIO pin number
		 * @param duty_cycle Duty cycle as float (0.0 to 1.0)
		 */
		virtual void set_duty_cycle(uint8_t pin, float duty_cycle) = 0;

		/**
		 * @brief Set wrap value (counter reload value) for PWM.
		 * @param wrap Wrap value for the PWM counter
		 */
		virtual void set_wrap(uint32_t wrap) = 0;

		/**
		 * @brief Enable PWM on the specified pin.
		 * @param pin GPIO pin number
		 */
		virtual void enable(uint8_t pin) = 0;

		/**
		 * @brief Disable PWM on the specified pin.
		 * @param pin GPIO pin number
		 */
		virtual void disable(uint8_t pin) = 0;
	};
}
