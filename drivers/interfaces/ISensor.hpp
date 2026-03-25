/**
 * @file ISensor.hpp
 * @brief Common sensor interface for driver modules.
 */

#pragma once

#include <stdint.h>

namespace drivers
{
	/**
	 * @brief Generic sensor interface.
	 *
	 * Provides a common API for all sensor drivers with standard
	 * lifecycle methods: init, read, update, and state query.
	 */
	class ISensor
	{
	public:
		virtual ~ISensor() = default;

		/**
		 * @brief Initialize the sensor.
		 * 
		 * Must be called before any other sensor operations.
		 * 
		 * @return true if initialization succeeded, false otherwise
		 */
		virtual bool init() = 0;

		/**
		 * @brief Check if sensor is initialized.
		 * @return true if sensor is ready to use
		 */
		virtual bool is_initialized() const = 0;

	protected:
		ISensor() = default;
	};

	/**
	 * @brief Template interface for sensors with typed data.
	 * @tparam T Data type specific to the sensor
	 */
	template<typename T>
	class ITypedSensor : public ISensor
	{
	public:
		/**
		 * @brief Read sensor data directly.
		 * 
		 * Performs an immediate read from the sensor and returns
		 * the data without updating internal state.
		 * 
		 * @param data Reference to data structure to fill
		 * @return true if read succeeded, false otherwise
		 */
		virtual bool read(T& data) = 0;

		/**
		 * @brief Update internal state with new measurement.
		 * 
		 * Reads sensor data and updates the internal state variable.
		 * Use get_data() to retrieve the updated state.
		 * 
		 * @return true if update succeeded, false otherwise
		 */
		virtual bool update() = 0;

		/**
		 * @brief Get the latest sensor data from internal state.
		 * 
		 * Returns the most recently updated sensor data without
		 * performing a new read operation.
		 * 
		 * @param data Reference to data structure to fill
		 * @return true if data is valid, false otherwise
		 */
		virtual bool get_data(T& data) const = 0;

	protected:
		ITypedSensor() = default;
	};
} // namespace drivers
