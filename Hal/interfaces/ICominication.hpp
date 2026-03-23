/**
 * @file ICominication.hpp
 * @brief Common communication interface for HAL modules.
 */

#pragma once

#include <stdint.h>

namespace hal
{
    /**
     * @brief Generic communication interface.
     *
     * Provides a common API for HAL drivers that support
     * basic write and receive operations.
     */
    class ICommunication
    {
    public:
        virtual ~ICommunication() = default;

        /**
         * @brief Write data to the device.
         * @param data Pointer to data buffer
         * @param length Number of bytes to write
         * @return true on success, false on failure
         */
        virtual bool write(const uint8_t* data, uint32_t length) = 0;

        /**
         * @brief Receive data from the device.
         * @param buffer Pointer to receive buffer
         * @param length In: buffer size, Out: number of bytes received
         * @return true on success, false on failure
         */
        virtual bool receive(uint8_t* buffer, uint32_t& length) = 0;

    protected:
        ICommunication() = default;
    };
} // namespace hal
