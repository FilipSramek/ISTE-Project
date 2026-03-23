/**
 * @file IFlashBackend.hpp
 * @brief Backend interface for internal flash memory access.
 */

#pragma once

#include "pico/stdlib.h"

namespace hal
{
    /**
     * @brief Abstract interface for internal flash backend implementation.
     */
    class IFlashBackend
    {
    public:
        virtual ~IFlashBackend() = default;

        /**
         * @brief Initialize flash memory access.
         */
        virtual void init() = 0;

        /**
         * @brief Read data from flash memory.
         * @param offset Offset in flash from start of user data area
         * @param buffer Pointer to buffer to read into
         * @param length Number of bytes to read
         * @return Number of bytes successfully read
         */
        virtual uint32_t read(uint32_t offset, uint8_t* buffer, uint32_t length) = 0;

        /**
         * @brief Write data to flash memory.
         * @param offset Offset in flash from start of user data area
         * @param buffer Pointer to buffer containing data to write
         * @param length Number of bytes to write
         * @return Number of bytes successfully written
         */
        virtual uint32_t write(uint32_t offset, const uint8_t* buffer, uint32_t length) = 0;

        /**
         * @brief Erase a sector of flash memory.
         * @param offset Offset in flash from start of user data area
         * @return True if erase was successful, false otherwise
         */
        virtual bool erase(uint32_t offset) = 0;

        /**
         * @brief Get the total size of available flash for user data.
         * @return Size in bytes
         */
        virtual uint32_t get_size() const = 0;

        /**
         * @brief Get the sector size for erase operations.
         * @return Sector size in bytes
         */
        virtual uint32_t get_sector_size() const = 0;

        /**
         * @brief Flush the XIP cache after programming/erasing flash.
         * Required after write/erase operations to clear stale cached data.
         * See RP2040 datasheet section 2.6.3.2 for details.
         */
        virtual void flush_cache() = 0;

        /**
         * @brief Invalidate cache lines for a specific address range.
         * Use this for selective cache invalidation without full flush.
         * See RP2040 datasheet section 2.6.3.2 for details.
         * @param address Start address in flash address space (0x10xxxxxxxx format)
         * @param length Size of range to invalidate
         */
        virtual void invalidate_cache_range(uint32_t address, uint32_t length) = 0;
    };
}
