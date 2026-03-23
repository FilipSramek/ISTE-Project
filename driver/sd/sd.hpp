/**
 * @file sd.hpp
 * @brief SD Card HAL interface and Pico backend implementation.
 */

#pragma once

#include <stdint.h>
#include "hal/spi/spi.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/timer/timer.hpp"
#include "utils/assert/assert.hpp"

namespace hal
{
	/**
	 * @brief SD Card backend interface for testability.
	 */
	class ISDCardBackend
	{
	public:
		virtual ~ISDCardBackend() = default;
		virtual void init() = 0;
		virtual bool send_command(uint8_t cmd, uint32_t arg, uint8_t& response) = 0;
		virtual bool read_block(uint32_t block_address, uint8_t* buffer, uint32_t block_size) = 0;
		virtual bool write_block(uint32_t block_address, const uint8_t* buffer, uint32_t block_size) = 0;
		virtual bool is_ready() = 0;
	};

	/**
	 * @brief Pico-specific SD Card backend using SPI.
	 */
	class PicoSDCardBackend final : public ISDCardBackend
	{
	public:
		explicit PicoSDCardBackend(SPI& spi, GPIO& chip_select, Timer& timer);

		void init() override;
		bool send_command(uint8_t cmd, uint32_t arg, uint8_t& response) override;
		bool read_block(uint32_t block_address, uint8_t* buffer, uint32_t block_size) override;
		bool write_block(uint32_t block_address, const uint8_t* buffer, uint32_t block_size) override;
		bool is_ready() override;

	private:
		static constexpr uint32_t CMD_TIMEOUT_MS = 100U;
		static constexpr uint32_t DATA_TIMEOUT_MS = 500U;
		static constexpr uint32_t MAX_READ_ATTEMPTS = 1000U;
		static constexpr uint8_t DATA_START_TOKEN = 0xFEU;
		static constexpr uint8_t DATA_RESPONSE_MASK = 0x1FU;
		static constexpr uint8_t DATA_ACCEPTED = 0x05U;

		bool send_cmd_(uint8_t cmd, uint32_t arg, uint8_t& response);
		bool read_response_(uint8_t& response);
		bool wait_for_token_(uint8_t token, uint32_t timeout_ms);
		void chip_select_on_();
		void chip_select_off_();

		SPI& spi_;
		GPIO& chip_select_;
		Timer& timer_;
		bool initialized_;
	};

	/**
	 * @brief SD Card driver with configuration management.
	 */
	class SDCard final
	{
	public:
		/**
		 * @brief SD Card configuration structure.
		 */
		struct Config
		{
			uint32_t block_size;      ///< Block size in bytes (typically 512)
			uint32_t max_blocks;      ///< Maximum number of blocks
			uint32_t cmd_timeout_ms;  ///< Command timeout in milliseconds
			uint32_t data_timeout_ms; ///< Data transfer timeout in milliseconds
		};

		/**
		 * @brief Construct SD Card driver with configuration and backend.
		 * @param config SD Card configuration
		 * @param backend Reference to backend implementation
		 */
		SDCard(const Config& config, ISDCardBackend& backend);

		/**
		 * @brief Initialize SD Card driver.
		 * @return true if initialization succeeded
		 */
		bool init();

		/**
		 * @brief Read one or more blocks from SD Card.
		 * @param block_address Starting block address
		 * @param buffer Buffer to read into
		 * @param num_blocks Number of blocks to read
		 * @param blocks_read Output parameter for actual blocks read
		 * @return true if read succeeded
		 */
		bool read_blocks(uint32_t block_address,
		                 uint8_t* buffer,
		                 uint32_t num_blocks,
		                 uint32_t& blocks_read);

		/**
		 * @brief Write one or more blocks to SD Card.
		 * @param block_address Starting block address
		 * @param buffer Buffer containing data to write
		 * @param num_blocks Number of blocks to write
		 * @param blocks_written Output parameter for actual blocks written
		 * @return true if write succeeded
		 */
		bool write_blocks(uint32_t block_address,
		                  const uint8_t* buffer,
		                  uint32_t num_blocks,
		                  uint32_t& blocks_written);

		/**
		 * @brief Get the block size of the SD Card.
		 * @param block_size Output parameter for block size in bytes
		 * @return true if succeeded
		 */
		bool get_block_size(uint32_t& block_size) const;

		/**
		 * @brief Get the total capacity of the SD Card.
		 * @param capacity Output parameter for capacity in bytes
		 * @return true if succeeded
		 */
		bool get_capacity(uint32_t& capacity) const;

		/**
		 * @brief Write a typed struct to SD Card at specified block address.
		 * @tparam T Type of struct to write
		 * @param block_address Block address to write to
		 * @param data Reference to data structure
		 * @return true if write succeeded
		 */
		template<typename T>
		bool write_struct(uint32_t block_address, const T& data)
		{
			ASSERT(is_initialized_);
			ASSERT(sizeof(T) <= config_.block_size);

			if (!is_initialized_ || (sizeof(T) > config_.block_size))
			{
				return false;
			}

			uint8_t buffer[512] = {0U};
			const uint8_t* src = reinterpret_cast<const uint8_t*>(&data);
			for (uint32_t i = 0U; i < sizeof(T); ++i)
			{
				buffer[i] = src[i];
			}

			uint32_t blocks_written = 0U;
			return write_blocks(block_address, buffer, 1U, blocks_written) && (blocks_written == 1U);
		}

		/**
		 * @brief Read a typed struct from SD Card at specified block address.
		 * @tparam T Type of struct to read
		 * @param block_address Block address to read from
		 * @param data Reference to data structure to fill
		 * @return true if read succeeded
		 */
		template<typename T>
		bool read_struct(uint32_t block_address, T& data)
		{
			ASSERT(is_initialized_);
			ASSERT(sizeof(T) <= config_.block_size);

			if (!is_initialized_ || (sizeof(T) > config_.block_size))
			{
				return false;
			}

			uint8_t buffer[512] = {0U};
			uint32_t blocks_read = 0U;
			if (!read_blocks(block_address, buffer, 1U, blocks_read) || (blocks_read != 1U))
			{
				return false;
			}

			uint8_t* dst = reinterpret_cast<uint8_t*>(&data);
			for (uint32_t i = 0U; i < sizeof(T); ++i)
			{
				dst[i] = buffer[i];
			}

			return true;
		}

		/**
		 * @brief Check if SD Card is initialized.
		 * @return true if initialized
		 */
		bool is_initialized() const;

	private:
		bool is_valid_config_() const;
		bool is_block_in_bounds_(uint32_t block_address, uint32_t num_blocks) const;

		Config config_;
		ISDCardBackend& backend_;
		bool is_initialized_;
	};
}
