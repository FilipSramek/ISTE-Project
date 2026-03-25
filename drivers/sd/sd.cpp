/**
 * @file sd.cpp
 * @brief SD Card HAL implementation.
 */

#include "drivers/sd/sd.hpp"

#include "utils/assert/assert.hpp"

namespace hal
{
	PicoSDCardBackend::PicoSDCardBackend(SPI& spi, GPIO& chip_select, Timer& timer)
		: spi_(spi), chip_select_(chip_select), timer_(timer), initialized_(false)
	{
	}

	void PicoSDCardBackend::init()
	{
		ASSERT(!initialized_);
		initialized_ = true;
	}

	bool PicoSDCardBackend::send_command(uint8_t cmd, uint32_t arg, uint8_t& response)
	{
		ASSERT(initialized_);

		if (!initialized_)
		{
			return false;
		}

		return send_cmd_(cmd, arg, response);
	}

	bool PicoSDCardBackend::read_block(uint32_t block_address, uint8_t* buffer, uint32_t block_size)
	{
		ASSERT(initialized_);
		ASSERT(buffer != nullptr);
		ASSERT(block_size == 512U);

		if (!initialized_ || (buffer == nullptr) || (block_size != 512U))
		{
			return false;
		}

		chip_select_on_();

		uint8_t cmd_buffer[6] = {0x51U, 0U, 0U, 0U, 0U, 0xFFU};
		cmd_buffer[1] = static_cast<uint8_t>((block_address >> 24) & 0xFFU);
		cmd_buffer[2] = static_cast<uint8_t>((block_address >> 16) & 0xFFU);
		cmd_buffer[3] = static_cast<uint8_t>((block_address >> 8) & 0xFFU);
		cmd_buffer[4] = static_cast<uint8_t>(block_address & 0xFFU);

		uint8_t rx[6] = {0U};
		uint32_t len = 6U;
		if (!spi_.transfer(cmd_buffer, rx, len))
		{
			chip_select_off_();
			return false;
		}

		if (!wait_for_token_(DATA_START_TOKEN, DATA_TIMEOUT_MS))
		{
			chip_select_off_();
			return false;
		}

		uint8_t rx_buffer[512] = {0U};
		uint32_t read_len = 512U;
		if (!spi_.receive(rx_buffer, read_len))
		{
			chip_select_off_();
			return false;
		}

		for (uint32_t i = 0U; i < block_size; ++i)
		{
			buffer[i] = rx_buffer[i];
		}

		uint8_t crc[2] = {0U};
		uint32_t crc_len = 2U;
		if (!spi_.receive(crc, crc_len))
		{
			chip_select_off_();
			return false;
		}

		chip_select_off_();
		return true;
	}

	bool PicoSDCardBackend::write_block(uint32_t block_address, const uint8_t* buffer, uint32_t block_size)
	{
		ASSERT(initialized_);
		ASSERT(buffer != nullptr);
		ASSERT(block_size == 512U);

		if (!initialized_ || (buffer == nullptr) || (block_size != 512U))
		{
			return false;
		}

		chip_select_on_();

		uint8_t cmd_buffer[6] = {0x58U, 0U, 0U, 0U, 0U, 0xFFU};
		cmd_buffer[1] = static_cast<uint8_t>((block_address >> 24) & 0xFFU);
		cmd_buffer[2] = static_cast<uint8_t>((block_address >> 16) & 0xFFU);
		cmd_buffer[3] = static_cast<uint8_t>((block_address >> 8) & 0xFFU);
		cmd_buffer[4] = static_cast<uint8_t>(block_address & 0xFFU);

		uint8_t rx[6] = {0U};
		uint32_t tx_len = 6U;
		if (!spi_.transfer(cmd_buffer, rx, tx_len))
		{
			chip_select_off_();
			return false;
		}

		uint8_t token = DATA_START_TOKEN;
		if (!spi_.write(&token, 1U))
		{
			chip_select_off_();
			return false;
		}

		uint8_t tx_buffer[512] = {0U};
		for (uint32_t i = 0U; i < block_size; ++i)
		{
			tx_buffer[i] = buffer[i];
		}

		if (!spi_.write(tx_buffer, 512U))
		{
			chip_select_off_();
			return false;
		}

		uint8_t crc[2] = {0xFFU, 0xFFU};
		if (!spi_.write(crc, 2U))
		{
			chip_select_off_();
			return false;
		}

		uint8_t response = 0U;
		if (!read_response_(response))
		{
			chip_select_off_();
			return false;
		}

		if ((response & DATA_RESPONSE_MASK) != DATA_ACCEPTED)
		{
			chip_select_off_();
			return false;
		}

		if (!wait_for_token_(0xFFU, DATA_TIMEOUT_MS))
		{
			chip_select_off_();
			return false;
		}

		chip_select_off_();
		return true;
	}

	bool PicoSDCardBackend::is_ready()
	{
		return initialized_;
	}

	bool PicoSDCardBackend::send_cmd_(uint8_t cmd, uint32_t arg, uint8_t& response)
	{
		ASSERT(cmd < 0xFEU);

		if (cmd >= 0xFEU)
		{
			return false;
		}

		chip_select_on_();

		uint8_t cmd_buffer[6] = {0U};
		cmd_buffer[0] = static_cast<uint8_t>(0x40U | cmd);
		cmd_buffer[1] = static_cast<uint8_t>((arg >> 24) & 0xFFU);
		cmd_buffer[2] = static_cast<uint8_t>((arg >> 16) & 0xFFU);
		cmd_buffer[3] = static_cast<uint8_t>((arg >> 8) & 0xFFU);
		cmd_buffer[4] = static_cast<uint8_t>(arg & 0xFFU);
		cmd_buffer[5] = 0x95U;

		uint8_t rx[6] = {0U};
		uint32_t cmd_len = 6U;
		if (!spi_.transfer(cmd_buffer, rx, cmd_len))
		{
			chip_select_off_();
			return false;
		}

		const bool ok = read_response_(response);
		chip_select_off_();
		return ok;
	}

	bool PicoSDCardBackend::read_response_(uint8_t& response)
	{
		ASSERT(MAX_READ_ATTEMPTS > 0U);

		for (uint32_t i = 0U; i < MAX_READ_ATTEMPTS; ++i)
		{
			uint8_t rx = 0xFFU;
			uint32_t rsp_len = 1U;
			if (!spi_.receive(&rx, rsp_len))
			{
				return false;
			}

			if ((rx & 0x80U) == 0U)
			{
				response = rx;
				return true;
			}
		}

		return false;
	}

	bool PicoSDCardBackend::wait_for_token_(uint8_t token, uint32_t timeout_ms)
	{
		ASSERT(timeout_ms > 0U);
		ASSERT(MAX_READ_ATTEMPTS > 0U);

		if (timeout_ms == 0U)
		{
			return false;
		}

		const uint64_t start_time = timer_.now();
		for (uint32_t i = 0U; i < MAX_READ_ATTEMPTS; ++i)
		{
			uint8_t rx = 0xFFU;
			uint32_t tok_len = 1U;
			if (!spi_.receive(&rx, tok_len))
			{
				return false;
			}

			if (rx == token)
			{
				return true;
			}

			if (timer_.is_timeout_ms(start_time, timeout_ms))
			{
				return false;
			}
		}

		return false;
	}

	void PicoSDCardBackend::chip_select_on_()
	{
		(void)chip_select_.write(false);
	}

	void PicoSDCardBackend::chip_select_off_()
	{
		(void)chip_select_.write(true);
	}

	SDCard::SDCard(const Config& config, ISDCardBackend& backend)
		: config_(config), backend_(backend), is_initialized_(false)
	{
	}

	bool SDCard::init()
	{
		ASSERT(!is_initialized_);
		ASSERT(is_valid_config_());

		if (is_initialized_ || !is_valid_config_())
		{
			return false;
		}

		backend_.init();
		is_initialized_ = true;
		return true;
	}

	bool SDCard::read_blocks(uint32_t block_address,
	                          uint8_t* buffer,
	                          uint32_t num_blocks,
	                          uint32_t& blocks_read)
	{
		ASSERT(is_initialized_);
		ASSERT(buffer != nullptr);
		ASSERT(num_blocks > 0U);

		blocks_read = 0U;
		if (!is_initialized_ || (buffer == nullptr) || (num_blocks == 0U))
		{
			return false;
		}

		if (!is_block_in_bounds_(block_address, num_blocks))
		{
			return false;
		}

		for (uint32_t i = 0U; i < num_blocks; ++i)
		{
			const uint32_t offset = i * config_.block_size;
			if (!backend_.read_block(block_address + i, &buffer[offset], config_.block_size))
			{
				return false;
			}
			++blocks_read;
		}

		return true;
	}

	bool SDCard::write_blocks(uint32_t block_address,
	                           const uint8_t* buffer,
	                           uint32_t num_blocks,
	                           uint32_t& blocks_written)
	{
		ASSERT(is_initialized_);
		ASSERT(buffer != nullptr);
		ASSERT(num_blocks > 0U);

		blocks_written = 0U;
		if (!is_initialized_ || (buffer == nullptr) || (num_blocks == 0U))
		{
			return false;
		}

		if (!is_block_in_bounds_(block_address, num_blocks))
		{
			return false;
		}

		for (uint32_t i = 0U; i < num_blocks; ++i)
		{
			const uint32_t offset = i * config_.block_size;
			if (!backend_.write_block(block_address + i, &buffer[offset], config_.block_size))
			{
				return false;
			}
			++blocks_written;
		}

		return true;
	}

	bool SDCard::get_block_size(uint32_t& block_size) const
	{
		ASSERT(is_initialized_);

		if (!is_initialized_)
		{
			return false;
		}

		block_size = config_.block_size;
		return true;
	}

	bool SDCard::get_capacity(uint32_t& capacity) const
	{
		ASSERT(is_initialized_);

		if (!is_initialized_)
		{
			return false;
		}

		capacity = config_.block_size * config_.max_blocks;
		return true;
	}

	bool SDCard::is_initialized() const
	{
		return is_initialized_;
	}

	bool SDCard::is_valid_config_() const
	{
		ASSERT(config_.block_size > 0U);
		ASSERT(config_.max_blocks > 0U);
		ASSERT(config_.cmd_timeout_ms > 0U);
		ASSERT(config_.data_timeout_ms > 0U);

		if ((config_.block_size == 0U) || (config_.max_blocks == 0U) ||
		    (config_.cmd_timeout_ms == 0U) || (config_.data_timeout_ms == 0U))
		{
			return false;
		}

		return true;
	}

	bool SDCard::is_block_in_bounds_(uint32_t block_address, uint32_t num_blocks) const
	{
		ASSERT(num_blocks > 0U);

		if (num_blocks == 0U)
		{
			return false;
		}

		if (block_address >= config_.max_blocks)
		{
			return false;
		}

		const uint32_t end_block = block_address + num_blocks;
		if (end_block > config_.max_blocks)
		{
			return false;
		}

		return true;
	}
}
