/**
 * @file sx1278.cpp
 * @brief SX1278 LoRa transceiver driver implementation.
 */

#include "drivers/sx1278/sx1278.hpp"

#include "utils/assert/assert.hpp"

namespace drivers
{
	SX1278::SX1278(hal::SPI& spi,
	               hal::GPIO& nss,
	               hal::GPIO& reset,
	               hal::GPIO& dio0,
	               hal::Timer& timer,
	               const Config& config)
		: spi_(spi),
		  nss_(nss),
		  reset_(reset),
		  dio0_(dio0),
		  timer_(timer),
		  config_(config),
		  current_data_{{0U}, 0U, 0, 0, false, false},
		  reassembly_{{0U}, 0U, 0U, 0U, 0U, 0U, 0U, false},
		  next_message_id_(1U),
		  initialized_(false)
	{
	}

	bool SX1278::init()
	{
		ASSERT(!initialized_);
		ASSERT(config_.frequency_hz != 0U);

		if (initialized_ || !is_valid_config_())
		{
			return false;
		}

		if (!reset_radio_() || !set_mode_(MODE_SLEEP))
		{
			return false;
		}

		uint8_t version = 0U;
		if (!read_register_(Register::VERSION, version))
		{
			return false;
		}

		if ((version != SX1278_VERSION) || !configure_radio_())
		{
			return false;
		}

		initialized_ = true;
		current_data_.valid = false;
		reset_reassembly_();
		return true;
	}

	bool SX1278::read(Data& data)
	{
		ASSERT(initialized_);
		ASSERT(config_.rx_timeout_ms != 0U);

		if (!initialized_)
		{
			data.valid = false;
			return false;
		}

		if (!clear_irq_flags_() || !write_register_(Register::FIFO_ADDR_PTR, 0x00U))
		{
			data.valid = false;
			return false;
		}

		if (!set_mode_(MODE_RX_SINGLE) || !wait_for_dio0_(config_.rx_timeout_ms))
		{
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		uint8_t irq_flags = 0U;
		if (!read_register_(Register::IRQ_FLAGS, irq_flags))
		{
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		if ((irq_flags & IRQ_RX_DONE) == 0U)
		{
			(void)clear_irq_flags_();
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		data.crc_error = ((irq_flags & IRQ_PAYLOAD_CRC_ERROR) != 0U);
		if (data.crc_error)
		{
			(void)clear_irq_flags_();
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		uint8_t rx_addr = 0U;
		uint8_t rx_bytes = 0U;
		if (!read_register_(Register::FIFO_RX_CURRENT_ADDR, rx_addr) ||
		    !read_register_(Register::RX_NB_BYTES, rx_bytes) ||
		    !write_register_(Register::FIFO_ADDR_PTR, rx_addr))
		{
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		data.length = static_cast<uint32_t>(rx_bytes);
		if ((data.length == 0U) ||
		    (data.length > MAX_PAYLOAD_BYTES) ||
		    !read_burst_(Register::FIFO, data.payload, data.length) ||
		    !read_packet_metrics_(data.rssi_dbm, data.snr_cdb))
		{
			(void)clear_irq_flags_();
			(void)enter_standby_();
			data.valid = false;
			return false;
		}

		data.valid = clear_irq_flags_() && enter_standby_();
		return data.valid;
	}

	bool SX1278::update()
	{
		ASSERT(initialized_);
		ASSERT(config_.rx_timeout_ms != 0U);

		if (!initialized_)
		{
			return false;
		}

		return read(current_data_);
	}

	bool SX1278::get_data(Data& data) const
	{
		ASSERT(initialized_);
		ASSERT(config_.frequency_hz != 0U);

		if (!initialized_)
		{
			return false;
		}

		data = current_data_;
		return current_data_.valid;
	}

	bool SX1278::is_initialized() const
	{
		return initialized_;
	}

	bool SX1278::send(const uint8_t* payload, uint32_t length)
	{
		ASSERT(initialized_);
		ASSERT(payload != nullptr);

		if ((!initialized_) || (payload == nullptr) || (length == 0U) || (length > MAX_PAYLOAD_BYTES))
		{
			return false;
		}

		if (!enter_standby_() ||
		    !clear_irq_flags_() ||
		    !write_register_(Register::FIFO_ADDR_PTR, 0x00U) ||
		    !write_burst_(Register::FIFO, payload, length) ||
		    !write_register_(Register::PAYLOAD_LENGTH, static_cast<uint8_t>(length)) ||
		    !set_mode_(MODE_TX) ||
		    !wait_for_dio0_(config_.rx_timeout_ms))
		{
			(void)enter_standby_();
			return false;
		}

		uint8_t irq_flags = 0U;
		return read_register_(Register::IRQ_FLAGS, irq_flags) &&
		       ((irq_flags & IRQ_TX_DONE) != 0U) &&
		       clear_irq_flags_() &&
		       enter_standby_();
	}

	bool SX1278::send_message(const uint8_t* payload, uint32_t length)
	{
		ASSERT(initialized_);
		ASSERT(payload != nullptr);

		if ((!initialized_) || (payload == nullptr) || (length == 0U) || (length > MAX_MESSAGE_BYTES))
		{
			return false;
		}

		const uint16_t message_id = next_message_id_;
		next_message_id_ = static_cast<uint16_t>(next_message_id_ + 1U);
		if (next_message_id_ == 0U)
		{
			next_message_id_ = 1U;
		}

		const uint16_t payload_crc16 = crc16_ccitt_(payload, length);
		const uint8_t fragment_count = static_cast<uint8_t>((length + MAX_FRAGMENT_PAYLOAD_BYTES - 1U) / MAX_FRAGMENT_PAYLOAD_BYTES);
		uint32_t offset = 0U;

		for (uint8_t fragment_index = 0U; fragment_index < fragment_count; ++fragment_index)
		{
			uint8_t frame[MAX_PAYLOAD_BYTES] = {0U};
			uint32_t frame_length = 0U;

			if (!build_fragment_(payload,
			                     length,
			                     message_id,
			                     payload_crc16,
			                     fragment_index,
			                     fragment_count,
			                     offset,
			                     frame,
			                     frame_length) ||
			    !send(frame, frame_length))
			{
				return false;
			}

			offset += static_cast<uint32_t>(frame[9]);
		}

		return true;
	}

	bool SX1278::read_message(uint8_t* payload, uint32_t capacity, uint32_t& length)
	{
		ASSERT(initialized_);
		ASSERT(payload != nullptr);

		length = 0U;
		if ((!initialized_) || (payload == nullptr) || (capacity == 0U) || (capacity > MAX_MESSAGE_BYTES))
		{
			return false;
		}

		reset_reassembly_();
		for (uint32_t i = 0U; i < MAX_FRAGMENT_COUNT; ++i)
		{
			Data packet;
			FragmentHeader header = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
			const uint8_t* fragment_payload = nullptr;

			if (!read(packet) || !parse_fragment_(packet, header, fragment_payload) || !accept_fragment_(header, fragment_payload))
			{
				reset_reassembly_();
				return false;
			}

			if (reassembly_.active && (reassembly_.next_fragment_index >= reassembly_.fragment_count))
			{
				if ((reassembly_.total_length > capacity) ||
				    (crc16_ccitt_(reassembly_.payload, reassembly_.total_length) != reassembly_.payload_crc16))
				{
					reset_reassembly_();
					return false;
				}

				for (uint32_t j = 0U; j < reassembly_.total_length; ++j)
				{
					payload[j] = reassembly_.payload[j];
				}

				length = reassembly_.total_length;
				reset_reassembly_();
				return true;
			}
		}

		reset_reassembly_();
		return false;
	}

	bool SX1278::is_valid_config_() const
	{
		ASSERT(config_.frequency_hz >= 137000000U);
		ASSERT(config_.frequency_hz <= 525000000U);

		if ((config_.frequency_hz < 137000000U) || (config_.frequency_hz > 525000000U))
		{
			return false;
		}

		if ((config_.tx_power_dbm < 2) || (config_.tx_power_dbm > 17))
		{
			return false;
		}

		if ((config_.preamble_length == 0U) || (config_.rx_timeout_ms == 0U))
		{
			return false;
		}

		return true;
	}

	bool SX1278::reset_radio_()
	{
		ASSERT(RESET_PULSE_MS != 0U);
		ASSERT(RESET_SETTLE_MS != 0U);

		if (!reset_.write(false))
		{
			return false;
		}

		timer_.delay_ms(RESET_PULSE_MS);
		if (!reset_.write(true))
		{
			return false;
		}

		timer_.delay_ms(RESET_SETTLE_MS);
		return true;
	}

	bool SX1278::write_register_(Register reg, uint8_t value)
	{
		ASSERT(static_cast<uint8_t>(reg) <= 0x7FU);
		ASSERT(REG_WRITE_MASK == 0x80U);

		uint8_t tx[2] = {static_cast<uint8_t>(static_cast<uint8_t>(reg) | REG_WRITE_MASK), value};
		uint8_t rx[2] = {0U, 0U};
		uint32_t length = 2U;

		if (!nss_.write(false))
		{
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, length);
		const bool deassert_ok = nss_.write(true);
		return ok && deassert_ok;
	}

	bool SX1278::read_register_(Register reg, uint8_t& value)
	{
		ASSERT(static_cast<uint8_t>(reg) <= 0x7FU);
		ASSERT(REG_READ_MASK == 0x7FU);

		uint8_t tx[2] = {static_cast<uint8_t>(static_cast<uint8_t>(reg) & REG_READ_MASK), 0x00U};
		uint8_t rx[2] = {0U, 0U};
		uint32_t length = 2U;

		if (!nss_.write(false))
		{
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, length);
		const bool deassert_ok = nss_.write(true);
		value = rx[1];
		return ok && deassert_ok;
	}

	bool SX1278::write_burst_(Register reg, const uint8_t* data, uint32_t length)
	{
		ASSERT(data != nullptr);
		ASSERT(length <= MAX_PAYLOAD_BYTES);

		if ((data == nullptr) || (length == 0U) || (length > MAX_PAYLOAD_BYTES))
		{
			return false;
		}

		uint8_t tx[MAX_PAYLOAD_BYTES + 1U] = {0U};
		uint8_t rx[MAX_PAYLOAD_BYTES + 1U] = {0U};
		tx[0] = static_cast<uint8_t>(static_cast<uint8_t>(reg) | REG_WRITE_MASK);
		for (uint32_t i = 0U; i < length; ++i)
		{
			tx[i + 1U] = data[i];
		}

		uint32_t transfer_len = length + 1U;
		if (!nss_.write(false))
		{
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, transfer_len);
		const bool deassert_ok = nss_.write(true);
		return ok && deassert_ok;
	}

	bool SX1278::read_burst_(Register reg, uint8_t* data, uint32_t length)
	{
		ASSERT(data != nullptr);
		ASSERT(length <= MAX_PAYLOAD_BYTES);

		if ((data == nullptr) || (length == 0U) || (length > MAX_PAYLOAD_BYTES))
		{
			return false;
		}

		uint8_t tx[MAX_PAYLOAD_BYTES + 1U] = {0U};
		uint8_t rx[MAX_PAYLOAD_BYTES + 1U] = {0U};
		tx[0] = static_cast<uint8_t>(static_cast<uint8_t>(reg) & REG_READ_MASK);

		uint32_t transfer_len = length + 1U;
		if (!nss_.write(false))
		{
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, transfer_len);
		const bool deassert_ok = nss_.write(true);
		for (uint32_t i = 0U; i < length; ++i)
		{
			data[i] = rx[i + 1U];
		}
		return ok && deassert_ok;
	}

	bool SX1278::set_mode_(uint8_t mode_bits)
	{
		ASSERT((mode_bits & 0x07U) == mode_bits);
		ASSERT(MODE_LONG_RANGE == 0x80U);

		return write_register_(Register::OP_MODE, static_cast<uint8_t>(MODE_LONG_RANGE | mode_bits));
	}

	bool SX1278::enter_standby_()
	{
		ASSERT(MODE_STDBY == 0x01U);
		ASSERT(MODE_SLEEP == 0x00U);

		return set_mode_(MODE_STDBY);
	}

	bool SX1278::clear_irq_flags_()
	{
		ASSERT(IRQ_RX_DONE == 0x40U);
		ASSERT(IRQ_TX_DONE == 0x08U);

		return write_register_(Register::IRQ_FLAGS, 0xFFU);
	}

	bool SX1278::wait_for_dio0_(uint32_t timeout_ms)
	{
		ASSERT(timeout_ms != 0U);
		ASSERT(MAX_IRQ_WAIT_ITERS != 0U);

		const uint64_t start_time = timer_.now();
		for (uint32_t i = 0U; i < MAX_IRQ_WAIT_ITERS; ++i)
		{
			bool dio0_high = false;
			if (!dio0_.read(dio0_high))
			{
				return false;
			}

			if (dio0_high)
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

	bool SX1278::configure_radio_()
	{
		ASSERT(config_.sync_word != 0U);
		ASSERT(config_.preamble_length != 0U);

		return enter_standby_() &&
		       set_frequency_() &&
		       write_register_(Register::FIFO_TX_BASE_ADDR, 0x00U) &&
		       write_register_(Register::FIFO_RX_BASE_ADDR, 0x00U) &&
		       write_register_(Register::LNA, 0x23U) &&
		       write_register_(Register::SYNC_WORD, config_.sync_word) &&
		       write_register_(Register::DIO_MAPPING1, 0x00U) &&
		       set_tx_power_() &&
		       configure_modem_() &&
		       clear_irq_flags_();
	}

	bool SX1278::set_frequency_()
	{
		ASSERT(config_.frequency_hz >= 137000000U);
		ASSERT(config_.frequency_hz <= 525000000U);

		const int64_t frf = (static_cast<int64_t>(config_.frequency_hz) << 19) / 32000000LL;
		return write_register_(Register::FRF_MSB, static_cast<uint8_t>((frf >> 16) & 0xFFLL)) &&
		       write_register_(Register::FRF_MID, static_cast<uint8_t>((frf >> 8) & 0xFFLL)) &&
		       write_register_(Register::FRF_LSB, static_cast<uint8_t>(frf & 0xFFLL));
	}

	bool SX1278::set_tx_power_()
	{
		ASSERT(config_.tx_power_dbm >= 2);
		ASSERT(config_.tx_power_dbm <= 17);

		const uint8_t power_bits = static_cast<uint8_t>(config_.tx_power_dbm - 2);
		return write_register_(Register::PA_CONFIG, static_cast<uint8_t>(PA_BOOST_ENABLE | power_bits)) &&
		       write_register_(Register::PA_DAC, 0x04U);
	}

	bool SX1278::configure_modem_()
	{
		ASSERT(static_cast<uint8_t>(config_.bandwidth) <= 0x09U);
		ASSERT(static_cast<uint8_t>(config_.coding_rate) >= 0x01U);

		const uint8_t modem_config1 = static_cast<uint8_t>((static_cast<uint8_t>(config_.bandwidth) << 4) |
		                                                  (static_cast<uint8_t>(config_.coding_rate) << 1));
		const uint8_t sf_bits = static_cast<uint8_t>(config_.spreading_factor);
		const uint8_t modem_config2 = static_cast<uint8_t>((sf_bits << 4) |
		                                                  (config_.crc_enabled ? 0x04U : 0x00U));
		const uint8_t detection_opt = (config_.spreading_factor == SpreadingFactor::SF6) ? 0x05U : 0x03U;
		const uint8_t modem_config3 = 0x04U;

		return write_register_(Register::MODEM_CONFIG1, modem_config1) &&
		       write_register_(Register::MODEM_CONFIG2, modem_config2) &&
		       write_register_(Register::MODEM_CONFIG3, modem_config3) &&
		       write_register_(Register::PREAMBLE_MSB, static_cast<uint8_t>((config_.preamble_length >> 8) & 0xFFU)) &&
		       write_register_(Register::PREAMBLE_LSB, static_cast<uint8_t>(config_.preamble_length & 0xFFU)) &&
		       write_register_(Register::DETECTION_OPTIMIZE, detection_opt);
	}

	bool SX1278::read_packet_metrics_(int16_t& rssi_dbm, int16_t& snr_cdb)
	{
		ASSERT(config_.frequency_hz >= 137000000U);
		ASSERT(config_.frequency_hz <= 525000000U);

		uint8_t snr_raw = 0U;
		uint8_t rssi_raw = 0U;
		if (!read_register_(Register::PKT_SNR_VALUE, snr_raw) || !read_register_(Register::PKT_RSSI_VALUE, rssi_raw))
		{
			return false;
		}

		const int8_t snr_qdb = static_cast<int8_t>(snr_raw);
		snr_cdb = static_cast<int16_t>(snr_qdb * 25);
		rssi_dbm = static_cast<int16_t>(static_cast<int16_t>(rssi_raw) - 157);
		return true;
	}

	bool SX1278::build_fragment_(const uint8_t* payload,
	                            uint32_t total_length,
	                            uint16_t message_id,
	                            uint16_t payload_crc16,
	                            uint8_t fragment_index,
	                            uint8_t fragment_count,
	                            uint32_t offset,
	                            uint8_t* frame,
	                            uint32_t& frame_length) const
	{
		ASSERT(payload != nullptr);
		ASSERT(frame != nullptr);

		if ((payload == nullptr) || (frame == nullptr) || (offset >= total_length) ||
		    (fragment_count == 0U) || (fragment_count > MAX_FRAGMENT_COUNT) || (fragment_index >= fragment_count))
		{
			return false;
		}

		const uint32_t remaining = total_length - offset;
		const uint8_t fragment_length = static_cast<uint8_t>((remaining > MAX_FRAGMENT_PAYLOAD_BYTES) ? MAX_FRAGMENT_PAYLOAD_BYTES : remaining);

		frame[0] = FRAGMENT_VERSION;
		frame[1] = static_cast<uint8_t>((message_id >> 8) & 0xFFU);
		frame[2] = static_cast<uint8_t>(message_id & 0xFFU);
		frame[3] = static_cast<uint8_t>((total_length >> 8) & 0xFFU);
		frame[4] = static_cast<uint8_t>(total_length & 0xFFU);
		frame[5] = static_cast<uint8_t>((payload_crc16 >> 8) & 0xFFU);
		frame[6] = static_cast<uint8_t>(payload_crc16 & 0xFFU);
		frame[7] = fragment_index;
		frame[8] = fragment_count;
		frame[9] = fragment_length;

		for (uint32_t i = 0U; i < fragment_length; ++i)
		{
			frame[FRAGMENT_HEADER_BYTES + i] = payload[offset + i];
		}

		frame_length = FRAGMENT_HEADER_BYTES + fragment_length;
		return true;
	}

	bool SX1278::parse_fragment_(const Data& packet, FragmentHeader& header, const uint8_t*& payload_ptr) const
	{
		ASSERT(packet.length <= MAX_PAYLOAD_BYTES);
		ASSERT(payload_ptr == nullptr);

		if (packet.length < FRAGMENT_HEADER_BYTES)
		{
			return false;
		}

		header.version = packet.payload[0];
		header.message_id = static_cast<uint16_t>((static_cast<uint16_t>(packet.payload[1]) << 8) | packet.payload[2]);
		header.total_length = static_cast<uint16_t>((static_cast<uint16_t>(packet.payload[3]) << 8) | packet.payload[4]);
		header.payload_crc16 = static_cast<uint16_t>((static_cast<uint16_t>(packet.payload[5]) << 8) | packet.payload[6]);
		header.fragment_index = packet.payload[7];
		header.fragment_count = packet.payload[8];
		header.fragment_length = packet.payload[9];

		if ((header.version != FRAGMENT_VERSION) ||
		    (header.total_length == 0U) ||
		    (header.total_length > MAX_MESSAGE_BYTES) ||
		    (header.fragment_count == 0U) ||
		    (header.fragment_count > MAX_FRAGMENT_COUNT) ||
		    (header.fragment_index >= header.fragment_count) ||
		    (header.fragment_length == 0U) ||
		    (header.fragment_length > MAX_FRAGMENT_PAYLOAD_BYTES) ||
		    ((FRAGMENT_HEADER_BYTES + header.fragment_length) != packet.length))
		{
			return false;
		}

		payload_ptr = &packet.payload[FRAGMENT_HEADER_BYTES];
		return true;
	}

	void SX1278::reset_reassembly_()
	{
		reassembly_.bytes_received = 0U;
		reassembly_.total_length = 0U;
		reassembly_.payload_crc16 = 0U;
		reassembly_.message_id = 0U;
		reassembly_.fragment_count = 0U;
		reassembly_.next_fragment_index = 0U;
		reassembly_.active = false;
	}

	bool SX1278::accept_fragment_(const FragmentHeader& header, const uint8_t* payload_ptr)
	{
		ASSERT(payload_ptr != nullptr);
		ASSERT(header.fragment_count <= MAX_FRAGMENT_COUNT);

		if (payload_ptr == nullptr)
		{
			return false;
		}

		if (!reassembly_.active)
		{
			reassembly_.active = true;
			reassembly_.bytes_received = 0U;
			reassembly_.total_length = header.total_length;
			reassembly_.payload_crc16 = header.payload_crc16;
			reassembly_.message_id = header.message_id;
			reassembly_.fragment_count = header.fragment_count;
			reassembly_.next_fragment_index = 0U;
		}

		if ((header.message_id != reassembly_.message_id) ||
		    (header.total_length != reassembly_.total_length) ||
		    (header.payload_crc16 != reassembly_.payload_crc16) ||
		    (header.fragment_count != reassembly_.fragment_count) ||
		    (header.fragment_index != reassembly_.next_fragment_index) ||
		    ((reassembly_.bytes_received + header.fragment_length) > reassembly_.total_length))
		{
			return false;
		}

		for (uint32_t i = 0U; i < header.fragment_length; ++i)
		{
			reassembly_.payload[reassembly_.bytes_received + i] = payload_ptr[i];
		}

		reassembly_.bytes_received += header.fragment_length;
		reassembly_.next_fragment_index = static_cast<uint8_t>(reassembly_.next_fragment_index + 1U);

		if ((reassembly_.next_fragment_index == reassembly_.fragment_count) &&
		    (reassembly_.bytes_received != reassembly_.total_length))
		{
			return false;
		}

		return true;
	}

	uint16_t SX1278::crc16_ccitt_(const uint8_t* data, uint32_t length)
	{
		ASSERT(data != nullptr);
		ASSERT(length <= MAX_MESSAGE_BYTES);

		uint16_t crc = 0xFFFFU;
		for (uint32_t i = 0U; i < length; ++i)
		{
			crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
			for (uint8_t bit = 0U; bit < 8U; ++bit)
			{
				if ((crc & 0x8000U) != 0U)
				{
					crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
				}
				else
				{
					crc = static_cast<uint16_t>(crc << 1);
				}
			}
		}

		return crc;
	}
} // namespace drivers