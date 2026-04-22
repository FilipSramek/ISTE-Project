/**
 * @file sx1278.cpp
 * @brief SX1278 LoRa transceiver driver implementation.
 */

#include "drivers/sx1278/sx1278.hpp"

#include <stdio.h>
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

		printf("[SX1278] Initializing radio (freq=%lu Hz, SF=%u, BW=%u)\n",
		        config_.frequency_hz,
		        static_cast<uint32_t>(config_.spreading_factor),
		        static_cast<uint32_t>(config_.bandwidth));

		if (initialized_ || !is_valid_config_())
		{
			printf("[SX1278] Init failed: invalid config or already initialized\n");
			return false;
		}

		printf("[SX1278] DEBUG: Resetting radio...\n");
		if (!reset_radio_())
		{
			printf("[SX1278] DEBUG: Reset failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG: Reset done, setting mode to SLEEP...\n");
		
		if (!set_mode_(MODE_SLEEP))
		{
			printf("[SX1278] DEBUG: Failed to set mode to SLEEP!\n");
			return false;
		}
		printf("[SX1278] DEBUG: Mode set to SLEEP\n");

		// Test DIO0 pin
		printf("[SX1278] DEBUG: Testing DIO0 pin...\n");
		bool dio0_state = false;
		for (int i = 0; i < 5; i++)
		{
			if (!dio0_.read(dio0_state))
			{
				printf("[SX1278] DEBUG: DIO0 read failed!\n");
				return false;
			}
			printf("[SX1278] DEBUG: DIO0 state=%d (read %d/5)\n", dio0_state ? 1 : 0, i + 1);
			timer_.delay_ms(10);
		}

		printf("[SX1278] DEBUG: Reading VERSION register...\n");
		uint8_t version = 0U;
		if (!read_register_(Register::VERSION, version))
		{
			printf("[SX1278] Init failed: could not read version (SPI error)\n");
			return false;
		}

		printf("[SX1278] Version: 0x%02X (expected 0x%02X)\n", version, SX1278_VERSION);
		
		if (version == 0x00U)
		{
			printf("[SX1278] ERROR: Version is 0x00 - possible SPI connection issue!\n");
			printf("[SX1278] Check: NSS pin, MOSI/MISO/CLK, RESET pin, hardware connection\n");
			return false;
		}

		if ((version != SX1278_VERSION) || !configure_radio_())
		{
			printf("[SX1278] Init failed: version mismatch (got 0x%02X, expected 0x%02X)\n", version, SX1278_VERSION);
			return false;
		}

		initialized_ = true;
		current_data_.valid = false;
		reset_reassembly_();
		printf("[SX1278] Initialization complete\n");
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
			printf("[SX1278] RX error: CRC failure\n");
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
		if (data.valid)
		{
			printf("[SX1278] RX: %lu bytes, RSSI=%d dBm, SNR=%d cB\n",
			        data.length, data.rssi_dbm, data.snr_cdb);
		}
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
			printf("[SX1278] TX error: invalid parameters\n");
			return false;
		}

		printf("[SX1278] TX: sending %lu bytes\n", length);

		printf("[SX1278] DEBUG TX: entering standby...\n");
		if (!enter_standby_())
		{
			printf("[SX1278] DEBUG TX: enter_standby failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: standby OK\n");

		printf("[SX1278] DEBUG TX: clearing IRQ flags...\n");
		if (!clear_irq_flags_())
		{
			printf("[SX1278] DEBUG TX: clear_irq_flags failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: IRQ flags cleared\n");

		printf("[SX1278] DEBUG TX: setting FIFO_ADDR_PTR to 0x00...\n");
		if (!write_register_(Register::FIFO_ADDR_PTR, 0x00U))
		{
			printf("[SX1278] DEBUG TX: write FIFO_ADDR_PTR failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: FIFO_ADDR_PTR set\n");

		printf("[SX1278] DEBUG TX: writing %lu bytes to FIFO...\n", length);
		if (!write_burst_(Register::FIFO, payload, length))
		{
			printf("[SX1278] DEBUG TX: write_burst failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: payload written to FIFO\n");

		printf("[SX1278] DEBUG TX: setting PAYLOAD_LENGTH to %u...\n", static_cast<uint32_t>(length));
		if (!write_register_(Register::PAYLOAD_LENGTH, static_cast<uint8_t>(length)))
		{
			printf("[SX1278] DEBUG TX: write PAYLOAD_LENGTH failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: PAYLOAD_LENGTH set\n");

		printf("[SX1278] DEBUG TX: setting MODE_TX...\n");
		if (!set_mode_(MODE_TX))
		{
			printf("[SX1278] DEBUG TX: set_mode(MODE_TX) failed!\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: MODE_TX set\n");

		// Ověř OP_MODE register
		uint8_t op_mode = 0U;
		if (!read_register_(Register::OP_MODE, op_mode))
		{
			printf("[SX1278] DEBUG TX: failed to read OP_MODE\n");
			return false;
		}
		printf("[SX1278] DEBUG TX: OP_MODE verification read=0x%02X (MODE_TX=0x%02X)\n", 
		       op_mode, MODE_LONG_RANGE | MODE_TX);

		printf("[SX1278] DEBUG TX: MODE_TX set, waiting for TX_DONE (timeout=%u ms)...\n", config_.rx_timeout_ms);

		if (!wait_for_dio0_(config_.rx_timeout_ms))
		{
			printf("[SX1278] DEBUG TX: TX_DONE timeout! Checking IRQ_FLAGS anyway...\n");
		}

		uint8_t irq_flags = 0U;
		if (!read_register_(Register::IRQ_FLAGS, irq_flags))
		{
			printf("[SX1278] TX error: failed to read IRQ_FLAGS\n");
			(void)enter_standby_();
			return false;
		}
		printf("[SX1278] DEBUG TX: Final IRQ_FLAGS=0x%02X (TX_DONE=%d, RX_DONE=%d, CAD_DONE=%d)\n", 
		       irq_flags, 
		       (irq_flags & 0x08) ? 1 : 0,  // Bit 3 = TX_DONE
		       (irq_flags & 0x40) ? 1 : 0,  // Bit 6 = RX_DONE
		       (irq_flags & 0x04) ? 1 : 0); // Bit 2 = CAD_DONE

		const bool success = ((irq_flags & IRQ_TX_DONE) != 0U) && 
		                     clear_irq_flags_() && 
		                     enter_standby_();

		if (success)
		{
			printf("[SX1278] TX: transmission complete (irq_flags=0x%02X)\n", irq_flags);
		}
		else
		{
			printf("[SX1278] TX error: transmission failed (irq_flags=0x%02X, TX_DONE=%d)\n", 
			       irq_flags, ((irq_flags & IRQ_TX_DONE) != 0U) ? 1 : 0);
		}
		return success;
	}

	bool SX1278::send_message(const uint8_t* payload, uint32_t length)
	{
		ASSERT(initialized_);
		ASSERT(payload != nullptr);

		if ((!initialized_) || (payload == nullptr) || (length == 0U) || (length > MAX_MESSAGE_BYTES))
		{
			printf("[SX1278] Message TX error: invalid parameters\n");
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

		printf("[SX1278] Message TX: ID=%u, len=%lu, fragments=%u\n",
		        message_id, length, fragment_count);

		for (uint8_t fragment_index = 0U; fragment_index < fragment_count; ++fragment_index)
		{
			uint8_t frame[MAX_PAYLOAD_BYTES] = {0U};
			uint32_t frame_length = 0U;

			printf("[SX1278] Message TX: sending fragment %u/%u\n",
			        fragment_index + 1U, fragment_count);

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
				printf("[SX1278] Message TX error: fragment %u failed\n", fragment_index + 1U);
				return false;
			}

			offset += static_cast<uint32_t>(frame[9]);
		}

		printf("[SX1278] Message TX: complete\n");
		return true;
	}

	bool SX1278::read_message(uint8_t* payload, uint32_t capacity, uint32_t& length)
	{
		ASSERT(initialized_);
		ASSERT(payload != nullptr);

		length = 0U;
		if ((!initialized_) || (payload == nullptr) || (capacity == 0U) || (capacity > MAX_MESSAGE_BYTES))
		{
			printf("[SX1278] Message RX error: invalid parameters\n");
			return false;
		}

		printf("[SX1278] Message RX: starting reassembly (capacity=%lu)\n", capacity);
		reset_reassembly_();
		for (uint32_t i = 0U; i < MAX_FRAGMENT_COUNT; ++i)
		{
			Data packet;
			FragmentHeader header = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
			const uint8_t* fragment_payload = nullptr;

			if (!read(packet) || !parse_fragment_(packet, header, fragment_payload) || !accept_fragment_(header, fragment_payload))
			{
				printf("[SX1278] Message RX error: fragment parse/accept failed\n");
				reset_reassembly_();
				return false;
			}

			printf("[SX1278] Message RX: fragment %u/%u received (%lu bytes)\n",
			        reassembly_.next_fragment_index, reassembly_.fragment_count,
			        static_cast<uint32_t>(header.fragment_length));

			if (reassembly_.active && (reassembly_.next_fragment_index >= reassembly_.fragment_count))
			{
				if ((reassembly_.total_length > capacity) ||
				    (crc16_ccitt_(reassembly_.payload, reassembly_.total_length) != reassembly_.payload_crc16))
				{
					printf("[SX1278] Message RX error: capacity exceeded or CRC mismatch\n");
					reset_reassembly_();
					return false;
				}

				for (uint32_t j = 0U; j < reassembly_.total_length; ++j)
				{
					payload[j] = reassembly_.payload[j];
				}

				length = reassembly_.total_length;
				printf("[SX1278] Message RX: complete (%lu bytes)\n", length);
				reset_reassembly_();
				return true;
			}
		}

		printf("[SX1278] Message RX error: timeout waiting for fragments\n");
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

		if (reg == Register::OP_MODE)
		{
			printf("[SX1278] DEBUG write_register OP_MODE: tx[0]=0x%02X (0x01|0x80), tx[1]=0x%02X (value)\n", 
			       tx[0], tx[1]);
		}

		if (!nss_.write(false))
		{
			printf("[SX1278] DEBUG write_register: NSS low failed\n");
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, length);
		const bool deassert_ok = nss_.write(true);
		
		if (reg == Register::OP_MODE)
		{
			printf("[SX1278] DEBUG write_register OP_MODE: SPI ok=%d, deassert=%d\n", 
			       ok ? 1 : 0, deassert_ok ? 1 : 0);
		}
		
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
			printf("[SX1278] DEBUG read_register: NSS low failed\n");
			return false;
		}

		const bool ok = spi_.transfer(tx, rx, length);
		const bool deassert_ok = nss_.write(true);
		
		if (!ok)
		{
			printf("[SX1278] DEBUG read_register: SPI transfer failed for reg 0x%02X\n", static_cast<uint8_t>(reg));
		}
		if (!deassert_ok)
		{
			printf("[SX1278] DEBUG read_register: NSS high failed\n");
		}
		
		value = rx[1];
		if (reg == Register::VERSION)
		{
			printf("[SX1278] DEBUG read_register: VERSION - tx[0]=0x%02X, rx[1]=0x%02X (ok=%d, deassert=%d)\n",
			       tx[0], rx[1], ok ? 1 : 0, deassert_ok ? 1 : 0);
		}
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

		const uint8_t full_mode = static_cast<uint8_t>(MODE_LONG_RANGE | mode_bits);
		printf("[SX1278] DEBUG set_mode: mode_bits=0x%02X, MODE_LONG_RANGE=0x80, full_mode=0x%02X\n",
		       mode_bits, full_mode);
		
		const bool result = write_register_(Register::OP_MODE, full_mode);
		printf("[SX1278] DEBUG set_mode: write_register returned %d\n", result ? 1 : 0);
		
		return result;
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
		printf("[SX1278] DEBUG: wait_for_tx_done start - polling IRQ_FLAGS for TX_DONE, timeout=%u ms\n", timeout_ms);

		// Primárně čekáme na TX_DONE bit (bit 3) v IRQ_FLAGS registru
		// DIO0 pin není potřeba - Registry se čte přímo
		for (uint32_t i = 0U; i < MAX_IRQ_WAIT_ITERS; ++i)
		{
			uint8_t irq_flags = 0U;
			if (!read_register_(Register::IRQ_FLAGS, irq_flags))
			{
				printf("[SX1278] DEBUG: Failed to read IRQ_FLAGS at iteration %u\n", i);
				return false;
			}

			// Loguj každých 5000 iterací aby se log nezaplnil
			if (i % 5000 == 0)
			{
				printf("[SX1278] DEBUG: Iteration %u - IRQ_FLAGS=0x%02X (TX_DONE=%d, RX_DONE=%d, CAD_DONE=%d)\n", 
				       i, irq_flags, 
				       (irq_flags & 0x08) ? 1 : 0,  // Bit 3 = TX_DONE
				       (irq_flags & 0x40) ? 1 : 0,  // Bit 6 = RX_DONE
				       (irq_flags & 0x04) ? 1 : 0); // Bit 2 = CAD_DONE
			}

			// Čekáme na TX_DONE bit (bit 3)
			if ((irq_flags & 0x08) != 0U)
			{
				printf("[SX1278] DEBUG: TX_DONE bit set in IRQ_FLAGS at iteration %u (took ~%u ms)\n", 
				       i, timer_.elapsed_ms(start_time));
				return true;
			}

			// Pokud RX_DONE bit (bit 6) - pro RX mód
			if ((irq_flags & 0x40) != 0U)
			{
				printf("[SX1278] DEBUG: RX_DONE bit set in IRQ_FLAGS at iteration %u (took ~%u ms)\n", 
				       i, timer_.elapsed_ms(start_time));
				return true;
			}

			if (timer_.is_timeout_ms(start_time, timeout_ms))
			{
				printf("[SX1278] DEBUG: Timeout after %u ms (iteration %u/%u)\n", 
				       timeout_ms, i, MAX_IRQ_WAIT_ITERS);
				// Čteme IRQ_FLAGS ještě jednou na konci
				uint8_t irq_final = 0U;
				if (!read_register_(Register::IRQ_FLAGS, irq_final))
				{
					irq_final = 0xFF;
				}
				printf("[SX1278] DEBUG: Final IRQ_FLAGS=0x%02X at timeout (TX_DONE=%d, RX_DONE=%d)\n", 
				       irq_final, (irq_final & 0x08) ? 1 : 0, (irq_final & 0x40) ? 1 : 0);
				return false;
			}
		}

		printf("[SX1278] DEBUG: Exceeded MAX_IRQ_WAIT_ITERS (%u) without TX/RX_DONE or timeout\n", MAX_IRQ_WAIT_ITERS);
		return false;
	}

	bool SX1278::configure_radio_()
	{
		ASSERT(config_.sync_word != 0U);
		ASSERT(config_.preamble_length != 0U);

		printf("[SX1278] DEBUG: configure_radio start\n");

		if (!enter_standby_())
		{
			printf("[SX1278] DEBUG: enter_standby failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: enter_standby OK\n");

		if (!set_frequency_())
		{
			printf("[SX1278] DEBUG: set_frequency failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: set_frequency OK\n");

		if (!write_register_(Register::FIFO_TX_BASE_ADDR, 0x00U))
		{
			printf("[SX1278] DEBUG: write FIFO_TX_BASE_ADDR failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: FIFO_TX_BASE_ADDR OK\n");

		if (!write_register_(Register::FIFO_RX_BASE_ADDR, 0x00U))
		{
			printf("[SX1278] DEBUG: write FIFO_RX_BASE_ADDR failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: FIFO_RX_BASE_ADDR OK\n");

		if (!write_register_(Register::LNA, 0x23U))
		{
			printf("[SX1278] DEBUG: write LNA failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: LNA OK\n");

		if (!write_register_(Register::SYNC_WORD, config_.sync_word))
		{
			printf("[SX1278] DEBUG: write SYNC_WORD failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: SYNC_WORD OK\n");

		if (!write_register_(Register::DIO_MAPPING1, 0x00U))
		{
			printf("[SX1278] DEBUG: write DIO_MAPPING1 failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: DIO_MAPPING1 OK (0x00)\n");

		// Ověř DIO_MAPPING1 zpátky
		uint8_t dio_mapping1_read = 0U;
		if (!read_register_(Register::DIO_MAPPING1, dio_mapping1_read))
		{
			printf("[SX1278] DEBUG: read DIO_MAPPING1 failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: DIO_MAPPING1 verification read=0x%02X\n", dio_mapping1_read);

		if (!set_tx_power_())
		{
			printf("[SX1278] DEBUG: set_tx_power failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: set_tx_power OK\n");

		if (!configure_modem_())
		{
			printf("[SX1278] DEBUG: configure_modem failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: configure_modem OK\n");

		if (!clear_irq_flags_())
		{
			printf("[SX1278] DEBUG: clear_irq_flags failed\n");
			return false;
		}
		printf("[SX1278] DEBUG: clear_irq_flags OK\n");

		printf("[SX1278] DEBUG: configure_radio complete\n");
		return true;
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