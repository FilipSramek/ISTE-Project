/**
 * @file sx1278.hpp
 * @brief SX1278 LoRa transceiver driver for telemetry packet TX/RX.
 */

#pragma once

#include <stdint.h>
#include "drivers/interfaces/ISensor.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"
#include "utils/assert/assert.hpp"

namespace drivers
{
	struct SX1278Packet
	{
		uint8_t payload[255];
		uint32_t length;
		int16_t rssi_dbm;
		int16_t snr_cdb;
		bool crc_error;
		bool valid;
	};

	class SX1278 final : public ITypedSensor<SX1278Packet>
	{
	public:
		using Data = SX1278Packet;

		enum class Bandwidth : uint8_t
		{
			BW_7_8_KHZ   = 0x00U,
			BW_10_4_KHZ  = 0x01U,
			BW_15_6_KHZ  = 0x02U,
			BW_20_8_KHZ  = 0x03U,
			BW_31_25_KHZ = 0x04U,
			BW_41_7_KHZ  = 0x05U,
			BW_62_5_KHZ  = 0x06U,
			BW_125_KHZ   = 0x07U,
			BW_250_KHZ   = 0x08U,
			BW_500_KHZ   = 0x09U
		};

		enum class CodingRate : uint8_t
		{
			CR_4_5 = 0x01U,
			CR_4_6 = 0x02U,
			CR_4_7 = 0x03U,
			CR_4_8 = 0x04U
		};

		enum class SpreadingFactor : uint8_t
		{
			SF6  = 6U,
			SF7  = 7U,
			SF8  = 8U,
			SF9  = 9U,
			SF10 = 10U,
			SF11 = 11U,
			SF12 = 12U
		};

		struct Config
		{
			uint32_t frequency_hz;
			Bandwidth bandwidth;
			CodingRate coding_rate;
			SpreadingFactor spreading_factor;
			uint16_t preamble_length;
			uint8_t sync_word;
			int8_t tx_power_dbm;
			bool crc_enabled;
			uint32_t rx_timeout_ms;
		};

		SX1278(hal::SPI& spi,
		       hal::GPIO& nss,
		       hal::GPIO& reset,
		       hal::GPIO& dio0,
		       hal::Timer& timer,
		       const Config& config);

		bool init() override;
		bool read(Data& data) override;
		bool update() override;
		bool get_data(Data& data) const override;
		bool is_initialized() const override;

		bool send(const uint8_t* payload, uint32_t length);
		bool send_message(const uint8_t* payload, uint32_t length);
		bool read_message(uint8_t* payload, uint32_t capacity, uint32_t& length);

		template<typename T>
		bool send_struct(const T& packet)
		{
			const uint32_t length = static_cast<uint32_t>(sizeof(T));
			ASSERT(length <= MAX_MESSAGE_BYTES);
			ASSERT(is_initialized());

			if (length > MAX_MESSAGE_BYTES)
			{
				return false;
			}

			return send_message(reinterpret_cast<const uint8_t*>(&packet), length);
		}

		template<typename T>
		bool read_struct(T& packet)
		{
			uint32_t length = 0U;
			ASSERT(sizeof(T) <= MAX_MESSAGE_BYTES);
			ASSERT(is_initialized());

			if (!read_message(reinterpret_cast<uint8_t*>(&packet), static_cast<uint32_t>(sizeof(T)), length))
			{
				return false;
			}

			return (length == static_cast<uint32_t>(sizeof(T)));
		}

	private:
		struct FragmentHeader
		{
			uint8_t version;
			uint16_t message_id;
			uint16_t total_length;
			uint16_t payload_crc16;
			uint8_t fragment_index;
			uint8_t fragment_count;
			uint8_t fragment_length;
		};

		struct ReassemblyState
		{
			uint8_t payload[1024];
			uint32_t bytes_received;
			uint16_t total_length;
			uint16_t payload_crc16;
			uint16_t message_id;
			uint8_t fragment_count;
			uint8_t next_fragment_index;
			bool active;
		};

		enum class Register : uint8_t
		{
			FIFO                 = 0x00U,
			OP_MODE              = 0x01U,
			FRF_MSB              = 0x06U,
			FRF_MID              = 0x07U,
			FRF_LSB              = 0x08U,
			PA_CONFIG            = 0x09U,
			LNA                  = 0x0CU,
			FIFO_ADDR_PTR        = 0x0DU,
			FIFO_TX_BASE_ADDR    = 0x0EU,
			FIFO_RX_BASE_ADDR    = 0x0FU,
			FIFO_RX_CURRENT_ADDR = 0x10U,
			IRQ_FLAGS            = 0x12U,
			RX_NB_BYTES          = 0x13U,
			PKT_SNR_VALUE        = 0x19U,
			PKT_RSSI_VALUE       = 0x1AU,
			MODEM_CONFIG1        = 0x1DU,
			MODEM_CONFIG2        = 0x1EU,
			PREAMBLE_MSB         = 0x20U,
			PREAMBLE_LSB         = 0x21U,
			PAYLOAD_LENGTH       = 0x22U,
			MODEM_CONFIG3        = 0x26U,
			DETECTION_OPTIMIZE   = 0x31U,
			SYNC_WORD            = 0x39U,
			DIO_MAPPING1         = 0x40U,
			VERSION              = 0x42U,
			PA_DAC               = 0x4DU
		};

		bool is_valid_config_() const;
		bool reset_radio_();
		bool write_register_(Register reg, uint8_t value);
		bool read_register_(Register reg, uint8_t& value);
		bool write_burst_(Register reg, const uint8_t* data, uint32_t length);
		bool read_burst_(Register reg, uint8_t* data, uint32_t length);
		bool set_mode_(uint8_t mode_bits);
		bool enter_standby_();
		bool clear_irq_flags_();
		bool wait_for_dio0_(uint32_t timeout_ms);
		bool configure_radio_();
		bool set_frequency_();
		bool set_tx_power_();
		bool configure_modem_();
		bool read_packet_metrics_(int16_t& rssi_dbm, int16_t& snr_cdb);
		bool build_fragment_(const uint8_t* payload,
		                     uint32_t total_length,
		                     uint16_t message_id,
		                     uint16_t payload_crc16,
		                     uint8_t fragment_index,
		                     uint8_t fragment_count,
		                     uint32_t offset,
		                     uint8_t* frame,
		                     uint32_t& frame_length) const;
		bool parse_fragment_(const Data& packet, FragmentHeader& header, const uint8_t*& payload_ptr) const;
		void reset_reassembly_();
		bool accept_fragment_(const FragmentHeader& header, const uint8_t* payload_ptr);
		static uint16_t crc16_ccitt_(const uint8_t* data, uint32_t length);

		hal::SPI& spi_;
		hal::GPIO& nss_;
		hal::GPIO& reset_;
		hal::GPIO& dio0_;
		hal::Timer& timer_;
		Config config_;
		Data current_data_;
		ReassemblyState reassembly_;
		uint16_t next_message_id_;
		bool initialized_;

		static constexpr uint8_t REG_WRITE_MASK = 0x80U;
		static constexpr uint8_t REG_READ_MASK = 0x7FU;
		static constexpr uint8_t MODE_LONG_RANGE = 0x80U;
		static constexpr uint8_t MODE_SLEEP = 0x00U;
		static constexpr uint8_t MODE_STDBY = 0x01U;
		static constexpr uint8_t MODE_TX = 0x03U;
		static constexpr uint8_t MODE_RX_SINGLE = 0x06U;
		static constexpr uint8_t PA_BOOST_ENABLE = 0x80U;
		static constexpr uint8_t IRQ_RX_DONE = 0x40U;
		static constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR = 0x20U;
		static constexpr uint8_t IRQ_TX_DONE = 0x08U;
		static constexpr uint8_t SX1278_VERSION = 0x12U;
		static constexpr uint32_t MAX_PAYLOAD_BYTES = 255U;
		static constexpr uint32_t MAX_MESSAGE_BYTES = 1024U;
		static constexpr uint8_t FRAGMENT_VERSION = 1U;
		static constexpr uint32_t FRAGMENT_HEADER_BYTES = 10U;
		static constexpr uint32_t MAX_FRAGMENT_PAYLOAD_BYTES = MAX_PAYLOAD_BYTES - FRAGMENT_HEADER_BYTES;
		static constexpr uint32_t MAX_FRAGMENT_COUNT = (MAX_MESSAGE_BYTES + MAX_FRAGMENT_PAYLOAD_BYTES - 1U) / MAX_FRAGMENT_PAYLOAD_BYTES;
		static constexpr uint32_t MAX_IRQ_WAIT_ITERS = 200000U;
		static constexpr uint32_t RESET_PULSE_MS = 1U;
		static constexpr uint32_t RESET_SETTLE_MS = 10U;
	};
} // namespace drivers