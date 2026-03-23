/**
 * @file test_sx1278.cpp
 * @brief Unit tests for the SX1278 LoRa driver.
 */

#include "drivers/sx1278/sx1278.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/interfaces/IGPIOBackend.hpp"
#include "hal/interfaces/ITimerBackend.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"
#include "pico/stdlib.h"

#include <cstdio>

namespace
{
	constexpr uint32_t MAX_PAYLOAD_BYTES = 255U;
	constexpr uint32_t MAX_MESSAGE_BYTES = 1024U;
	constexpr uint32_t FRAGMENT_HEADER_BYTES = 10U;
	constexpr uint32_t MAX_FRAGMENT_PAYLOAD_BYTES = MAX_PAYLOAD_BYTES - FRAGMENT_HEADER_BYTES;
	constexpr uint32_t MAX_FRAMES = 8U;

	struct MockRadioState
	{
		bool dio0_high;
		bool simulate_crc_error;
		uint8_t rx_payloads[MAX_FRAMES][MAX_PAYLOAD_BYTES];
		uint8_t rx_lengths[MAX_FRAMES];
		uint8_t rx_frame_count;
		uint8_t rx_frame_index;
		uint8_t active_rx_frame;
		bool rx_frame_loaded;
		int8_t snr_qdb;
		uint8_t rssi_raw;
	};

	class MockGPIOBackend final : public hal::IGPIOBackend
	{
	public:
		explicit MockGPIOBackend(MockRadioState* state = nullptr)
			: state_(state), last_written_(true), is_output_(false)
		{
		}

		void init(uint8_t pin) override { (void)pin; }
		void set_dir(uint8_t pin, bool is_output) override { (void)pin; is_output_ = is_output; }
		void set_pull(uint8_t pin, bool pull_up, bool pull_down) override { (void)pin; (void)pull_up; (void)pull_down; }
		void write(uint8_t pin, bool value) override { (void)pin; last_written_ = value; }
		bool read(uint8_t pin) override
		{
			(void)pin;
			if (state_ != nullptr)
			{
				return state_->dio0_high;
			}
			return last_written_;
		}
		void toggle(uint8_t pin) override { (void)pin; last_written_ = !last_written_; }

		MockRadioState* state_;
		bool last_written_;
		bool is_output_;
	};

	class MockTimerBackend final : public hal::ITimerBackend
	{
	public:
		MockTimerBackend() : time_us_(0U), advance_us_(500U) {}
		void init() override {}
		uint64_t get_time_us() override { time_us_ += advance_us_; return time_us_; }
		uint64_t get_elapsed_us(uint64_t start_time) override { time_us_ += advance_us_; return time_us_ - start_time; }
		void delay_us(uint32_t us) override { time_us_ += us; }
		void delay_ms(uint32_t ms) override { time_us_ += static_cast<uint64_t>(ms) * 1000ULL; }
		bool is_timeout(uint64_t start_time, uint64_t timeout_us) override
		{
			time_us_ += advance_us_;
			return ((time_us_ - start_time) >= timeout_us);
		}

		uint64_t time_us_;
		uint64_t advance_us_;
	};

	class MockSPIBackend final : public hal::ISPIBackend
	{
	public:
		explicit MockSPIBackend(MockRadioState& state)
			: state_(state),
			  tx_frame_count_(0U),
			  pending_tx_length_(0U),
			  fifo_rx_addr_(0U)
		{
			for (uint32_t i = 0U; i < 128U; ++i)
			{
				registers_[i] = 0U;
			}

			for (uint32_t frame = 0U; frame < MAX_FRAMES; ++frame)
			{
				tx_lengths_[frame] = 0U;
				for (uint32_t i = 0U; i < MAX_PAYLOAD_BYTES; ++i)
				{
					pending_tx_[i] = 0U;
					tx_payloads_[frame][i] = 0U;
				}
			}

			registers_[0x42U] = 0x12U;
		}

		void init(spi_inst_t* instance, uint32_t baudrate) override { (void)instance; (void)baudrate; }
		void set_pins(uint8_t clk_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t cs_pin) override
		{
			(void)clk_pin;
			(void)mosi_pin;
			(void)miso_pin;
			(void)cs_pin;
		}

		int write(spi_inst_t* instance, const uint8_t* data, uint32_t length) override
		{
			(void)instance;
			(void)data;
			(void)length;
			return -1;
		}

		int read(spi_inst_t* instance, uint8_t* buffer, uint32_t length) override
		{
			(void)instance;
			(void)buffer;
			(void)length;
			return -1;
		}

		int transfer(spi_inst_t* instance, const uint8_t* tx_data, uint8_t* rx_buffer, uint32_t length) override
		{
			(void)instance;
			if ((tx_data == nullptr) || (rx_buffer == nullptr) || (length == 0U))
			{
				return -1;
			}

			for (uint32_t i = 0U; i < length; ++i)
			{
				rx_buffer[i] = 0U;
			}

			const uint8_t address = static_cast<uint8_t>(tx_data[0] & 0x7FU);
			const bool is_write = ((tx_data[0] & 0x80U) != 0U);
			if (is_write)
			{
				return handle_write_(address, tx_data, length);
			}

			return handle_read_(address, rx_buffer, length);
		}

		int handle_write_(uint8_t address, const uint8_t* tx_data, uint32_t length)
		{
			if (address == 0x00U)
			{
				pending_tx_length_ = static_cast<uint8_t>(length - 1U);
				for (uint32_t i = 1U; i < length; ++i)
				{
					pending_tx_[i - 1U] = tx_data[i];
				}
				return static_cast<int>(length);
			}

			if (length >= 2U)
			{
				registers_[address] = tx_data[1];
			}

			if (address == 0x01U)
			{
				const uint8_t mode = static_cast<uint8_t>(tx_data[1] & 0x07U);
				if (mode == 0x03U)
				{
					if (tx_frame_count_ < MAX_FRAMES)
					{
						tx_lengths_[tx_frame_count_] = pending_tx_length_;
						for (uint32_t i = 0U; i < pending_tx_length_; ++i)
						{
							tx_payloads_[tx_frame_count_][i] = pending_tx_[i];
						}
						++tx_frame_count_;
					}

					registers_[0x12U] = 0x08U;
					state_.dio0_high = true;
				}

				if ((mode == 0x06U) && (state_.rx_frame_index < state_.rx_frame_count))
				{
					state_.active_rx_frame = state_.rx_frame_index;
					state_.rx_frame_loaded = true;
					registers_[0x10U] = fifo_rx_addr_;
					registers_[0x13U] = state_.rx_lengths[state_.active_rx_frame];
					registers_[0x19U] = static_cast<uint8_t>(state_.snr_qdb);
					registers_[0x1AU] = state_.rssi_raw;
					registers_[0x12U] = state_.simulate_crc_error ? 0x60U : 0x40U;
					state_.dio0_high = true;
				}
			}

			if (address == 0x0DU)
			{
				fifo_rx_addr_ = tx_data[1];
			}

			if ((address == 0x12U) && (length >= 2U) && (tx_data[1] == 0xFFU))
			{
				registers_[0x12U] = 0x00U;
				state_.dio0_high = false;
				if (state_.rx_frame_loaded)
				{
					state_.rx_frame_loaded = false;
					if (state_.rx_frame_index < state_.rx_frame_count)
					{
						++state_.rx_frame_index;
					}
				}
			}

			return static_cast<int>(length);
		}

		int handle_read_(uint8_t address, uint8_t* rx_buffer, uint32_t length)
		{
			if (address == 0x00U)
			{
				const uint8_t frame_index = state_.active_rx_frame;
				for (uint32_t i = 1U; i < length; ++i)
				{
					rx_buffer[i] = state_.rx_payloads[frame_index][i - 1U];
				}
				return static_cast<int>(length);
			}

			if (length >= 2U)
			{
				rx_buffer[1] = registers_[address];
			}
			return static_cast<int>(length);
		}

		MockRadioState& state_;
		uint8_t registers_[128];
		uint8_t pending_tx_[MAX_PAYLOAD_BYTES];
		uint8_t pending_tx_length_;
		uint8_t tx_payloads_[MAX_FRAMES][MAX_PAYLOAD_BYTES];
		uint8_t tx_lengths_[MAX_FRAMES];
		uint8_t tx_frame_count_;
		uint8_t fifo_rx_addr_;
	};

	static hal::SPI make_spi(MockSPIBackend& backend)
	{
		const hal::SPI::Config config = {
			reinterpret_cast<spi_inst_t*>(0x1),
			18U,
			19U,
			16U,
			17U,
			1000000U
		};

		hal::SPI spi(config, backend);
		(void)spi.init();
		return spi;
	}

	static drivers::SX1278::Config make_config()
	{
		drivers::SX1278::Config config = {
			433000000U,
			drivers::SX1278::Bandwidth::BW_125_KHZ,
			drivers::SX1278::CodingRate::CR_4_5,
			drivers::SX1278::SpreadingFactor::SF7,
			8U,
			0x12U,
			17,
			true,
			50U
		};
		return config;
	}

	static uint16_t crc16_ccitt(const uint8_t* data, uint32_t length)
	{
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

	template<typename T>
	void copy_bytes(uint8_t* destination, const T& source)
	{
		const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&source);
		for (uint32_t i = 0U; i < sizeof(T); ++i)
		{
			destination[i] = bytes[i];
		}
	}

	struct TelemetryStruct
	{
		int32_t altitude_mm;
		int32_t velocity_mm_s;
		uint32_t counter;
	};

	struct LargeTelemetryStruct
	{
		uint32_t counter;
		uint8_t payload[300];
	};

	void enqueue_frame(MockRadioState& state, uint8_t frame_index, const uint8_t* data, uint8_t length)
	{
		state.rx_lengths[frame_index] = length;
		for (uint32_t i = 0U; i < length; ++i)
		{
			state.rx_payloads[frame_index][i] = data[i];
		}
	}

	void load_fragment_frames(MockRadioState& state, const uint8_t* message, uint32_t message_length, uint16_t message_id)
	{
		const uint16_t payload_crc = crc16_ccitt(message, message_length);
		const uint8_t fragment_count = static_cast<uint8_t>((message_length + MAX_FRAGMENT_PAYLOAD_BYTES - 1U) / MAX_FRAGMENT_PAYLOAD_BYTES);
		uint32_t offset = 0U;

		state.rx_frame_count = fragment_count;
		for (uint8_t fragment_index = 0U; fragment_index < fragment_count; ++fragment_index)
		{
			const uint32_t remaining = message_length - offset;
			const uint8_t fragment_length = static_cast<uint8_t>((remaining > MAX_FRAGMENT_PAYLOAD_BYTES) ? MAX_FRAGMENT_PAYLOAD_BYTES : remaining);
			uint8_t frame[MAX_PAYLOAD_BYTES] = {0U};

			frame[0] = 1U;
			frame[1] = static_cast<uint8_t>((message_id >> 8) & 0xFFU);
			frame[2] = static_cast<uint8_t>(message_id & 0xFFU);
			frame[3] = static_cast<uint8_t>((message_length >> 8) & 0xFFU);
			frame[4] = static_cast<uint8_t>(message_length & 0xFFU);
			frame[5] = static_cast<uint8_t>((payload_crc >> 8) & 0xFFU);
			frame[6] = static_cast<uint8_t>(payload_crc & 0xFFU);
			frame[7] = fragment_index;
			frame[8] = fragment_count;
			frame[9] = fragment_length;

			for (uint32_t i = 0U; i < fragment_length; ++i)
			{
				frame[FRAGMENT_HEADER_BYTES + i] = message[offset + i];
			}

			enqueue_frame(state, fragment_index, frame, static_cast<uint8_t>(FRAGMENT_HEADER_BYTES + fragment_length));
			offset += fragment_length;
		}
	}

	bool make_radio(MockRadioState& state,
	                MockSPIBackend& spi_backend,
	                MockGPIOBackend& nss_backend,
	                MockGPIOBackend& reset_backend,
	                MockGPIOBackend& dio0_backend,
	                MockTimerBackend& timer_backend,
	                drivers::SX1278& radio)
	{
		(void)state;
		(void)spi_backend;
		(void)nss_backend;
		(void)reset_backend;
		(void)dio0_backend;
		(void)timer_backend;
		return radio.init();
	}
}

bool test_sx1278_init()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 0, 0U};
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	return make_radio(state, spi_backend, nss_backend, reset_backend, dio0_backend, timer_backend, radio) && radio.is_initialized();
}

bool test_sx1278_send_bytes()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 0, 0U};
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	const uint8_t payload[4] = {1U, 2U, 3U, 4U};
	return radio.send(payload, 4U) &&
	       (spi_backend.tx_frame_count_ == 1U) &&
	       (spi_backend.tx_lengths_[0] == 4U) &&
	       (spi_backend.tx_payloads_[0][0] == 1U) &&
	       (spi_backend.tx_payloads_[0][3] == 4U);
}

bool test_sx1278_send_struct()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 0, 0U};
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	TelemetryStruct packet = {1000, -42, 7U};
	return radio.send_struct(packet) &&
	       (spi_backend.tx_frame_count_ == 1U) &&
	       (spi_backend.tx_lengths_[0] == (sizeof(TelemetryStruct) + FRAGMENT_HEADER_BYTES)) &&
	       (spi_backend.tx_payloads_[0][0] == 1U) &&
	       (spi_backend.tx_payloads_[0][7] == 0U) &&
	       (spi_backend.tx_payloads_[0][8] == 1U);
}

bool test_sx1278_send_large_struct_fragmented()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 0, 0U};
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	LargeTelemetryStruct packet = {17U, {0U}};
	for (uint32_t i = 0U; i < sizeof(packet.payload); ++i)
	{
		packet.payload[i] = static_cast<uint8_t>(i & 0xFFU);
	}

	return radio.send_struct(packet) &&
	       (spi_backend.tx_frame_count_ == 2U) &&
	       (spi_backend.tx_payloads_[0][7] == 0U) &&
	       (spi_backend.tx_payloads_[1][7] == 1U) &&
	       (spi_backend.tx_payloads_[0][8] == 2U) &&
	       (spi_backend.tx_payloads_[1][8] == 2U);
}

bool test_sx1278_receive_packet()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 1U, 0U, 0U, false, 8, 200U};
	state.rx_payloads[0][0] = 0x10U;
	state.rx_payloads[0][1] = 0x20U;
	state.rx_payloads[0][2] = 0x30U;
	state.rx_lengths[0] = 3U;
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	drivers::SX1278::Data data;
	if (!radio.read(data))
	{
		return false;
	}

	return data.valid && (data.length == 3U) && (data.payload[2] == 0x30U) && (data.rssi_dbm == 43) && (data.snr_cdb == 200);
}

bool test_sx1278_receive_struct()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 4, 190U};
	TelemetryStruct source = {2222, -11, 55U};
	uint8_t frame[MAX_PAYLOAD_BYTES] = {0U};
	const uint8_t* src = reinterpret_cast<const uint8_t*>(&source);
	const uint16_t payload_crc = crc16_ccitt(src, sizeof(TelemetryStruct));

	frame[0] = 1U;
	frame[1] = 0U;
	frame[2] = 1U;
	frame[3] = 0U;
	frame[4] = static_cast<uint8_t>(sizeof(TelemetryStruct));
	frame[5] = static_cast<uint8_t>((payload_crc >> 8) & 0xFFU);
	frame[6] = static_cast<uint8_t>(payload_crc & 0xFFU);
	frame[7] = 0U;
	frame[8] = 1U;
	frame[9] = static_cast<uint8_t>(sizeof(TelemetryStruct));
	for (uint32_t i = 0U; i < sizeof(TelemetryStruct); ++i)
	{
		frame[FRAGMENT_HEADER_BYTES + i] = src[i];
	}
	enqueue_frame(state, 0U, frame, static_cast<uint8_t>(FRAGMENT_HEADER_BYTES + sizeof(TelemetryStruct)));
	state.rx_frame_count = 1U;

	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	TelemetryStruct packet = {0, 0, 0U};
	return radio.read_struct(packet) &&
	       (packet.altitude_mm == 2222) &&
	       (packet.velocity_mm_s == -11) &&
	       (packet.counter == 55U);
}

bool test_sx1278_receive_large_struct_reassembled()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 4, 190U};
	LargeTelemetryStruct source = {91U, {0U}};
	for (uint32_t i = 0U; i < sizeof(source.payload); ++i)
	{
		source.payload[i] = static_cast<uint8_t>((i * 3U) & 0xFFU);
	}
	load_fragment_frames(state,
	                     reinterpret_cast<const uint8_t*>(&source),
	                     static_cast<uint32_t>(sizeof(LargeTelemetryStruct)),
	                     9U);

	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	LargeTelemetryStruct packet = {0U, {0U}};
	return radio.read_struct(packet) &&
	       (packet.counter == source.counter) &&
	       (packet.payload[0] == source.payload[0]) &&
	       (packet.payload[299] == source.payload[299]);
}

bool test_sx1278_crc_error()
{
	MockRadioState state = {false, true, {{0U}}, {0U}, 1U, 0U, 0U, false, 0, 180U};
	state.rx_payloads[0][0] = 1U;
	state.rx_payloads[0][1] = 2U;
	state.rx_lengths[0] = 2U;
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, make_config());
	if (!radio.init())
	{
		return false;
	}

	drivers::SX1278::Data data;
	return !radio.read(data) && !data.valid;
}

bool test_sx1278_timeout()
{
	MockRadioState state = {false, false, {{0U}}, {0U}, 0U, 0U, 0U, false, 0, 0U};
	MockSPIBackend spi_backend(state);
	MockGPIOBackend nss_backend;
	MockGPIOBackend reset_backend;
	MockGPIOBackend dio0_backend(&state);
	MockTimerBackend timer_backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO nss({17U, true, true, false, false}, nss_backend);
	hal::GPIO reset({20U, true, true, false, false}, reset_backend);
	hal::GPIO dio0({21U, false, false, false, false}, dio0_backend);
	hal::Timer timer(timer_backend);
	(void)nss.init();
	(void)reset.init();
	(void)dio0.init();

	drivers::SX1278::Config config = make_config();
	config.rx_timeout_ms = 2U;
	drivers::SX1278 radio(spi, nss, reset, dio0, timer, config);
	if (!radio.init())
	{
		return false;
	}

	drivers::SX1278::Data data;
	return !radio.read(data) && !data.valid;
}

int run_sx1278_tests()
{
	bool all_passed = true;
	all_passed = all_passed && test_sx1278_init();
	all_passed = all_passed && test_sx1278_send_bytes();
	all_passed = all_passed && test_sx1278_send_struct();
	all_passed = all_passed && test_sx1278_send_large_struct_fragmented();
	all_passed = all_passed && test_sx1278_receive_packet();
	all_passed = all_passed && test_sx1278_receive_struct();
	all_passed = all_passed && test_sx1278_receive_large_struct_reassembled();
	all_passed = all_passed && test_sx1278_crc_error();
	all_passed = all_passed && test_sx1278_timeout();
	return all_passed ? 0 : 1;
}

int main()
{
	stdio_init_all();
	sleep_ms(2000);

	const int result = run_sx1278_tests();
	printf("TEST SX1278: %s\n", (result == 0) ? "PASS" : "FAIL");
	fflush(stdout);
	sleep_ms(100);

	return result;
}