/**
 * @file example_sx1278.cpp
 * @brief Example SX1278 telemetry TX/RX usage.
 */

#include "drivers/sx1278/sx1278.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"
#include "pico/stdlib.h"

namespace
{
	struct TelemetryPacket
	{
		int32_t altitude_mm;
		int32_t vertical_speed_mm_s;
		int32_t alpha_cdeg;
		int32_t beta_cdeg;
		uint32_t counter;
	};

	struct LargeTelemetryPacket
	{
		int32_t altitude_mm;
		int32_t vertical_speed_mm_s;
		int32_t alpha_cdeg;
		int32_t beta_cdeg;
		uint32_t counter;
		uint8_t log_bytes[300];
	};
}

void example_sx1278_transmitter()
{
	hal::PicoSPIBackend spi_backend;
	hal::SPI::Config spi_config = {
		.instance = spi0,
		.clk_pin = 18U,
		.mosi_pin = 19U,
		.miso_pin = 16U,
		.cs_pin = 17U,
		.baudrate = 1000000U
	};
	hal::SPI spi(spi_config, spi_backend);
	if (!spi.init())
	{
		return;
	}

	hal::PicoGPIOBackend gpio_backend_nss;
	hal::PicoGPIOBackend gpio_backend_reset;
	hal::PicoGPIOBackend gpio_backend_dio0;
	hal::GPIO nss({17U, true, true, false, false}, gpio_backend_nss);
	hal::GPIO reset({20U, true, true, false, false}, gpio_backend_reset);
	hal::GPIO dio0({21U, false, false, false, false}, gpio_backend_dio0);
	if (!nss.init() || !reset.init() || !dio0.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	drivers::SX1278::Config config = {
		.frequency_hz = 433000000U,
		.bandwidth = drivers::SX1278::Bandwidth::BW_125_KHZ,
		.coding_rate = drivers::SX1278::CodingRate::CR_4_5,
		.spreading_factor = drivers::SX1278::SpreadingFactor::SF7,
		.preamble_length = 8U,
		.sync_word = 0x12U,
		.tx_power_dbm = 17,
		.crc_enabled = true,
		.rx_timeout_ms = 100U
	};

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, config);
	if (!radio.init())
	{
		return;
	}

	TelemetryPacket packet = {12500, -350, 420, -30, 0U};
	constexpr uint32_t MAX_PACKETS = 10U;
	for (uint32_t i = 0U; i < MAX_PACKETS; ++i)
	{
		packet.counter = i;
		(void)radio.send_struct(packet);
		sleep_ms(100U);
	}
}

void example_sx1278_large_struct_transmitter()
{
	hal::PicoSPIBackend spi_backend;
	hal::SPI::Config spi_config = {
		.instance = spi0,
		.clk_pin = 18U,
		.mosi_pin = 19U,
		.miso_pin = 16U,
		.cs_pin = 17U,
		.baudrate = 1000000U
	};
	hal::SPI spi(spi_config, spi_backend);
	if (!spi.init())
	{
		return;
	}

	hal::PicoGPIOBackend gpio_backend_nss;
	hal::PicoGPIOBackend gpio_backend_reset;
	hal::PicoGPIOBackend gpio_backend_dio0;
	hal::GPIO nss({17U, true, true, false, false}, gpio_backend_nss);
	hal::GPIO reset({20U, true, true, false, false}, gpio_backend_reset);
	hal::GPIO dio0({21U, false, false, false, false}, gpio_backend_dio0);
	if (!nss.init() || !reset.init() || !dio0.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	drivers::SX1278::Config config = {
		.frequency_hz = 433000000U,
		.bandwidth = drivers::SX1278::Bandwidth::BW_125_KHZ,
		.coding_rate = drivers::SX1278::CodingRate::CR_4_5,
		.spreading_factor = drivers::SX1278::SpreadingFactor::SF7,
		.preamble_length = 8U,
		.sync_word = 0x12U,
		.tx_power_dbm = 17,
		.crc_enabled = true,
		.rx_timeout_ms = 100U
	};

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, config);
	if (!radio.init())
	{
		return;
	}

	LargeTelemetryPacket packet = {12500, -350, 420, -30, 0U, {0U}};
	for (uint32_t i = 0U; i < sizeof(packet.log_bytes); ++i)
	{
		packet.log_bytes[i] = static_cast<uint8_t>(i & 0xFFU);
	}

	constexpr uint32_t MAX_PACKETS = 10U;
	for (uint32_t i = 0U; i < MAX_PACKETS; ++i)
	{
		packet.counter = i;
		(void)radio.send_struct(packet);
		sleep_ms(100U);
	}
}

void example_sx1278_large_struct_receiver()
{
	hal::PicoSPIBackend spi_backend;
	hal::SPI::Config spi_config = {
		.instance = spi0,
		.clk_pin = 18U,
		.mosi_pin = 19U,
		.miso_pin = 16U,
		.cs_pin = 17U,
		.baudrate = 1000000U
	};
	hal::SPI spi(spi_config, spi_backend);
	if (!spi.init())
	{
		return;
	}

	hal::PicoGPIOBackend gpio_backend_nss;
	hal::PicoGPIOBackend gpio_backend_reset;
	hal::PicoGPIOBackend gpio_backend_dio0;
	hal::GPIO nss({17U, true, true, false, false}, gpio_backend_nss);
	hal::GPIO reset({20U, true, true, false, false}, gpio_backend_reset);
	hal::GPIO dio0({21U, false, false, false, false}, gpio_backend_dio0);
	if (!nss.init() || !reset.init() || !dio0.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	drivers::SX1278::Config config = {
		.frequency_hz = 433000000U,
		.bandwidth = drivers::SX1278::Bandwidth::BW_125_KHZ,
		.coding_rate = drivers::SX1278::CodingRate::CR_4_5,
		.spreading_factor = drivers::SX1278::SpreadingFactor::SF7,
		.preamble_length = 8U,
		.sync_word = 0x12U,
		.tx_power_dbm = 17,
		.crc_enabled = true,
		.rx_timeout_ms = 100U
	};

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, config);
	if (!radio.init())
	{
		return;
	}

	LargeTelemetryPacket packet = {0, 0, 0, 0, 0U, {0U}};
	if (radio.read_struct(packet))
	{
		const int32_t altitude = packet.altitude_mm;
		const int32_t alpha = packet.alpha_cdeg;
		const uint8_t first_log_byte = packet.log_bytes[0];
		(void)altitude;
		(void)alpha;
		(void)first_log_byte;
	}
}

void example_sx1278_receiver()
{
	hal::PicoSPIBackend spi_backend;
	hal::SPI::Config spi_config = {
		.instance = spi0,
		.clk_pin = 18U,
		.mosi_pin = 19U,
		.miso_pin = 16U,
		.cs_pin = 17U,
		.baudrate = 1000000U
	};
	hal::SPI spi(spi_config, spi_backend);
	if (!spi.init())
	{
		return;
	}

	hal::PicoGPIOBackend gpio_backend_nss;
	hal::PicoGPIOBackend gpio_backend_reset;
	hal::PicoGPIOBackend gpio_backend_dio0;
	hal::GPIO nss({17U, true, true, false, false}, gpio_backend_nss);
	hal::GPIO reset({20U, true, true, false, false}, gpio_backend_reset);
	hal::GPIO dio0({21U, false, false, false, false}, gpio_backend_dio0);
	if (!nss.init() || !reset.init() || !dio0.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	drivers::SX1278::Config config = {
		.frequency_hz = 433000000U,
		.bandwidth = drivers::SX1278::Bandwidth::BW_125_KHZ,
		.coding_rate = drivers::SX1278::CodingRate::CR_4_5,
		.spreading_factor = drivers::SX1278::SpreadingFactor::SF7,
		.preamble_length = 8U,
		.sync_word = 0x12U,
		.tx_power_dbm = 17,
		.crc_enabled = true,
		.rx_timeout_ms = 100U
	};

	drivers::SX1278 radio(spi, nss, reset, dio0, timer, config);
	if (!radio.init())
	{
		return;
	}

	TelemetryPacket packet = {0, 0, 0, 0, 0U};
	if (radio.read_struct(packet))
	{
		const int32_t altitude = packet.altitude_mm;
		const int32_t alpha = packet.alpha_cdeg;
		(void)altitude;
		(void)alpha;
	}
}