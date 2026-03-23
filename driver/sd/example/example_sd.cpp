/**
 * @file example_sd.cpp
 * @brief Example SD Card driver usage for telemetry storage.
 */

#include "driver/sd/sd.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/spi/spi.hpp"
#include "hal/timer/timer.hpp"
#include "pico/stdlib.h"

namespace
{
	/**
	 * @brief Large telemetry struct that spans multiple blocks when needed.
	 */
	struct TelemetryRecord
	{
		uint32_t timestamp_ms;
		int32_t altitude_mm;
		int32_t vertical_speed_mm_s;
		int32_t alpha_cdeg;
		int32_t beta_cdeg;
		int16_t temperature_cdeg;
		uint16_t voltage_mv;
		uint8_t status_flags;
		uint8_t reserved[485];
	};
}

void example_sd_write_telemetry()
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

	hal::PicoGPIOBackend cs_backend;
	hal::GPIO chip_select({17U, true, true, false, false}, cs_backend);
	if (!chip_select.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	hal::PicoSDCardBackend sd_backend(spi, chip_select, timer);
	hal::SDCard::Config config = {
		.block_size = 512U,
		.max_blocks = 65536U,
		.cmd_timeout_ms = 100U,
		.data_timeout_ms = 500U
	};
	hal::SDCard sd_card(config, sd_backend);

	if (!sd_card.init())
	{
		return;
	}

	uint32_t block_size = 0U;
	uint32_t capacity = 0U;
	if (!sd_card.get_block_size(block_size) || !sd_card.get_capacity(capacity))
	{
		return;
	}

	TelemetryRecord record = {
		.timestamp_ms = 0U,
		.altitude_mm = 12500,
		.vertical_speed_mm_s = -350,
		.alpha_cdeg = 420,
		.beta_cdeg = -30,
		.temperature_cdeg = 2500,
		.voltage_mv = 12000U,
		.status_flags = 0x01U,
		.reserved = {0U}
	};

	constexpr uint32_t MAX_RECORDS = 100U;
	for (uint32_t i = 0U; i < MAX_RECORDS; ++i)
	{
		record.timestamp_ms = i * 10U;
		if (!sd_card.write_struct(i, record))
		{
			return;
		}
		sleep_ms(10U);
	}
}

void example_sd_read_telemetry()
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

	hal::PicoGPIOBackend cs_backend;
	hal::GPIO chip_select({17U, true, true, false, false}, cs_backend);
	if (!chip_select.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	hal::PicoSDCardBackend sd_backend(spi, chip_select, timer);
	hal::SDCard::Config config = {
		.block_size = 512U,
		.max_blocks = 65536U,
		.cmd_timeout_ms = 100U,
		.data_timeout_ms = 500U
	};
	hal::SDCard sd_card(config, sd_backend);

	if (!sd_card.init())
	{
		return;
	}

	TelemetryRecord record = {0,  0, 0, 0, 0, 0, 0, 0, {0U}};

	if (sd_card.read_struct(0U, record))
	{
		const uint32_t timestamp = record.timestamp_ms;
		const int32_t altitude = record.altitude_mm;
		const int16_t temperature = record.temperature_cdeg;
		(void)timestamp;
		(void)altitude;
		(void)temperature;
	}
}

void example_sd_block_operations()
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

	hal::PicoGPIOBackend cs_backend;
	hal::GPIO chip_select({17U, true, true, false, false}, cs_backend);
	if (!chip_select.init())
	{
		return;
	}

	hal::PicoTimerBackend timer_backend;
	hal::Timer timer(timer_backend);

	hal::PicoSDCardBackend sd_backend(spi, chip_select, timer);
	hal::SDCard::Config config = {
		.block_size = 512U,
		.max_blocks = 65536U,
		.cmd_timeout_ms = 100U,
		.data_timeout_ms = 500U
	};
	hal::SDCard sd_card(config, sd_backend);

	if (!sd_card.init())
	{
		return;
	}

	uint8_t write_buffer[512] = {0U};
	for (uint32_t i = 0U; i < 512U; ++i)
	{
		write_buffer[i] = static_cast<uint8_t>(i & 0xFFU);
	}

	uint32_t blocks_written = 0U;
	if (!sd_card.write_blocks(0U, write_buffer, 1U, blocks_written))
	{
		return;
	}

	uint8_t read_buffer[512] = {0U};
	uint32_t blocks_read = 0U;
	if (!sd_card.read_blocks(0U, read_buffer, 1U, blocks_read))
	{
		return;
	}

	bool data_matches = true;
	for (uint32_t i = 0U; i < 512U; ++i)
	{
		if (read_buffer[i] != write_buffer[i])
		{
			data_matches = false;
			break;
		}
	}

	(void)data_matches;
}
