/**
 * @file test_sd.cpp
 * @brief Unit tests for the SD Card driver.
 */

#include "drivers/sd/sd.hpp"
#include "pico/stdlib.h"

#include <cstdio>

namespace
{
	class MockSPIBackend final : public hal::ISPIBackend
	{
	public:
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
			if ((data == nullptr) || (length == 0U))
			{
				return -1;
			}

			for (uint32_t i = 0U; i < length && i < 512U; ++i)
			{
				last_tx_[i] = data[i];
			}
			last_tx_length_ = length;
			return static_cast<int>(length);
		}

		int read(spi_inst_t* instance, uint8_t* buffer, uint32_t length) override
		{
			(void)instance;
			if ((buffer == nullptr) || (length == 0U))
			{
				return -1;
			}

			for (uint32_t i = 0U; i < length && i < 512U; ++i)
			{
				buffer[i] = simulated_rx_[i];
			}
			return static_cast<int>(length);
		}

		int transfer(spi_inst_t* instance,
		             const uint8_t* tx_data,
		             uint8_t* rx_buffer,
		             uint32_t length) override
		{
			(void)instance;
			if ((tx_data == nullptr) || (rx_buffer == nullptr) || (length == 0U))
			{
				return -1;
			}

			for (uint32_t i = 0U; i < length && i < 512U; ++i)
			{
				last_tx_[i] = tx_data[i];
				rx_buffer[i] = simulated_rx_[i];
			}
			last_tx_length_ = length;
			return static_cast<int>(length);
		}

		uint8_t last_tx_[512];
		uint32_t last_tx_length_;
		uint8_t simulated_rx_[512];
	};

	class MockGPIOBackend final : public hal::IGPIOBackend
	{
	public:
		void init(uint8_t pin) override { (void)pin; }
		void set_dir(uint8_t pin, bool is_output) override { (void)pin; (void)is_output; }
		void set_pull(uint8_t pin, bool pull_up, bool pull_down) override { (void)pin; (void)pull_up; (void)pull_down; }
		void write(uint8_t pin, bool value) override { (void)pin; last_written_ = value; }
		bool read(uint8_t pin) override { (void)pin; return last_written_; }
		void toggle(uint8_t pin) override { (void)pin; last_written_ = !last_written_; }

		bool last_written_;
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

	class MockSDCardBackend final : public hal::ISDCardBackend
	{
	public:
		void init() override { initialized_ = true; }
		bool send_command(uint8_t cmd, uint32_t arg, uint8_t& response) override
		{
			(void)cmd;
			(void)arg;
			response = 0x00U;
			return true;
		}

		bool read_block(uint32_t block_address, uint8_t* buffer, uint32_t block_size) override
		{
			if ((buffer == nullptr) || (block_size != 512U))
			{
				return false;
			}

			const uint32_t index = block_address % 100U;
			for (uint32_t i = 0U; i < 512U; ++i)
			{
				buffer[i] = simulated_blocks[index][i];
			}

			return true;
		}

		bool write_block(uint32_t block_address, const uint8_t* buffer, uint32_t block_size) override
		{
			if ((buffer == nullptr) || (block_size != 512U))
			{
				return false;
			}

			const uint32_t index = block_address % 100U;
			for (uint32_t i = 0U; i < 512U; ++i)
			{
				simulated_blocks[index][i] = buffer[i];
			}

			return true;
		}

		bool is_ready() override { return initialized_; }

		bool initialized_;
		uint8_t simulated_blocks[100][512];

		MockSDCardBackend() : initialized_(false)
		{
			for (uint32_t block = 0U; block < 100U; ++block)
			{
				for (uint32_t i = 0U; i < 512U; ++i)
				{
					simulated_blocks[block][i] = 0U;
				}
			}
		}
	};

	struct TestTelemetry
	{
		uint32_t counter;
		int32_t altitude_mm;
		int32_t velocity_mm_s;
		uint16_t temperature_cdeg;
		uint8_t status;
		uint8_t padding[493];
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

	static hal::GPIO make_gpio(MockGPIOBackend& backend)
	{
		hal::GPIO gpio({17U, true, true, false, false}, backend);
		(void)gpio.init();
		return gpio;
	}

	static hal::Timer make_timer(MockTimerBackend& backend)
	{
		hal::Timer timer(backend);
		return timer;
	}

	static hal::SDCard::Config make_config()
	{
		hal::SDCard::Config config = {
			.block_size = 512U,
			.max_blocks = 100U,
			.cmd_timeout_ms = 100U,
			.data_timeout_ms = 500U
		};
		return config;
	}
}

bool test_sd_init()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	return sd_card.init() && sd_card.is_initialized();
}

bool test_sd_write_read_block()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	uint8_t write_buffer[512] = {0U};
	for (uint32_t i = 0U; i < 512U; ++i)
	{
		write_buffer[i] = static_cast<uint8_t>(i & 0xFFU);
	}

	uint32_t blocks_written = 0U;
	if (!sd_card.write_blocks(0U, write_buffer, 1U, blocks_written))
	{
		return false;
	}

	if (blocks_written != 1U)
	{
		return false;
	}

	uint8_t read_buffer[512] = {0U};
	uint32_t blocks_read = 0U;
	if (!sd_card.read_blocks(0U, read_buffer, 1U, blocks_read))
	{
		return false;
	}

	if (blocks_read != 1U)
	{
		return false;
	}

	for (uint32_t i = 0U; i < 512U; ++i)
	{
		if (read_buffer[i] != write_buffer[i])
		{
			return false;
		}
	}

	return true;
}

bool test_sd_write_struct()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	TestTelemetry data = {
		.counter = 42U,
		.altitude_mm = 12500,
		.velocity_mm_s = -350,
		.temperature_cdeg = 2500U,
		.status = 0x01U,
		.padding = {0U}
	};

	if (!sd_card.write_struct(0U, data))
	{
		return false;
	}

	return sizeof(TestTelemetry) <= 512U;
}

bool test_sd_read_struct()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	TestTelemetry write_data = {
		.counter = 99U,
		.altitude_mm = 5000,
		.velocity_mm_s = 100,
		.temperature_cdeg = 1800U,
		.status = 0x02U,
		.padding = {0U}
	};

	if (!sd_card.write_struct(5U, write_data))
	{
		return false;
	}

	TestTelemetry read_data = {0, 0, 0, 0, 0, {0U}};
	if (!sd_card.read_struct(5U, read_data))
	{
		return false;
	}

	return (read_data.counter == write_data.counter) &&
	       (read_data.altitude_mm == write_data.altitude_mm) &&
	       (read_data.velocity_mm_s == write_data.velocity_mm_s) &&
	       (read_data.temperature_cdeg == write_data.temperature_cdeg) &&
	       (read_data.status == write_data.status);
}

bool test_sd_get_capacity()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	uint32_t block_size = 0U;
	if (!sd_card.get_block_size(block_size))
	{
		return false;
	}

	if (block_size != 512U)
	{
		return false;
	}

	uint32_t capacity = 0U;
	if (!sd_card.get_capacity(capacity))
	{
		return false;
	}

	return (capacity == 512U * 100U);
}

bool test_sd_out_of_bounds()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	uint8_t buffer[512] = {0U};
	uint32_t blocks_written = 0U;
	return !sd_card.write_blocks(99U, buffer, 2U, blocks_written);
}

bool test_sd_multiple_blocks()
{
	MockSPIBackend spi_backend;
	MockGPIOBackend gpio_backend;
	MockTimerBackend timer_backend;
	MockSDCardBackend backend;

	hal::SPI spi = make_spi(spi_backend);
	hal::GPIO gpio = make_gpio(gpio_backend);
	hal::Timer timer = make_timer(timer_backend);

	hal::SDCard sd_card(make_config(), backend);
	if (!sd_card.init())
	{
		return false;
	}

	uint8_t write_buffer[1024] = {0U};
	for (uint32_t i = 0U; i < 1024U; ++i)
	{
		write_buffer[i] = static_cast<uint8_t>((i * 7U) & 0xFFU);
	}

	uint32_t blocks_written = 0U;
	if (!sd_card.write_blocks(0U, write_buffer, 2U, blocks_written))
	{
		return false;
	}

	if (blocks_written != 2U)
	{
		return false;
	}

	uint8_t read_buffer[1024] = {0U};
	uint32_t blocks_read = 0U;
	if (!sd_card.read_blocks(0U, read_buffer, 2U, blocks_read))
	{
		return false;
	}

	if (blocks_read != 2U)
	{
		return false;
	}

	for (uint32_t i = 0U; i < 1024U; ++i)
	{
		if (read_buffer[i] != write_buffer[i])
		{
			return false;
		}
	}

	return true;
}

int run_sd_tests()
{
	bool all_passed = true;
	all_passed = all_passed && test_sd_init();
	all_passed = all_passed && test_sd_write_read_block();
	all_passed = all_passed && test_sd_write_struct();
	all_passed = all_passed && test_sd_read_struct();
	all_passed = all_passed && test_sd_get_capacity();
	all_passed = all_passed && test_sd_out_of_bounds();
	all_passed = all_passed && test_sd_multiple_blocks();
	return all_passed ? 0 : 1;
}

int main()
{
	stdio_init_all();
	sleep_ms(2000);

	const int result = run_sd_tests();
	printf("TEST SD: %s\n", (result == 0) ? "PASS" : "FAIL");
	fflush(stdout);
	sleep_ms(100);

	return result;
}
