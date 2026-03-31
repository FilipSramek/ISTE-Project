#include "drivers/soil_moisture/soil_moisture.hpp"
#include <cassert>
#include <cstdio>

// Mock nebo stub pro testování bez HW
class MockADCBackend : public hal::IADCBackend {
public:
    mutable uint16_t value = 2000;
    void init(uint8_t) override {}
    uint16_t read(uint8_t) override { return value; }
    void set_clock_div(uint8_t, float) override {}
};

int main() {
    MockADCBackend backend;
    hal::ADC::Config config{.channel = 0, .clock_div = 0.0f};
    hal::ADC adc(config, backend);

    soil_moisture::SoilMoistureSensor sensor(0);
    // Test surové hodnoty
    backend.value = 3500;
    assert(sensor.read_percent() == 0.0f);
    backend.value = 1500;
    assert(sensor.read_percent() == 100.0f);
    backend.value = 2500;
    float percent = sensor.read_percent();
    printf("Test vlhkosti: %.1f %%\n", percent);
    assert(percent > 0.0f && percent < 100.0f);
    printf("Všechny testy prošly.\n");
    return 0;
}
