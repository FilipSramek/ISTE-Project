#include "drivers/soil_moisture/soil_moisture.hpp"
#include <cstdio>

int main() {
    // Příklad pro ADC kanál 0
    soil_moisture::SoilMoistureSensor sensor(0);
    sensor.init();

    uint16_t raw = sensor.read_raw();
    float percent = sensor.read_percent();

    printf("Surová hodnota: %u\n", raw);
    printf("Vlhkost půdy: %.1f %%\n", percent);
    return 0;
}
