#pragma once
#include <cstdint>

namespace soil_moisture {

class SoilMoistureSensor {
public:
    // Konstruktor: adc_channel je číslo kanálu ADC
    SoilMoistureSensor(int adc_channel);

    // Inicializace senzoru (pokud je potřeba)
    void init();

    // Získání surové hodnoty z ADC
    uint16_t read_raw() const;

    // Získání vlhkosti v procentech (0-100 %)
    float read_percent() const;

private:
    int m_adc_channel;
};

} // namespace soil_moisture
