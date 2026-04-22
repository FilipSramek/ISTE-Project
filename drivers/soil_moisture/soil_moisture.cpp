#include "soil_moisture.hpp"
#include "hal/adc/adc.hpp"
#include <stdio.h>

namespace soil_moisture {

SoilMoistureSensor::SoilMoistureSensor(int adc_channel)
    : m_adc_channel(adc_channel) {}

void SoilMoistureSensor::init() {
    // Inicializace ADC kanálu pro moisture senzor.
    adc_init();
    adc_gpio_init(26 + m_adc_channel);  // ADC channels 0-4 are on GPIO 26-30
}

uint16_t SoilMoistureSensor::read_raw() const {
    // Čtení surové hodnoty z ADC
    adc_select_input(m_adc_channel);
    return adc_read();
}

float SoilMoistureSensor::read_percent() const {
    // Surová hodnota z ADC (předpoklad: 12bit ADC, rozsah 0-4095)
    uint16_t raw = read_raw();
    //test
    //printf("[DEBUG] SoilMoistureSensor::read_percent() - raw ADC value: %u\n", raw);
    // Kalibrace: upravte min/max podle reálného senzoru
    constexpr uint16_t DRY = 3500;   // hodnota pro "sucho"
    constexpr uint16_t WET = 1500;   // hodnota pro "mokro"
    //printf("[DEBUG] SoilMoistureSensor::read_percent() - DRY: %u, WET: %u\n", DRY, WET);
    if (raw >= DRY) return 0.0f;
    if (raw <= WET) return 100.0f;
    return 100.0f * (DRY - raw) / (DRY - WET);
}

} // namespace soil_moisture
