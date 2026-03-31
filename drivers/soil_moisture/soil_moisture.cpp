#include "soil_moisture.hpp"
#include "hal/adc/adc.hpp"

namespace soil_moisture {

SoilMoistureSensor::SoilMoistureSensor(int adc_channel)
    : m_adc_channel(adc_channel) {}

void SoilMoistureSensor::init() {
    // Inicializace ADC kanálu, pokud je potřeba
    // Předpokládáme, že ADC je již inicializováno v systému
}

uint16_t SoilMoistureSensor::read_raw() const {
    // Čtení surové hodnoty z ADC
    return hal::adc::read(m_adc_channel);
}

float SoilMoistureSensor::read_percent() const {
    // Surová hodnota z ADC (předpoklad: 12bit ADC, rozsah 0-4095)
    uint16_t raw = read_raw();
    // Kalibrace: upravte min/max podle reálného senzoru
    constexpr uint16_t DRY = 3500;   // hodnota pro "sucho"
    constexpr uint16_t WET = 1500;   // hodnota pro "mokro"
    if (raw >= DRY) return 0.0f;
    if (raw <= WET) return 100.0f;
    return 100.0f * (DRY - raw) / (DRY - WET);
}

} // namespace soil_moisture
