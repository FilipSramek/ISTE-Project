#include "soil_moisture.hpp"
#include "hal/adc/adc.hpp"
#include "app/app_init.hpp"

namespace soil_moisture {

app::AppContext* g_app_context_ptr = nullptr;

SoilMoistureSensor::SoilMoistureSensor(int adc_channel)
    : m_adc_channel(adc_channel) {}

void SoilMoistureSensor::init() {
    // Inicializace ADC kanálu, pokud je potřeba
    // Předpokládáme, že ADC je již inicializováno v systému
}

uint16_t SoilMoistureSensor::read_raw() const {
    // Čtení surové hodnoty z ADC
    if (g_app_context_ptr) {
        return g_app_context_ptr->adc_backend.read(static_cast<uint8_t>(m_adc_channel));
    }
    return 0;
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
