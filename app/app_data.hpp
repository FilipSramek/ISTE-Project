#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
#include <ctime>

namespace app {

struct Diagnostics {
    uint32_t error_flags = 0; // Bitová pole pro chyby
    uint32_t warning_flags = 0;
    uint32_t info_flags = 0;
    // Můžeš rozšířit dle potřeby
};

struct SensorData {
    // Čas zápisu (UNIX timestamp, sekundy)
    uint32_t timestamp = 0;

    // Napětí na baterii (V)
    float battery_voltage = 0.0f;

    // Vlhkost půdy (%)
    float soil_moisture = 0.0f;

    // BMP280
    float temperature = 0.0f; // °C
    float pressure = 0.0f;    // hPa

    // INA219
    float shunt_voltage = 0.0f; // V
    float power_draw = 0.0f;    // W
    float shunt_current = 0.0f; // A

    // Diagnostická data
    Diagnostics diag;
};

// Globální proměnná pro poslední známá data
extern SensorData g_last_sensor_data;

} // namespace app
