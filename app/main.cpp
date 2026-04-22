#include <stdio.h>

#include "app/app_init.hpp"
#include "app/state_machine.hpp"
#include "app/app_data.hpp"
#include "pico/stdlib.h"

#include <string.h>
#include "pico/stdio_usb.h"


int main()
{
    stdio_init_all();

    printf("Starting application...\n");
    while (true)
    {
        app::AppContext app_context;
        if (!app::init(app_context))
        {
            printf("Application init failed: %s\n", app::last_init_error());
            sleep_ms(1000);
            continue;
        }

        printf("Application init successful\n");

        app::StateMachine state_machine;

        // Nastav výchozí stav podle režimu
        if constexpr (app::ENABLE_TEST_MEASURE) {
            printf("[INFO] ENABLE_TEST_MEASURE = true, přecházím do TEST_MEASURE režimu!\n");
            state_machine.current = app::State::TEST_MEASURE;
        } else {
            bool radio_failed = (strcmp(app::last_init_error(), "sx1278.init() failed") == 0);
            if (radio_failed) {
                printf("[ERROR] SX1278 radio init failed, přecházím do TEST_MEASURE režimu!\n");
                state_machine.current = app::State::TEST_MEASURE;
            } else {
                state_machine.current = app::State::MEASURE;
                printf("[INFO] Výchozí stav: MEASURE\n");
            }
        }
        printf("[STATE] Počáteční stav: %s\n", app::StateMachine::state_to_string(state_machine.current));

        while (true)
        {
            printf("[STATE] Aktuální stav: %s\n", app::StateMachine::state_to_string(state_machine.current));
            uint64_t now = app_context.timer.now_ms();
            switch (state_machine.current) {
                case app::State::TEST_MEASURE: {
                    // Humidity senzor zůstává zapnutý
                    app_context.gpio_humidity_power.write(true);

                    printf("[TEST_MEASURE] --- Begin measurement and send ---\n");

                    drivers::VoltageDivider::Data vdiv_data;
                    bool vdiv_update = app_context.voltage_divider_1.update();
                    bool vdiv_ok = app_context.voltage_divider_1.get_data(vdiv_data);
                    printf("[TEST_MEASURE] Battery update=%d get_data=%d valid=%d initialized=%d\n",
                        vdiv_update ? 1 : 0,
                        vdiv_ok ? 1 : 0,
                        vdiv_data.valid ? 1 : 0,
                        app_context.voltage_divider_1.is_initialized() ? 1 : 0);
                    if (vdiv_ok && vdiv_data.valid) {
                        app::g_last_sensor_data.battery_voltage = vdiv_data.input_voltage;
                    }
                    printf("[TEST_MEASURE] Battery voltage: %.2f V\n", app::g_last_sensor_data.battery_voltage);

                    app::g_last_sensor_data.soil_moisture = app_context.soil_sensor.read_percent();
                    printf("[TEST_MEASURE] Soil moisture: %.2f %%\n", app::g_last_sensor_data.soil_moisture);

                    drivers::BMP280::Data bmp_data;
                    bool bmp_update = app_context.bmp280.update();
                    bool bmp_ok = app_context.bmp280.get_data(bmp_data);
                    printf("[TEST_MEASURE] BMP280 update=%d get_data=%d valid=%d initialized=%d\n",
                        bmp_update ? 1 : 0,
                        bmp_ok ? 1 : 0,
                        bmp_data.valid ? 1 : 0,
                        app_context.bmp280.is_initialized() ? 1 : 0);
                    if (!bmp_ok || !bmp_data.valid) {
                        printf("[TEST_MEASURE] BMP280 last_error: %s\n", app_context.bmp280.last_error());
                    }
                    if (bmp_ok && bmp_data.valid) {
                        app::g_last_sensor_data.temperature = bmp_data.temperature * 0.01f;
                        app::g_last_sensor_data.pressure = bmp_data.pressure / 100.0f;
                    }
                    printf("[TEST_MEASURE] Temperature: %.2f °C, Pressure: %.2f hPa\n", app::g_last_sensor_data.temperature, app::g_last_sensor_data.pressure);

                    drivers::INA219::Data ina_data;
                    bool ina_update = app_context.ina219.update();
                    bool ina_ok = app_context.ina219.get_data(ina_data);
                    printf("[TEST_MEASURE] INA219 update=%d get_data=%d valid=%d initialized=%d\n",
                        ina_update ? 1 : 0,
                        ina_ok ? 1 : 0,
                        ina_data.valid ? 1 : 0,
                        app_context.ina219.is_initialized() ? 1 : 0);
                    if (!ina_ok || !ina_data.valid) {
                        printf("[TEST_MEASURE] INA219 last_error: %s\n", app_context.ina219.last_error());
                    }
                    if (ina_ok && ina_data.valid) {
                        app::g_last_sensor_data.shunt_voltage = ina_data.shunt_voltage;
                        app::g_last_sensor_data.power_draw = ina_data.power;
                        app::g_last_sensor_data.shunt_current = ina_data.current;
                    }
                    printf("[TEST_MEASURE] Shunt voltage: %.3f V, Power: %.3f W, Current: %.3f A\n",
                        app::g_last_sensor_data.shunt_voltage,
                        app::g_last_sensor_data.power_draw,
                        app::g_last_sensor_data.shunt_current);

                    app::g_last_sensor_data.timestamp = time(NULL);
                    state_machine.last_measure_time = now;

                    // Vždy posli data, bez ohledu na baterii
                    printf("[TEST_MEASURE] Sending data via radio (no battery check)...\n");
                    app_context.sx1278.send_struct(app::g_last_sensor_data);
                    state_machine.last_send_time = now;
                    printf("[TEST_MEASURE] Send complete\n");

                    printf("[STATE] TEST_MEASURE done\n");
                    break;
                }
                case app::State::MEASURE: {
                    // Zapni napájení humidity senzoru (MOSFET)
                    app_context.gpio_humidity_power.write(true);
                    printf("[HUMIDITY] Power ON, čekám 1 minutu na stabilizaci...\n");
                    uint64_t start_wait = app_context.timer.now_ms();
                    while (app_context.timer.now_ms() - start_wait < 60000) {
                        sleep_ms(100);
                    }

                    // Voltage divider
                    drivers::VoltageDivider::Data vdiv_data;
                    if (app_context.voltage_divider_1.get_data(vdiv_data) && vdiv_data.valid)
                        app::g_last_sensor_data.battery_voltage = vdiv_data.input_voltage;
                    printf("[MEASURE] Battery voltage: %.2f V\n", app::g_last_sensor_data.battery_voltage);

                    // Soil moisture
                    app::g_last_sensor_data.soil_moisture = app_context.soil_sensor.read_percent();
                    printf("[MEASURE] Soil moisture: %.2f %%\n", app::g_last_sensor_data.soil_moisture);

                    // BMP280
                    drivers::BMP280::Data bmp_data;
                    if (app_context.bmp280.get_data(bmp_data) && bmp_data.valid) {
                        app::g_last_sensor_data.temperature = bmp_data.temperature * 0.01f;
                        app::g_last_sensor_data.pressure = bmp_data.pressure / 100.0f; // hPa
                    }
                    printf("[MEASURE] Temperature: %.2f °C, Pressure: %.2f hPa\n", 
                        app::g_last_sensor_data.temperature, app::g_last_sensor_data.pressure);

                    // INA219
                    drivers::INA219::Data ina_data;
                    if (app_context.ina219.get_data(ina_data) && ina_data.valid) {
                        app::g_last_sensor_data.shunt_voltage = ina_data.shunt_voltage;
                        app::g_last_sensor_data.power_draw = ina_data.power;
                        app::g_last_sensor_data.shunt_current = ina_data.current;
                    }
                    printf("[MEASURE] Shunt voltage: %.3f V, Power: %.3f W, Current: %.3f A\n",
                        app::g_last_sensor_data.shunt_voltage,
                        app::g_last_sensor_data.power_draw,
                        app::g_last_sensor_data.shunt_current);

                    // Vypni napájení humidity senzoru
                    app_context.gpio_humidity_power.write(false);
                    printf("[HUMIDITY] Power OFF\n");

                    app::g_last_sensor_data.timestamp = time(NULL);
                    state_machine.last_measure_time = now;
                    
                    // Zkus poslat data
                    printf("[MEASURE] Attempting to send data via radio...\n");
                    if (app::g_last_sensor_data.battery_voltage > app::MIN_BATTERY_VOLTAGE) {
                        printf("[MEASURE] Battery OK (%.2f V > %.2f V)\n", 
                            app::g_last_sensor_data.battery_voltage, app::MIN_BATTERY_VOLTAGE);
                        app_context.sx1278.send_struct(app::g_last_sensor_data);
                        state_machine.last_send_time = now;
                        printf("[MEASURE] Send complete\n");
                    } else {
                        printf("[MEASURE] Send skipped - low battery (%.2f V < %.2f V)\n",
                            app::g_last_sensor_data.battery_voltage, app::MIN_BATTERY_VOLTAGE);
                    }
                    
                    state_machine.transition(app::State::STANDBY, app_context.timer);
                    printf("[STATE] MEASURE done\n");
                    break;
                }
                case app::State::SEND:
                    if (app::g_last_sensor_data.battery_voltage > app::MIN_BATTERY_VOLTAGE) {
                        app_context.sx1278.send_struct(app::g_last_sensor_data);
                        state_machine.last_send_time = now;
                        printf("[STATE] SEND done\n");
                    } else {
                        printf("[STATE] SEND skipped (low battery)\n");
                    }
                    state_machine.transition(app::State::STANDBY, app_context.timer);
                    break;
                case app::State::STANDBY:
                    if (state_machine.current == app::State::TEST_MEASURE) {
                        if (now - state_machine.last_measure_time > app::TEST_MEASURE_INTERVAL_MS) {
                            state_machine.transition(app::State::TEST_MEASURE, app_context.timer);
                        }
                    } else {
                        if (now - state_machine.last_measure_time > app::MEASURE_INTERVAL_MS) {
                            state_machine.transition(app::State::MEASURE, app_context.timer);
                        } else if (now - state_machine.last_send_time > app::SEND_INTERVAL_MS) {
                            state_machine.transition(app::State::SEND, app_context.timer);
                        }
                    }
                    break;
            }
            sleep_ms(1000);
        }
    }
}
