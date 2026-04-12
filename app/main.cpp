#include <stdio.h>

#include "app/app_init.hpp"
#include "app/state_machine.hpp"
#include "app/app_data.hpp"
#include "pico/stdlib.h"
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

        while (true)
        {
            uint64_t now = app_context.timer.now_ms();
            switch (state_machine.current) {
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

                    // Soil moisture
                    app::g_last_sensor_data.soil_moisture = app_context.soil_sensor.read_percent();

                    // BMP280
                    drivers::BMP280::Data bmp_data;
                    if (app_context.bmp280.get_data(bmp_data) && bmp_data.valid) {
                        app::g_last_sensor_data.temperature = bmp_data.temperature * 0.01f;
                        app::g_last_sensor_data.pressure = bmp_data.pressure / 100.0f; // hPa
                    }

                    // INA219
                    drivers::INA219::Data ina_data;
                    if (app_context.ina219.get_data(ina_data) && ina_data.valid) {
                        app::g_last_sensor_data.shunt_voltage = ina_data.shunt_voltage;
                        app::g_last_sensor_data.power_draw = ina_data.power;
                        app::g_last_sensor_data.shunt_current = ina_data.current;
                    }

                    // Vypni napájení humidity senzoru
                    app_context.gpio_humidity_power.write(false);
                    printf("[HUMIDITY] Power OFF\n");

                    app::g_last_sensor_data.timestamp = time(NULL);
                    state_machine.last_measure_time = now;
                    state_machine.transition(app::State::SEND, app_context.timer);
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
                    if (now - state_machine.last_measure_time > app::MEASURE_INTERVAL_MS) {
                        state_machine.transition(app::State::MEASURE, app_context.timer);
                    } else if (now - state_machine.last_send_time > app::SEND_INTERVAL_MS) {
                        state_machine.transition(app::State::SEND, app_context.timer);
                    }
                    break;
            }
            sleep_ms(1000);
        }
    }
}
