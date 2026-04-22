#pragma once
#include <cstdint>
#include "hal/timer/timer.hpp"

namespace app {

constexpr float MIN_BATTERY_VOLTAGE = 3.5f; // minimální napětí pro odeslání dat
constexpr uint32_t MEASURE_INTERVAL_MS = 1 * 60 * 1000; // 1 minut
constexpr uint32_t SEND_INTERVAL_MS = 1 * 60 * 60 * 1000;   // 1 hodina


constexpr uint32_t TEST_MEASURE_INTERVAL_MS = 5000; // 5 sekund

constexpr bool ENABLE_TEST_MEASURE = true; // Nastav na true pro automatický testovací režim

enum class State {
    MEASURE,
    SEND,
    STANDBY,
    TEST_MEASURE
};

struct StateMachine {
    State current = State::STANDBY;
    uint64_t last_measure_time = 0;
    uint64_t last_send_time = 0;
    uint64_t last_state_change = 0;

    static const char* state_to_string(State s) {
        switch (s) {
            case State::MEASURE: return "MEASURE";
            case State::SEND: return "SEND";
            case State::STANDBY: return "STANDBY";
            case State::TEST_MEASURE: return "TEST_MEASURE";
            default: return "UNKNOWN";
        }
    }

    void transition(State next, hal::Timer& timer) {
        current = next;
        last_state_change = timer.now_ms();
    }
};

} // namespace app
