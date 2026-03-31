#pragma once
#include <cstdint>
#include "hal/timer/timer.hpp"

namespace app {

constexpr float MIN_BATTERY_VOLTAGE = 3.3f; // minimální napětí pro odeslání dat
constexpr uint32_t MEASURE_INTERVAL_MS = 5 * 60 * 1000; // 5 minut
constexpr uint32_t SEND_INTERVAL_MS = 15 * 60 * 1000;   // 15 minut

enum class State {
    MEASURE,
    SEND,
    STANDBY
};

struct StateMachine {
    State current = State::STANDBY;
    uint64_t last_measure_time = 0;
    uint64_t last_send_time = 0;
    uint64_t last_state_change = 0;

    void transition(State next, hal::Timer& timer) {
        current = next;
        last_state_change = timer.now_ms();
    }
};

} // namespace app
