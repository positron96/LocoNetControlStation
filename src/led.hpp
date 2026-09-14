#pragma once

#include <Arduino.h>

#include <etl/bitset.h>
#include <etl/enum_type.h>
#include <freertos/timers.h>

namespace led {

    struct State {
        enum enum_type {
            normal,
            attention,
            error,
        };
        ETL_DECLARE_ENUM_TYPE(State, uint8_t)
        ETL_ENUM_TYPE(normal, "normal")
        ETL_ENUM_TYPE(attention, "attention")
        ETL_ENUM_TYPE(error, "error")
        ETL_END_ENUM_TYPE
    };

    class Led {
    public:
        explicit Led(uint8_t pin) : pin{pin} {}

        Led(const Led&) = delete;
        Led& operator=(const Led&) = delete;

        ~Led() {
            if(timer != nullptr) {
                xTimerStop(timer, 0);
                xTimerDelete(timer, 0);
                timer = nullptr;
            }
        }

        void begin(uint8_t initial_value = LOW) {
            value = initial_value;
            pinMode(pin, OUTPUT);
            digitalWrite(pin, value);
        }

        void enable_state(State state) {
            set_state(state, true);
        }

        void disable_state(State state) {
            set_state(state, false);
        }

        void set_state(State state, bool enabled) {
            priority_state.set(state.get_value(), enabled);
            apply_priority_state();
        }

        void start_blinking(uint32_t ms = 1000, uint8_t initial_value = HIGH) {
            if(ms == 0) ms = 1000;
            value = initial_value;
            digitalWrite(pin, value);

            if(timer == nullptr) {
                timer = xTimerCreate("LedBlink", pdMS_TO_TICKS(ms), pdTRUE,
                    static_cast<void*>(this), &Led::timer_func);
            }

            if(timer == nullptr) return;

            xTimerChangePeriod(timer, pdMS_TO_TICKS(ms), 0);
            xTimerStart(timer, 0);
        }

        void stop_blinking() {
            if(timer != nullptr) xTimerStop(timer, 0);
            value = LOW;
            digitalWrite(pin, value);
        }

    private:
        static constexpr unsigned state_count = 3;
        static constexpr uint32_t state_intervals[state_count] = {1000, 500, 250};

        uint8_t pin;
        uint8_t value{LOW};
        etl::bitset<state_count> priority_state;
        TimerHandle_t timer{nullptr};

        static void timer_func(TimerHandle_t timer) {
            auto* inst = static_cast<Led*>(pvTimerGetTimerID(timer));
            inst->toggle();
        }

        void apply_priority_state() {
            for(unsigned priority = state_count; priority > 0; --priority) {
                if(priority_state.test(priority - 1)) {
                    start_blinking(state_intervals[priority - 1]);
                    return;
                }
            }

            stop_blinking(); // fallback
        }

        void toggle() {
            value = value == LOW ? HIGH : LOW;
            digitalWrite(pin, value);
        }
    };

}
