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

        void enable_state(State state, uint8_t blinks = 1) {
            blink_count[state.get_value()] = blinks == 0 ? 1 : blinks;
            set_state(state, true);
        }

        void disable_state(State state) {
            set_state(state, false);
        }

        void set_state(State state, bool enabled) {
            priority_state.set(state.get_value(), enabled);
            apply_priority_state();
        }

        void stop_blinking() {
            if(timer != nullptr) xTimerStop(timer, 0);
            value = LOW;
            digitalWrite(pin, value);
        }

    private:
        static constexpr unsigned state_count = 3;
        // On/off duration of a single blink, per priority - faster blink = more urgent,
        // but kept slow enough (>=250ms) that blink counts stay easy to count visually.
        static constexpr uint32_t blink_intervals[state_count] = {400, 300, 250};
        // Pause between blink groups, per priority - shorter for urgent states so they repeat sooner.
        static constexpr uint32_t pause_intervals[state_count] = {800, 600, 500};
        static constexpr uint32_t off_time_ms = 150;

        uint8_t pin;
        uint8_t value{LOW};
        etl::bitset<state_count> priority_state;
        uint8_t blink_count[state_count] = {1, 1, 1};
        TimerHandle_t timer{nullptr};
        unsigned active_priority{0};
        uint8_t blinks_done{0};
        enum class Phase { on, off, pause } phase{Phase::pause};

        static void timer_func(TimerHandle_t timer) {
            auto* inst = static_cast<Led*>(pvTimerGetTimerID(timer));
            inst->advance_pattern();
        }

        void apply_priority_state() {
            for(unsigned priority = state_count; priority > 0; --priority) {
                if(priority_state[priority - 1]) {
                    start_pattern(priority - 1);
                    return;
                }
            }

            stop_blinking(); // nothing is on, turn off.
        }

        void start_pattern(unsigned priority) {
            active_priority = priority;
            blinks_done = 0;
            phase = Phase::on;
            value = HIGH;
            digitalWrite(pin, value);
            schedule(blink_intervals[active_priority]);
        }

        void schedule(uint32_t ms) {
            if(ms == 0) ms = 1;

            if(timer == nullptr) {
                timer = xTimerCreate("LedBlink", pdMS_TO_TICKS(ms), pdFALSE,
                    static_cast<void*>(this), &Led::timer_func);
                if(timer == nullptr) return;
                xTimerStart(timer, 0);
                return;
            }

            xTimerChangePeriod(timer, pdMS_TO_TICKS(ms), 0);
        }

        // Steps through on -> off -> ... -> pause -> on, counting blinks per priority's code.
        void advance_pattern() {
            const uint32_t on_time_ms = blink_intervals[active_priority];
            const uint8_t total_blinks = blink_count[active_priority];

            switch(phase) {  // transitioning from this phase
                case Phase::on:
                    value = LOW;
                    digitalWrite(pin, value);
                    ++blinks_done;
                    if(blinks_done < total_blinks) {
                        phase = Phase::off;
                        schedule(off_time_ms);
                    } else {
                        phase = Phase::pause;
                        schedule(pause_intervals[active_priority]);
                    }
                    break;

                case Phase::off:
                    value = HIGH;
                    digitalWrite(pin, value);
                    phase = Phase::on;
                    schedule(on_time_ms);
                    break;

                case Phase::pause:
                    blinks_done = 0;
                    value = HIGH;
                    digitalWrite(pin, value);
                    phase = Phase::on;
                    schedule(on_time_ms);
                    break;
            }
        }
    };

}
