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
            set_state(state, true, blinks);
        }

        void disable_state(State state) {
            set_state(state, false, 1);
        }

        void set_state(State state, bool enabled, uint8_t blinks) {
            auto v = state.get_value();
            if(!enabled) blinks = 1;
            if(priorities_enabled[v] == enabled && blinks == blink_count[v]) {
                // same configuration already set, don't restart sequence
                return;
            }

            priorities_enabled[state.get_value()] = enabled;
            blink_count[v] = blinks;
            apply_priority_pattern();
        }

        void stop_blinking() {
            if(timer != nullptr) xTimerStop(timer, 0);
            value = LOW;
            digitalWrite(pin, value);
        }

    private:
        static constexpr unsigned state_count = 3;
        /** normal priority: sequence of beats, count = state identifier,
         *   followed by a long off pause between groups (except a single beat,
         *   which has no pauses).
         * attention/error: continuous pattern, no pause,
         *   qualitatively distinct from "normal".
         */
        static constexpr uint32_t on_intervals[state_count]    = {500,  300, 150};
        static constexpr uint32_t off_intervals[state_count]   = {200,  300, 150};
        static constexpr uint32_t pause_intervals[state_count] = {1000, 0,   0};
        static constexpr bool continuous_pattern[state_count] = {false, true, true};

        uint8_t pin;
        uint8_t value{LOW};
        etl::bitset<state_count> priorities_enabled;
        uint8_t blink_count[state_count] = {1, 1, 1};
        TimerHandle_t timer{nullptr};
        unsigned active_priority{0};
        uint8_t blinks_done{0};
        enum class Phase { on, off, pause } phase{Phase::pause};

        static void timer_func(TimerHandle_t timer) {
            auto* inst = static_cast<Led*>(pvTimerGetTimerID(timer));
            inst->advance_pattern();
        }

        void apply_priority_pattern() {
            for(unsigned priority = state_count; priority > 0; --priority) {
                if(priorities_enabled[priority - 1]) {
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
            schedule(on_intervals[active_priority]);
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

        // Steps through on -> off -> ... -> pause -> on,
        //   counting blinks per priority's code.
        // Continuous priorities (attention/error) skip the pause and just keep toggling,
        //   as does normal with a single beat (heartbeat, e.g. STA).
        void advance_pattern() {
            const uint32_t on_time_ms = on_intervals[active_priority];
            const uint32_t off_time_ms = off_intervals[active_priority];
            const uint8_t total_blinks = blink_count[active_priority];
            const bool is_continuous = continuous_pattern[active_priority];

            switch(phase) {  // transitioning from this phase
                case Phase::on:
                    value = LOW;
                    digitalWrite(pin, value);
                    ++blinks_done;
                    if(is_continuous || blinks_done < total_blinks) {
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
