#pragma once

#include <Arduino.h>

#include <etl/chrono.h>


namespace fast_clock {

    // a custom ETL clock with second precision for usage in DCC fast clocks.

    class clock {
    public:
        using rep = uint32_t;
        using period = etl::ratio<1>;
        using duration = etl::chrono::duration<rep, period>;
        using time_point = etl::chrono::time_point<clock>;

        static constexpr bool is_steady = true;

        static time_point now() noexcept {
            return time_point{duration{seconds}};
        }

        static void update() {
            uint32_t now = millis();
            uint32_t elapsed = now - millis_at_last_update;
            millis_at_last_update = now;
            seconds += elapsed * rate / 1000;
        }

        static unsigned getRate() { return rate; }
        static void setRate(unsigned newRate) { rate = newRate; }

        static void setSeconds(uint32_t newSeconds) {
            seconds = newSeconds;
            millis_at_last_update = millis();
        }

    private:
        static rep seconds;
        static unsigned rate; // for now, integer multiplier to world clock.
        static uint32_t millis_at_last_update;

    };

    inline clock::rep clock::seconds;
    inline unsigned clock::rate;
    inline uint32_t clock::millis_at_last_update;
}
