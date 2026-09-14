#pragma once

#include <Arduino.h>

#include <etl/observer.h>

#include <freertos/timers.h>


namespace fast_clock {

    using rep = uint32_t;

    struct ClockChangedEvent {
        rep seconds;
        uint8_t rate;
        bool big_change;
    };

    using clock_observer = etl::observer<const ClockChangedEvent&>;

    class Clock: public etl::observable<clock_observer, 10> {
    public:

        bool isRunning() { return rate != 0; }

        rep getSeconds() noexcept {
            return seconds;
        }

        void getDHMS(unsigned &days, uint8_t &hrs, uint8_t &mins, uint8_t &secs) {
            mins = (seconds / 60) % 60;
            hrs = (seconds / 3600) % 24;
            days = seconds / 86400;
            secs = seconds % 60;
        }

        void setSeconds(rep newSeconds) {
            seconds = newSeconds;
            millis_at_last_update = millis();
            notify_observers(ClockChangedEvent{seconds, rate, true});
        }

        /** 0=stopped, 1=realtime, 2=twice as fast as realtime,... */
        uint8_t getRate() { return rate; }
        void setRate(uint8_t newRate) {
            if(rate == newRate) return;
            rate = newRate;
            notify_observers(ClockChangedEvent{seconds, rate, true});

            if(rate==0) {
                if(timer!=nullptr) xTimerStop(timer, 0);
            } else {
                if(timer==nullptr) {
                    timer = xTimerCreate("FastClock", pdMS_TO_TICKS(1000),
                        pdTRUE, static_cast<void*>(this), &Clock::timer_func );
                }
                xTimerStart(timer, 0);
            }
        }

    private:
        rep seconds{0};
        uint8_t rate{0}; /// for now, integer multiplier to world clock.
        uint32_t millis_at_last_update{0};
        TimerHandle_t timer{nullptr};

        static void timer_func(TimerHandle_t tim) {
            Clock* inst = static_cast<Clock*>(pvTimerGetTimerID(tim));
            inst->tick1s();
        }

        void tick1s() {
            bool close_to_overflow = seconds % 60 > 30;
            seconds += rate;
            bool overflown = seconds % 60 < 30;
            if(close_to_overflow && overflown) {
                // send event every minute when seconds overflow
                notify_observers(ClockChangedEvent{seconds, rate, false});
            }
        }
    };

    inline Clock clock{};

}
