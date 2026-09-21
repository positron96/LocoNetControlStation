#pragma once

#include <esp32-hal.h>
#include <type_traits>

using millis_t = decltype(millis());

/**
 * A watchdog class that checks against up to 2 timeout periods.
 *
 * @param TIMEOUT how long to wait for update before timing out, ms
 * @param FUTURE_COMPENSATION - if somewhy kick happens "after" timedOut, then math inside
 *      timedOut goes crazy and fires timedOut immediately. For this reason, kick stored timestamp
 *      rolled back some time in the past (by this value, in ms)
 * @param TIMEOUT2 (optional) if not 0, enables second timeout, ms.
 */
template<millis_t TIMEOUT, millis_t FUTURE_COMPENSATION=0, millis_t TIMEOUT2=0>
class Watchdog {
    millis_t lastUpdate;

public:
    Watchdog(): lastUpdate(0) {}

    void kick() { lastUpdate = millis()-FUTURE_COMPENSATION; }

    millis_t getLastUpdate() const { return lastUpdate; }

    bool timedOut() const {
        millis_t ms = millis();
        //if(ms-lastUpdate >= TIMEOUT) W_LOGI("timeout at %ld, last update was at %ld", ms, getLastUpdate() );
        return ms-lastUpdate >= TIMEOUT;
    }

    template <millis_t X=TIMEOUT2>
    std::enable_if_t<X!=0, bool> timedOut2() const {
        return millis()-lastUpdate >= TIMEOUT2;
    }
};
