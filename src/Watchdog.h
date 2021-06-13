#pragma once

#include <esp32-hal.h>
#include <type_traits>

using millis_t = decltype(millis());

#define W_LOGI(format, ...)   do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__);  } while(0)

/**
 * @param TIMEOUT how long to wait for update before timing out, ms
 * @param FUTURE_COMPENSATION - if somewhy kick happens "after" timedOut, then math inside
 *      timedOut goes crazy and fires timedOut immediately. For this reason, kick stored timestamp
 *      rolled back some time in the past (by this value, in ms)
 */
template<millis_t TIMEOUT, millis_t FUTURE_COMPENSATION=0, millis_t TIMEOUT2=0>
class Watchdog {
    millis_t lastUpdate;

public:
    Watchdog(): lastUpdate(0) {}

    void kick() { lastUpdate = millis()-FUTURE_COMPENSATION; }

    millis_t getLastUpdate() { return lastUpdate; }

    bool timedOut() {
        millis_t ms = millis();
        //if(ms-lastUpdate >= TIMEOUT) W_LOGI("timeout at %ld, last update was at %ld", ms, getLastUpdate() );
        return ms-lastUpdate >= TIMEOUT;
    }

    template <millis_t X=TIMEOUT2>
    std::enable_if_t<X!=0, bool> timedOut2() {
        return millis()-lastUpdate >= TIMEOUT2;
    }
};

