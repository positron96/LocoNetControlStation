#pragma once

#include "base_channel.hpp"
#include "esp32_timer_channel.hpp"

namespace dcc {

/**
 * ESP32-based timer for DCC handling.
 *
 * Uses a hardware timer for outputting bits to track,
 *  and a half-software timer for checking for current consumption regularly.
 */
class ESP32Timer {

public:
    explicit ESP32Timer(uint8_t timerNum = 1);

    void setProgChannel(ESP32TimerChannel * ch) { prog = ch;}
    void setMainChannel(ESP32TimerChannel * ch) { main = ch;}

    /**
     * Starts half-bit timer.
     * To get 58us tick we need divisor of 58us/0.0125us(80mhz) = 4640,
     * separate this into 464 prescaler and 10 timer alarm.
     */
    void begin();

    void end();

private:
    hw_timer_t * _timer = nullptr;
    uint8_t _timerNum;
    esp_timer_handle_t _adcTimer;
    ESP32TimerChannel *main = nullptr;
    ESP32TimerChannel *prog = nullptr;

    friend void timerCallback();
    friend void adcTimerCallback(void*);

    void IRAM_ATTR timerFunc();
    void IRAM_ATTR adcTimerFunc();
};

}
