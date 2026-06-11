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
     */
    void begin();

    void end();

private:
    hw_timer_t * _timer{nullptr};
    uint8_t _timerNum;
    ESP32TimerChannel *main{nullptr};
    ESP32TimerChannel *prog{nullptr};

    static void timerFunc_c(void* arg);

    void timerFunc();
};

}
