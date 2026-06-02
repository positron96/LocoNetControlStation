#pragma once

#include "DCC.h"


class DCCESP32SignalGenerator {

public:
    explicit DCCESP32SignalGenerator(uint8_t timerNum = 1);

    void setProgChannel(IDCCChannel * ch) { prog = ch;}
    void setMainChannel(IDCCChannel * ch) { main = ch;}

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
    IDCCChannel *main = nullptr;
    IDCCChannel *prog = nullptr;

    friend void timerCallback();
    friend void adcTimerCallback(void*);

    void IRAM_ATTR timerFunc();
    void IRAM_ATTR adcTimerFunc();
};
