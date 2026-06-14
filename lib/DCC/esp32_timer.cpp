#include "esp32_timer.hpp"

#include <esp32-hal-timer.h>
#include <esp_timer.h>

namespace dcc {

ESP32Timer::ESP32Timer()
{
}

void ESP32Timer::begin() {
    if (main!=nullptr) main->begin();
    if (prog!=nullptr) prog->begin();

    constexpr size_t TIMER_TICK_US = 58;

    _timer = timerBegin(1000000 / TIMER_TICK_US);
    timerAttachInterruptArg(_timer, timerFunc_c, this);
    timerAlarm(_timer, 1, true, 0);
    timerStart(_timer);

}

void ESP32Timer::end() {
    if(_timer!=nullptr) {
        timerStop(_timer);
        timerEnd(_timer);
        _timer = nullptr;
    }
    if (main!=nullptr) main->end();
    if (prog!=nullptr) prog->end();
}

void IRAM_ATTR ESP32Timer::timerFunc_c(void* arg) {
    static_cast<ESP32Timer*>(arg)->timerFunc();
}

void IRAM_ATTR ESP32Timer::timerFunc() {
    if (main!=nullptr) main->timerFunc();
    if (prog!=nullptr) prog->timerFunc();
}

}
