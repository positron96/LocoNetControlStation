#include "esp32_timer.hpp"

namespace dcc {


static ESP32Timer * _inst = nullptr;


void IRAM_ATTR timerCallback() {
    _inst->timerFunc();
}

void adcTimerCallback(void* arg) {
    ((ESP32Timer*)arg)->adcTimerFunc();
}


ESP32Timer::ESP32Timer(uint8_t timerNum)
    : _timerNum(timerNum)
{
    _inst = this;
}

void ESP32Timer::begin() {
    if (main!=nullptr) main->begin();
    if (prog!=nullptr) prog->begin();

    constexpr size_t TIMER_TICK_US = 58;

    (void)_timerNum;
    _timer = timerBegin(1000000 / TIMER_TICK_US);
    timerAttachInterrupt(_timer, timerCallback);
    timerAlarm(_timer, 10, true, 0);
    timerStart(_timer);

    esp_timer_create_args_t cfg{adcTimerCallback, this, ESP_TIMER_TASK, "adc"};
    esp_timer_create(&cfg, &_adcTimer);
    esp_timer_start_periodic(_adcTimer, 1000);  // 1ms
}

void ESP32Timer::end() {
    if(_timer!=nullptr) {
        timerStop(_timer);
        timerEnd(_timer);
        _timer = nullptr;
    }
    esp_timer_stop(_adcTimer);
    esp_timer_delete(_adcTimer);
    if (main!=nullptr) main->end();
    if (prog!=nullptr) prog->end();
}

void ESP32Timer::timerFunc() {

    if (main!=nullptr) main->timerFunc();
    if (prog!=nullptr) prog->timerFunc();

}

void ESP32Timer::adcTimerFunc() {
    if (main!=nullptr) main->updateCurrent();
    if (prog!=nullptr) prog->updateCurrent();
}

}
