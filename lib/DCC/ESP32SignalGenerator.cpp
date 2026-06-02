#include "ESP32SignalGenerator.h"



//char _msg[1024];
//char _buf[100];
/*
void DCCESP32Channel::PacketSlot::initPackets(){
    activePacket = packet;
    updatePacket = packet+1;
} */


static DCCESP32SignalGenerator * _inst = nullptr;


void IRAM_ATTR timerCallback() {
    _inst->timerFunc();
}

void adcTimerCallback(void* arg) {
    ((DCCESP32SignalGenerator*)arg)->adcTimerFunc();
}


DCCESP32SignalGenerator::DCCESP32SignalGenerator(uint8_t timerNum)
    : _timerNum(timerNum)
{
    _inst = this;
}

void DCCESP32SignalGenerator::begin() {
    if (main!=nullptr) main->begin();
    if (prog!=nullptr) prog->begin();

    _timer = timerBegin(_timerNum, 464, true);
    timerAttachInterrupt(_timer, timerCallback, true);
    timerAlarmWrite(_timer, 10, true);
    timerAlarmEnable(_timer);
    timerStart(_timer);

    esp_timer_create_args_t cfg{adcTimerCallback, this, ESP_TIMER_TASK, "adc"};
    esp_timer_create(&cfg, &_adcTimer);
    esp_timer_start_periodic(_adcTimer, 1000);  // 1ms
}

void DCCESP32SignalGenerator::end() {
    if(_timer!=nullptr) {
        if(timerStarted(_timer) ) { timerStop(_timer); }
        timerEnd(_timer);
        _timer = nullptr;
    }
    esp_timer_stop(_adcTimer);
    esp_timer_delete(_adcTimer);
    if (main!=nullptr) main->end();
    if (prog!=nullptr) prog->end();
}

void DCCESP32SignalGenerator::timerFunc() {

    if (main!=nullptr) main->timerFunc();
    if (prog!=nullptr) prog->timerFunc();

}

void DCCESP32SignalGenerator::adcTimerFunc() {
    if (main!=nullptr) main->updateCurrent();
    if (prog!=nullptr) prog->updateCurrent();
}
