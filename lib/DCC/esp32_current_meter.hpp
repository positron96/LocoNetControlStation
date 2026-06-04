#pragma once

#include "base_channel.hpp"

namespace dcc {

    class ESP32CurrentMeter: public CurrentMeter {
    public:
        void begin() {
            esp_timer_create_args_t cfg{adcTimerFunc_c, this, ESP_TIMER_TASK, "adc"};
            esp_timer_create(&cfg, &_adcTimer);
            esp_timer_start_periodic(_adcTimer, 1000);  // 1ms

            for(auto ch: channels) {
                ch->resetMaxCurrent();
            }
        }

        void end() {
            esp_timer_stop(_adcTimer);
            esp_timer_delete(_adcTimer);
            _adcTimer = nullptr;
        }

    private:
        esp_timer_handle_t _adcTimer;

        static void adcTimerFunc_c(void* arg) {
            ((ESP32CurrentMeter*)arg)->adcTimerFunc();
        }
        void adcTimerFunc() {
            for(auto ch: channels) {
                ch->updateCurrent();
            }
        }
    };
}
