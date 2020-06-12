#pragma once

#include <esp32-hal-timer.h>

class DCCESP32Channel {
    public:

    DCCESP32Channel(uint8_t outputPin=10, uint8_t sensePin=12, bool isOps=true, uint8_t timerId): 
        _outputPin(outputPin), _sensePin(sensePin)
    {
        _inst = this;
    }

    void begin() {
        
        _timer = timerBegin(_timerId, 480, true);

        timerAttachInterrupt(_timer, timerCallback, true);

        timerAlarmWrite(_timer, 10, true);
        
        pinMode(_outputPin, OUTPUT);
        //pinMode(_progOutputPin, OUTPUT);
    }

    void end() {

    }

    private:
    uint8_t _timerId;
    hw_timer_t * _timer;
    uint8_t _outputPin;    
    uint8_t _sensePin;

    uint8_t _cBit;
    uint8_t _cByte;
    uint8_t *_cPacket;
    uint8_t _packetLen;

    /** How manu timer periods are left for this bit. */
    uint8_t _ticksLeft;
    /** Timer tick value when to flip pin. Half of total bit length. */
    uint8_t _ticksBitFlip;

    static DCCESP32Channel * _inst;
    static void timerCallback() {
        _inst->timerFunc();
    }

    void IRAM_ATTR nextBit();
    void IRAM_ATTR timerFunc();
}