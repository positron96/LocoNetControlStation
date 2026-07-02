#pragma once

#include "DCC.h"

namespace dcc {

class ESP32Channel : public BaseChannel {
public:
    ESP32Channel(
        uint8_t outputPin,
        uint8_t enPin,
        uint8_t sensePin,
        BasePacketList &packets,
        float mvTomA = 1.0f
    ) : BaseChannel{packets},
        _outputPin{outputPin},
        _enPin{enPin},
        _sensePin{sensePin},
        _mvTomA{mvTomA}
    {}

    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_outputPin, LOW);
        digitalWrite(_enPin, LOW);

        //analogRead(_sensePin); // without it, analogSetPinAttenuation throws error
        //analogSetPinAttenuation(_sensePin, ADC_0db);
    }

    void end() override {
        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v, PowerEvent::Reason reason = PowerEvent::Reason::Normal) override {
        if(v == getPower()) return;
        DCC_LOGI("setPower(%d)", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
        BaseChannel::setPower(v, reason);
        notify_observers(PowerEvent{v, reason, this});
    }

    bool getPower() const override {
        return digitalRead(_enPin) == HIGH;
    }

    void updateCurrent() override {
        const uint16_t mv = analogReadMilliVolts(_sensePin);

        if(current > overCurrentThreshold) {
            overCurrentFlag = true;
            overCurrentEventPending = true;
            digitalWrite(_enPin, LOW); // act immediately and without notifications
        }

        current = static_cast<uint16_t>(mv * _mvTomA);
        if (current > maxCurrent) {
            maxCurrent = current.load();
        }
    }

    /**
     * Sets the voltage to current conversion coefficient.
     *
     * It depends on schematic of the board, so cannot be hardcoded.
     */
    void setVoltageToCurrentCoef(float v) {
        _mvTomA = v;
    }

protected:
    uint8_t _outputPin;
    uint8_t _enPin;
    uint8_t _sensePin;

private:
    float _mvTomA;
};

}
