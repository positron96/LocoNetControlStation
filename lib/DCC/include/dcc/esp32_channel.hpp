#pragma once

#include "DCC.h"

#include <array>
#include <limits>

namespace dcc {

/**
 * Utility class for maintaining a sliding window of values and calculating a trimmed mean.
 **/
template<typename T, std::size_t WINDOW_SIZE>
class SlidingWindow {
public:
    void add(T value) {
        _values[_writeIndex] = value;
        _writeIndex = (_writeIndex + 1) % WINDOW_SIZE;
    }

    T trimmedValue() const {
        T minValue = std::numeric_limits<T>::max();
        T maxValue = std::numeric_limits<T>::lowest();
        T sum = 0;

        for (T value : _values) {
            sum += value;
            if (value < minValue) minValue = value;
            if (value > maxValue) maxValue = value;
        }

        static_assert(WINDOW_SIZE > 2, "Window size must be greater than 2 to calculate trimmed value.");
        return (sum - minValue - maxValue) / ((T)WINDOW_SIZE - 2);
    }

private:
    etl::array<T, WINDOW_SIZE> _values{};
    size_t _writeIndex = 0;
};

/**
 * ESP32-based DCC channel.
 *
 * Because there are several implementation of DCC output for ESP32 (timer, RMT),
 *   this is a abstract base class.
 * Main functionality here is current measurement using ESP32 APIs.
 *
 */
class ESP32Channel : public BaseChannel {
public:
    ESP32Channel(
        uint8_t outputPin,
        uint8_t enPin,
        uint8_t sensePin,
        BasePacketList &packets,
        float mvTomA = 1.0f
    ) : BaseChannel{packets},
        pinData{outputPin},
        pinEn{enPin},
        pinSense{sensePin},
        mvTomA{mvTomA}
    {}

    void begin() override {
        pinMode(pinData, OUTPUT);
        pinMode(pinEn, OUTPUT);
        digitalWrite(pinData, LOW);
        digitalWrite(pinEn, LOW);

        //analogRead(_sensePin); // without it, analogSetPinAttenuation throws error
        //analogSetPinAttenuation(_sensePin, ADC_0db);
    }

    void end() override {
        pinMode(pinData, INPUT);
        pinMode(pinEn, INPUT);
    }

    void setPower(bool v, PowerEvent::Reason reason = PowerEvent::Reason::Normal) override {
        if(v == getPower()) return;
        DCC_LOGI("setPower(%d)", v);
        digitalWrite(pinEn, v ? HIGH : LOW);
        powerState = v;
        BaseChannel::setPower(v, reason);
        notify_observers(PowerEvent{v, reason, this});
    }

    bool getPower() const override {
        return powerState;
    }

    void updateCurrent() override {

        _currentWindow.add((uint16_t)analogReadMilliVolts(pinSense));

        const uint16_t mv = _currentWindow.trimmedValue();
        uint16_t cur = static_cast<uint16_t>(mv * mvTomA);
        current = cur;
        if (current > maxCurrent) {
            maxCurrent = cur;
        }

        if(cur > overCurrentThreshold) {
            digitalWrite(pinEn, LOW); // act immediately before any notifications
            DCC_LOGW("Overcurrent: %humV, %hu mA > %hu mA\n", mv, cur, overCurrentThreshold);
            overCurrentFlag = true;
            overCurrentEventPending = true;
        }
    }

    /**
     * Sets the voltage to current conversion coefficient.
     *
     * It depends on schematic of the board, so cannot be hardcoded.
     */
    void setVoltageToCurrentCoef(float v) {
        mvTomA = v;
    }

protected:
    uint8_t pinData;
    uint8_t pinEn;
    uint8_t pinSense;
    bool powerState{false};

private:

    float mvTomA;
    SlidingWindow<uint16_t, 4> _currentWindow;
};

}
