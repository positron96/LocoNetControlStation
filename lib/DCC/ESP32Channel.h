#pragma once

#include "DCC.h"
#include "PacketList.hpp"

namespace dcc {


class DCCESP32Channel: public IDCCChannel {
public:

    DCCESP32Channel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin, BasePacketList &packets):
        IDCCChannel{packets},
        _outputPin{outputPin}, _enPin{enPin}, _sensePin{sensePin}
    {
    }


    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_enPin, LOW);

        //DCC_LOGI("DCCESP32Channel(enPin=%d)::begin", _enPin);

        //analogSetCycles(16);
        //analogSetWidth(11);
        analogSetPinAttenuation(_sensePin, ADC_0db);
        /*esp_adc_cal_value_t ar = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_chars);
        if (ar == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            DCC_LOGI("eFuse Vref");
        } else {
            DCC_LOGI("Default vref");
        }*/

        // during loadPacket there is time when new index is enabled for refresh, but urgentPacket is not yet loaded.
        //loadPacket(1, idlePacket, 2, 0);
    }

    void end() override {
        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v) override {
        DCC_LOGI("setPower(%d)", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() override {
        return digitalRead(_enPin) == HIGH;
    }

    void updateCurrent() override {
        uint16_t c = analogRead(_sensePin);
        current = c;
        if(c > maxCurrent) maxCurrent = c;
    }

    void IRAM_ATTR timerFunc() override {
        timerPeriodsLeft--;
        //DCC_DEBUGF_ISR("DCCESP32Channel::timerFunc, periods left: %d, total: %d\n", R.timerPeriodsLeft, R.timerPeriodsHalf*2);
        if(timerPeriodsLeft == timerPeriodsHalf) {
            digitalWrite(_outputPin, HIGH );
        }
        if(timerPeriodsLeft == 0) {
            digitalWrite(_outputPin, LOW );
            nextBit();
        }

        //current = readCurrentAdc();
    }


protected:

    /** Tries to load a packet for a specified duration. */
    bool loadPacket(etl::span<uint8_t> packet, size_t nRepeat, size_t timeout_ms=1000) override {
        for(size_t i=0; i<timeout_ms; i++) {
            if (packets.put_generic_packet(packet, nRepeat)) return true;
            delay(1);
        }
        return false;
    }

    void unloadSlot(const LocoAddress addr) override {
        packets.clear_loco(addr);
    }

private:

    uint8_t _outputPin;
    uint8_t _enPin;
    uint8_t _sensePin;
    //esp_adc_cal_characteristics_t adc_chars;

    PacketWithRepeats currentPacket;
    size_t current_bit;

    /* how many 58us periods needed for half-cycle (1 for "1", 2 for "0") */
    volatile uint8_t timerPeriodsHalf = 1; // first thing a timerfunc does is decrement this, so make it not underflow
    /* how many 58us periods are left (at start, 2 for "1", 4 for "0"). */
    volatile uint8_t timerPeriodsLeft = 2; // some sane nonzero value.

    void IRAM_ATTR nextBit() {
        auto *p = &currentPacket;
        //DCC_DEBUGF_ISR("nextBit: currentPacket=%d, activePacket=%d, cbit=%d, bits=%d", R.currentIdx(),  R.currentPacket->activeIdx(), R.currentBit, p->nBits );

        // end of packet, either repeat this or get next one
        if (current_bit == p->packet.len) {
            current_bit = 0;
            if (p->nRepeats>0) {
                p->nRepeats--;
                DCC_LOGD_ISR("repeat packet = %d", p->nRepeat);
            } else {
                if(!packets.fetch_next_packet(*p)) {
                    *p = {idle_packet_bits, 1};
                }
            }
        }

        set_bit_timings();
        current_bit++;
    }

    inline bool current_bit_value() const {
        return (currentPacket.packet.buf[current_bit/8] & 1<<(7-current_bit%8) )!= 0;
    }


    inline void set_bit_timings() {
        if ( current_bit_value() ) {
            /* For "1" bit, we need 1 58us timer tick for each signal level */
            DCC_LOGD_ISR("bit %d (0x%02x) = 1", current_bit, currentPacket.packet.buf[current_bit/8] );
            timerPeriodsHalf = 1;
            timerPeriodsLeft = 2;
        } else {  /* ELSE it is a ZERO bit */
            /* For "0" bit, we need 2 58us timer ticks for each signal level */
            DCC_LOGD_ISR("bit %d (0x%02x) = 0", current_bit, currentPacket.packet.buf[current_bit/8] );
            timerPeriodsHalf = 2;
            timerPeriodsLeft = 4;
        }
    }

    //void IRAM_ATTR timerFunc();

};

}
