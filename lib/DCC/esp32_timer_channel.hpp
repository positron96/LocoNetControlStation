#pragma once

#include "esp32_channel.hpp"

namespace dcc {

/**
 * A DCC channel that outputs DCC waveform using ESP32 timer.
 *
 * Use in conjunction with ESP32Timer.
 *
 */
class ESP32TimerChannel: public ESP32Channel {
public:

    ESP32TimerChannel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin, BasePacketList &packets):
        ESP32Channel{outputPin, enPin, sensePin, packets}
    {
    }

    void IRAM_ATTR timerFunc() {
        timerPeriodsLeft = timerPeriodsLeft - 1;
        //DCC_DEBUGF_ISR("ESP32TimerChannel::timerFunc, periods left: %d, total: %d\n", R.timerPeriodsLeft, R.timerPeriodsHalf*2);
        if(timerPeriodsLeft == timerPeriodsHalf) {
            gpio_set_level(static_cast<gpio_num_t>(_outputPin), 1);
        }
        if(timerPeriodsLeft == 0) {
            gpio_set_level(static_cast<gpio_num_t>(_outputPin), 0);
            nextBit();
        }
    }

private:

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
                DCC_LOGD_ISR("repeat packet = %d", p->nRepeats);
            } else {
                if(!packets.fetch_next_packet(*p)) {
                    *p = {idle_packet_bits, 1};
                }
                DCC_LOGD_ISR("next packet: [%d]=[%02X %02X...]*%d", p->packet.len, p->packet.buf[0], p->packet.buf[1], p->nRepeats);
            }
        }

        set_bit_timings();
        current_bit++;
    }

    inline void set_bit_timings() {
        if ( currentPacket.packet.bit_at(current_bit) != 0 ) {
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

};

}
