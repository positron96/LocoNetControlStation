#pragma once

#include "LocoAddress.h"
#include "LocoSpeed.h"
#include "packet.hpp"
#include "PacketList.hpp"

#include <esp32-hal-timer.h>
//#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#include <etl/map.h>
#include <etl/bitset.h>

#include <Arduino.h>

#include <atomic>

#define DCC_DEBUG

#ifdef DCC_DEBUG
#define DCC_LOGD(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#define DCC_LOGD_ISR(...)
#define DCC_LOGI(format, ...) log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define DCC_LOGI_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define DCC_LOGW(format, ...) log_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
//extern char _msg[1024];
//extern char _buf[100];
//#define DCC_DEBUG_ISR(...)  do{ snprintf(_buf, 100, __VA_ARGS__); snprintf(_msg, 1024, "%s%s\n", _msg, _buf ); } while(0)
//#define DCC_DEBUG_ISR_DUMP()  do{ Serial.print(_msg); _msg[0]=0; } while(0);
#else
#define DCC_LOGD(...)
#define DCC_LOGD_ISR(...)
#define DCC_LOGI(...)
#define DCC_LOGI_ISR( ...)
#define DCC_LOGW( ...)
#endif


namespace dcc {

constexpr float ADC_RESISTANCE = 0.1;
constexpr float ADC_MAX_MV = 1100;
constexpr float ADC_TO_MV = ADC_MAX_MV/4096;
constexpr float ADC_TO_MA = ADC_TO_MV / ADC_RESISTANCE;
constexpr uint16_t MAX_CURRENT = 2000;

extern uint8_t idlePacket[3];
extern uint8_t resetPacket[3];
extern PacketBits idle_packet_bits;


class BaseChannel {

public:

    BaseChannel(BasePacketList &packets): packets{packets} {}

    virtual void begin()=0;

    virtual void end()=0;

    virtual void setPower(bool v)=0;

    virtual bool getPower()=0;

    /**
     *
     */
    void sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd);
    void sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn);
    //void sendFunction(LocoAddress addr, uint8_t fByte, uint8_t eByte=0);
    /**
     * @param addr11 is 1-based.
     */
    void sendAccessory(uint16_t addr11, bool thr);
    /**
     * @param addr9 is 1-based
     * @param ch is 0-based.
     */
    // void sendAccessory(uint16_t addr9, uint8_t ch, bool);

    int16_t readCVProg(int cv);
    bool verifyCVByteProg(uint16_t cv, uint8_t bValue);
    bool writeCVByteProg(int cv, uint8_t bValue);
    bool writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue);
    void writeCVByteMain(LocoAddress addr, int cv, uint8_t bValue);
    void writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue);

    void unloadSlot(const LocoAddress addr) { packets.clear_loco(addr); }

    bool checkOvercurrent() {
        uint16_t v = getCurrent();
        float mA = v * ADC_TO_MA;
        //if(v!=0) DCC_LOGI("%d, %d", v, (int)mA);
        if(mA>MAX_CURRENT) {
            setPower(false);
            return false;
        }
        else return true;
    }

    void resetMaxCurrent() { maxCurrent = 0; }
    uint16_t getMaxCurrent() const { return maxCurrent; }
    uint16_t getCurrent() const { return current; }
    virtual void updateCurrent() = 0;

    virtual ~BaseChannel() = default;

protected:
    std::atomic<uint16_t> current;
    std::atomic<uint16_t> maxCurrent;

    BasePacketList &packets;

    virtual void timerFunc()=0;

    /** Tries to load a packet for a specified duration. */
    bool loadPacket(etl::span<uint8_t> packet, size_t nRepeat, size_t timeout_ms=1000) {
        for(size_t i=0; i<timeout_ms; i++) {
            if (packets.put_generic_packet(packet, nRepeat)) return true;
            delay(1);
        }
        return false;
    }

    uint getBaselineCurrent() const;
    bool checkCurrentResponse(uint baseline) const;

private:
    friend class DCCESP32SignalGenerator;
};

}
