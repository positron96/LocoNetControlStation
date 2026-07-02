#pragma once

#include "LocoAddress.h"
#include "LocoSpeed.h"
#include "accessory_address.hpp"
#include "packet.hpp"
#include "PacketList.hpp"
#include "power_event.hpp"
#include "log.hpp"

#include <etl/map.h>
#include <etl/bitset.h>
#include <etl/observer.h>

#include <Arduino.h>

#include <atomic>


namespace dcc {

extern Packet idlePacket;
extern Packet resetPacket;
extern PacketBits idle_packet_bits;

/**
 * A (abstract) class that manages one DCC track.
 *
 * It outputs DCC waveforms and reads current consumption
 *   for both CV operations and overpower protection.
 */
class BaseChannel: public etl::observable<PowerObserver, 5> {

public:

    BaseChannel(BasePacketList &packets): packets{packets} {}

    virtual void begin()=0;

    virtual void end()=0;

    virtual void setPower(bool v, PowerEvent::Reason reason = PowerEvent::Reason::Normal) {
        if(v) overCurrentFlag = false;
    }

    virtual bool getPower() const = 0;

    /**
     */
    void sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd);

    /** Sends a function group command to a locomotive.
     * Can either put it in a refreshing slot or only send once.
     */
    void sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn);

    /**
     * @param addr is accessory decoder address.
     */
    void sendAccessory(const AccessoryAddress &addr, bool thr);

    int16_t readCVProg(int cv);
    bool verifyCVByteProg(uint16_t cv, uint8_t bValue);
    bool writeCVByteProg(int cv, uint8_t bValue);
    bool writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue);
    void writeCVByteMain(LocoAddress addr, int cv, uint8_t bValue);
    void writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue);

    void unloadSlot(const LocoAddress addr) { packets.clear_loco(addr); }

    /** Different channels may have different thresholds. */
    void setOvercurrentThreshold(uint16_t mA) { overCurrentThreshold = mA; }
    bool checkOvercurrent() {
        uint16_t mA = getCurrent();
        if(mA > overCurrentThreshold) {
            overCurrentFlag = true;
            setPower(false, PowerEvent::Reason::Overcurrent);
            return false;
        }
        else return true;
    }
    bool getOvercurrentStatus() const {
        return overCurrentFlag;
    }

    void resetMaxCurrent() { maxCurrent = 0; }
    /** Max current encountered so far after resetMaxCurrent() was called. */
    uint16_t getMaxCurrent() const { return maxCurrent; }
    /**  Current consumption in mA */
    uint16_t getCurrent() const { return current; }
    /** Reads current consumption and updates internal state. */
    virtual void updateCurrent() = 0;

    virtual ~BaseChannel() = default;

protected:
    uint16_t overCurrentThreshold{std::numeric_limits<uint16_t>::max()}; ///< disabled until explicitly set
    std::atomic<uint16_t> current{0};
    std::atomic<uint16_t> maxCurrent{0};
    bool overCurrentFlag{false};

    BasePacketList &packets;

    /** Tries to schedule a packet for a specified duration and waits until it's sent to tracks. */
    bool sendPacketFully(const etl::span<uint8_t> packet, size_t nRepeat, size_t timeout_ms=1000);

    uint getBaselineCurrent() const;
    bool checkCurrentResponse(uint baseline) const;

};


/**
 * Thin container of channels to be current-monitored.
 **/
class CurrentMeter {
public:
    constexpr static size_t MAX_CHANNELS = 2;

    virtual void  begin() = 0;

    virtual void end() = 0;

    void addChannel(BaseChannel &ch) {
        channels.push_back(&ch);
    }

    void update() {
        for(auto ch: channels) {
            ch->updateCurrent();
        }
    }

    void checkOvercurrent() {
        for(auto ch: channels) {
            ch->checkOvercurrent();
        }
    }

protected:
    etl::vector<BaseChannel*, MAX_CHANNELS> channels;
};

}
