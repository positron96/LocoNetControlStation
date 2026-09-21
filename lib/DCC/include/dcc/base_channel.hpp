#pragma once

#include "loco_address.hpp"
#include "loco_speed.hpp"
#include "accessory_address.hpp"
#include "packet.hpp"
#include "packet_list.hpp"
#include "power_event.hpp"
#include "log.hpp"

#include <etl/map.h>
#include <etl/bitset.h>
#include <etl/observer.h>
#include <etl/enum_type.h>
#include <etl/expected.h>

#include <Arduino.h>

#include <atomic>


namespace dcc {

extern Packet idlePacket;
extern Packet resetPacket;
extern PacketBits idle_packet_bits;

struct CvCommError {
    enum enum_type {
        Timeout,
        NoResponse,
        InvalidState,
   };

   ETL_DECLARE_ENUM_TYPE(CvCommError, unsigned)
   ETL_ENUM_TYPE(Timeout, "Timeout")
   ETL_ENUM_TYPE(NoResponse, "No Response")
   ETL_ENUM_TYPE(InvalidState, "Invalid State")
   ETL_END_ENUM_TYPE
};

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
        if(v) {
            // clear overcurrent states
            overCurrentFlag = false;
            overCurrentEventPending = false;
        }
    }

    virtual bool getPower() const = 0;

    /**
     * Sends throttle command and resends it periodically.
     */
    void sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd);

    /** Sends throttle command once. */
    void sendThrottleOnce(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd);

    /** Sends a function group command to a locomotive.
     * Can either put it in a refreshing slot or send once directly to tracks.
     */
    void sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn);

    /**
     * @param addr is accessory decoder address.
     */
    void sendAccessory(const AccessoryAddress &addr, bool thr);

    etl::expected<uint8_t, CvCommError> readCVProg(uint16_t cv);
    etl::expected<bool, CvCommError> verifyCVByteProg(uint16_t cv, uint8_t value);
    etl::expected<void, CvCommError> writeCVByteProg(uint16_t cv, uint8_t value);
    etl::expected<void, CvCommError> writeCVBitProg(uint16_t cv, uint8_t bit_num, uint8_t value);
    void writeCVByteMain(LocoAddress addr, uint16_t cv, uint8_t value);
    void writeCVBitMain(LocoAddress addr, uint16_t cv, uint8_t bit_num, uint8_t value);

    void unloadSlot(const LocoAddress addr) { packets.clear_loco(addr); }

    /** Different channels may have different thresholds. */
    void setOvercurrentThreshold(uint16_t mA) { overCurrentThreshold = mA; }
    /** Should be called periodically to broadcast overcurrent events. */
    bool checkOvercurrent() {
        if(overCurrentEventPending) {
            overCurrentEventPending = false;
            notify_observers(PowerEvent{false, PowerEvent::Reason::Overcurrent, this});
        }
        return overCurrentFlag;
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

    virtual size_t diagGetPacketsSent() const { return 0; }

protected:
    uint16_t overCurrentThreshold{std::numeric_limits<uint16_t>::max()}; ///< disabled until explicitly set
    std::atomic<uint16_t> current{0};
    std::atomic<uint16_t> maxCurrent{0};
    bool overCurrentFlag{false}; // retained until power is turned back on.
    bool overCurrentEventPending{false}; // set when overcurrent is detected, checked in checkOvercurrent()

    BasePacketList &packets;

    /** Tries to schedule a packet for a specified duration and waits until it's sent to tracks. */
    bool sendPacketFully(const etl::span<uint8_t> packet, size_t nRepeat, size_t timeout_ms=1000);

    unsigned getBaselineCurrent();
    bool checkCurrentResponse(unsigned baseline) const;

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

    // void update() {
    //     for(auto ch: channels) {
    //         ch->updateCurrent();
    //     }
    // }

    void checkOvercurrent() {
        for(auto ch: channels) {
            ch->checkOvercurrent();
        }
    }

protected:
    etl::vector<BaseChannel*, MAX_CHANNELS> channels;
};

}
