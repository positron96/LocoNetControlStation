#pragma once

#include <Arduino.h>
#include <LocoNet2.h>
#include <etl/map.h>
#include "CommandStation.h"
#include "FastClock.hpp"

class LocoNetSlotManager : public LocoNetConsumer, public fast_clock::clock_observer  {

public:
    LocoNetSlotManager(LocoNetBus * const ln);

    //void initSlot(uint8_t i, uint8_t addrHi=0, uint8_t addrLo=0);

    LN_STATUS onMessage(const lnMsg& msg) override {
        processMessage(&msg);
        return LN_IDLE;
    }

    void processMessage(const lnMsg* msg);

    void notification(const fast_clock::ClockChangedEvent &event) override;

    bool isFastClockMaster() const { return isClockMaster; }

    void setFastClockMaster(bool v);


private:

    LocoNetBus * const _ln;

    uint8_t dispatchedSlot;

    struct LnSlotData {
        uint8_t ss2;
        uint8_t id1;
        uint8_t id2;
        LnSlotData(): ss2(0),id1(0),id2(0) {}
    };

    etl::map<uint8_t, LnSlotData, CommandStation::MAX_SLOTS> extra;

    static constexpr uint32_t CLOCK_SEND_INTL = 60'000; // send every minute
    bool isClockMaster{false}; ///< clock master sends periodic clock updates to the bus
    uint16_t clockSetterId{0}; ///< who set the clock. 0 means nobody has set it yet, 7F,7x means PC
    uint32_t clockSentTime{0};

    bool slotValid(uint8_t slot) {
        return (slot>=1) && (slot < CommandStation::MAX_SLOTS);
    }

    bool haveDispatchedSlot() { return slotValid(dispatchedSlot); }
    void removeDispatchedSlot() { dispatchedSlot = 0;}

    int locateSlot(uint8_t hi, uint8_t lo);

    void releaseSlot(uint8_t slot);

    void sendSlotData(uint8_t slot);

    void sendLack(uint8_t cmd, uint8_t arg=0);

    void sendProgData(progTaskMsg, uint8_t pstat, uint8_t value );

    void processDirf(uint8_t slot, uint v) ;

    void processSnd(uint8_t slot, uint8_t snd);

    void processStat1(uint8_t slot, uint8_t stat) ;

    void processSpd(uint8_t slot, uint8_t spd);

    void processProgMsg(const progTaskMsg &msg);

    void fillSlotMsg(uint8_t slot, rwSlotDataMsg &msg);

    void processFastClockMsg(const fastClockMsg &msg);

    void sendFastClock();

};
