#pragma once

#include <Arduino.h>
#include <LocoNet.h>
#include <etl/map.h>
#include "CommandStation.h"

class LocoNetSlotManager : public LocoNetConsumer {

public:
    LocoNetSlotManager(LocoNetBus * const ln);

    //void initSlot(uint8_t i, uint8_t addrHi=0, uint8_t addrLo=0);

    LN_STATUS onMessage(const lnMsg& msg) override {
        processMessage(&msg);
        return LN_DONE;
    }

    void processMessage(const lnMsg* msg);


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

};
