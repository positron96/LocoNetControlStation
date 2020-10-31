#pragma once

#include <Arduino.h>
#include <LocoNet.h>
#include "CommandStation.h"

class LocoNetSlotManager : public LocoNetConsumer {

public:
    LocoNetSlotManager(LocoNetBus * const ln);

    void initSlot(uint8_t i, uint8_t addrHi=0, uint8_t addrLo=0);

    LN_STATUS onMessage(const lnMsg& msg) override {
        processMessage(&msg);
        return LN_DONE;
    }

    void processMessage(const lnMsg* msg);


private:

    LocoNetBus * const _ln;

    static const int MAX_SLOTS = CommandStation::MAX_SLOTS;

    rwSlotDataMsg _slots[MAX_SLOTS];
    
    bool slotValid(uint8_t slot) {
        return (slot>=1) && (slot < MAX_SLOTS);
    }

    int locateSlot(uint8_t hi, uint8_t lo);

    void releaseSlot(uint8_t slot);

    void sendSlotData(uint8_t slot);

    void sendLack(uint8_t cmd, uint8_t arg=0);

    void sendProgData(progTaskMsg, uint8_t pstat, uint16_t cv, uint8_t value );

    void processDirf(uint8_t slot, uint v) ;

    void processSnd(uint8_t slot, uint8_t snd);

    void processStat1(uint8_t slot, uint8_t stat) ;

    void processSpd(uint8_t slot, uint8_t spd);


    void processProgMsg(const progTaskMsg &msg);

};
