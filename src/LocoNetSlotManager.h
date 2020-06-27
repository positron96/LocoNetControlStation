#pragma once

#include <Arduino.h>
#include <LocoNet.h>
#include "CommandStation.h"

#define LNSM_DEBUG

#ifdef LNSM_DEBUG
#define LNSM_DEBUGF(...)  { Serial.printf(__VA_ARGS__); }
#else
#define LNSM_DEBUGF(...)
#endif

class LocoNetSlotManager : public LocoNetConsumer {

public:
    LocoNetSlotManager(LocoNetBus * const ln): _ln(ln) {
        for(int i=0; i<MAX_SLOTS; i++) {
            initSlot(i);
        }

        ln->addConsumer(this);
    }

    void initSlot(uint8_t i, uint8_t addrHi=0, uint8_t addrLo=0) {
        rwSlotDataMsg &sd = _slots[i];
        sd.command = 0xE7;
        sd.mesg_size = 14;
        sd.slot = i;
        sd.stat = DEC_MODE_128 | LOCO_FREE;
        sd.adr = addrLo; 
        sd.spd = 0; 
        sd.dirf = DIRF_DIR;  // FWD
        sd.trk = GTRK_IDLE | GTRK_POWER | GTRK_MLOK1; // POWER ON & Loconet 1.1 by default; 
        sd.ss2 = 0; 
        sd.adr2 = addrHi; 
        sd.snd = 0; 
        sd.id1 = i; 
        sd.id2 = 0;
    }

    virtual LN_STATUS onMessage(const lnMsg& msg) {
        processMessage(&msg);
        return LN_DONE;
    }

    void processMessage(const lnMsg* msg) {

        switch(msg->data[0]) {
            case OPC_GPON:
                CS.setPowerState(true);
                break;
            case OPC_GPOFF:
                CS.setPowerState(false);
                break;
            case OPC_LOCO_ADR: {
                int slot = locateSlot( msg->la.adr_hi,  msg->la.adr_lo );
                if(slot<=0) {
                    sendLack(OPC_LOCO_ADR);
                    Serial.printf("OPC_LOCO_ADR for addr %d, no slots\n", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                    break;
                }
                
                sendSlotData(slot);
                LNSM_DEBUGF("OPC_LOCO_ADR for addr %d, slot is %d\n", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                if( msg->sm.dest!=msg->sm.src || !slotValid(msg->sm.dest) || !slotValid(msg->sm.src) ) {
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    uint8_t slot = msg->ss.slot;
                    LNSM_DEBUGF("OPC_MOVE_SLOTS NULL MOVE for slot %d\n", slot );
                    _slots[slot].stat |= LOCO_IN_USE;
                    CS.setLocoRefresh(slot, true);
                    sendSlotData(slot);
                }
                break;
            }
            case OPC_SLOT_STAT1: {
                uint8_t slot = msg->ss.slot;
                if( !slotValid(slot) ) { sendLack(OPC_LOCO_SND); break; } 
                processStat1(slot, msg->ss.stat);
                break;
            }
            case OPC_LOCO_SND: {
                uint8_t slot = msg->ls.slot;
                if( !slotValid(slot) ) { sendLack(OPC_LOCO_SND); break; } 
                processSnd(slot, msg->ls.snd);
                break;
            }
            case OPC_LOCO_DIRF: {
                uint8_t slot = msg->ldf.slot;
                if( !slotValid(slot) ) { sendLack(OPC_LOCO_DIRF); break; } 
                processDirf(slot, msg->ldf.dirf);
                break;
            }
            case OPC_LOCO_SPD : {
                uint8_t slot = msg->lsp.slot;
                if( !slotValid(slot) ) { sendLack(OPC_LOCO_SPD); break; } 
                processSpd(slot, msg->lsp.spd);
                break;
            }
            case OPC_WR_SL_DATA: {
                const rwSlotDataMsg & m = msg->sd;
                uint8_t slot = m.slot;
                if( !slotValid(slot) ) { sendLack(OPC_WR_SL_DATA); break; } 
                rwSlotDataMsg &_slot = _slots[slot];

                if(_slot.stat != m.stat) processStat1(slot, m.stat);
                if( !CS.isSlotAllocated(slot) ) return; // stat1 sets slot to inactive, do not continue
                if(_slot.spd != m.spd) processSpd(slot, m.spd);
                if(_slot.dirf != m.dirf) processDirf(slot, m.dirf);
                if(_slot.snd != m.snd) processSnd(slot, m.snd);
                
                _slot.adr = m.adr;
                _slot.trk = m.trk;
                _slot.ss2 = m.ss2;
                _slot.adr2 = m.adr2;
                _slot.id1 = m.id1;
                _slot.id2 = m.id2;

                //_slot = msg->sd;

                LNSM_DEBUGF("OPC_WR_SL_DATA slot %d\n", slot);
                break;
            }
            case OPC_RQ_SL_DATA: {
                uint8_t slot = msg->sr.slot;
                if( !slotValid(slot) ) { sendLack(OPC_RQ_SL_DATA); break;} 
                LNSM_DEBUGF("OPC_RQ_SL_DATA slot %d\n", slot);
                sendSlotData(slot);
            }
        }
        
    } 


private:

    LocoNetBus * const _ln;

    static const int MAX_SLOTS = CommandStation::MAX_SLOTS;

    rwSlotDataMsg _slots[MAX_SLOTS];

    bool slotValid(uint8_t slot) {
        return (slot>=1) && (slot < MAX_SLOTS);
    }

    int locateSlot(uint8_t hi, uint8_t lo) {
        LocoAddress addr = hi==0 ? LocoAddress::shortAddr(lo) : LocoAddress::longAddr(ADDR(hi,lo));
        uint8_t slot = CS.findLocoSlot(addr);
        if(slot==0) {
            slot = CS.locateFreeSlot();
            if(slot==0) return 0;
            CS.initLocoSlot(slot, addr);
            initSlot(slot, hi, lo);
        }
        return slot;
    }

    void releaseSlot(uint8_t slot) {
        CS.releaseLocoSlot(slot);
        _slots[slot].stat &= ~STAT1_SL_BUSY;
    }

    void sendSlotData(uint8_t slot) {        
        LnMsg ret;
        ret.sd = _slots[slot];
        LNSM_DEBUGF("LocoNetSlotManager::sendSlotData: sending ");
        for(uint8_t i=0; i<ret.length(); i++) {
            LNSM_DEBUGF(" %02X", ret.data[i]);
        }
        LNSM_DEBUGF("\n");

        _ln->broadcast(ret, this);
    }

    void sendLack(uint8_t cmd, uint8_t arg=0) {
        LnMsg lack = makeLongAck(cmd, arg); 
        _ln->broadcast(lack, this);
    }

    void processDirf(uint8_t slot, uint v) {
        LNSM_DEBUGF("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, v);
        _slots[slot].dirf = v;
        uint8_t dir = (v & DIRF_DIR) == DIRF_DIR ? 1 : 0;
        CS.setLocoDir(slot, dir);
        CS.setLocoFns(slot, 0x1F, (v & B00001111)<<1 | (v & B00010000)>>4 );  // fn order in this byte is 04321
    }

    void processSnd(uint8_t slot, uint8_t snd) {
        LNSM_DEBUGF("OPC_LOCO_SND slot %d snd %02x\n", slot, snd);
        CS.setLocoFns(slot, 0x1E0, snd << 5 );
        _slots[slot].snd = snd;
    }

    void processStat1(uint8_t slot, uint8_t stat) {
        LNSM_DEBUGF("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, stat);

        if( (_slots[slot].stat & LOCOSTAT_MASK) != (stat&LOCOSTAT_MASK) ) {
            LNSM_DEBUGF("Changing active+busy: %02x\n", stat&LOCOSTAT_MASK);
            if( (stat & STAT1_SL_BUSY) == 0) { 
                releaseSlot(slot);
                return;
            }

            CS.setLocoRefresh(slot, (stat & STAT1_SL_ACTIVE) != 0);
        }
        _slots[slot].stat = stat;
    }

    void processSpd(uint8_t slot, uint8_t spd) {
        LNSM_DEBUGF("OPC_LOCO_SPD slot %d spd %d\n", slot, spd);
        CS.setLocoSpeed(slot, spd);
        _slots[slot].spd = spd;
    }

};
