#pragma once

#include <Arduino.h>

#include <LocoNet.h>

#include "DCC.h"


class LocoNetSlotManager {

public:
    LocoNetSlotManager(LocoNet * const ln): _ln(ln) {
        for(int i=0; i<MAX_SLOTS; i++) {
            initSlot(i);
        }
    }

    void initSlot(uint8_t i, uint8_t addrHi=0, uint8_t addrLo=0) {
        rwSlotDataMsg &sd = _slots[i];
        sd.command = 0xE7;
        sd.mesg_size = 14;
        sd.slot = i;
        sd.stat = DEC_MODE_128 | LOCO_FREE;
        sd.adr = addrLo; 
        sd.spd = 0; 
        sd.dirf = 0;
        sd.trk = GTRK_POWER & GTRK_MLOK1; // POWER ON & Loconet 1.1 by default; 
        sd.ss2 = 0; 
        sd.adr2 = addrHi; 
        sd.snd = 0; 
        sd.id1 = i; 
        sd.id2 = 0;
    }

    void registerCallbacks() {
        _ln->onPacket(CALLBACK_FOR_ALL_OPCODES, [this](lnMsg* msg) { processMessage(msg); } );
    }

    void processMessage(lnMsg* msg) {

        switch(msg->data[0]) {
            case OPC_LOCO_ADR: {

                int slot = locateSlot( msg->la.adr_hi,  msg->la.adr_lo );
                if(slot<0) {
                    lnMsg lack = makeLongAck(OPC_LOCO_ADR, 0);
                    _ln->send(&lack);
                    Serial.printf("OPC_LOCO_ADR for addr %d, no slots\n", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                    break;
                }
                
                sendSlotData(slot);
                Serial.printf("OPC_LOCO_ADR for addr %d, slot is %d\n", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                if( msg->sm.dest!=msg->sm.src || !slotValid(msg->sm.dest) || !slotValid(msg->sm.src) ) {
                    lnMsg lack = makeLongAck(OPC_MOVE_SLOTS, 0);
                    _ln->send(&lack);
                } else {
                    rwSlotDataMsg & slot = _slots[msg->sm.src];
                    Serial.printf("OPC_MOVE_SLOTS NULL MOVE for slot %d\n", slot.slot );
                    slot.stat |= LOCO_IN_USE;
                    notifyDcc(slot.slot);
                    sendSlotData(slot.slot);
                }
                break;
            }
            case OPC_LOCO_SND: {
                uint8_t slot = msg->ls.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_SND, 0); _ln->send(&lack); break;} 
                _slots[slot].snd = msg->ls.snd;
                Serial.printf("OPC_LOCO_SND slot %d snd %02x\n", slot, msg->ls.snd);
                notifyDcc(slot, NOTIFY_SND);
                break;
            }
            case OPC_LOCO_DIRF: {
                uint8_t slot = msg->ldf.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_DIRF, 0); _ln->send(&lack); break;} 
                _slots[slot].dirf = msg->ldf.dirf;
                Serial.printf("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, msg->ldf.dirf);
                notifyDcc(slot, NOTIFY_DIRF);
                break;
            }
            case OPC_LOCO_SPD : {
                uint8_t slot = msg->lsp.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_SPD, 0); _ln->send(&lack); break;} 
                _slots[slot].spd = msg->lsp.spd;
                Serial.printf("OPC_LOCO_SPD slot %d spd %02x\n", slot, msg->lsp.spd);
                notifyDcc(slot, NOTIFY_SPD);
                break;
            }
            case OPC_WR_SL_DATA: {
                uint8_t slot = msg->sd.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_WR_SL_DATA, 0); _ln->send(&lack); break;} 
                _slots[slot] = msg->sd;
                Serial.printf("OPC_WR_SL_DATA slot %d\n", slot);
                notifyDcc(slot);
                break;
            }
            case OPC_RQ_SL_DATA: {
                uint8_t slot = msg->sr.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_RQ_SL_DATA, 0); _ln->send(&lack); break;} 
                Serial.printf("OPC_RQ_SL_DATA slot %d\n", slot);
                sendSlotData(slot);
            }
            case OPC_SLOT_STAT1: {
                uint8_t slot = msg->ss.slot;
                if( (_slots[slot].stat & LOCOSTAT_MASK) != (msg->ss.stat&LOCOSTAT_MASK) )
                    Serial.println("Changing active+busy");
                _slots[slot].stat = msg->ss.stat;
                Serial.printf("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, msg->ss.stat);
                notifyDcc(slot, NOTIFY_STAT);
                break;
            }
        }
        
    } 

    void setDccMainChannel(DCCESP32Channel * dcc) { _dccMain = dcc; }

private:

    LocoNet * const _ln;

    DCCESP32Channel * _dccMain;

    static constexpr uint8_t MAX_SLOTS = 10;

    rwSlotDataMsg _slots[MAX_SLOTS];

    bool slotValid(uint8_t slot) {
        return (slot>=1) && (slot<MAX_SLOTS);
    }

    static void setMasked(uint8_t &val, uint8_t mask, uint8_t v) {
        val = ( val & ~mask) | v;
    }

    int locateSlot(uint8_t hi, uint8_t lo) {
        uint8_t firstFreeSlot = 0;
        for(uint8_t i=1; i<MAX_SLOTS; i++) {
            Serial.printf("testing slot %d, stat=%02x, addr1=%d, addr2=%d\n",
                i, _slots[i].stat, _slots[i].adr, _slots[i].adr2
                );
            if( (_slots[i].stat & LOCOSTAT_MASK) == LOCO_FREE)  { if(firstFreeSlot==0) firstFreeSlot=i; } 
            else {
                if(_slots[i].adr==lo  && _slots[i].adr2==hi) return i; // found slot with this loco
            }
        }
        if(firstFreeSlot!=0) { 
            // allocate new slot for this loco
            initSlot(firstFreeSlot, hi, lo);
            return firstFreeSlot;
        }
        return -1;
    }

    static const uint8_t NOTIFY_DIRF = 0x1;
    static const uint8_t NOTIFY_SND  = 0x2;
    static const uint8_t NOTIFY_SS2  = 0x4;
    static const uint8_t NOTIFY_SPD  = 0x8;
    static const uint8_t NOTIFY_STAT = 0x10;

    void notifyDcc(uint8_t nslot, uint8_t changeMask=0xFF) {
        rwSlotDataMsg * slot = & _slots[nslot];
        if(_dccMain!=nullptr) {
            uint16_t addr = ADDR(slot->adr2, slot->adr);
            if( changeMask & NOTIFY_STAT) {
                if(slot->stat & STAT1_SL_ACTIVE) {
                    // add this loco
                } else {
                    // remove this loco
                }
            }
            if( (slot->stat & STAT1_SL_ACTIVE) == 0) return; // do not refresh
            if( changeMask & (NOTIFY_DIRF|NOTIFY_SPD) ) _dccMain->setThrottle(slot->slot, addr, slot->spd, bitRead(slot->dirf, 5)  );
            if( changeMask & NOTIFY_DIRF) _dccMain->setFunction(slot->slot, addr, 128 | (slot->dirf & B00011111) );
            if( changeMask & NOTIFY_SND ) _dccMain->setFunction(slot->slot, addr, 176 | (slot->snd & B00001111) );
        }
    }

    void sendSlotData(uint8_t slot) {
        
        lnMsg ret;
        ret.sd = _slots[slot];
        Serial.print("tx'd");
        for(uint8_t i=0; i<lnPacketSize(&ret); i++) {
            Serial.printf(" %02X", ret.data[i]);
        }
        Serial.println("");
        

        _ln->send(&ret);
    }


};