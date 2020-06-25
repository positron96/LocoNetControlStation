#pragma once

#include <Arduino.h>

#include <LocoNet.h>

#include "CommandStation.h"

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
                //int slot = CS.locate
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
                    CS.setLocoRefresh(slot.slot, true);
                    sendSlotData(slot.slot);
                }
                break;
            }
            case OPC_SLOT_STAT1: {
                uint8_t slot = msg->ss.slot;
                Serial.printf("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, msg->ss.stat);

                if( (_slots[slot].stat & LOCOSTAT_MASK) != (msg->ss.stat&LOCOSTAT_MASK) ) {
                    Serial.println("Changing active+busy");
                    CS.setLocoRefresh(slot, (msg->ss.stat & STAT1_SL_ACTIVE) != 0);
                }
                _slots[slot].stat = msg->ss.stat;
                break;
            }
            case OPC_LOCO_SND: {
                uint8_t slot = msg->ls.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_SND, 0); _ln->send(&lack); break;} 
                _slots[slot].snd = msg->ls.snd;
                Serial.printf("OPC_LOCO_SND slot %d snd %02x\n", slot, msg->ls.snd);
                CS.setLocoFnGroup(slot, 0x1E0, msg->ls.snd << 5 );
                break;
            }
            case OPC_LOCO_DIRF: {
                uint8_t slot = msg->ldf.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_DIRF, 0); _ln->send(&lack); break;} 
                uint8_t v = msg->ldf.dirf;
                _slots[slot].dirf = v;
                Serial.printf("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, v);
                CS.setLocoDir(slot, v);
                CS.setLocoFnGroup(slot, 0x1F, (v & 0xF)<<1 | (v & 0x10)>>4 );
                break;
            }
            case OPC_LOCO_SPD : {
                uint8_t slot = msg->lsp.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_LOCO_SPD, 0); _ln->send(&lack); break;} 
                _slots[slot].spd = msg->lsp.spd;
                Serial.printf("OPC_LOCO_SPD slot %d spd %02x\n", slot, msg->lsp.spd);
                CS.setLocoSpeed(slot, msg->lsp.spd);
                break;
            }
            case OPC_WR_SL_DATA: {
                uint8_t slot = msg->sd.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_WR_SL_DATA, 0); _ln->send(&lack); break;} 
                _slots[slot] = msg->sd;
                Serial.printf("OPC_WR_SL_DATA slot %d\n", slot);
                //notifyDcc(slot);
                break;
            }
            case OPC_RQ_SL_DATA: {
                uint8_t slot = msg->sr.slot;
                if( !slotValid(slot) ) { lnMsg lack = makeLongAck(OPC_RQ_SL_DATA, 0); _ln->send(&lack); break;} 
                Serial.printf("OPC_RQ_SL_DATA slot %d\n", slot);
                sendSlotData(slot);
            }
        }
        
    } 


private:

    LocoNet * const _ln;

    static const int MAX_SLOTS = CommandStation::MAX_SLOTS;

    rwSlotDataMsg _slots[MAX_SLOTS];

    bool slotValid(uint8_t slot) {
        return (slot>=1) && (slot < MAX_SLOTS);
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
