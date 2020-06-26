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
                int slot = locateSlot( msg->la.adr_hi,  msg->la.adr_lo );
                if(slot<=0) {
                    sendLack(OPC_LOCO_ADR);
                    Serial.printf("OPC_LOCO_ADR for addr %d, no slots\n", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                    break;
                }
                
                sendSlotData(slot);
                Serial.printf("OPC_LOCO_ADR for addr %d, slot is %d\n", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                if( msg->sm.dest!=msg->sm.src || !slotValid(msg->sm.dest) || !slotValid(msg->sm.src) ) {
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    uint8_t slot = msg->ss.slot;
                    Serial.printf("OPC_MOVE_SLOTS NULL MOVE for slot %d\n", slot );
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
                rwSlotDataMsg & m = msg->sd;
                uint8_t slot = m.slot;
                if( !slotValid(slot) ) { sendLack(OPC_WR_SL_DATA); break; } 
                rwSlotDataMsg &_slot = _slots[slot];
                if(_slot.spd != m.spd) processSpd(slot, m.spd);
                if(_slot.dirf != m.dirf) processDirf(slot, m.dirf);
                if(_slot.stat != m.stat) processStat1(slot, m.stat);
                if(_slot.snd != m.snd) processSnd(slot, m.snd);
                
                _slot.adr = m.adr;
                _slot.trk = m.trk;
                _slot.ss2 = m.ss2;
                _slot.adr2 = m.adr2;
                _slot.id1 = m.id1;
                _slot.id2 = m.id2;

                //_slot = msg->sd;

                Serial.printf("OPC_WR_SL_DATA slot %d\n", slot);
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
        LocoAddress addr = hi==0 ? LocoAddress::shortAddr(lo) : LocoAddress::longAddr(ADDR(hi,lo));
        return CS.findOrAllocateLocoSlot(addr);
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

    void sendLack(uint8_t cmd, uint8_t arg=0) {
        lnMsg lack = makeLongAck(cmd, arg); 
        _ln->send(&lack); 
    }

    void processDirf(uint8_t slot, uint v) {
        Serial.printf("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, v);
        _slots[slot].dirf = v;
        CS.setLocoDir(slot, v);
        CS.setLocoFnGroup(slot, 0x1F, (v & 0xF)<<1 | (v & 0x10)>>4 );// fn order in this byte is 04321
    }

    void processSnd(uint8_t slot, uint8_t snd) {
        _slots[slot].snd = snd;
        Serial.printf("OPC_LOCO_SND slot %d snd %02x\n", slot, snd);
        CS.setLocoFnGroup(slot, 0x1E0, msg->ls.snd << 5 );
    }

    void processStat1(uint8_t slot, uint8_t stat) {
        Serial.printf("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, stat);

        if( (_slots[slot].stat & LOCOSTAT_MASK) != (stat&LOCOSTAT_MASK) ) {
            Serial.println("Changing active+busy");
            CS.setLocoRefresh(slot, (stat & STAT1_SL_ACTIVE) != 0);
        }
        _slots[slot].stat = stat;
    }

    void processSpd(uint8_t slot, uint8_t spd) {
        _slots[slot].spd = spd;
        Serial.printf("OPC_LOCO_SPD slot %d spd %02x\n", slot, spd);
        CS.setLocoSpeed(slot, spd);
    }

};
