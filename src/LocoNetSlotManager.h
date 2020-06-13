#pragma once

#include <Arduino.h>
#include <tuple>

#include <LocoNet.h>


class LocoNetSlotManager {

public:
    LocoNetSlotManager(LocoNet * const ln): _ln(ln)
    {
        for(int i=0; i<MAX_SLOTS; i++) {
            rwSlotDataMsg &sd = _slots[i];
            sd.command = 0xE7;
            sd.mesg_size = 14;
            sd.slot = i;
            sd.stat = DEC_MODE_128 | LOCO_FREE;
            sd.adr = 0; 
            sd.spd = 0; 
            sd.dirf = 1;
            sd.trk = 7; 
            sd.ss2 = 2; 
            sd.adr2 = 0; 
            sd.snd = 0; 
            sd.id1 = i; 
            sd.id2 = 0;
        }
    }

    static lnMsg makeLack(uint8_t cmd, uint8_t resp=0) {
        lnMsg lack;
        lack.lack.opcode = OPC_LONG_ACK;
        lack.lack.command = cmd;
        lack.lack.ack1 = resp;
        return lack;
    }

    void registerCallbacks() {
        _ln->onPacket(CALLBACK_FOR_ALL_OPCODES, [this](lnMsg* msg) {

            switch(msg->data[0]) {
                case OPC_LOCO_ADR: {

                    int slot = locateSlot( msg->la.adr_hi,  msg->la.adr_lo );
                    if(slot<0) {
                        lnMsg lack = makeLack(OPC_LOCO_ADR, 0);
                        _ln->send(&lack);
                        Serial.printf("OPC_LOCO_ADR for addr %d, no slots\n", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                        return;
                    }
                    
                    _slots[slot].adr = msg->la.adr_lo;
                    _slots[slot].adr2 = msg->la.adr_hi;
                    sendSlotData(slot);
                    Serial.printf("OPC_LOCO_ADR for addr %d, slot is %d\n", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                    break;
                }
                case OPC_MOVE_SLOTS: {
                    if(msg->sm.src == msg->sm.dest) {
                        uint8_t slot = msg->sm.src;
                        Serial.printf("OPC_MOVE_SLOTS NULL MOVE for slot %d\n", slot);
                        setMasked(_slots[slot].stat, LOCOSTAT_MASK, LOCO_IN_USE);
                        sendSlotData(slot);
                    } else {
                        lnMsg lack = makeLack(OPC_MOVE_SLOTS, 0);
                        _ln->send(&lack);
                    }
                    break;
                }
                case OPC_LOCO_SND: {
                    uint8_t slot = msg->ls.slot;
                    _slots[slot].snd = msg->ls.snd;
                    Serial.printf("OPC_LOCO_SND slot %d snd %02x\n", slot, msg->ls.snd);
                    break;
                }
                case OPC_LOCO_DIRF: {
                    uint8_t slot = msg->ldf.slot;
                    _slots[slot].dirf = msg->ldf.dirf;
                    Serial.printf("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, msg->ldf.dirf);
                    break;
                }
                case OPC_LOCO_SPD : {
                    uint8_t slot = msg->lsp.slot;
                    _slots[slot].spd = msg->lsp.spd;
                    Serial.printf("OPC_LOCO_SPD slot %d spd %02x\n", slot, msg->lsp.spd);
                    break;
                }
                case OPC_WR_SL_DATA: {
                    uint8_t slot = msg->sd.slot;
                    _slots[slot] = msg->sd;
                    Serial.printf("OPC_WR_SL_DATA slot %d\n", slot);
                    break;
                }
                case OPC_RQ_SL_DATA: {
                    uint8_t slot = msg->sr.slot;
                    Serial.printf("OPC_RQ_SL_DATA slot %d\n", slot);
                    sendSlotData(slot);
                }
                case OPC_SLOT_STAT1: {
                    uint8_t slot = msg->ss.slot;
                    _slots[slot].stat = msg->ss.stat;
                    Serial.printf("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, msg->ss.stat);
                    break;
                }
            }

            
        } );

        
    }

private:

    LocoNet * const _ln;

    static constexpr uint8_t MAX_SLOTS = 10;

    rwSlotDataMsg _slots[MAX_SLOTS];

    static void setMasked(uint8_t &val, uint8_t mask, uint8_t v) {
        val = ( val & ~mask) | v;
    }

    int locateSlot(uint8_t hi, uint8_t lo) {
        uint8_t firstFreeSlot = 0xFF;
        for(uint8_t i=0; i<MAX_SLOTS; i++) {
            Serial.printf("testing slot %d, stat=%02x, addr1=%d, addr2=%d\n",
                i, _slots[i].stat, _slots[i].adr, _slots[i].adr2
                );
            if( (_slots[i].stat & LOCOSTAT_MASK) == LOCO_FREE)  { if(firstFreeSlot==0xFF) firstFreeSlot=i; } 
            else {
                if(_slots[i].adr==lo  && _slots[i].adr2==hi) return i;
            }
        }
        if(firstFreeSlot!=0xFF) return firstFreeSlot;
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