#include "LocoNetSlotManager.h"

#define LNSM_DEBUG

#ifdef LNSM_DEBUG
#define LNSM_DEBUGF(...)  { Serial.printf(__VA_ARGS__); }
#else
#define LNSM_DEBUGF(...)
#endif


static LocoAddress lnAddr(uint16_t addr) {
    if(addr<=127) return LocoAddress::shortAddr(addr);
    return LocoAddress::longAddr(addr);
}

    LocoNetSlotManager::LocoNetSlotManager(LocoNetBus * const ln): _ln(ln) {
        for(int i=0; i<MAX_SLOTS; i++) {
            initSlot(i);
        }

        ln->addConsumer(this);
    }

    void LocoNetSlotManager::initSlot(uint8_t i, uint8_t addrHi, uint8_t addrLo) {
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

    void LocoNetSlotManager::processMessage(const lnMsg* msg) {

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
                    CS.setLocoSlotRefresh(slot, true);
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
                if(m.slot == PRG_SLOT) {
                    processProgMsg(msg->pt);
                    return;
                }
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



    int LocoNetSlotManager::locateSlot(uint8_t hi, uint8_t lo) {
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

    void LocoNetSlotManager::releaseSlot(uint8_t slot) {
        CS.releaseLocoSlot(slot);
        _slots[slot].stat &= ~STAT1_SL_BUSY;
    }

    void LocoNetSlotManager::sendSlotData(uint8_t slot) {        
        LnMsg ret;
        ret.sd = _slots[slot];
        LNSM_DEBUGF("LocoNetSlotManager::sendSlotData: sending ");
        for(uint8_t i=0; i<ret.length(); i++) {
            LNSM_DEBUGF(" %02X", ret.data[i]);
        }
        LNSM_DEBUGF("\n");

        _ln->broadcast(ret, this);
    }

    void LocoNetSlotManager::sendLack(uint8_t cmd, uint8_t arg) {
        LnMsg lack = makeLongAck(cmd, arg); 
        _ln->broadcast(lack, this);
    }

    void LocoNetSlotManager::processDirf(uint8_t slot, uint v) {
        LNSM_DEBUGF("OPC_LOCO_DIRF slot %d dirf %02x\n", slot, v);
        _slots[slot].dirf = v;
        uint8_t dir = (v & DIRF_DIR) == DIRF_DIR ? 1 : 0;
        CS.setLocoDir(slot, dir);
        CS.setLocoFns(slot, 0x1F, (v & B00001111)<<1 | (v & B00010000)>>4 );  // fn order in this byte is 04321
    }

    void LocoNetSlotManager::processSnd(uint8_t slot, uint8_t snd) {
        LNSM_DEBUGF("OPC_LOCO_SND slot %d snd %02x\n", slot, snd);
        CS.setLocoFns(slot, 0x1E0, snd << 5 );
        _slots[slot].snd = snd;
    }

    void LocoNetSlotManager::processStat1(uint8_t slot, uint8_t stat) {
        LNSM_DEBUGF("OPC_SLOT_STAT1 slot %d stat1 %02x\n", slot, stat);

        if( (_slots[slot].stat & LOCOSTAT_MASK) != (stat&LOCOSTAT_MASK) ) {
            LNSM_DEBUGF("Changing active+busy: %02x\n", stat&LOCOSTAT_MASK);
            if( (stat & STAT1_SL_BUSY) == 0) { 
                releaseSlot(slot);
                return;
            }

            CS.setLocoSlotRefresh(slot, (stat & STAT1_SL_ACTIVE) != 0);
        }
        _slots[slot].stat = stat;
    }

    void LocoNetSlotManager::processSpd(uint8_t slot, uint8_t spd) {
        LNSM_DEBUGF("OPC_LOCO_SPD slot %d spd %d\n", slot, spd);
        CS.setLocoSpeed(slot, spd);
        _slots[slot].spd = spd;
    }

void LocoNetSlotManager::sendProgData(uint8_t pcmd, uint8_t pstat, uint16_t cv, uint8_t value ) {

}

void LocoNetSlotManager::processProgMsg(const progTaskMsg &msg) {
    uint16_t cv = PROG_CV_NUM(msg);
    uint8_t mode = PCMD_MODE_MASK & msg.pcmd;
    uint8_t val = PROG_DATA(msg);
    uint16_t addr = (msg.hopsa&0x7F)<<7 | (msg.lopsa & 0x7F);
    bool read = (msg.pcmd & PCMD_RW)==0;
    if(read) {
        switch(mode) {
            case DIR_BYTE_ON_SRVC_TRK: {
                sendLack(0x7F, 1); // ack ok
                int16_t ret = CS.readCVProg(cv);
                sendProgData(msg.pcmd, ret?0:PSTAT_READ_FAIL, cv, ret>0?ret:0);
                break;
            }
            default:
                sendLack(0x7F, 0x7F); // not implemented
                break;
        }
    } else { // write
        switch(mode) {
            case SRVC_TRK_RESERVED: {// make it a verify command.
                sendLack(0x7F, 1); // ack ok
                bool ret = CS.verifyCVProg(cv, val);
                sendProgData(msg.pcmd, ret?0:PSTAT_READ_FAIL, cv, val);
                break;
            }
            case DIR_BYTE_ON_SRVC_TRK: {
                sendLack(0x7F, 1); // ack ok
                bool ret = CS.writeCvProg(cv, val);
                sendProgData(msg.pcmd, ret?0:PSTAT_WRITE_FAIL, cv, val);
                break;
            }
            /*case DIR_BIT_ON_SRVC_TRK:
                sendLack(0x7F, 1); // ack ok
                bool ret = CS.writeCvProgBit(cv, 0, val);
                break;*/
            case OPS_BYTE_NO_FEEDBACK:
                sendLack(0x7F, 0x40); // ack ok, no reply will follow
                CS.writeCvMain(lnAddr(addr), cv, val);
                break;
            /*case OPS_BIT_NO_FEEDBACK:
                sendLack(0x7F, 0x40); // ack ok, no reply will follow
                CS.writeCvMainBit(lnAddr(addr), cv, val);
                break;*/
            default:
                sendLack(0x7F, 0x7F); // not implemented
                break;

        }
    }
    
    sendLack(0x7F, 1); // ack ok
    sendLack(0x7F, 0x40); // ack ok, no reply will follow
    
}