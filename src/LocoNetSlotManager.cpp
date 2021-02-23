#include "LocoNetSlotManager.h"

#define LNSM_DEBUG


#ifdef LNSM_DEBUG
#define LNSM_LOGD(...) 
#define LNSM_LOGI(format, ...) log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define LNSM_LOGW(format, ...) log_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
#else
#define LNSM_LOGD(...) 
#define LNSM_LOGI(...) 
#define LNSM_LOGW(...) 
#endif

/// LocoNet 1.0 tells 0x7F, but JMRI expects OPC_WR_SL_DATA
constexpr uint8_t PROG_LACK = OPC_WR_SL_DATA;//0x7F;

static LocoAddress lnAddr(uint16_t addr) {
    if(addr<=127) { return LocoAddress::shortAddr(addr); }
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

    #define LNSM_LOGI_SLOT(TAG, I, S) LNSM_LOGI( TAG \
        " slot %d: ADDR=%d STAT=%02X(%s) ID=%02X%02X", I, \
        ADDR(S.adr2, S.adr), S.stat, LOCO_STAT(S.stat), S.id1, S.id2 )

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
                    LNSM_LOGI("OPC_LOCO_ADR for addr %d, no available slots", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                    sendLack(OPC_LOCO_ADR);
                    break;
                }
                
                LNSM_LOGI("OPC_LOCO_ADR for addr %d, slot is %d", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                sendSlotData(slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                if( msg->sm.dest!=msg->sm.src || !slotValid(msg->sm.dest) || !slotValid(msg->sm.src) ) {
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    uint8_t slot = msg->ss.slot;
                    LNSM_LOGI("OPC_MOVE_SLOTS NULL MOVE for slot %d", slot );
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

                LNSM_LOGI_SLOT("OPC_WR_SL_DATA", slot, _slot);

                break;
            }
            case OPC_RQ_SL_DATA: {
                uint8_t slot = msg->sr.slot;
                if( !slotValid(slot) ) { sendLack(OPC_RQ_SL_DATA); break;} 
                LNSM_LOGI("OPC_RQ_SL_DATA slot %d", slot);
                sendSlotData(slot);
                break;
            }
            default: break;
        }
        
    } 



    int LocoNetSlotManager::locateSlot(uint8_t hi, uint8_t lo) {
        LocoAddress addr = (hi==0) ? LocoAddress::shortAddr(lo) : LocoAddress::longAddr(ADDR(hi,lo));
        uint8_t slot = CS.findLocoSlot(addr);
        if(slot==0) {
            slot = CS.locateFreeSlot();
            if(slot==0) { return 0; }
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
        rwSlotDataMsg *s = &ret.sd;
        
        LNSM_LOGI_SLOT("Sending", slot, (*s));
        
        writeChecksum(ret);
        _ln->broadcast(ret, this);
    }

    void LocoNetSlotManager::sendLack(uint8_t cmd, uint8_t arg) {
        LnMsg lack = makeLongAck(cmd, arg); 
        _ln->broadcast(lack, this);
    }

    void LocoNetSlotManager::processDirf(uint8_t slot, uint v) {
        LNSM_LOGI("OPC_LOCO_DIRF slot %d dirf %02x", slot, v);
        _slots[slot].dirf = v;
        uint8_t dir = ((v & DIRF_DIR) == DIRF_DIR) ? 1 : 0;
        CS.setLocoDir(slot, dir);
        CS.setLocoFns(slot, 0x1F, (v & B00001111)<<1 | (v & B00010000)>>4 );  // fn order in this byte is 04321
    }

    void LocoNetSlotManager::processSnd(uint8_t slot, uint8_t snd) {
        LNSM_LOGI("OPC_LOCO_SND slot %d snd %02x", slot, snd);
        CS.setLocoFns(slot, 0x1E0, snd << 5 );
        _slots[slot].snd = snd;
    }

    void LocoNetSlotManager::processStat1(uint8_t slot, uint8_t stat) {
        LNSM_LOGI("OPC_SLOT_STAT1 slot %d stat1 %02x", slot, stat);

        if( (_slots[slot].stat & LOCOSTAT_MASK) != (stat&LOCOSTAT_MASK) ) {
            LNSM_LOGI("Changing active+busy bits: %02x", stat&LOCOSTAT_MASK);
            if( (stat & STAT1_SL_BUSY) == 0) { 
                releaseSlot(slot);
                return;
            }

            CS.setLocoSlotRefresh(slot, (stat & STAT1_SL_ACTIVE) != 0);
        }
        _slots[slot].stat = stat;
    }

    void LocoNetSlotManager::processSpd(uint8_t slot, uint8_t spd) {
        LNSM_LOGI("OPC_LOCO_SPD slot %d spd %d", slot, spd);
        CS.setLocoSpeed(slot, spd);
        _slots[slot].spd = spd;
    }

void LocoNetSlotManager::sendProgData(progTaskMsg ret, uint8_t pstat, uint8_t value ) {

    LNSM_LOGI("pstat=%02xh, val=%d", pstat, value);

    ret.command = OPC_SL_RD_DATA;
    ret.mesg_size = 14;
    ret.slot = PRG_SLOT;
    ret.pstat = pstat;
    //value = (((progTaskMsg.cvh & CVH_D7) << 6) | (progTaskMsg.data7 & 0x7f))
    bitWrite(ret.cvh, 1, (value>>7));
    ret.data7 = value & 0x7F;
    
    LnMsg msg; msg.pt = ret;
    writeChecksum(msg);
    _ln->broadcast(msg, this);
}

void LocoNetSlotManager::processProgMsg(const progTaskMsg &msg) {
    uint16_t cv = PROG_CV_NUM(msg)+1;
    uint8_t mode = PCMD_MODE_MASK & msg.pcmd;
    uint8_t val = PROG_DATA(msg);
    uint16_t addr = (msg.hopsa&0x7F)<<7 | (msg.lopsa & 0x7F);
    bool read = (msg.pcmd & PCMD_RW)==0;
    if(read) {
        switch(mode) {
            case DIR_BYTE_ON_SRVC_TRK: {
                LNSM_LOGI("Read byte on prog CV%d", cv);
                sendLack(PROG_LACK, 1); // ack ok
                int16_t ret = CS.readCVProg(cv);
                sendProgData(msg, (ret>=0) ? 0 : PSTAT_READ_FAIL, ret>=0?ret:0);
                break;
            }
            case SRVC_TRK_RESERVED: {// make it a verify command.
                LNSM_LOGI("Verify byte on prog CV%d==%d", cv, val);
                sendLack(PROG_LACK, 1); // ack ok
                bool ret = CS.verifyCVProg(cv, val);
                sendProgData(msg, ret?0:PSTAT_READ_FAIL, val);
                break;
            }
            default:
                sendLack(PROG_LACK, 0x7F); // not implemented
                break;
        }
    } else { // write
        switch(mode) {
            case DIR_BYTE_ON_SRVC_TRK: {
                LNSM_LOGI("Write byte on prog CV%d=%d", cv, val);
                sendLack(PROG_LACK, 1); // ack ok
                bool ret = CS.writeCvProg(cv, val);
                sendProgData(msg, ret?0:PSTAT_WRITE_FAIL, val);
                break;
            }
            /*case DIR_BIT_ON_SRVC_TRK:
                sendLack(0x7F, 1); // ack ok
                bool ret = CS.writeCvProgBit(cv, 0, val);
                break;*/
            case OPS_BYTE_NO_FEEDBACK:
                LNSM_LOGI("Read byte on prog CV%d", cv);            
                sendLack(PROG_LACK, 0x40); // ack ok, no reply will follow
                CS.writeCvMain(lnAddr(addr), cv, val);
                break;
            /*case OPS_BIT_NO_FEEDBACK:
                sendLack(0x7F, 0x40); // ack ok, no reply will follow
                CS.writeCvMainBit(lnAddr(addr), cv, val);
                break;*/
            default:
                sendLack(PROG_LACK, 0x7F); // not implemented
                break;

        }
    }
    
    //sendLack(PROG_LACK, 1); // ack ok
    //sendLack(PROG_LACK, 0x40); // ack ok, no reply will follow
    
}