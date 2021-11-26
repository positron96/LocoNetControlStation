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

// reverse to ADDR(hi,lo)  (   ((lo) | (((hi) & 0x0F ) << 7))    )
inline static uint8_t addrLo(const LocoAddress &addr) {
    return addr.addr() & 0b00011111;
}

inline static uint8_t addrHi(const LocoAddress &addr) {
    return (addr.addr() >> 7);
}

using LocoData = CommandStation::LocoData;
using SM = SpeedMode;

inline static uint8_t speedMode2int(SM sm) {
    switch(sm) {
        case SM::S128: return DEC_MODE_128;
        case SM::S28: return DEC_MODE_28;
        case SM::S14: return DEC_MODE_14;
    }
    LNSM_LOGW("bad speed mode: %d", (int)sm);
    return DEC_MODE_128;
}

inline static SM int2SpeedMode(uint8_t sm) {
    sm &= DEC_MODE_MASK;
    if(sm == DEC_MODE_128) return SM::S128;
    if(sm == DEC_MODE_14) return SM::S14;
    if(sm == DEC_MODE_28) return SM::S28;
    LNSM_LOGW("bad speed mode bits: %x", (int)sm);
    return SM::S128;
}

    LocoNetSlotManager::LocoNetSlotManager(LocoNetBus * const ln): _ln(ln) {
        ln->addConsumer(this);
    }

    void LocoNetSlotManager::fillSlotMsg(uint8_t slot, rwSlotDataMsg &sd) {
        sd.command = OPC_SL_RD_DATA;
        sd.mesg_size = 14;
        sd.slot = slot;

        if(!CS.isSlotAllocated(slot) ) {
            sd.stat = DEC_MODE_128 | LOCO_FREE;
            sd.adr = 0; 
            sd.spd = 0; 
            sd.spd = 0; 
            sd.spd = 0; 
            sd.dirf = DIRF_DIR;  // FWD
            sd.adr2 = 0; 
            sd.snd = 0; 

            sd.ss2 = 0; 
            sd.id1 = slot; 
            sd.id2 = 0;
        } else {
            const CommandStation::LocoData &d = CS.getSlotData(slot);
            uint32_t fns = d.fn.value<uint32_t>();
            sd.stat = speedMode2int(d.speedMode) | STAT1_SL_BUSY;
            if(d.refreshing) sd.stat |= STAT1_SL_ACTIVE;
            sd.adr = addrLo(d.addr); 
            sd.spd = d.speed.getSpeed128(); 
            sd.dirf = d.dir==1 ? DIRF_DIR : 0;
            sd.dirf |= fn15swap(fns);
            sd.adr2 = addrHi(d.addr); 
            sd.snd = (fns & 0b111100000)>>5; 

            const LnSlotData & e = extra[slot];
            sd.ss2 = e.ss2; 
            sd.id1 = e.id1; 
            sd.id2 = e.id2;
        }        
       
        sd.trk = GTRK_IDLE | GTRK_MLOK1; // POWER ON & Loconet 1.1 by default; 
        if(CS.getPowerState()) sd.trk |= GTRK_POWER;
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
                
                LNSM_LOGI("OPC_LOCO_ADR for addr %d, found slot %d", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                sendSlotData(slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                if( msg->sm.dest!=msg->sm.src || !slotValid(msg->sm.dest) || !slotValid(msg->sm.src) ) {
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    uint8_t slot = msg->ss.slot;
                    LNSM_LOGI("OPC_MOVE_SLOTS NULL MOVE for slot %d", slot );
                    CS.setLocoSlotRefresh(slot, true); // enable refresh
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
                    break;
                }
                if( !slotValid(slot) ) { sendLack(OPC_WR_SL_DATA); break; } 
                rwSlotDataMsg _slot;
                fillSlotMsg(slot, _slot);

                if(_slot.stat != m.stat) processStat1(slot, m.stat);
                if( !CS.isSlotAllocated(slot) ) break; // stat1 can set slot to inactive, do not continue in this case
                if(_slot.spd != m.spd) processSpd(slot, m.spd);
                if(_slot.dirf != m.dirf) processDirf(slot, m.dirf);
                if(_slot.snd != m.snd) processSnd(slot, m.snd);

                if(extra.find(slot) != extra.end() ) {
                    extra[slot] = LnSlotData{};
                }
                LnSlotData &e = extra[slot];
                e.ss2 = m.ss2;
                e.id1 = m.id1;
                e.id2 = m.id2;

                LNSM_LOGI_SLOT("OPC_WR_SL_DATA", slot, m);

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
            extra[slot] = LnSlotData{};
        }
        return slot;
    }

    void LocoNetSlotManager::releaseSlot(uint8_t slot) {
        CS.releaseLocoSlot(slot);
        extra.erase(slot);
    }

    void LocoNetSlotManager::sendSlotData(uint8_t slot) {        
        LnMsg ret;
        fillSlotMsg(slot, ret.sd);

        LNSM_LOGI_SLOT("Sending", slot, (ret.sd));
        
        writeChecksum(ret);
        _ln->broadcast(ret, this);
    }

    void LocoNetSlotManager::sendLack(uint8_t cmd, uint8_t arg) {
        LnMsg lack = makeLongAck(cmd, arg); 
        _ln->broadcast(lack, this);
    }

    void LocoNetSlotManager::processDirf(uint8_t slot, uint v) {
        LNSM_LOGI("OPC_LOCO_DIRF slot %d dirf %02x", slot, v);
        uint8_t dir = ((v & DIRF_DIR) == DIRF_DIR) ? 1 : 0;
        CS.setLocoDir(slot, dir);
        CS.setLocoFns(slot, 0b00011111, fn15swap(v) );  // fn order in this byte is 04321
    }

    void LocoNetSlotManager::processSnd(uint8_t slot, uint8_t snd) {
        LNSM_LOGI("OPC_LOCO_SND slot %d snd %02x", slot, snd);
        CS.setLocoFns(slot, 0x1E0, snd << 5 );
    }

    void LocoNetSlotManager::processStat1(uint8_t slot, uint8_t stat) {
        LNSM_LOGI("OPC_SLOT_STAT1 slot %d stat1 %02x", slot, stat);

        auto newSpeedMode = int2SpeedMode(stat);
        bool newActive = (stat & STAT1_SL_ACTIVE) == STAT1_SL_ACTIVE;
        bool newBusy = (stat & STAT1_SL_BUSY) == STAT1_SL_BUSY;
        if( !newBusy ) { 
            releaseSlot(slot);
            return;
        }

        const LocoData &dd = CS.getSlotData(slot);
        if(newSpeedMode != dd.speedMode) CS.setLocoSpeedMode(slot, newSpeedMode);
        if(newActive != dd.refreshing) CS.setLocoSlotRefresh(slot, newActive);
    }

    void LocoNetSlotManager::processSpd(uint8_t slot, uint8_t spd) {
        LNSM_LOGI("OPC_LOCO_SPD slot %d spd %d", slot, spd);
        CS.setLocoSpeed(slot, spd);
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