#include "LocoNetSlotManager.h"

#include "FastClock.hpp"

#define LOG_LEVEL  LEVEL_INFO
#include "log.h"

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

inline static uint8_t speedMode2int(SpeedMode sm) {
    using SM = SpeedMode;
    switch(sm) {
        case SM::S128: return DEC_MODE_128;
        case SM::S28: return DEC_MODE_28;
        case SM::S14: return DEC_MODE_14;
    }
    LOGW("bad speed mode: %d", (int)sm);
    return DEC_MODE_128;
}

inline static SpeedMode int2SpeedMode(uint8_t sm) {
    using SM = SpeedMode;
    sm &= DEC_MODE_MASK;
    if(sm == DEC_MODE_128) return SM::S128;
    if(sm == DEC_MODE_14) return SM::S14;
    if(sm == DEC_MODE_28) return SM::S28;
    LOGW("bad speed mode bits: %x", (int)sm);
    return SM::S128;
}

inline uint8_t trkByte() {
    uint8_t ret = GTRK_IDLE | GTRK_MLOK1; // no emgr across layout, & Loconet 1.1 by default;
    if(CS.getPowerState()) ret |= GTRK_POWER;
    return ret;
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
            sd.spd = d.speed.get128();
            sd.dirf = d.dir==1 ? DIRF_DIR : 0;
            sd.dirf |= moveBit1to5(fns);
            sd.adr2 = addrHi(d.addr);
            sd.snd = (fns & 0b1'1110'0000)>>5;

            const LnSlotData & e = extra[slot];
            sd.ss2 = e.ss2;
            sd.id1 = e.id1;
            sd.id2 = e.id2;
        }

        sd.trk = trkByte();
    }

    #define LOGI_SLOT(TAG, I, S) LOGI( TAG \
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
                    LOGI("OPC_LOCO_ADR for addr %d, no available slots", ADDR(msg->la.adr_hi, msg->la.adr_lo) );
                    sendLack(OPC_LOCO_ADR);
                    break;
                }

                LOGI("OPC_LOCO_ADR for addr %d, found slot %d", ADDR(msg->la.adr_hi, msg->la.adr_lo), slot);
                sendSlotData(slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                uint8_t srcSlot = msg->sm.src;
                uint8_t dstSlot = msg->sm.dest;
                if( dstSlot==srcSlot && slotValid(srcSlot)) {
                    LOGI("OPC_MOVE_SLOTS NULL MOVE for slot %d", srcSlot );
                    CS.setLocoSlotRefresh(srcSlot, true); // enable refresh
                    sendSlotData(srcSlot);
                } else
                if(dstSlot==0 && slotValid(srcSlot) ) {
                    LOGI("OPC_MOVE_SLOTS DISPATCH PUT for slot %d", srcSlot );
                    if(haveDispatchedSlot() ) {
                        sendLack(OPC_MOVE_SLOTS, 0);
                    } else {
                        dispatchedSlot = srcSlot;
                        sendSlotData(dispatchedSlot);
                    }
                } else
                if(srcSlot == 0 ) {
                    // DISPATCH GET
                    LOGI("OPC_MOVE_SLOTS DISPATCH GET" );
                    if(haveDispatchedSlot() ) {
                        sendSlotData(dispatchedSlot);
                        removeDispatchedSlot();
                    } else {
                        sendLack(OPC_MOVE_SLOTS, 0);
                    }
                } else
                if(slotValid(srcSlot) && slotValid(dstSlot)) {
                    // a valid move, but we don't support it atm
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    sendLack(OPC_MOVE_SLOTS);
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
                if(m.slot == FC_SLOT) {
                    processFastClockMsg(msg->fc);
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

                LOGI_SLOT("OPC_WR_SL_DATA", slot, m);

                break;
            }
            case OPC_RQ_SL_DATA: {
                uint8_t slot = msg->sr.slot;
                if(slot == FC_SLOT) {
                    sendFastClock();
                    break;
                }
                if( !slotValid(slot) ) { sendLack(OPC_RQ_SL_DATA); break;}
                LOGI("OPC_RQ_SL_DATA slot %d", slot);
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

        LOGI_SLOT("Sending", slot, (ret.sd));

        writeChecksum(ret);
        _ln->broadcast(ret, this);
    }

    void LocoNetSlotManager::sendLack(uint8_t cmd, uint8_t arg) {
        LnMsg lack = makeLongAck(cmd, arg);
        _ln->broadcast(lack, this);
    }

    void LocoNetSlotManager::processDirf(uint8_t slot, uint v) {
        LOGI("OPC_LOCO_DIRF slot %d dirf %02x", slot, v);
        uint8_t dir = ((v & DIRF_DIR) == DIRF_DIR) ? 0 : 1;
        CS.setLocoDir(slot, dir);
        // fn order in received byte is 04321, needs swapping
        CS.setLocoFns(slot, dcc::fn_group::F0_4, moveBit5to1(v) );
    }

    void LocoNetSlotManager::processSnd(uint8_t slot, uint8_t snd) {
        LOGI("OPC_LOCO_SND slot %d snd %02x", slot, snd);
        CS.setLocoFns(slot, dcc::fn_group::F5_8, (uint32_t)snd << 5);
    }

    void LocoNetSlotManager::processStat1(uint8_t slot, uint8_t stat) {
        LOGI("OPC_SLOT_STAT1 slot %d stat1 %02x", slot, stat);

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
        LOGI("OPC_LOCO_SPD slot %d spd %d", slot, spd);
        CS.setLocoSpeed(slot, LocoSpeed::from128(spd) );
    }

void LocoNetSlotManager::sendProgData(progTaskMsg ret, uint8_t pstat, uint8_t value ) {

    LOGI("pstat=%02xh, val=%d", pstat, value);

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
                LOGI("Read byte on prog CV%d", cv);
                sendLack(PROG_LACK, 1); // ack ok
                int16_t ret = CS.readCVProg(cv);
                sendProgData(msg, (ret>=0) ? 0 : PSTAT_READ_FAIL, ret>=0?ret:0);
                break;
            }
            case SRVC_TRK_RESERVED: {// make it a verify command.
                LOGI("Verify byte on prog CV%d==%d", cv, val);
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
                LOGI("Write byte on prog CV%d=%d", cv, val);
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
                LOGI("Read byte on prog CV%d", cv);
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
}

constexpr uint32_t TICK_MAX = 0x3FFF;

void LocoNetSlotManager::processFastClockMsg(const fastClockMsg &msg) {
    if( (msg.clk_cntrl & 0x40) == 0 ) {
        LOGI("Received fast clock message with invalid clock info, ignoring");
        return;
    }
    // magic numbers are simplified from LocoNetFastClock.cpp in LocoNet2 library.
    unsigned mins = (msg.mins_60 - (127-60));
    unsigned hrs = (msg.hours_24 - (128-24));
    unsigned days = msg.days;

    /*
    Interpretation of frac_minsh/frac_minsl is device-specific.
    Standard mandates that upon reception of the packet subinute counter must be reset.
    */
    unsigned ticks = TICK_MAX - ((msg.frac_minsh<<7) | msg.frac_minsl);
    unsigned rate = msg.clk_rate;

    clockId = (msg.id1 << 8) | msg.id2;

    fast_clock::clock.setRate(rate);
    fast_clock::clock.setSeconds(days*86400 + hrs*3600 + mins*60);

    LOGI("Received fast clock: days=%d, %02d:%02d .%02d, rate=%d:1", days, hrs, mins, ticks, rate);
}

void LocoNetSlotManager::sendFastClock() {
    uint32_t seconds = fast_clock::clock.getSeconds();
    unsigned mins = (seconds / 60) % 60;
    unsigned hrs = (seconds / 3600) % 24;
    unsigned days = seconds / 86400;
    // subminute counter; according to LocoNet2 library, a 14 bit counter, a minute is 0x7F*0x7F counts.
    unsigned ticks = TICK_MAX - (seconds % 60) * 0x7F*0x7F / 60;

    LnMsg ret;
    ret.fc.command = OPC_SL_RD_DATA;
    ret.fc.mesg_size = 14;
    ret.fc.slot = FC_SLOT;
    ret.fc.clk_rate = fast_clock::clock.getRate();
    ret.fc.frac_minsl = ticks & 0x7F;
    ret.fc.frac_minsh = (ticks >> 7) & 0x7F;
    ret.fc.mins_60 = (mins + (128-60)) & 0x7F;
    ret.fc.track_stat = trkByte();
    ret.fc.hours_24 = (hrs + (128-24)) & 0x7F;
    ret.fc.days = days;
    ret.fc.clk_cntrl = 0x40; // bit 6: 1=data is valid clock info; 0=ignore this reply
    ret.fc.id1 = clockId >> 8;
    ret.fc.id2 = clockId & 0xFF;

    LOGI("Sending fast clock");

    writeChecksum(ret);
    _ln->broadcast(ret, this);

}

void LocoNetSlotManager::notification(const fast_clock::ClockChangedEvent &event) {
    if(isClockMaster && millis() - clockSentTime > CLOCK_SEND_INTL) {
        clockSentTime = millis();
        sendFastClock();
    }
}

void LocoNetSlotManager::setFastClockMaster(bool v) {
    if(isClockMaster == v) return;
    isClockMaster = v;
    if(isClockMaster) {
        fast_clock::clock.add_observer(*this);
    } else {
        fast_clock::clock.remove_observer(*this);
    }
}
