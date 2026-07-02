#include "LocoNetManagers.h"

#include <dcc/accessory_address.hpp>

#include "FastClock.hpp"

#define LOG_LEVEL  LEVEL_INFO
#include "log.h"

/// LocoNet 1.0 tells 0x7F, but JMRI expects OPC_WR_SL_DATA
constexpr uint8_t PROG_LACK = OPC_WR_SL_DATA;//0x7F;

/** Creates LocoAddress (short/long) from LocoNet address. */
static LocoAddress fromLnAddr(uint16_t addr) {
    if(addr<=127) { return LocoAddress::shortAddr(addr); }
    return LocoAddress::longAddr(addr);
}

// reverse to ADDR(hi,lo)  (   ((lo) | (((hi) & 0x0F ) << 7))    )
static uint8_t addrLo(const LocoAddress &addr) {
    return addr.addr() & 0b00011111;
}

static uint8_t addrHi(const LocoAddress &addr) {
    return (addr.addr() >> 7);
}

/** Puts bit 0 of arg to 5th place, shifts bits 1-4 to right */
static uint8_t moveBit1to5(uint8_t normalByte) {
    return (normalByte & 0x1)<<4 | (normalByte & 0b1'1110)>>1;
}

/** Puts bit 5 of arg to 0th place, shifts bits 1-4 to left */
static uint8_t moveBit5to1(uint8_t dccByte) {
    return (dccByte & 0b0001'0000)>>4 | (dccByte & 0b0000'1111)<<1 ;
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

void sendLack(uint8_t cmd, uint8_t arg, LocoNetBus *_ln, LocoNetConsumer *sender) {
    LnMsg lack = makeLongAck(cmd, arg);
    _ln->broadcast(lack, sender);
}

    LocoNetSlotManager::LocoNetSlotManager(LocoNetBus * const ln): _ln(ln) {
        ln->addConsumer(this);
    }

    void LocoNetSlotManager::fillSlotMsg(uint8_t slot, rwSlotDataMsg &sd) {
        sd.command = OPC_SL_RD_DATA;
        sd.mesg_size = 14;
        sd.slot = slot;

        if(!isValidLocoSlot(slot) || !CS.isSlotAllocated(slot) ) {
            sd.stat = DEC_MODE_128 | LOCO_FREE;
            sd.adr = 0;
            sd.spd = 0;
            sd.spd = 0;
            sd.spd = 0;
            sd.dirf = DIRF_DIR;  // FWD
            sd.adr2 = 0;
            sd.snd = 0;

            sd.ss2 = 0;
            sd.id1 = 0;
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
            case OPC_GPON:  // FIXME: this is not related to slots.
                CS.setPowerState(true);
                break;
            case OPC_GPOFF:
                CS.setPowerState(false);
                break;
            case OPC_LOCO_ADR: {
                uint16_t addr = ADDR(msg->la.adr_hi, msg->la.adr_lo);
                int slot = findOrAllocateSlot(addr);
                if(slot<=0) {
                    LOGI("OPC_LOCO_ADR for addr %d, no available slots", addr );
                    sendLack(OPC_LOCO_ADR);
                    break;
                }

                LOGI("OPC_LOCO_ADR for addr %d, found slot %d", addr, slot);
                sendSlotData(slot);
                break;
            }
            case OPC_MOVE_SLOTS: {
                uint8_t srcSlot = msg->sm.src;
                uint8_t dstSlot = msg->sm.dest;
                if( dstSlot==srcSlot && isValidLocoSlot(srcSlot)) {
                    LOGI("OPC_MOVE_SLOTS NULL MOVE for slot %d", srcSlot );
                    CS.setLocoSlotRefresh(srcSlot, true); // enable refresh
                    sendSlotData(srcSlot);
                } else
                if(dstSlot==0 && isValidLocoSlot(srcSlot) ) {
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
                if(isValidLocoSlot(srcSlot) && isValidLocoSlot(dstSlot)) {
                    // a valid move, but we don't support it atm
                    sendLack(OPC_MOVE_SLOTS);
                } else {
                    sendLack(OPC_MOVE_SLOTS);
                }
                break;
            }
            case OPC_SLOT_STAT1: {
                uint8_t slot = msg->ss.slot;
                if( !isValidLocoSlot(slot) ) { sendLack(OPC_LOCO_SND); break; }
                processStat1(slot, msg->ss.stat);
                break;
            }
            case OPC_LOCO_SND: {
                uint8_t slot = msg->ls.slot;
                if( !isValidLocoSlot(slot) ) { sendLack(OPC_LOCO_SND); break; }
                processSnd(slot, msg->ls.snd);
                break;
            }
            case OPC_LOCO_DIRF: {
                uint8_t slot = msg->ldf.slot;
                if( !isValidLocoSlot(slot) ) { sendLack(OPC_LOCO_DIRF); break; }
                processDirf(slot, msg->ldf.dirf);
                break;
            }
            case OPC_LOCO_SPD : {
                uint8_t slot = msg->lsp.slot;
                if( !isValidLocoSlot(slot) ) { sendLack(OPC_LOCO_SPD); break; }
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
                if( !isValidLocoSlot(slot) ) { sendLack(OPC_WR_SL_DATA); break; }
                rwSlotDataMsg curSlot;
                fillSlotMsg(slot, curSlot);

                if(curSlot.stat != m.stat) processStat1(slot, m.stat);
                if( !CS.isSlotAllocated(slot) ) break; // stat1 can deallocate slot, do not continue in this case
                if(curSlot.spd != m.spd) processSpd(slot, m.spd);
                if(curSlot.dirf != m.dirf) processDirf(slot, m.dirf);
                if(curSlot.snd != m.snd) processSnd(slot, m.snd);

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
                // JMRI requests slot 0 on connect, so it's probably valid to read.
                if( isValidLocoSlot(slot) || slot==0) {
                    LOGI("OPC_RQ_SL_DATA slot %d", slot);
                    sendSlotData(slot);
                    break;
                }
                // TODO: slot 0x79 is a QuerySlot1, voltage/current meter slot, requested by JMRI on connect
                //   expected response is OPC_SL_RD_DATA_P2 with 21 bytes length;
                //   which is unsupported by the library.
                sendLack(OPC_RQ_SL_DATA);
                break;

            }
            default: break;
        }

    }


    int LocoNetSlotManager::findOrAllocateSlot(uint16_t ln_addr) {
        LocoAddress addr = fromLnAddr(ln_addr);
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

        LOGI_SLOT("Sending", slot, ret.sd);

        writeChecksum(ret);
        _ln->broadcast(ret, this);
    }

    void LocoNetSlotManager::sendLack(uint8_t cmd, uint8_t arg) {
        ::sendLack(cmd, arg, _ln, this);
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
        LOGI("OPC_SLOT_STAT1 slot=%d stat1=%02x", slot, stat);

        /*
        For bits D5(SL_BUSY) | D4(SL_ACTIVE):
        11 = IN_USE    loco adr in SLOT  -     REFRESHED
        10 = IDLE      loco adr in SLOT  - NOT refreshed
        01 = COMMON    loco adr IN SLOT  -     refreshed
        00 = FREE SLOT, no valid DATA    - not refreshed
        */

        auto newSpeedMode = int2SpeedMode(stat);
        bool newActive = (stat & STAT1_SL_ACTIVE) == STAT1_SL_ACTIVE;
        bool newBusy = (stat & STAT1_SL_BUSY) == STAT1_SL_BUSY;

        if(CS.isSlotAllocated(slot)) {
            if(!newActive && !newBusy) { // = FREE SLOT
                releaseSlot(slot);
                return;
            }
            const LocoData &dd = CS.getSlotData(slot);
            if(newSpeedMode != dd.speedMode) CS.setLocoSpeedMode(slot, newSpeedMode);
            if(newActive != dd.refreshing) CS.setLocoSlotRefresh(slot, newActive);
        } // else do we need to allocate this slot? I don't think so.
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
                CS.writeCvMain(fromLnAddr(addr), cv, val);
                break;
            /*case OPS_BIT_NO_FEEDBACK:
                sendLack(0x7F, 0x40); // ack ok, no reply will follow
                CS.writeCvMainBit(fromLnAddr(addr), cv, val);
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
    Standard mandates that upon reception of the packet subminute counter must be reset.
    */
    unsigned ticks = TICK_MAX - ((msg.frac_minsh<<7) | msg.frac_minsl);
    unsigned rate = msg.clk_rate;

    clockSetterId = (msg.id2 << 7) | msg.id1;

    fast_clock::clock.setRate(rate);
    fast_clock::clock.setSeconds(days*86400 + hrs*3600 + mins*60);

    LOGI("Received fast clock: days=%d, %02d:%02d .%02d, rate=%d:1", days, hrs, mins, ticks, rate);
}

void LocoNetSlotManager::sendFastClock() {
    uint8_t hrs, mins, secs;
    unsigned days;
    fast_clock::clock.getDHMS(days, hrs, mins, secs);

    // subminute counter; according to LocoNet2 library, a 14 bit counter, a minute is 0x7F*0x7F counts.
    // JMRI sends a completely different value.
    unsigned ticks = TICK_MAX - secs * 0x7F*0x7F / 60;

    LnMsg ret;
    ret.fc.command = OPC_SL_RD_DATA;
    ret.fc.mesg_size = 14;
    ret.fc.slot = FC_SLOT;
    ret.fc.clk_rate = fast_clock::clock.getRate();
    ret.fc.frac_minsl = ticks & 0x7F;
    ret.fc.frac_minsh = (ticks >> 7) & 0x7F;
    ret.fc.mins_60 = (mins + (127-60)) & 0x7F;
    ret.fc.track_stat = trkByte();
    ret.fc.hours_24 = (hrs + (128-24)) & 0x7F;
    ret.fc.days = days;
    ret.fc.clk_cntrl = 0x40; // bit 6: 1=data is valid clock info; 0=ignore this reply
    ret.fc.id1 = clockSetterId & 0x7F;
    ret.fc.id2 = clockSetterId >> 7;

    //LOGI("Sending fast clock");

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



static uint16_t lnSwitchAddr(uint8_t hi, uint8_t lo) {
    return (lo & 0b1111111) | ((hi & 0b1111)<<7); // +1 ?
}

static dcc::AccessoryAddress fromLnSwitchAddr(uint16_t ln_addr) {
    return dcc::AccessoryAddress::from11bit(ln_addr);
}


void LocoNetTurnoutManager::processMessage(const lnMsg* msg) {
    switch(msg->data[0]) {
        case OPC_SW_ACK: {  // switch command, sent to a command station from PC, command station emits DCC packets.
            processSwitchRequest(msg->srq, true);
            break;
        }
        case OPC_SW_REQ: {  // switch command, sent to decoders (or command station) from throttles
            const swReqMsg &req = msg->srq;
            if((req.sw1 & 0b1111'1100) == 0b0111'1000 && (req.sw2 & 0b1101'1111) == 0b0000'0111) {
                // interrogate devices on bus, used by JMRI on connection
                uint8_t bits = ((req.sw2 >> 3) & 0b100) | (req.sw1 & 0b11);
                LOGI("Interrogate devices with low bits 0b%d%d%d", (bits>>2)&1, (bits>>1)&1, bits&1);
                break;
            }
            if((req.sw2 & 0b1100'0000) == 0b0100'0000) {
                // switch input report?
                break;
            }
            if(propagateToDcc)
                processSwitchRequest(req, false);
            break;
        }
        case OPC_SW_STATE: { // request for switch state
            dcc::AccessoryAddress addr = fromLnSwitchAddr(lnSwitchAddr(msg->srq.sw1, msg->srq.sw2) );
            auto tt = CS.findTurnout(addr);
            if(tt.has_value() && tt.value().get().state != TurnoutState::UNKNOWN) {
                uint8_t ret = tt.value().get().state == TurnoutState::CLOSED ? OPC_SW_REQ_DIR : 0; // bit 5 = CLOSED
                ret |= OPC_SW_REQ_OUT; // bit 4 = ON
                ::sendLack(OPC_SW_STATE, ret, _ln, this);
                break;
            }
            break;
        }
        case OPC_SW_REP: {
            // sent by switch decoder:
            // external_evt=1: when decoder input (e.g. button) changes
            // external_evt=0: after it moves a switch (either from LocoNet command or button)
            bool external_evt = (msg->srp.sn2 & OPC_SW_REP_INPUTS) != 0;
            dcc::AccessoryAddress addr = fromLnSwitchAddr(lnSwitchAddr(msg->srp.sn1, msg->srp.sn2));
            bool closed = msg->srp.sn2 & OPC_SW_REP_CLOSED;
            bool thrown = msg->srp.sn2 & OPC_SW_REP_THROWN;
            LOGI("Switch report: addr %d, closed=%d, thrown=%d, external_evt=%d", addr, closed, thrown, external_evt);
            auto tt = CS.findTurnout(addr);
            if(tt.has_value()) {
                LOGI("Found this address in roster, state=%d", (int)tt.value().get().state);
            }
            break;
        }
        case OPC_INPUT_REP: { // sensor report, sent by occupancy detectors etc
            const inputRepMsg &ir = msg->ir;
            uint16_t addr = (ir.in1 & 0b0111'1111) << 1  // address bits 1..7
                | (ir.in2 & 0b1111) << 8                 // address bits 8..11
                | bitRead(ir.in2, 5);                    // address bit 0,
                                                         //  on DS64 0=AUX(A1..A4) 1=Switch(A1..A4)
            bool state = bitRead(ir.in2, 4);
            LOGI("Input report: addr=%d, state=%d", addr, state);
        }

        default: break;
    }
}

void LocoNetTurnoutManager::processSwitchRequest(const swReqMsg &msg, bool is_ack) {
    uint16_t addr = lnSwitchAddr(msg.sw1, msg.sw2);
    bool on = (msg.sw2 & 0b1'0000) != 0;
    bool thrown = (msg.sw2 & 0b10'0000) != 0;
    LOGI("OPC_SW_REQ for addr %d, thrown=%d, no=%d", addr, thrown, on);

    if (on) {
        CS.turnoutAction(fromLnSwitchAddr(addr), false,
            thrown ? TurnoutAction::THROW : TurnoutAction::CLOSE);
        ::sendLack(is_ack ? OPC_SW_ACK : OPC_SW_REQ, 0x7F, _ln, this);

    } else {
        //TODO: DCC packet supports this bit too, it's just not exposed in CommandStation and DCC library
        LOGI("OPC_SW_REQ with off bit - ignored");
        ::sendLack(is_ack ? OPC_SW_ACK : OPC_SW_REQ, 0, _ln, this);
    }
}
