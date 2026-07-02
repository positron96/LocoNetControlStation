#pragma once
/**
 * Contains all the stuff related to command station.
 * I.e. high-level DCC generation, turnout list and their respective
 * settings.
 */


#include <dcc/base_channel.hpp>
#include <dcc/packet.hpp>
#include <dcc/LocoAddress.h>
#include <dcc/accessory_address.hpp>
#include <LocoNet2.h>

#include "Watchdog.h"

#include <etl/map.h>
#include <etl/bitset.h>


#define CS_DEBUG

#ifdef CS_DEBUG
#define CS_DEBUGF(format, ...)  do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__);  } while(0)
#else
#define CS_DEBUGF
#endif

enum class TurnoutState {
    CLOSED,THROWN, UNKNOWN
};
inline TurnoutState toggleTurnout(const TurnoutState s) {
    return s==TurnoutState::THROWN ? TurnoutState::CLOSED : TurnoutState::THROWN;
}

enum class TurnoutAction {
    CLOSE, THROW, TOGGLE
};
inline TurnoutState actionToState(const TurnoutAction s) {
    return s==TurnoutAction::THROW ? TurnoutState::THROWN : TurnoutState::CLOSED;
}

/** Puts bit 0 of arg to 5th place, shifts bits 1-4 to right */
inline static uint8_t moveBit1to5(uint8_t normalByte) {
    return (normalByte & 0x1)<<4 | (normalByte & 0b1'1110)>>1;
}

/** Puts bit 5 of arg to 0th place, shifts bits 1-4 to left */
inline static uint8_t moveBit5to1(uint8_t dccByte) {
    return (dccByte & 0b0001'0000)>>4 | (dccByte & 0b0000'1111)<<1 ;
}

class CommandStation {
public:

    static constexpr uint8_t N_FUNCTIONS = 29;

    static constexpr uint8_t MAX_SLOTS = 10;

    static constexpr millis_t PURGE_DELAY = 200*1000; //200s

    CommandStation(): dccMain(nullptr), dccProg(nullptr), locoNet(nullptr) {
        loadTurnouts();
    }

    void setDccMain(dcc::BaseChannel * ch) { dccMain = ch; }
    void setDccProg(dcc::BaseChannel * ch) { dccProg = ch; }
    void setLocoNetBus(LocoNetBus *bus) { locoNet = bus; }

    void setPowerState(bool v) {
        if( dccMain!=nullptr ) dccMain->setPower(v);
    }

    bool getPowerState() const {
        return dccMain!=nullptr ? dccMain->getPower()
             //: dccProg!=nullptr ? dccProg->getPower()
             : false;
    }

    const dcc::BaseChannel *getMainTrack() const { return dccMain; }
    const dcc::BaseChannel *getProgTrack() const { return dccProg; }


    /* Define turnout object structures */
    struct TurnoutData {
        dcc::AccessoryAddress addr;
        int userTag;
        TurnoutState tStatus;
    };

    static constexpr int MAX_TURNOUTS = 15;
    using TurnoutMap = etl::map<dcc::AccessoryAddress, TurnoutData, MAX_TURNOUTS>;

    void loadTurnouts() {
        addTurnout({ dcc::AccessoryAddress::from9bit(1, 0), 0, TurnoutState::CLOSED });
        addTurnout({ dcc::AccessoryAddress::from9bit(1, 1), 1, TurnoutState::CLOSED });
        addTurnout({ dcc::AccessoryAddress::from9bit(1, 2), 2, TurnoutState::UNKNOWN });
        addTurnout({ dcc::AccessoryAddress::from9bit(1, 3), 3, TurnoutState::THROWN });

    }

    struct LocoData {
        using Fns = etl::bitset<N_FUNCTIONS>;
        LocoAddress addr;
        //uint8_t speed128; ///< <0=stop, 1=emgr, 2..127 = speed 0..max
        LocoSpeed speed;
        SpeedMode speedMode;
        int8_t dir; ///< 1 = FWD, 0 = REW
        Fns fn;
        bool refreshing;
        Watchdog<PURGE_DELAY, 500> wdt;
        bool allocated() const { return addr.isValid(); }
        void deallocate() { addr = LocoAddress(); }
        void kickWatchdog() { wdt.kick(); }
        uint8_t dccSpeedByte();
    };

    bool isSlotAllocated(uint8_t slot) const {
        if(slot<1 || slot>MAX_SLOTS) return true;
        return slots[slot-1].allocated();
    }

    bool isLocoAllocated(LocoAddress addr) {
        return locoSlot.find(addr) != locoSlot.end();
    }

    uint8_t findLocoSlot(LocoAddress addr) {
        auto it = locoSlot.find(addr);
        if(it != locoSlot.end() ) {
            return it->second;
        }
        return 0;
    }

    uint8_t locateFreeSlot() {
        if(locoSlot.size() < MAX_SLOTS) {
            for(int i=0; i<MAX_SLOTS; i++) {
                if(!slots[i].allocated() ) {
                    return i+1;
                }
            }
        }
        return 0;
    }

    void initLocoSlot(uint8_t slot, LocoAddress addr) {
        LocoData &_slot = getSlot(slot);
        _slot.addr = addr;
        _slot.dir = 1;
        _slot.fn = LocoData::Fns();
        _slot.refreshing = false;
        _slot.speed = LocoSpeed{};
        _slot.speedMode = SpeedMode::S128;
        _slot.kickWatchdog();
        locoSlot[addr] = slot;
    }

    uint8_t findOrAllocateLocoSlot(LocoAddress addr) {
        uint8_t slot = findLocoSlot(addr);
        if(slot==0) {
            slot = locateFreeSlot();
            if(slot!=0) initLocoSlot(slot, addr);
        }
        return slot;
    }

    void releaseLocoSlot(uint8_t slot) {
        if(slot==0) { CS_DEBUGF("invalid slot"); return; }
        uint8_t i = slot-1;
        CS_DEBUGF("releasing slot %d", slot);
        setLocoSlotRefresh(slot, false);
        locoSlot.erase( slots[i].addr );
        slots[i].deallocate();
    }

    /** Return view on slot numbers */
    auto getAllocatedSlots() const {
        return etl::views::values(etl::views::as_const(locoSlot));
    }

    size_t getAllocatedSlotsCount() const { return locoSlot.size(); }

    void setLocoSlotRefresh(uint8_t slot, bool refresh) {
        if(slot==0) { CS_DEBUGF("invalid slot"); return; }
        LocoData &dd = getSlot(slot);
        if(!dd.allocated()) { CS_DEBUGF("slot not allocated"); return; }
        if(dd.refreshing == refresh) return;
        CS_DEBUGF("slot %d refresh %c", slot, refresh?'Y':'N');
        dd.refreshing = refresh;

        if(refresh) {
            // no need to load, it will load itself on setLocoSpeed/setLocoFn
            dd.kickWatchdog();
        } else {
            // TODO: somehow send 0 speed to track
            dccMain->unloadSlot(dd.addr);
        }
    }

    void kickSlot(uint8_t slot) {
        LocoData &dd = getSlot(slot);
        if(!dd.allocated()) { CS_DEBUGF("slot not allocated"); return; }
        dd.kickWatchdog();
    }

    LocoAddress getLocoAddr(uint8_t slot) {
        if(!isSlotAllocated(slot)) return LocoAddress{};
        return getSlot(slot).addr;
    }

    const LocoData &getSlotData(uint8_t slot) {
        return getSlot(slot);
    }

    void setLocoSpeedMode(uint8_t slot, SpeedMode mode) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        if(dd.speedMode == mode) return;
        dd.speedMode = mode;
        if(dd.refreshing)
            dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir > 0);
    }

    SpeedMode getLocoSpeedMode(uint8_t slot) {
        return getSlot(slot).speedMode;
    }

    /** Changes one function. */
    void setLocoFn(uint8_t slot, uint8_t fn, bool val) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        if(dd.fn[fn] == val) return;
        // CS_DEBUGF("slot %d FN%d=%d", slot, fn, val);

        dd.fn[fn] = val;
        using dcc::fn_group;
        fn_group fg = dcc::fn_to_group(fn);
        uint32_t ifn = dd.fn.value<uint32_t>();

        dccMain->sendFunctionGroup(dd.addr, fg, ifn);
    }

    /** Changes bits of DCC function group. */
    void setLocoFns(uint8_t slot, dcc::fn_group fg, uint32_t vals) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        uint32_t current = dd.fn.value<uint32_t>();
        uint32_t mask = dcc::fn_group_mask(fg);
        vals = (current & ~mask) | (vals & mask);
        if(vals == current) return;
        // CS_DEBUGF("slot %d FN G%d = %d", slot, (int)fg, vals);

        dccMain->sendFunctionGroup(dd.addr, fg, vals);
        dd.fn = LocoData::Fns( vals );
    }

    /** Changes bits across multiple function groups. */
    void setLocoFns(uint8_t slot, uint32_t mask, uint32_t vals ) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        vals = vals & mask; // only take bits in mask, ignore others
        uint32_t current = dd.fn.value<uint32_t>();
        vals = (current & ~mask) | vals; // updated value for all bits
        uint32_t changed = current ^ vals;

        for(size_t g=0; g<dcc::FN_NUMBER; g++) {
            dcc::fn_group fg = static_cast<dcc::fn_group>(g);
            uint32_t gm = dcc::fn_group_mask(fg);
            // if required mask intersects function group mask
            //  and these bits differ from current value,
            // update bits (v=) and send function group
            if((mask & gm) != 0 && (changed & gm) != 0) {
                dccMain->sendFunctionGroup(dd.addr, fg, vals);
            }
        }

        dd.fn = LocoData::Fns( vals );
    }

    bool getLocoFn(uint8_t slot, uint8_t fn) {
        return  getSlot(slot).fn[fn] != 0;
    }

    /**
     *  @param speed DCC speed (0=sop, 1=EMGR stop)
     *  @param dir 1 - FWD, 0 - REW
     * */
    void setLocoDir(uint8_t slot, uint8_t dir) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        if(dd.dir==dir) return;
        dd.dir = dir;
        if(dd.refreshing)
            dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir);
    }

    uint8_t getLocoDir(uint8_t slot) {
        return getSlot(slot).dir;
    }

    /**
     * Updates slots that have not been used for a long time (PURGE_DELAY)
     */
    void loop() {
        for(const auto &i: locoSlot) {
            uint8_t slot = i.second;
            LocoData &dd = getSlot(slot);
            if(dd.refreshing && dd.wdt.timedOut()) {
                    CS_DEBUGF("slot %d timed out, current %lds, last update was at %lds", slot,
                        millis()/1000, dd.wdt.getLastUpdate()/1000 );
                    setLocoSlotRefresh(slot, false);
            }
        }
    }

    /// Sets speed
    void setLocoSpeed(uint8_t slot, LocoSpeed spd) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        if(dd.speed == spd) return;
        dd.speed = spd;
        if(dd.refreshing)
            dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir);
    }

    /// Returns speed
    LocoSpeed getLocoSpeed(uint8_t slot) {
        return getSlot(slot).speed;
    }

    void setLocoSpeedF(uint8_t slot, float spd) {
        setLocoSpeed(slot, LocoSpeed::fromFloat(spd) );
    }

    float getLocoSpeedF(uint8_t slot) {
        return getLocoSpeed(slot).getFloat();
    }

    int16_t readCVProg(uint16_t cv) {
        //IDCCChannel *dccProg = dccMain;
        if(dccProg==nullptr) return -2;
        return dccProg->readCVProg(cv);
    }
    bool verifyCVProg(uint16_t cv, uint8_t val) {
        //IDCCChannel *dccProg = dccMain;
        if(dccProg==nullptr) return false;
        return dccProg->verifyCVByteProg(cv, val);
    }
    bool writeCvProg(uint16_t cv, uint8_t val) {
        //IDCCChannel *dccProg = dccMain;
        if(dccProg ==nullptr) return false;
        return dccProg->writeCVByteProg(cv, val);
    }
    bool writeCvProgBit(uint16_t cv, uint8_t bit, bool val) {
        //IDCCChannel *dccProg = dccMain;
        if(dccProg ==nullptr) return false;
        return dccProg->writeCVBitProg(cv, bit, val);
    }
    void writeCvMain(LocoAddress addr, uint16_t cv, uint8_t val) {
        if(dccMain==nullptr) return;
        dccMain->writeCVByteMain(addr, cv, val);
    }
    void writeCvMainBit(LocoAddress addr, uint16_t cv, uint8_t bit, bool val) {
        if(dccMain==nullptr) return;
        dccMain->writeCVBitMain(addr, cv, bit, val?1:0);
    }

    auto getTurnouts() {
        return etl::views::values(etl::views::as_const(turnoutData));
    }

    size_t getTurnoutCount() { return turnoutData.size(); }

    TurnoutState turnoutToggle(const dcc::AccessoryAddress addr, bool fromRoster) {
        return turnoutAction(addr, fromRoster, TurnoutAction::TOGGLE);
    }

    TurnoutState getTurnoutState(const dcc::AccessoryAddress addr) {
        auto t = turnoutData.find(addr);
        if(t != turnoutData.end() ) {
            return t->second.tStatus;
        }
        return TurnoutState::UNKNOWN;
    }

    TurnoutState turnoutAction(dcc::AccessoryAddress addr, bool fromRoster, TurnoutAction action) {
        CS_DEBUGF("addr11=%d named=%d action=%d", addr.longAddr(), fromRoster, (int)action );

        TurnoutState newState = TurnoutState::THROWN;

        if(fromRoster) {
            auto t = turnoutData.find(addr);
            if(t != turnoutData.end() ) {
                if (action==TurnoutAction::TOGGLE) {
                    newState = toggleTurnout(t->second.tStatus);
                } else {  // throw or close
                    newState = actionToState(action);
                }

                t->second.tStatus = newState;
                addr = t->second.addr;
            } else {
                CS_DEBUGF("Did not find turnout in roster");
                return TurnoutState::UNKNOWN;
            }
        } else {
            if (action==TurnoutAction::TOGGLE) {
                CS_DEBUGF("Trying to toggle numeric turnout");
                newState = TurnoutState::THROWN;
            } else {  // throw or close
                newState = actionToState(action);
            }

            if(!turnoutData.full()) {
                // add turnout to roster
                addTurnout({addr, int(turnoutData.size()+1), newState});
                CS_DEBUGF("Added new turnout to roster: ID=%d, addr=%d", addr.get11bitAddr() );
            }
        }

        // send to DCC
        dccMain->sendAccessory(addr, newState==TurnoutState::THROWN);
        // send to LocoNet
        // FIXME: this is a dirty hack.
        // If LocoNet calls this function, it will be bounced back to bus.
        // Fortunately, right now, accessory commands from LocoNet do not get propagated to DCC
        // and this command is only called from WiThrottle code.
        if(locoNet!=nullptr) {
            LnMsg ttt = makeSwRec(addr.longAddr(), true, newState==TurnoutState::THROWN);
            locoNet->broadcast(ttt);
        }

        //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );

        return newState;
    }

private:
    dcc::BaseChannel * dccMain;
    dcc::BaseChannel * dccProg;
    LocoNetBus* locoNet;

    etl::map<LocoAddress, uint8_t, MAX_SLOTS> locoSlot;

    LocoData slots[MAX_SLOTS]; ///< slot 1 has index 0 in this array. Slot 0 is invalid.
    inline LocoData & getSlot(uint8_t slot) { return slots[slot-1]; }

    TurnoutMap turnoutData;

    void addTurnout(const TurnoutData &dd) {
        turnoutData[dd.addr] = dd;
    }

};

extern CommandStation CS;
