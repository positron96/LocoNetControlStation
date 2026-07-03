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
#include <etl/optional.h>
#include <etl/functional.h> // for reference_wrapper


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

class CommandStation {
public:

    static constexpr uint8_t N_FUNCTIONS = 29;

    static constexpr uint8_t MAX_SLOTS = 10;

    static constexpr millis_t PURGE_TIMEOUT = 120*1000; // 2min
    static constexpr millis_t SMALL_PURGE_TIMEOUT = 30'000;  // 30sec

    CommandStation(): dccMain(nullptr), dccProg(nullptr), locoNet(nullptr) {
        loadTurnouts();
    }

    /**
     * Updates slots that have not been used for a long time (PURGE_TIMEOUT)
     */
    void loop();

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


    struct LocoData {
        using Fns = etl::bitset<N_FUNCTIONS>;
        LocoAddress addr;
        LocoSpeed speed;
        SpeedMode speedMode;
        int8_t dir; ///< 1 = FWD, 0 = REW
        Fns fn;
        bool refreshing;
        Watchdog<PURGE_TIMEOUT, 500, SMALL_PURGE_TIMEOUT> wdt;
        void* owner; /// throttle that uses this slot
        bool allocated() const { return addr.isValid(); }
        void resetWatchdog() { wdt.kick(); }
        bool hasOwner() const { return owner!=nullptr;}
        uint8_t dccSpeedByte() const;
    private:
        void deallocate() { addr = LocoAddress(); }
        friend class CommandStation;
    };

    static bool isSlotSupported(uint8_t slot) { return slot>0 && slot<=MAX_SLOTS; }

    bool isSlotAllocated(uint8_t slot) const {
        if(!isSlotSupported(slot)) return true;
        return slots[slot-1].allocated();
    }

    bool isLocoAllocated(LocoAddress addr) {
        return locoSlot.find(addr) != locoSlot.end();
    }

    uint8_t findLocoSlot(LocoAddress addr);

    uint8_t locateFreeSlot();

    void initLocoSlot(uint8_t slot, LocoAddress addr);

    /** @returns 0 if slot wasn't created (no space) */
    uint8_t findOrAllocateLocoSlot(LocoAddress addr);

    void releaseLocoSlot(uint8_t slot);

    /** Return view on allocated slot data */
    auto getAllocatedSlots() const {
        return etl::views::values(etl::views::as_const(locoSlot));
    }

    size_t getAllocatedSlotsCount() const { return locoSlot.size(); }

    void setLocoSlotRefresh(uint8_t slot, bool refresh);

    void kickSlot(uint8_t slot) {
        assert(isSlotSupported(slot));
        LocoData &dd = getSlot(slot);
        if(!dd.allocated()) { CS_DEBUGF("slot not allocated"); return; }
        dd.resetWatchdog();
    }

    LocoAddress getLocoAddr(uint8_t slot) {
        if(!isSlotAllocated(slot)) return LocoAddress{};
        return getSlot(slot).addr;
    }

    const LocoData &getSlotData(uint8_t slot) {
        //assert(isSlotSupported(slot));
        return getSlot(slot);
    }

    void setSlotOwner(uint8_t slot, void* o);

        /**
     *  @param speed DCC speed (0=sop, 1=EMGR stop)
     *  @param dir 1 - FWD, 0 - REW
     * */
    void setLocoDir(uint8_t slot, uint8_t dir);
    uint8_t getLocoDir(uint8_t slot) { return getSlot(slot).dir; }


    void setLocoSpeed(uint8_t slot, LocoSpeed spd);
    LocoSpeed getLocoSpeed(uint8_t slot) { return getSlot(slot).speed;  }

    void setLocoSpeedF(uint8_t slot, float spd) { setLocoSpeed(slot, LocoSpeed::fromFloat(spd) ); }
    float getLocoSpeedF(uint8_t slot) { return getLocoSpeed(slot).getFloat(); }


    void setLocoSpeedMode(uint8_t slot, SpeedMode mode);
    SpeedMode getLocoSpeedMode(uint8_t slot) {
        return getSlot(slot).speedMode;
    }


    bool getLocoFn(uint8_t slot, uint8_t fn) { return  getSlot(slot).fn[fn] != 0; }

    /** Changes one function. */
    void setLocoFn(uint8_t slot, uint8_t fn, bool val);

    /** Changes bits of DCC function group. */
    void setLocoFns(uint8_t slot, dcc::fn_group fg, uint32_t vals);

    /** Changes bits across multiple function groups. */
    void setLocoFns(uint8_t slot, uint32_t mask, uint32_t vals );



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

    /*********** Turnouts ************/

    /* Define turnout object structures */
    struct TurnoutData {
        dcc::AccessoryAddress addr;
        int userTag;
        TurnoutState state;
    };

    static constexpr int MAX_TURNOUTS = 15;
    using TurnoutMap = etl::map<dcc::AccessoryAddress, TurnoutData, MAX_TURNOUTS>;

    void loadTurnouts();

    auto getTurnouts() {
        return etl::views::values(etl::views::as_const(turnoutData));
    }

    /** Might switch to ordinary const pointer in future. */
    etl::optional<etl::reference_wrapper<const TurnoutData>> findTurnout(const dcc::AccessoryAddress addr) {
        auto t = turnoutData.find(addr);
        if(t != turnoutData.end() ) {
            return etl::cref(t->second);
        }
        return etl::nullopt;
    }

    size_t getTurnoutCount() { return turnoutData.size(); }

    TurnoutState turnoutToggle(const dcc::AccessoryAddress addr, bool fromRoster);

    TurnoutState getTurnoutState(const dcc::AccessoryAddress addr);

    TurnoutState turnoutAction(dcc::AccessoryAddress addr, bool fromRoster, TurnoutAction action);

private:
    dcc::BaseChannel * dccMain;
    dcc::BaseChannel * dccProg;
    LocoNetBus* locoNet;

    using LocoSlotMap = etl::map<LocoAddress, uint8_t, MAX_SLOTS>;
    LocoSlotMap locoSlot;

    LocoData slots[MAX_SLOTS]; ///< slot 1 has index 0 in this array. Slot 0 is invalid.
    LocoData &getSlot(uint8_t slot) { return slots[slot-1]; }

    TurnoutMap turnoutData;

    void addTurnout(const TurnoutData &dd) {
        turnoutData[dd.addr] = dd;
    }

    LocoSlotMap::iterator releaseLocoSlot(LocoSlotMap::iterator it);

};

extern CommandStation CS;
