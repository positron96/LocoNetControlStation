#pragma once
/**
 * Contains all the stuff related to command station.
 * I.e. high-level DCC generation, turnout list and their respective 
 * settings.
 */

#include <etl/map.h>
#include <etl/bitset.h>

#include "DCC.h"
#include "LocoAddress.h"
#include <LocoNet.h>

#include "Watchdog.h"


#define CS_DEBUG

#ifdef CS_DEBUG
#define CS_DEBUGF(format, ...)  do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__);  } while(0)
#else
#define CS_DEBUGF
#endif

#define  STATES    CLOSED,THROWN
enum class TurnoutState {
    STATES, UNKNOWN
};
enum class TurnoutAction {
    STATES, TOGGLE
};

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

    void setDccMain(IDCCChannel * ch) { dccMain = ch; }
    void setDccProg(IDCCChannel * ch) { dccProg = ch; }
    void setLocoNetBus(LocoNetBus *bus) { locoNet = bus; }

    void setPowerState(bool v) {
        if( dccMain!=nullptr ) dccMain->setPower(v);
    }

    bool getPowerState() const { 
        return dccMain!=nullptr ? dccMain->getPower() 
             //: dccProg!=nullptr ? dccProg->getPower() 
             : false; 
    }


    /* Define turnout object structures */
    struct TurnoutData {
        uint16_t addr11;	
        int userTag;
        TurnoutState tStatus;
    };

    static constexpr int MAX_TURNOUTS = 15;
    using TurnoutMap = etl::map<uint16_t, TurnoutData, MAX_TURNOUTS>;
    TurnoutMap turnoutData;

    uint16_t getTurnoutCount() { return turnoutData.size(); }
    
    void loadTurnouts() {
        turnoutData[6] = { 6, 0, TurnoutState::CLOSED };
        turnoutData[7] = { 7, 1, TurnoutState::CLOSED };
        turnoutData[10] = { 10, 2, TurnoutState::UNKNOWN };
        turnoutData[11] = { 11, 3, TurnoutState::THROWN };
        
        /*sendDCCppCmd("T");
        waitForDCCpp();
        int t = 0;
        while(Serial.available()>0) {
            char data[maxCommandLength];
            sprintf(data, "%s", readResponse().c_str() );
            if (strlen(data)==0) break;
            int addr, sub, stat, id;
            int ret = sscanf(data, "%*c %d %d %d %d", &id, &addr, &sub, &stat );
            turnoutData[t] = { ((addr-1)*4+1) + (sub&0x3) , id, stat==0 ? 2 : 4};
            t++;
        }*/
    }

    //const TurnoutData& getTurnout(uint16_t i) { return turnoutData[i]; }

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

    void setLocoSlotRefresh(uint8_t slot, bool refresh) {
        if(slot==0) { CS_DEBUGF("invalid slot"); return; }
        LocoData &dd = getSlot(slot);
        if(!dd.allocated()) { CS_DEBUGF("slot not allocated"); return; }
        if(dd.refreshing == refresh) return;
        CS_DEBUGF("slot %d refresh %c", slot, refresh?'Y':'N'); 
        dd.refreshing = refresh;
        
        if(refresh) {
            // no need to load, it will load itself on setLocoSpeed
            dd.kickWatchdog();
        } else {
            dccMain->unloadSlot(slot);
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
            dccMain->sendThrottle(slot, dd.addr, dd.dccSpeedByte(), dd.speedMode, dd.dir);
    }

    SpeedMode getLocoSpeedMode(uint8_t slot) {
        return getSlot(slot).speedMode;
    }

    void setLocoFn(uint8_t slot, uint8_t fn, bool val) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        if(dd.fn[fn] == val) return;

        dd.fn[fn] = val;
        DCCFnGroup fg;
        
        uint32_t ifn = dd.fn.value<uint32_t>();
        if     (fn<5)  fg = DCCFnGroup::F0_4;
        else if(fn<9)  fg = DCCFnGroup::F5_8;
        else if(fn<13) fg = DCCFnGroup::F9_12;
        else if(fn<21) fg = DCCFnGroup::F13_20;
        else           fg = DCCFnGroup::F21_28;
        dccMain->sendFunctionGroup(slot, dd.addr, fg, ifn);
    }

    void setLocoFns(uint8_t slot, uint32_t m, uint32_t f ) {
        LocoData &dd = getSlot(slot);
        dd.kickWatchdog();
        uint32_t v = dd.fn.value<uint32_t>();
        // if required bits (m) intersect function group bits (GM) and these bits (f^v != 0) differ from current value,
        // update bits (v=) and send function group
        #define CHECK_SEND(GM, FG)  if(  ( (m&GM)!=0) && ( ( (v^f)&m&GM)!=0 ) )  \
            { v = (v&(0xFFFF'FFFF&~GM)) | (f&m&GM);   dccMain->sendFunctionGroup(slot, dd.addr, FG, v ); }  

        CHECK_SEND(      0x1F, DCCFnGroup::F0_4);
        CHECK_SEND(     0x1E0, DCCFnGroup::F5_8);
        CHECK_SEND(    0x1E00, DCCFnGroup::F9_12);
        CHECK_SEND( 0x1F'E000, DCCFnGroup::F13_20);
        CHECK_SEND(0x1FE'0000, DCCFnGroup::F21_28);
        dd.fn = LocoData::Fns( v );
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
            dccMain->sendThrottle(slot, dd.addr, dd.dccSpeedByte(), dd.speedMode, dd.dir);
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
            dccMain->sendThrottle(slot, dd.addr, dd.dccSpeedByte(), dd.speedMode, dd.dir);
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

    const TurnoutMap& getTurnouts() {
        return turnoutData;
    }

    TurnoutState turnoutToggle(uint16_t aAddr, bool fromRoster) {
        return turnoutAction(aAddr, fromRoster, TurnoutAction::TOGGLE);
    }

    TurnoutState getTurnoutState(uint16_t aAddr) {
        auto t = turnoutData.find(aAddr);
        if(t != turnoutData.end() ) {
            return t->second.tStatus;
        }
        return TurnoutState::UNKNOWN;
    }

    TurnoutState turnoutAction(uint16_t aAddr, bool fromRoster, TurnoutAction action) {
        CS_DEBUGF("addr=%d named=%d action=%d", aAddr, fromRoster, (int)action );

        TurnoutState newState = TurnoutState::THROWN;

        if(fromRoster) {
            auto t = turnoutData.find(aAddr);
            if(t != turnoutData.end() ) {
                if (action==TurnoutAction::TOGGLE) {
                    newState = t->second.tStatus==TurnoutState::THROWN ? TurnoutState::CLOSED : TurnoutState::THROWN;
                } else {  // throw or close
                    newState = (TurnoutState)(int)action;
                }
                
                //sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);
                //dccMain.sendAccessory(turnoutData[t].addr, turnoutData[t].subAddr, newStat);
                t->second.tStatus = newState;
                aAddr = t->second.addr11;
            } else {
                CS_DEBUGF("Did not find turnout in roster");
                return TurnoutState::UNKNOWN;
            }
        } else {
            if (action==TurnoutAction::TOGGLE) {
                CS_DEBUGF("Trying to toggle numeric turnout");
                newState = TurnoutState::THROWN;
            } else {  // throw or close
                newState = (TurnoutState)(int)action;
            }

            if(turnoutData.available()>0) {
                // add turnout to roster
                turnoutData[aAddr] = {aAddr, int(turnoutData.size()+1), newState};
                CS_DEBUGF("Added new turnout to roster: ID=%d, addr=%d", aAddr, aAddr );
            }
        }

        // send to DCC
        dccMain->sendAccessory(aAddr, newState==TurnoutState::THROWN);
        // send to LocoNet
        // FIXME: this is a dirty hack. 
        // If LocoNet calls this function, it will be bounced back to bus.
        // Fortunately, right now, accessory commands from LocoNet do not get propagated to DCC
        // and this command is only called from WiThrottle code.
        if(locoNet!=nullptr) {
            LnMsg ttt = makeSwRec(aAddr, true, newState==TurnoutState::THROWN);
            locoNet->broadcast(ttt);
        }
        
        //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );

        return newState;
    }

private:
    IDCCChannel * dccMain;
    IDCCChannel * dccProg;
    LocoNetBus* locoNet;

    etl::map<LocoAddress, uint8_t, MAX_SLOTS> locoSlot;

    LocoData slots[MAX_SLOTS]; ///< slot 1 has index 0 in this array. Slot 0 is invalid.
    inline LocoData & getSlot(uint8_t slot) { return slots[slot-1]; }

};

extern CommandStation CS;