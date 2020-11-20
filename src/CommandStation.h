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


#define CS_DEBUG

#ifdef CS_DEBUG
#include <Arduino.h>
#define CS_DEBUGF(...)  { Serial.printf(__VA_ARGS__); }
#else
#define CS_DEBUGF
#endif


enum class TurnoutState {
    CLOSED=0, THROWN=1
};

class CommandStation {
public:

    static const uint8_t MAX_SLOTS = 10;
    
    CommandStation(): dccMain(nullptr), dccProg(nullptr), locoNet(nullptr) { 
        loadTurnouts();  
    }

    void setDccMain(IDCCChannel * ch) { dccMain = ch; }
    void setDccProg(IDCCChannel * ch) { dccProg = ch; }
    void setLocoNetBus(LocoNetBus *bus) { locoNet = bus; }

    void setPowerState(bool v) {
        dccMain->setPower(v);
    }

    bool getPowerState() const { 
        return dccMain!=nullptr ? dccMain->getPower() 
             : dccProg!=nullptr ? dccProg->getPower() 
             : false; 
    }


    /* Define turnout object structures */
    struct TurnoutData {
        uint16_t addr;	
        uint8_t subAddr;
        uint16_t id;
        TurnoutState tStatus;
    };

    const static int MAX_TURNOUTS = 15;
    TurnoutData turnoutData[MAX_TURNOUTS];

    uint16_t getTurnoutCount() { return 2; }
    
    void loadTurnouts() {
        turnoutData[0] = { 10, 1, 100, TurnoutState::CLOSED };
        turnoutData[1] = { 20, 2, 200, TurnoutState::THROWN };
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

    const TurnoutData& getTurnout(uint16_t i) { return turnoutData[i]; }

    TurnoutState turnoutToggle(uint16_t aAddr, bool fromRoster) {
        return turnoutAction(aAddr, fromRoster, -1);
    }

    TurnoutState turnoutAction(uint16_t aAddr, bool fromRoster, TurnoutState newStat) {
        return turnoutAction(aAddr, fromRoster, (int)newStat);
    }

    bool isSlotAllocated(uint8_t slot) {
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
        _slot.speed = 0;
        _slot.speedMode = LocoData::SpeedMode::S128;
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
        if(slot==0) { CS_DEBUGF("CommandStation::releaseLocoSlot: invalid slot\n"); return; }
        uint8_t i = slot-1;
        CS_DEBUGF("CommandStation::releaseLocoSlot: releasing slot %d\n", slot); 
        setLocoSlotRefresh(slot, false);
        locoSlot.erase( slots[i].addr );        
        slots[i].deallocate();
    }

    void setLocoSlotRefresh(uint8_t slot, bool refresh) {
        if(slot==0) { CS_DEBUGF("CommandStation::setLocoSlotRefresh: invalid slot\n"); return; }
        LocoData &dd = getSlot(slot);
        if(dd.refreshing == refresh) return;
        CS_DEBUGF("CommandStation::setLocoSlotRefresh: slot %d refresh %d\n", slot, refresh); 
        dd.refreshing = refresh;
        if(refresh) {
            
        } else {
            dccMain->unloadSlot(slot);
        }
    }

    void setLocoFn(uint8_t slot, uint8_t fn, bool val) {
        LocoData &dd = getSlot(slot);
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
        uint32_t v = dd.fn.value<uint32_t>();
        // if required bits (m) intersect function group bits (GM) and these bits (f^v != 0) differ from current
        // update bits (v=) and set function group
        #define CHECK_SEND(GM, FG)  if(  ( (m&GM)!=0) && ( ( (v^f)&m&GM)!=0 ) )  \
            { v = (v&(0xFFFFFFFF&~GM)) | (f&m&GM);   dccMain->sendFunctionGroup(slot, dd.addr, FG, v ); }  

        CHECK_SEND(     0x1F, DCCFnGroup::F0_4);
        CHECK_SEND(    0x1E0, DCCFnGroup::F5_8);
        CHECK_SEND(   0x1E00, DCCFnGroup::F9_12);
        CHECK_SEND( 0x1FE000, DCCFnGroup::F13_20);
        CHECK_SEND(0x1FE0000, DCCFnGroup::F21_28);
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
        if(dd.dir==dir) return; 
        dd.dir = dir;
        dccMain->sendThrottle(slot, dd.addr, dd.speed, dd.dir);
    }

    uint8_t getLocoDir(uint8_t slot) { 
        return getSlot(slot).dir;
    }

    /// Sets DCC-formatted speed (0-stop, 1-EMGR stop, 2-... moving speed)
    void setLocoSpeed(uint8_t slot, uint8_t spd) {
        LocoData &dd = getSlot(slot);
        if(dd.speed == spd) return;
        dd.speed = spd;
        dccMain->sendThrottle(slot, dd.addr, dd.speed, dd.dir);
    }

    /// Returns DCC-formatted speed (0-stop, 1-EMGR stop, ...)
    uint8_t getLocoSpeed(uint8_t slot) {
        return getSlot(slot).speed;
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

private:
    IDCCChannel * dccMain;
    IDCCChannel * dccProg;
    LocoNetBus* locoNet;

    struct LocoData {
        using Fns = etl::bitset<29>;
        LocoAddress addr;
        uint8_t speed;
        enum class SpeedMode { S14, S28, S128 };
        SpeedMode speedMode;
        int8_t dir;
        Fns fn;
        bool refreshing;
        bool allocated() { return addr.isValid(); }
        void deallocate() { addr = LocoAddress(); }
    };

    etl::map<LocoAddress, uint8_t, MAX_SLOTS> locoSlot;

    LocoData slots[MAX_SLOTS]; ///< slot 1 has index 0 in this array. Slot 0 is invalid.
    LocoData & getSlot(uint8_t slot) { return slots[slot-1]; }

    TurnoutState turnoutAction(uint16_t aAddr, bool fromRoster, int8_t newStat) {
        CS_DEBUGF("CommandStation::turnoutAction addr=%d named=%d new state=%d\n", aAddr, fromRoster, newStat );

        if(fromRoster) {
            
            for (int t = 0 ; (t<MAX_TURNOUTS) && (turnoutData[t].addr!=0) ; t++) {
                if(turnoutData[t].addr==aAddr) {
                    // turnout command
                    if (newStat==-1) 
                        newStat = turnoutData[t].tStatus==TurnoutState::CLOSED ? 0 : 1;
                    
                    //sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);
                    //dccMain.sendAccessory(turnoutData[t].addr, turnoutData[t].subAddr, newStat);
                    turnoutData[t].tStatus = (TurnoutState)newStat;

                    //DEBUGS(String("parsed new status ")+newStat );
                    
                    break;
                }

            }

        } else {

            if(newStat==-1) 
                newStat=(int)TurnoutState::THROWN;
        }

        // send to DCC
        dccMain->sendAccessory(aAddr, newStat==1);
        // send to LocoNet
        if(locoNet!=nullptr) {
            LnMsg ttt = makeSwRec(aAddr, true, newStat==1);
            locoNet->broadcast(ttt);
        }
        
        //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );

        return (TurnoutState)newStat;
    }

};

extern CommandStation CS;