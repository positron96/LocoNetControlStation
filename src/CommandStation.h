#pragma once
/**
 * Contains all the stuff related to command station.
 * I.e. high-level DCC generation, turnout list and their respective 
 * settings.
 */

#include <etl/map.h>

#include "LocoNetSlotManager.h"
#include "DCC.h"
#include "debug.h"


enum class TurnoutState {
    CLOSED=0, THROWN=1
};

class LocoAddress {
public:
    LocoAddress() : num(0) {}
    static LocoAddress shortAddr(uint8_t addr) {  return LocoAddress(addr); }
    static LocoAddress longAddr(uint16_t addr) {  return LocoAddress(-addr); }
    bool isShort() const { return num>=0; }
    bool isLong() const { return num<=0; }
    uint16_t addr() const { return abs(num); }
    bool isValid() const { return num!=0; }
    bool operator < (const LocoAddress& a) const { return (num < a.num); }
    operator String() const {  return String( (isShort() ? 'S' : 'L') )+addr(); }
private:
    LocoAddress(int16_t num): num(num) { }
    int16_t num;
};

class CommandStation {
public:
    CommandStation() { loadTurnouts();  }//: dccMain(0, 0, 0) {}

    void turnPower(bool v) {

    }

    bool getPowerState() const { return false; }


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
        turnoutData[1] = { 20, 2, 200, TurnoutState::CLOSED };
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

    TurnoutState turnoutToggle(uint16_t aAddr, bool namedTurnout) {
        return turnoutAction(aAddr, namedTurnout, -1);
    }

    TurnoutState turnoutAction(uint16_t aAddr, bool namedTurnout, TurnoutState newStat) {
        return turnoutAction(aAddr, namedTurnout, (int)newStat);
    }

    bool isSlotTaken(uint8_t slot) {
        if(slot<1 || slot>MAX_SLOTS) return true;
        return slots[slot-1].addr.isValid();
    }

    bool isLocoTaken(LocoAddress addr) {
        return locoSlot.find(addr) != locoSlot.end();
    }

    uint8_t locateLocoSlot(LocoAddress addr) {

        auto it = locoSlot.find(addr);
        if(it != locoSlot.end() ) {
            return it->second;
        }
        if(locoSlot.size() < MAX_SLOTS) {
            for(int i=0; i<MAX_SLOTS; i++) {
                if(!slots[i].allocated() ) {
                    slots[i].addr = addr;
                    uint8_t slot = i+1;
                    locoSlot[addr] = slot;
                    return slot;
                }
            }
        }
        return 0;
    }

    void releaseLocoSlot(uint8_t slot) {
        if(slot==0) { DEBUGS("CommandStation::releaseSlot: invalid slot"); return; }
        uint8_t i = slot-1;        
        locoSlot.erase( slots[i].addr );
        slots[i].deallocate();
    }

    void setlocoRefresh(uint8_t slot, bool refresh) {
        if(slot==0) { DEBUGS("CommandStation::releaseSlot: invalid slot"); return; }

        getSlot(slot).refreshing = refresh;
    }

    void setLocoFn(uint8_t slot, uint8_t fn, bool val) {
        if(val) 
            getSlot(slot).fnMask |= (1<<fn);
        else
            getSlot(slot).fnMask &= ~ (1<<fn);
    }

    bool getLocoFn(uint8_t slot, uint8_t fn) {
        return (getSlot(slot).fnMask & (1<<fn) ) != 0;
    }

    void setLocoDir(uint8_t slot, uint8_t dir) {
        getSlot(slot).dir = dir;
    }

    uint8_t getLocoDir(uint8_t slot) { 
        return getSlot(slot).dir;
    }

    /// Sets DCC-formatted speed (0-stop, 1-EMGR stop, 2-... moving speed)
    void setLocoSpeed(uint8_t slot, uint8_t spd) {
        getSlot(slot).speed = spd;
    }

    /// Returns DCC-formatted speed (0-stop, 1-EMGR stop, ...)
    uint8_t getLocoSpeed(uint8_t slot) {
        return getSlot(slot).speed;
    }

    //const LocoNetSlotManager slotMan;
private:
    //DCCESP32Channel dccMain;

    struct LocoData {
        LocoAddress addr;
        uint8_t speed;
        enum class SpeedMode { S14, S28, S128 };
        SpeedMode speedMode;
        int8_t dir;
        uint32_t fnMask;
        bool refreshing;
        bool allocated() { return addr.isValid(); }
        void deallocate() { addr = LocoAddress(); }
    };

    static const uint8_t MAX_SLOTS = 10;

    etl::map<LocoAddress, uint8_t, MAX_SLOTS> locoSlot;

    LocoData slots[MAX_SLOTS]; ///< slot 1 has index 0 in this array. Slot 0 is invalid.
    LocoData & getSlot(uint8_t slot) { return slots[slot-1]; }

    TurnoutState turnoutAction(uint16_t aAddr, bool namedTurnout, int8_t newStat) {
        DEBUGS(String("turnout action, addr=")+aAddr+"; named:"+namedTurnout );

        if(namedTurnout) {
            
            for (int t = 0 ; (t<MAX_TURNOUTS) && (turnoutData[t].addr!=0) ; t++) {
                if(turnoutData[t].addr==aAddr) {
                    // turnout command
                    if (newStat==-1) 
                        newStat = turnoutData[t].tStatus==TurnoutState::CLOSED ? 0 : 1;
                    
                    //sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);
                    //dccMain.setAccessory(turnoutData[t].addr, turnoutData[t].subAddr, newStat);
                    turnoutData[t].tStatus = (TurnoutState)newStat;

                    //DEBUGS(String("parsed new status ")+newStat );
                    
                    break;
                }

            }

        } else {

            if(newStat==-1) 
                newStat=(int)TurnoutState::THROWN;
            // accessory command
            int addr = ((aAddr-1) >> 2) + 1; 
            int sub  = (aAddr-1) & 0x3; 
            //dccMain.setAccessory(addr, sub, newStat);
            //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );
            
        }

        return (TurnoutState)newStat;
    }

};

extern CommandStation CS;