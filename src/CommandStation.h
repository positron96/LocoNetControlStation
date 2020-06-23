#pragma once
/**
 * Contains all the stuff related to command station.
 * I.e. high-level DCC generation, turnout list and their respective 
 * settings.
 */


#include "LocoNetSlotManager.h"
#include "DCC.h"
#include "debug.h"

enum class TurnoutState {
    CLOSED, THROWN, TOGGLE
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
    CommandStation() : dccMain(0, 0, 0) {}

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

    const static int MAX_TURNOUTS = 100;
    TurnoutData turnoutData[MAX_TURNOUTS];

    uint16_t getTurnoutCount() { return 0; }
    
    void loadTurnouts() {
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

    TurnoutState turnoutAction(uint16_t aAddr, bool namedTurnout, TurnoutState newStat) {

        DEBUGS(String("turnout action, addr=")+aAddr+"; named:"+namedTurnout );

        if(namedTurnout) {
            
            for (int t = 0 ; (t<MAX_TURNOUTS) && (turnoutData[t].addr!=0) ; t++) {
                if(turnoutData[t].addr==aAddr) {
                    // turnout command
                    if (newStat==TurnoutState::TOGGLE) 
                        newStat = turnoutData[t].tStatus==TurnoutState::CLOSED ? TurnoutState::THROWN : TurnoutState::CLOSED;
                    
                    //sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);
                    dccMain.setAccessory(turnoutData[t].addr, turnoutData[t].subAddr, newStat==TurnoutState::THROWN ? 1 : 0);
                    turnoutData[t].tStatus = newStat;

                    //DEBUGS(String("parsed new status ")+newStat );
                    
                    break;
                }

            }

        } else {

            if(newStat==TurnoutState::TOGGLE) 
                newStat=TurnoutState::THROWN;
            // accessory command    
            int addr = ((aAddr-1) >> 2) + 1; 
            int sub  = (aAddr-1) & 0x3; 
            dccMain.setAccessory(addr, sub, newStat==TurnoutState::THROWN ? 1 : 0);
            //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );
            
        }

        return newStat;

    }

    bool isSlotTaken(uint8_t slot) {
        return false;
    }

    bool isLocoTaken(LocoAddress addr) {
        return false;
    }

    uint8_t locateLocoSlot(LocoAddress addr) {
        return 0;
    }

    void releaseLocoSlot(uint8_t slot) {

    }

    void setLocoFn(uint8_t slot, uint8_t fn, bool val) {

    }

    bool getLocoFn(uint8_t slot, uint8_t fn) {
        return false;
    }

    void setLocoDir(uint8_t slot, uint8_t dir) {

    }

    uint8_t getLocoDir(uint8_t slot) { 
        return 0; 
    }

    void setLocoSpeed(uint8_t slot, uint8_t spd) {

    }

    uint8_t getLocoSpeed(uint8_t slot) {
        return 0;
    }

    //const LocoNetSlotManager slotMan;
private:
    DCCESP32Channel dccMain;

};

extern CommandStation CS;