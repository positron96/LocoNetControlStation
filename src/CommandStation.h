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

class CommandStation {
public:
    void turnPower(bool v) {

    }


    /* Define turnout object structures */
    struct TurnoutData {
        uint16_t addr;	
        uint8_t subAddr;
        uint16_t id;
        TurnoutState tStatus;
    } ;
    const int MAX_TURNOUTS = 100;
    TurnoutData turnoutData[MAX_TURNOUTS];
    

    TurnoutState turnoutAction(uint16_t aAddr, bool namedTurnout, TurnoutState newStat) {

        DEBUGS(String("turnout action, addr=")+aAddr+"; named:"+namedTurnout );

        if(namedTurnout) {
            
            for (int t = 0 ; (t<MAX_TURNOUTS) && (turnoutData[t].addr!=0) ; t++) {
                if(turnoutData[t].addr==aAddr) {
                    // turnout command
                    if (newStat==TurnoutState::TOGGLE) 
                        newStat = turnoutData[t].tStatus==TurnoutState::CLOSED ? TurnoutState::THROWN : TurnoutState::CLOSED;
                    
                    //sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);
                    dccMain.setAccessory(tunoutData[t].addr, turnoutData[t].subAddr, newStat==TurnoutState::THROWN ? 1 : 0);
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

    //const LocoNetSlotManager slotMan;
private:
    DCCESP32Channel dccMain;

};

extern CommandStation CS;