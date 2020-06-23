#include "WiThrottle.h"
    

StringSumHelper & operator +(const StringSumHelper &lhs, const LocoAddress a) {
    return lhs + (a.isShort() ? 'S' : 'L') +a.addr();
}

LocoAddress parseAddr(String addr) {
    uint16_t iLocoAddr = addr.substring(1).toInt();
    if (addr.charAt(0) == 'S') return LocoAddress::shortAddr(iLocoAddr);
    if (addr.charAt(0) == 'L') return LocoAddress::longAddr(iLocoAddr);
    return LocoAddress();
}

inline int invert(int value) {
    return value == 0 ? 1 : 0;
};

void WiThrottleServer::throttleStart(int iClient) {
    clients[iClient].flush();
    clients[iClient].setTimeout(500);
    DEBUGS("New client");
    ClientData & cc = clientData[iClient];

    wifiPrintln(iClient, "VN2.0");
    wifiPrintln(iClient, "RL0");
    wifiPrintln(iClient, "PPA"+powerStatus );
    wifiPrintln(iClient, "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{"+String(TURNOUT_CLOSED)+"]\\[Thrown}|{"+String(TURNOUT_THROWN) );
    wifiPrint(iClient, "PTL");
    for (int t = 0 ; t<CS.getTurnoutCount(); t++) {
        const CommandStation::TurnoutData &tt = CS.getTurnout(t);
        wifiPrint(iClient, String("]\\[")+TURNOUT_PREF+tt.addr+"}|{"+tt.id+"}|{"
            +(tt.tStatus==TurnoutState::THROWN ? TURNOUT_THROWN : TURNOUT_CLOSED) );
    }
    wifiPrintln(iClient, "");
    wifiPrintln(iClient, "*"+String(cc.heartbeatTimeout));
    cc.connected = true;
}

void WiThrottleServer::throttleStop(int iClient) {
    clients[iClient].stop();
    DEBUGS("Client lost");
    //alreadyConnected[iClient] = false;
    //heartbeatEnable[iClient] = false;
    ClientData &client = clientData[iClient];
    
    for(const auto& thrSlots: client.slots) {
        for(const auto& slots: thrSlots.second) {
            CS.releaseLocoSlot(slots.second);
        }
    }
    client.slots.clear();
    client.heartbeatEnabled = false;
    client.connected = false;

}

void WiThrottleServer::locoAdd(char th, String sLocoAddr, int iClient) {
    LocoAddress addr = parseAddr(sLocoAddr);
    wifiPrintln(iClient, String("M")+th+"+"+sLocoAddr+"<;>");
    for (int fKey=0; fKey<29; fKey++) {
        wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>F0"+String(fKey));
    }
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>V0");
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>R1");
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>s1");
    DEBUGS("loco add thr="+String(th)+"; addr"+String(sLocoAddr) );

    uint8_t slot = CS.locateLocoSlot(addr);
    clientData[iClient].slots[th][addr] = slot;

}

void WiThrottleServer::locoRelease(char th, String sLocoAddr, int iClient) {
    wifiPrintln(iClient, String("M")+th+"-"+sLocoAddr+"<;>");
    DEBUGS("loco release thr="+String(th)+"; addr"+String(sLocoAddr) );
    // stop now
    ClientData &client = clientData[iClient];
    LocoAddress iLocoAddr = parseAddr(sLocoAddr);
    CS.releaseLocoSlot( client.slots[th][iLocoAddr] );
    client.slots[th].erase(iLocoAddr);
}

void WiThrottleServer::locoAction(char th, String sLocoAddr, String actionVal, int iClient) {
    ClientData &client = clientData[iClient];

    LocoAddress iLocoAddr;
    if (sLocoAddr == "*") {
        iLocoAddr = client.slots[th].begin()->first;
    } else iLocoAddr = parseAddr(sLocoAddr);

    uint8_t slot = client.slots[th][iLocoAddr];

    DEBUGS("loco action thr="+String(th)+"; action="+actionVal+"; DCC "+iLocoAddr );
    if (actionVal.startsWith("F1")) {
        int fKey = actionVal.substring(2).toInt();
        bool newVal = ! CS.getLocoFn(slot, fKey);
        CS.setLocoFn(slot, fKey, newVal );
        wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>" + (newVal?"F1":"F0")+String(fKey));
    }
    else if (actionVal.startsWith("qV")) {
        //DEBUGS("query speed for loco "+String(dccLocoAddr) );
        wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>" + "V"+String(CS.getLocoSpeed(slot)));							
    }
    else if (actionVal.startsWith("V")) {
        //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
        CS.setLocoSpeed(slot, actionVal.substring(1).toInt());
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));

    }
    else if (actionVal.startsWith("qR")) {
        //DEBUGS("query dir for loco "+String(dccLocoAddr) );
        wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>" + "R"+String(CS.getLocoDir(slot) ));							
    }
    else if (actionVal.startsWith("R")) {
        //DEBUGS("Sending dir to addr "+String(dccLocoAddr) );
        CS.setLocoDir(slot, actionVal.substring(1).toInt() );

    }
    else if (actionVal.startsWith("X")) { // EMGR stop
        CS.setLocoSpeed(slot, 1);
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
    }
    else if (actionVal.startsWith("I")) { // idle
        // sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
        CS.setLocoSpeed(slot, 0);
    }
    else if (actionVal.startsWith("Q")) { // quit
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
        CS.setLocoSpeed(slot, 0);
    }
}

void WiThrottleServer::checkHeartbeat(int iClient) {
    ClientData &c = clientData[iClient];
    
    if (c.lastHeartbeat > 0 && c.lastHeartbeat + c.heartbeatTimeout * 1000 < millis()) {
        // stop loco
        c.lastHeartbeat = 0;
        for(const auto& throttle: c.slots)
            for(const auto& slot: throttle.second) {
                CS.setLocoSpeed(slot.second, 1); // emgr
                wifiPrintln(iClient, String("M")+throttle.first+"A"+slot.first+"<;>V0");
            }
        
    }
}

void WiThrottleServer::accessoryToggle(int aAddr, char aStatus, bool namedTurnout) {
    TurnoutState newStat = TurnoutState::TOGGLE;
    switch(aStatus) {
        case 'T': newStat=TurnoutState::THROWN; break;
        case 'C': newStat=TurnoutState::CLOSED; break;
    }

    DEBUGS(String("turnout action, addr=")+aAddr+"; named:"+namedTurnout );

    newStat = CS.turnoutAction(aAddr, namedTurnout, newStat);

    int wStat = 3; // unknown
    switch(newStat) {
        case TurnoutState::THROWN: wStat = TURNOUT_THROWN; break;
        case TurnoutState::CLOSED: wStat = TURNOUT_CLOSED; break;
        default: break; //should not get here
    }

    for (int i=0; i<MAX_CLIENTS; i++) {
        if(clients[i])
        wifiPrintln(i, String("PTA")+wStat+(namedTurnout?TURNOUT_PREF:"")+aAddr);
    }

}