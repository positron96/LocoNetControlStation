#include "WiThrottle.h"

#include <etl/vector.h>

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED '2'
#define TURNOUT_THROWN '4'
    
/*
StringSumHelper & operator +(const StringSumHelper &lhs, const LocoAddress a) {
    return lhs + (a.isShort() ? 'S' : 'L') +a.addr();
}
*/

String addr2str(const LocoAddress & a) {
    return (a.isShort() ? 'S' : 'L') + String(a.addr());
}

LocoAddress str2addr(String addr) {
    uint16_t iLocoAddr = addr.substring(1).toInt();
    if (addr.charAt(0) == 'S') return LocoAddress::shortAddr(iLocoAddr);
    if (addr.charAt(0) == 'L') return LocoAddress::longAddr(iLocoAddr);
    return LocoAddress();
}

inline int invert(int value) {
    return value == 0 ? 1 : 0;
};


void WiThrottleServer::loop() {
    
    for (int iClient=0; iClient<MAX_CLIENTS; iClient++) {
        WiFiClient& cli = clients[iClient];
        ClientData& cc = clientData[iClient];
        if (!cli) {
            if(cc.connected) clientStop(iClient);
            cli = server.available();
            if(cli) clientStart(iClient);
        } 
        if (cli) { // connected client
            size_t a = cli.available();
            while(a-->0) { 
                /*uint32_t t0 = millis();
                String dataStr = cli.readStringUntil('\n'); 
                WT_LOGI("Read new cmd(%d b), took %d ms", dataStr.length(), millis()-t0);
                */
                char c = cli.read();
                //if(c>=32) WT_LOGI("read char '%c'", c); else WT_LOGI("read char #%d", c);
                if(c=='\n') {
                    cc.cmdline[cc.cmdpos] = 0;
                    processCmd(iClient);
                    cc.cmdpos = 0;
                } else {
                    if (cc.cmdpos<sizeof(cc.cmdline) ) 
                        cc.cmdline[cc.cmdpos++] = c;
                }
                if(cc.heartbeatEnabled) { 
                    cc.lastHeartbeat = millis();
                }
                
            }
            if (cc.heartbeatEnabled) {
                checkHeartbeat(iClient);
            }
        }
        
    }
}

void WiThrottleServer::processCmd(int iClient) {
    ClientData& cc = clientData[iClient];
    String dataStr = cc.cmdline;
    WT_LOGI("Read new cmd (%db): %d", dataStr.length(), cc.cmdline[0]);
    WT_LOGI("WTRX %s", cc.cmdline);
    switch(dataStr.charAt(0)) {
    case '*': {
        if(dataStr.length()>1) {
            switch(dataStr.charAt(1) ) {
                case '+' : cc.heartbeatEnabled = true; break;
                case '-' : cc.heartbeatEnabled = false; break;
            } 
        }
        break;
    }             
    case 'P': {
        if (dataStr.startsWith("PPA") ) {
            turnPower(dataStr.charAt(3) );
        } else if (dataStr.startsWith("PTA")) {
            char aStatus = dataStr.charAt(3);
            int aAddr;
            bool named;
            if(dataStr.substring(4,6)==TURNOUT_PREF) {
                // named turnout
                aAddr = dataStr.substring(6).toInt();
                named = true;
            } else {
                aAddr = dataStr.substring(4).toInt();
                named = false;
            }
            accessoryToggle(aAddr, aStatus, named);
        }
        break;
    }
    case 'N': { // device name
        WT_LOGI("Device ID: %s", cc.cmdline+1 );
        wifiPrintln(iClient, "*" + String(cc.heartbeatTimeout));
        break;
    }
    case 'H': {
        WT_LOGI("Hardware ID: %s", cc.cmdline+1 );
        break;
    }
    case 'M': {
        char th = dataStr.charAt(1);
        char action = dataStr.charAt(2);
        String actionData = dataStr.substring(3);
        int delimiter = actionData.indexOf(";");
        String actionKey = actionData.substring(0, delimiter-1);
        String actionVal = actionData.substring(delimiter+2);
        if (action == '+') {
            locoAdd(th, actionKey, iClient);
        } else if (action == '-') {
            locoRelease(th, actionKey, iClient);
        } else if (action == 'A') {
            locoAction(th, actionKey, actionVal, iClient);
        }
        break;
    }
    case 'Q':
        // quit
        clientStop(iClient);
        break;
    }
}

void WiThrottleServer::clientStart(int iClient) {
    clients[iClient].flush();
    clients[iClient].setTimeout(500);
    WT_LOGI( "New client " );
    ClientData & cc = clientData[iClient];

    wifiPrintln(iClient, "VN2.0");
    wifiPrintln(iClient, "RL0");
    wifiPrintln(iClient, String("PPA")+powerStatus );
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
    cc.cmdpos = 0;
}

void WiThrottleServer::clientStop(int iClient) {
    clients[iClient].stop();
    WT_LOGI("Client stopping");
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
    LocoAddress addr = str2addr(sLocoAddr);
    wifiPrintln(iClient, String("M")+th+"+"+sLocoAddr+"<;>");
    for (int fKey=0; fKey<29; fKey++) {
        wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>F0"+String(fKey));
    }
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>V0");
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>R1");
    wifiPrintln(iClient, String("M")+th+"A"+sLocoAddr+"<;>s1"); // TODO: this is speed steps 128 -> 1, 28->2 14->8

    //DEBUGS("loco add thr="+String(th)+"; addr"+String(sLocoAddr) );

    uint8_t slot = CS.findOrAllocateLocoSlot(addr);
    clientData[iClient].slots[th][addr] = slot;
    CS.setLocoSlotRefresh(slot, true);
}

void WiThrottleServer::locoRelease(char th, String sLocoAddr, int iClient) {
    //wifiPrintln(iClient, String("M")+th+"-"+sLocoAddr+"<;>");
    WT_LOGI("loco release thr=%c; addr=%s", th, String(sLocoAddr).c_str() );
    ClientData &client = clientData[iClient];

    if(sLocoAddr=="*") { 
        etl::vector<LocoAddress, MAX_LOCOS_PER_THROTTLE> tmp;
        for(const auto& slot: client.slots[th]) {
            tmp.push_back(slot.first);
        }
        for(const auto& addr: tmp) 
            locoRelease(th, addr, iClient);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoRelease(th, iLocoAddr, iClient);
    }
}

void WiThrottleServer::locoRelease(char th, LocoAddress addr, int iClient) {
    wifiPrintln(iClient, String("M")+th+"-"+addr2str(addr)+"<;>");
    //DEBUGS("loco release thr="+String(th)+"; addr "+String(addr) );

    ClientData &client = clientData[iClient];
    CS.releaseLocoSlot( client.slots[th][addr] );
    client.slots[th].erase(addr);
}


void WiThrottleServer::locoAction(char th, String sLocoAddr, String actionVal, int iClient) {
    //DEBUGS("loco action thr="+String(th)+"; action="+actionVal+"; addr "+sLocoAddr );
    ClientData &client = clientData[iClient];

    if(sLocoAddr=="*") { 
        for(const auto& slot: client.slots[th]) 
            locoAction(th, slot.first, actionVal, iClient);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoAction(th, iLocoAddr,  actionVal, iClient);
    }
}


void WiThrottleServer::locoAction(char th, LocoAddress iLocoAddr, String actionVal, int iClient) {
    ClientData &client = clientData[iClient];

    uint8_t slot = client.slots[th][iLocoAddr];

    WT_LOGI("loco action thr=%c; action=%s; addr %s ", th, actionVal.c_str(), String(iLocoAddr).c_str() );
    if (actionVal.startsWith("F1")) {
        int fKey = actionVal.substring(2).toInt();
        bool newVal = ! CS.getLocoFn(slot, fKey);
        CS.setLocoFn(slot, fKey, newVal );
        wifiPrintln(iClient, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + (newVal?"F1":"F0")+String(fKey));
    }
    else if (actionVal.startsWith("qV")) {
        //DEBUGS("query speed for loco "+String(dccLocoAddr) );
        wifiPrintln(iClient, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + "V"+String(CS.getLocoSpeed(slot)));							
    }
    else if (actionVal.startsWith("V")) {
        //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
        CS.setLocoSpeed(slot, actionVal.substring(1).toInt());
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));
    }
    else if (actionVal.startsWith("qR")) {
        //DEBUGS("query dir for loco "+String(dccLocoAddr) );
        wifiPrintln(iClient, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + "R"+String(CS.getLocoDir(slot) ));							
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
    else if (actionVal.startsWith("Q")) { // quit, TODO: kill throttle here
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
        CS.setLocoSpeed(slot, 0);
    }
}

void WiThrottleServer::checkHeartbeat(int iClient) {
    ClientData &c = clientData[iClient];
    if(! c.heartbeatEnabled) return;
    
    if ( (c.lastHeartbeat > 0) && (c.lastHeartbeat + c.heartbeatTimeout*1000 < millis() ) ) {
        // stop loco
        WT_LOGI("timeout exceeded: last: %d, current %d", c.lastHeartbeat/1000, millis()/1000 );
        c.lastHeartbeat = 0;
        for(const auto& throttle: c.slots)
            for(const auto& slot: throttle.second) {
                CS.setLocoSpeed(slot.second, 1); // emgr
                wifiPrintln(iClient, String("M")+throttle.first+"A"+addr2str(slot.first)+"<;>V"+CS.getLocoSpeed(slot.second));
            }
        
    }
}

void WiThrottleServer::accessoryToggle(int aAddr, char aStatus, bool namedTurnout) {

    WT_LOGI("turnout action, addr=%d; named: %c", aAddr, namedTurnout?'Y':'N' );

    TurnoutState newStat;
    switch(aStatus) {
        case TURNOUT_THROWN:
        case 'T': 
            newStat = CS.turnoutAction(aAddr, namedTurnout, TurnoutState::THROWN);
            break;
        case TURNOUT_CLOSED:
        case 'C': 
            newStat = CS.turnoutAction(aAddr, namedTurnout, TurnoutState::CLOSED);
            break;
        case '3': 
            newStat = CS.turnoutToggle(aAddr, namedTurnout);
            break;
        default: return;
    }


    char cStat = '3'; // unknown
    switch(newStat) {
        case TurnoutState::THROWN: cStat = TURNOUT_THROWN; break;
        case TurnoutState::CLOSED: cStat = TURNOUT_CLOSED; break;
    }

    for (int i=0; i<MAX_CLIENTS; i++) {
        if(clients[i]) wifiPrintln(i, String("PTA")+cStat+(namedTurnout?TURNOUT_PREF:"")+aAddr);
    }

}