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
    return (value == 0) ? 1 : 0;
};


WiThrottleServer::WiThrottleServer(uint16_t port) : port(port), server(port) {
    server.onClient( [this](void*, AsyncClient* cli ) {
        if(clients.full()) {
            WT_LOGI("onConnect: Not accepting client: %s", cli->remoteIP().toString().c_str() );
            cli->close();
            return;
        }
        //cli->setKeepAlive(10000, 2);
        WT_LOGI("onConnect: New client(%X): %s", (intptr_t)cli, cli->remoteIP().toString().c_str() );
        clientStart(cli); 

        cli->onDisconnect([this](void*, AsyncClient* cli) {
            WT_LOGI("onDisconnect: Client(%X) disconnected", (intptr_t)cli );
            clientStop(cli);
        });

        cli->onData( [this](void*, AsyncClient* cli, void *data, size_t len) {
            ClientData &cc = clients[cli];
            if(cc.heartbeatEnabled) { 
                cc.lastHeartbeat = millis();
            }
            for(size_t i=0; i<len; i++) {
                char c = ((char*)data)[i];
                if(c=='\n') {
                    cc.cmdline[cc.cmdpos] = 0;
                    processCmd(cli);
                    cc.cmdpos = 0;
                } else {
                    if (cc.cmdpos<sizeof(cc.cmdline) ) 
                        cc.cmdline[cc.cmdpos++] = c;
                }
            }
        });

        cli->onError([this](void*, AsyncClient* cli, int8_t err) { 
            WT_LOGI("onError(%X): %d", (intptr_t)cli, err);  
        });
        cli->onTimeout([this](void*, AsyncClient* cli, uint32_t time) { 
            WT_LOGI("onTimeout(%X): %d", (intptr_t)cli, time); 
            cli->close(); 
        });
    }, nullptr);
}

void WiThrottleServer::begin() {

    WT_LOGI("WiThrottleServer::begin");

    server.begin();

    //MDNS.begin(hostString);
    MDNS.addService("withrottle","tcp", port);
    //MDNS.setInstanceName("DCC++ Network Interface");

    notifyPowerStatus();

}

void WiThrottleServer::loop() {
    
    for (auto &p: clients) {
        ClientData &cc = p.second;
        if (cc.heartbeatEnabled) {
                checkHeartbeat(p.first);
        }
    }

}

void WiThrottleServer::processCmd(AsyncClient* cli) {
    ClientData& cc = clients[cli];
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
        } else {}
        break;
    }
    case 'N': { // device name
        WT_LOGI("Device ID: %s", cc.cmdline+1 );
        wifiPrintln(cli, "*" + String(cc.heartbeatTimeout));
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
            locoAdd(th, actionKey, cli);
        } else if (action == '-') {
            locoRelease(th, actionKey, cli);
        } else if (action == 'A') {
            locoAction(th, actionKey, actionVal, cli);
        }
        break;
    }
    case 'Q':
        // quit
        clientStop(cli);
        break;
    default:
        break;
    }
}

void WiThrottleServer::clientStart(AsyncClient* cli) {
    
    ClientData cc;
    WT_LOGI( "New client " );

    wifiPrintln(cli, "VN2.0");
    wifiPrintln(cli, "RL0");
    wifiPrintln(cli, String("PPA")+powerStatus );
    wifiPrintln(cli, "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{"+String(TURNOUT_CLOSED)+"]\\[Thrown}|{"+String(TURNOUT_THROWN) );
    wifiPrint(cli, "PTL");
    for(const auto &t: CS.getTurnouts() ) {
    //for (int t = 0 ; t<CS.getTurnoutCount(); t++) {
        //const CommandStation::TurnoutData &tt = CS.getTurnout(t);
        const CommandStation::TurnoutData &tt = t.second;
        wifiPrint(cli, String("]\\[")+TURNOUT_PREF+tt.addr11+"}|{"+tt.id+"}|{"
            +(tt.tStatus==TurnoutState::THROWN ? TURNOUT_THROWN : TURNOUT_CLOSED) );
    }
    wifiPrintln(cli, "");
    wifiPrintln(cli, "*"+String(cc.heartbeatTimeout));

    cc.connected = true;
    cc.cmdpos = 0;

    clients[cli] = cc;
}

void WiThrottleServer::clientStop(AsyncClient* cli) {
    //clients[cli].stop();
    WT_LOGI("Client stopping");
    //alreadyConnected[iClient] = false;
    //heartbeatEnable[iClient] = false;
    ClientData &client = clients[cli];
    
    for(const auto& thrSlots: client.slots) {
        for(const auto& slots: thrSlots.second) {
            CS.releaseLocoSlot(slots.second);
        }
    }
    client.slots.clear();
    client.heartbeatEnabled = false;
    client.connected = false;
    if(cli->connected() ) cli->stop();
    clients.erase(cli);

}

void WiThrottleServer::locoAdd(char th, String sLocoAddr, AsyncClient* cli) {
    LocoAddress addr = str2addr(sLocoAddr);
    wifiPrintln(cli, String("M")+th+"+"+sLocoAddr+"<;>");
    for (int fKey=0; fKey<29; fKey++) {
        wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>F0"+String(fKey));
    }
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>V0");
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>R1");
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>s1"); // TODO: this is speed steps 128 -> 1, 28->2 14->8

    //DEBUGS("loco add thr="+String(th)+"; addr"+String(sLocoAddr) );

    uint8_t slot = CS.findOrAllocateLocoSlot(addr);
    clients[cli].slots[th][addr] = slot;
    CS.setLocoSlotRefresh(slot, true);
}

void WiThrottleServer::locoRelease(char th, String sLocoAddr, AsyncClient* cli) {
    //wifiPrintln(iClient, String("M")+th+"-"+sLocoAddr+"<;>");
    WT_LOGI("loco release thr=%c; addr=%s", th, String(sLocoAddr).c_str() );
    ClientData &client = clients[cli];

    if(sLocoAddr=="*") { 
        etl::vector<LocoAddress, MAX_LOCOS_PER_THROTTLE> tmp;
        for(const auto& slot: client.slots[th]) {
            tmp.push_back(slot.first);
        }
        for(const auto& addr: tmp) 
            locoRelease(th, addr, cli);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoRelease(th, iLocoAddr, cli);
    }
}

void WiThrottleServer::locoRelease(char th, LocoAddress addr, AsyncClient* cli) {
    wifiPrintln(cli, String("M")+th+"-"+addr2str(addr)+"<;>");
    //DEBUGS("loco release thr="+String(th)+"; addr "+String(addr) );

    ClientData &client = clients[cli];
    CS.releaseLocoSlot( client.slots[th][addr] );
    client.slots[th].erase(addr);
}


void WiThrottleServer::locoAction(char th, String sLocoAddr, String actionVal, AsyncClient* cli) {
    //DEBUGS("loco action thr="+String(th)+"; action="+actionVal+"; addr "+sLocoAddr );
    ClientData &client = clients[cli];

    if(sLocoAddr=="*") { 
        for(const auto& slot: client.slots[th]) 
            locoAction(th, slot.first, actionVal, cli);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoAction(th, iLocoAddr,  actionVal, cli);
    }
}


void WiThrottleServer::locoAction(char th, LocoAddress iLocoAddr, String actionVal, AsyncClient* cli) {
    ClientData &client = clients[cli];

    uint8_t slot = client.slots[th][iLocoAddr];

    WT_LOGI("loco action thr=%c; action=%s; addr %s ", th, actionVal.c_str(), String(iLocoAddr).c_str() );
    if (actionVal.startsWith("F1")) {
        int fKey = actionVal.substring(2).toInt();
        bool newVal = ! CS.getLocoFn(slot, fKey);
        CS.setLocoFn(slot, fKey, newVal );
        wifiPrintln(cli, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + (newVal?"F1":"F0")+String(fKey));
    }
    else if (actionVal.startsWith("qV")) {
        //DEBUGS("query speed for loco "+String(dccLocoAddr) );
        wifiPrintln(cli, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + "V"+String(CS.getLocoSpeed(slot)));							
    }
    else if (actionVal.startsWith("V")) {
        //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
        CS.setLocoSpeed(slot, actionVal.substring(1).toInt());
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));
    }
    else if (actionVal.startsWith("qR")) {
        //DEBUGS("query dir for loco "+String(dccLocoAddr) );
        wifiPrintln(cli, String("M")+th+"A"+addr2str(iLocoAddr)+"<;>" + "R"+String(CS.getLocoDir(slot) ));							
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

void WiThrottleServer::checkHeartbeat(AsyncClient* cli) {
    ClientData &c = clients[cli];
    if(! c.heartbeatEnabled) return;
    
    if ( (c.lastHeartbeat > 0) && (c.lastHeartbeat + c.heartbeatTimeout*1000 < millis() ) ) {
        // stop loco
        WT_LOGI("timeout exceeded: last: %d, current %d", c.lastHeartbeat/1000, millis()/1000 );
        c.lastHeartbeat = 0;
        for(const auto& throttle: c.slots)
            for(const auto& slot: throttle.second) {
                CS.setLocoSpeed(slot.second, 1); // emgr
                wifiPrintln(cli, String("M")+throttle.first+"A"+addr2str(slot.first)+"<;>V"+CS.getLocoSpeed(slot.second));
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

    for (auto p: clients) {
        wifiPrintln(p.first, String("PTA")+cStat+(namedTurnout?TURNOUT_PREF:"")+aAddr);
    }

}