#include "WiThrottle.h"

#include <etl/vector.h>

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_UNKNOWN '1'
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
            WT_LOGI("onConnect: Not accepting client: %s, no space left", cli->remoteIP().toString().c_str() );
            cli->close();
            return;
        }
        // here keepalives are built into protocol, no need for TCP keepalives
        //cli->setKeepAlive(10000, 2); 
        clientStart(cli); 
        cli->setAckTimeout(HEARTBEAT_INTL*1000*3); 
        WT_LOGI("onConnect: New client(%X): %s, have %d clients", 
                (intptr_t)cli, cli->remoteIP().toString().c_str(), clients.size() );

        cli->onDisconnect([this](void*, AsyncClient* cli) {
            WT_LOGI("onDisconnect: Client(%X) disconnected", (intptr_t)cli );
            if(clients.find(cli)==clients.end() ) {
                WT_LOGI("Not clearing up client(%X), it is not registered", (intptr_t)cli);
                return;
            }
            clientStop(clients[cli]);
        });

        cli->onData( [this](void*, AsyncClient* cli, void *data, size_t len) {
            if(clients.find(cli)==clients.end() ) { 
                // after sending Q(quit), the client might send other commands
                WT_LOGI("Got some data from client(%X), but it is not recognised", (intptr_t)cli);
                return;
            }
            ClientData &cc = clients[cli];
            if(cc.heartbeatEnabled) { 
                cc.updateHeartbeat();
            }
            for(size_t i=0; i<len; i++) {
                char c = ((char*)data)[i];
                if(c=='\n') {
                    cc.cmdline[cc.cmdpos] = 0;
                    processCmd(cc);
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
            cc.checkHeartbeat();
        }
    }
}

void WiThrottleServer::processCmd(ClientData & cc) {
    String dataStr = cc.cmdline;
    WT_LOGI("Read new cmd '%s'(%db)", cc.cmdline, dataStr.length());
    switch(dataStr.charAt(0)) {
    case '*': {
        if(dataStr.length()>1) {
            switch(dataStr.charAt(1) ) {
                case '+' : cc.heartbeatEnabled = true; break;
                case '-' : cc.heartbeatEnabled = false; break;
            } 
        }
        WT_LOGI("Heartbeat is %s", cc.heartbeatEnabled?"ON":"OFF");
        break;
    }             
    case 'P': {
        if (dataStr.startsWith("PPA") ) {
            turnPower(dataStr.charAt(3) );
        } else if (dataStr.startsWith("PTA")) {
            char action = dataStr.charAt(3);
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
            accessoryToggle(aAddr, action, named, cc);
        } else {}
        break;
    }
    case 'N': { // device name
        WT_LOGI("Device ID: '%s'", cc.cmdline+1 );
        wifiPrintln(cc.cli, "*" + String(HEARTBEAT_INTL));
        break;
    }
    case 'H': {
        WT_LOGI("Hardware ID: '%s'", cc.cmdline+1 );
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
            cc.locoAdd(th, actionKey);
        } else if (action == '-') {
            cc.locoRelease(th, actionKey);
        } else if (action == 'A') {
            cc.locoAction(th, actionKey, actionVal);
        }
        break;
    }
    case 'Q':
        // quit
        //clientStop(cc);
        cc.cli->close();
        break;
    default:
        break;
    }
}

char turnoutState2Chr(TurnoutState s) {
    switch(s) {
        case TurnoutState::THROWN : return TURNOUT_THROWN;
        case TurnoutState::CLOSED: return TURNOUT_CLOSED;
        case TurnoutState::UNKNOWN : return TURNOUT_UNKNOWN;
        default: 
            CS_DEBUGF("Unknown turnout state: %d", (int)s);
            return TURNOUT_UNKNOWN;
    }
}

void WiThrottleServer::clientStart(AsyncClient *cli) {
    
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
        wifiPrint(cli, String("]\\[")+TURNOUT_PREF+tt.addr11+"}|{"+tt.userTag+"}|{"+turnoutState2Chr(tt.tStatus) );
    }
    wifiPrintln(cli, "");
    wifiPrintln(cli, "*"+String(HEARTBEAT_INTL));

    cc.cmdpos = 0;
    cc.cli = cli;
    cc.heartbeatEnabled = false;
    cc.updateHeartbeat();

    clients[cli] = cc;
}

void WiThrottleServer::clientStop(ClientData &client) {
    WT_LOGI("Client stopping");
    
    for(const auto& thrSlots: client.slots) {
        for(const auto& slots: thrSlots.second) {
            CS.releaseLocoSlot(slots.second);
        }
    }
    client.slots.clear();
    client.heartbeatEnabled = false;
    AsyncClient *cli = client.cli;
    // this method should be called only from onDisconnected, so this should be false
    if(cli->connected() ) cli->stop(); 
    clients.erase(cli);
}

void WiThrottleServer::ClientData::locoAdd(char th, String sLocoAddr) {
    LocoAddress addr = str2addr(sLocoAddr);
    AddrToSlotMap &slotmap = slots[th];
    if(slotmap.available()==0 && slotmap.find(addr)==slotmap.end() ) {
        WT_LOGI("locoAdd(thr=%c, addr=%d) no space for new loco\n", th, addr.addr() );
        return;
    }

    uint8_t slot = CS.findOrAllocateLocoSlot(addr); // TODO: check result
    slotmap[addr] = slot;

    wifiPrintln(cli, String("M")+th+"+"+sLocoAddr+"<;>");
    for (int fKey=0; fKey<CommandStation::N_FUNCTIONS; fKey++) {
        wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>F"+(CS.getLocoFn(slot, fKey)?'1':'0')+String(fKey));
    }
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>V0");
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>R1");
    wifiPrintln(cli, String("M")+th+"A"+sLocoAddr+"<;>s1"); // TODO: this is speed steps 128 -> 1, 28->2 14->8

    //DEBUGS("loco add thr="+String(th)+"; addr"+String(sLocoAddr) );

    CS.setLocoSlotRefresh(slot, true);
}

void WiThrottleServer::ClientData::locoRelease(char th, String sLocoAddr) {
    //wifiPrintln(iClient, String("M")+th+"-"+sLocoAddr+"<;>");
    WT_LOGI("loco release thr=%c; addr=%s", th, String(sLocoAddr).c_str() );

    if(sLocoAddr=="*") { 
        etl::vector<LocoAddress, MAX_LOCOS_PER_THROTTLE> tmp;
        for(const auto& slot: slots[th]) {
            tmp.push_back(slot.first);
        }
        for(const auto& addr: tmp) {
            locoRelease(th, addr);
        }
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoRelease(th, iLocoAddr);
    }
}

void WiThrottleServer::ClientData::locoRelease(char th, LocoAddress addr) {
    wifiPrintln(cli, String("M")+th+"-"+addr2str(addr)+"<;>");
    //DEBUGS("loco release thr="+String(th)+"; addr "+String(addr) );

    CS.releaseLocoSlot( slots[th][addr] );
    slots[th].erase(addr);
}


void WiThrottleServer::ClientData::locoAction(char th, String sLocoAddr, String actionVal) {
    //DEBUGS("loco action thr="+String(th)+"; action="+actionVal+"; addr "+sLocoAddr );

    if(sLocoAddr=="*") { 
        for(const auto& slot: slots[th]) 
            locoAction(th, slot.first, actionVal);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoAction(th, iLocoAddr,  actionVal);
    }
}


void WiThrottleServer::ClientData::locoAction(char th, LocoAddress iLocoAddr, String actionVal) {

    uint8_t slot = slots[th][iLocoAddr];

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
        CS.setLocoSpeed(slot, SPEED_EMGR);
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
    }
    else if (actionVal.startsWith("I")) { // idle
        // sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
        CS.setLocoSpeed(slot, SPEED_IDLE);
    }
    else if (actionVal.startsWith("Q")) { // quit
        //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
        CS.setLocoSpeed(slot, SPEED_IDLE);
        cli->close();
    }
}

void WiThrottleServer::ClientData::checkHeartbeat() {
    if(! heartbeatEnabled) return;
    
    //if ( lastHeartbeat > 0) {
        if (wdt.timedOut() && heartbeatsLost==0) {
            // stop loco
            WT_LOGI("timeout exceeded: last: %ds, current %ds", wdt.getLastUpdate()/1000, millis()/1000 );
            heartbeatsLost++;
            for(const auto& throttle: slots)
                for(const auto& slot: throttle.second) {
                    CS.setLocoSpeed(slot.second, 1); // emgr
                    wifiPrintln(cli, String("M")+throttle.first+"A"+addr2str(slot.first)+"<;>V"+CS.getLocoSpeed(slot.second));
                }
        }
        if (wdt.timedOut2() && heartbeatsLost==1) {
            WT_LOGI("timeout exceeded twice: closing connection" );
            heartbeatsLost++;
            cli->close();
        }
        
    //}
}

void WiThrottleServer::ClientData::sendMessage(String msg, bool alert) {
    wifiPrintln(cli, String("H")+(alert?'M':'m')+msg);
}

void WiThrottleServer::accessoryToggle(int aAddr, char action, bool isNamed, ClientData &cc) {

    WT_LOGI("Turnout addr=%d(named: %c) action=%c", aAddr, isNamed?'Y':'N', action );

    TurnoutAction a;
    switch(action) {
        case 'T':  a = TurnoutAction::THROWN;  break;
        case 'C':  a = TurnoutAction::CLOSED;  break;
        case '2':  a = TurnoutAction::TOGGLE;  break;
        default: 
            cc.sendMessage("Unknown turnout command!", true);
            return;
    }
    TurnoutState newStat = CS.turnoutAction(aAddr, isNamed, a);

    if(newStat==TurnoutState::UNKNOWN) cc.sendMessage("Could not change turnout!", true); 
    char cStat = turnoutState2Chr(newStat); 
    
    for (auto p: clients) {
        wifiPrintln(p.first, String("PTA")+cStat+(isNamed?TURNOUT_PREF:"")+aAddr);
    }

}