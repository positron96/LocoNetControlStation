#include "WiThrottle.h"

#include "DCC.h"
#define FILE_LOG_LEVEL  LEVEL_DEBUG
#include "log.h"

#include <etl/vector.h>
#include <etl/to_arithmetic.h>

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_UNKNOWN '1'
#define TURNOUT_CLOSED '2'
#define TURNOUT_THROWN '4'

#define DELIM "<;>"


String addr2str(const LocoAddress & a) {
    return (a.isShort() ? 'S' : 'L') + String(a.addr());
}

LocoAddress str2addr(etl::string_view addr) {
    uint16_t iLocoAddr = etl::to_arithmetic<uint16_t>(addr.substr(1)).value();
    if (addr[0] == 'S') return LocoAddress::shortAddr((uint8_t)iLocoAddr);
    if (addr[0] == 'L') return LocoAddress::longAddr(iLocoAddr);
    return LocoAddress();
}

inline int invert(int value) {
    return (value == 0) ? 1 : 0;
};

char turnoutState2Chr(TurnoutState s) {
    switch(s) {
        case TurnoutState::THROWN : return TURNOUT_THROWN;
        case TurnoutState::CLOSED: return TURNOUT_CLOSED;
        case TurnoutState::UNKNOWN : return TURNOUT_UNKNOWN;
        default:
            LOGW("Unknown turnout state: %d", (int)s);
            return TURNOUT_UNKNOWN;
    }
}

int speedMode2int(SpeedMode sm) {
    switch(sm) {
        case SpeedMode::S128: return 1;
        case SpeedMode::S14: return 8;
        case SpeedMode::S28: return 2;
        default: return 0;
    }
}

int speed2int(LocoSpeed s) {
    if(s==SPEED_EMGR) return -1;
    if(s==SPEED_IDLE) return 0;
    return s.get128()-1; // 2..127 -> 1..126
}

LocoSpeed int2speed(int s) {
    if(s<0) return SPEED_EMGR;
    if(s==0) return SPEED_IDLE;
    return LocoSpeed::from128((uint8_t)(s+1)); // 1..126 -> 2..127
}


void WiThrottleServer::onNewClient(AsyncClient* cli) {
    if(clients.full()) {
        LOGI("onConnect: Not accepting client: %s, no space left", cli->remoteIP().toString().c_str() );
        cli->close();
        return;
    }
    // here keepalives are built into protocol, no need for TCP keepalives
    //cli->setKeepAlive(10000, 2);
    bool initNewClient = true;
    for(auto it = clients.cbegin(); it!=clients.cend(); ) {
        const auto &c = it->first;
        if(c->getRemoteAddress() == cli->getRemoteAddress()) {
            LOGI("onConnect: There is a client(%X) with this addr already", (intptr_t)c);
            initNewClient = false;
            clients[cli] = it->second;  // copy ClientData
            ClientData& cc = clients[cli];
            cc.rxpos = 0;
            cc.cli = cli;
            cc.updateHeartbeat();
            it = clients.erase(it); // onDisconnect(c) won't find the client and won't cleanup ClientData
            c->close();
            break;
        } else it++;
    }
    if(initNewClient) clientStart(cli);

    cli->setAckTimeout(HEARTBEAT_INTL*1000*3);
    LOGI("onConnect: New client(%X): %s, now have %d clients",
            (intptr_t)cli, cli->remoteIP().toString().c_str(), clients.size() );

    cli->onDisconnect([this](void*, AsyncClient* cli_) {
        LOGI("onDisconnect: Client(%X) disconnected", (intptr_t)cli_ );
        auto it = clients.find(cli_);
        if(it==clients.end() ) {
            LOGW("Not clearing up client(%X), it is not registered", (intptr_t)cli_);
            return;
        }
        ClientData &cc = it->second;
        clientStop(cc);
    });

    cli->onData([this](void*, AsyncClient* cli_, void *data, size_t len) {
        auto it = clients.find(cli_);
        if(it==clients.end() ) {
            // after sending Q(quit), the client might send other commands, ignore them
            LOGW("Got some data from client(%X), but it is not recognised", (intptr_t)cli_);
            return;
        }
        ClientData &cc = it->second;
        if(cc.heartbeatEnabled) {
            cc.updateHeartbeat();
        }
        for(size_t i=0; i<len; i++) {
            char c = ((char*)data)[i];
            if(c=='\n') {
                cc.rx[cc.rxpos] = 0;
                processCmd(cc);
                cc.rxpos = 0;
            } else {
                if (cc.rxpos<ClientData::RX_SIZE) {
                    cc.rx[cc.rxpos] = c;
                    cc.rxpos++;
                } else {
                    LOGW("Input buffer overflow, dropping char '%c'", c);
                }
            }
        }
    });

    cli->onError([this](void*, AsyncClient* cli_, int8_t err) {
        LOGW("onError(%X): %d", (intptr_t)cli_, err);
    });
    cli->onTimeout([this](void*, AsyncClient* cli_, uint32_t time) {
        LOGI("onTimeout(%X): %d", (intptr_t)cli_, time);
        cli_->close();
    });
}

WiThrottleServer::WiThrottleServer(uint16_t port, const char* name) :
        port{port}, name{name}, server(port)
{
    server.onClient([this](void*, AsyncClient* cli ){this->onNewClient(cli);}, nullptr);
}

void WiThrottleServer::begin() {

    LOGI("WiThrottleServer::begin");

    server.begin();

    MDNS.addService("withrottle","tcp", port);

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
    etl::string_view dataStr{cc.rx};
    LOGD("RX '%s'(len %d)", cc.rx, dataStr.length());
    switch(dataStr[0]) {
    case '*': {
        if(dataStr.length()>1) {
            switch(dataStr[1] ) {
                case '+' : cc.heartbeatEnabled = true; cc.updateHeartbeat(); break;
                case '-' : cc.heartbeatEnabled = false; break;
                default: LOGW("Unknown * cmd: %s", dataStr.data());
            }
        }
        LOGI("Heartbeat is %s", cc.heartbeatEnabled?"ON":"OFF");
        break;
    }
    case 'P': {
        if (dataStr.starts_with("PPA") ) {
            turnPower(dataStr[3]);
        } else if (dataStr.starts_with("PTA")) {
            char action = dataStr[3];
            unsigned aAddr;
            bool named;
            if(dataStr.substr(4,2)==TURNOUT_PREF) {
                // named turnout
                aAddr = etl::to_arithmetic<unsigned>(dataStr.substr(6)).value();
                named = true;
            } else {
                aAddr = etl::to_arithmetic<unsigned>(dataStr.substr(4)).value();
                named = false;
            }
            accessoryToggle(aAddr, action, named, cc);
        }
        break;
    }
    case 'N': { // device name
        auto remoteName = dataStr.substr(1);
        LOGI("Device name: '%s'",  remoteName.data());
        break;
    }
    case 'H': {
        if(dataStr[1]=='U') {
            auto hwId = dataStr.substr(2);
            LOGI("Hardware ID: '%s'", hwId.data() );
            cc.hwId = String(hwId.data());
            // kill other clients with same ID (assume it's reconnect from same device)
            const auto cli = cc.cli;
            for(auto it = clients.cbegin(); it!=clients.cend(); ) {
                const auto &c = it->first;
                if(c == cli) {it++; continue;}
                if(it->second.hwId == hwId.data()) {
                    LOGW(" There is a client(%X) with this ID already. Should not happen!", (intptr_t)c);
                    cc = it->second;  // copy ClientData
                    cc.rxpos = 0;
                    cc.cli = cli;
                    cc.updateHeartbeat();
                    it = clients.erase(it); // onDisconnect(c) won't find the client and won't cleanup ClientData
                    c->close();
                    break;
                } else it++;
            }
        }
        break;
    }
    case 'M': {
        char th = dataStr[1];
        char action = dataStr[2];
        auto actionData = dataStr.substr(3);
        size_t delimiter = actionData.find(DELIM);
        if(delimiter == -1) {
            LOGW("Malformed M command");
            break;
        }
        auto actionKey = actionData.substr(0, delimiter);
        auto actionVal = actionData.substr(delimiter+3);
        if (action == '+') {
            cc.locoAdd(th, actionKey);
        } else if (action == '-') {
            cc.locosRelease(th, actionKey);
        } else if (action == 'A') {
            cc.locosAction(th, actionKey, actionVal);
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

void WiThrottleServer::clientStart(AsyncClient *cli) {
    ClientData cc;
    LOGI( "New client " );

    wifiPrintln(cli, "VN2.0");
    if(name!=nullptr) wifiPrintln(cli, String("Ht")+name );
    wifiPrintln(cli, "RL0"); // roster list is size 0
    notifyPowerStatus(cli);
    wifiPrintln(cli, "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{"+String(TURNOUT_CLOSED)+"]\\[Thrown}|{"+String(TURNOUT_THROWN) );
    wifiPrint(cli, "PTL");
    for(const auto &t: CS.getTurnouts() ) {
    //for (int t = 0 ; t<CS.getTurnoutCount(); t++) {
        //const CommandStation::TurnoutData &tt = CS.getTurnout(t);
        const CommandStation::TurnoutData &tt = t.second;
        wifiPrint(cli, String("]\\[")+TURNOUT_PREF+tt.addr11+"}|{"+tt.userTag+"}|{"+turnoutState2Chr(tt.tStatus) );
    }
    // wifiPrintln(cli, "PW8888"); // Web port
    wifiPrintln(cli, "");
    wifiPrintln(cli, "*"+String(HEARTBEAT_INTL));

    cc.rxpos = 0;
    cc.cli = cli;
    cc.heartbeatEnabled = false;
    cc.updateHeartbeat();

    clients[cli] = cc;
}

void WiThrottleServer::clientStop(ClientData &client) {
    LOGI("Clearing client data");

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

void WiThrottleServer::ClientData::locoAdd(char th, etl::string_view sLocoAddr) {
    LocoAddress addr = str2addr(sLocoAddr);
    AddrToSlotMap &slotmap = slots[th];
    if(slotmap.available()==0 && slotmap.find(addr)==slotmap.end() ) {
        sendMessage("No space for new loco", true);
        LOGI("locoAdd(thr=%c, addr=%d) no space for new loco\n", th, addr.addr() );
        return;
    }

    uint8_t slot = CS.findOrAllocateLocoSlot(addr); // TODO: check result
    slotmap[addr] = slot;

    sendThrottleMsg(th,'+',addr, "");
    for (uint8_t fKey=0; fKey<CommandStation::N_FUNCTIONS; fKey++) {
        sendThrottleMsg(th,'A',addr, String("F")+(CS.getLocoFn(slot, fKey)?'1':'0')+String(fKey) );
    }
    sendThrottleMsg(th,'A',addr, String("V")+speed2int(CS.getLocoSpeed(slot)) );
    sendThrottleMsg(th,'A',addr, String("R")+CS.getLocoDir(slot) );
    sendThrottleMsg(th,'A',addr, String("s")+speedMode2int(CS.getLocoSpeedMode(slot)) );

    //DEBUGS("loco add thr="+String(th)+"; addr"+String(sLocoAddr) );

    CS.setLocoSlotRefresh(slot, true);
}

void WiThrottleServer::ClientData::locosRelease(char th, etl::string_view sLocoAddr) {
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
    String sAddr = addr;
    LOGI("loco release thr=%c; addr=%s", th, sAddr.c_str() );
    sendThrottleMsg(th,'-',addr, "" );

    auto &throttle = slots[th];
    auto it = throttle.find(addr);
    if(it == throttle.end()) {
        LOGW("No loco '%s' in this throttle", sAddr.c_str());
        return;
    }
    uint8_t slot = it->second;
    CS.releaseLocoSlot(slot);
    slots[th].erase(addr);
}


void WiThrottleServer::ClientData::locosAction(char th, etl::string_view sLocoAddr, etl::string_view actionVal) {
    if(sLocoAddr=="*") {
        for(const auto& slot: slots[th])
            locoAction(th, slot.first, actionVal);
    } else {
        LocoAddress iLocoAddr = str2addr(sLocoAddr);
        locoAction(th, iLocoAddr,  actionVal);
    }
}

void WiThrottleServer::ClientData::locoAction(char th, LocoAddress iLocoAddr, etl::string_view actionVal) {
    auto &throttle = slots[th];
    auto it = throttle.find(iLocoAddr);
    if(it == throttle.end()) {
        LOGW("No loco '%s' in this throttle", String(iLocoAddr).c_str());
        return;
    }
    uint8_t slot = it->second;

    LOGI("loco action thr=%c; addr=%s; action=%s", th, String(iLocoAddr).c_str(), actionVal.data() );
    switch(actionVal[0] ) {
        case 'F': {
            uint8_t fKey = etl::to_arithmetic<uint8_t>(actionVal.substr(2)).value();
            bool val = CS.getLocoFn(slot, fKey);
            if(actionVal[1]=='1'){
                val = !val;
                CS.setLocoFn(slot, fKey, val);
            }
            sendThrottleMsg(th, 'A', iLocoAddr, (val?"F1":"F0")+String(fKey) );
            break;
        }
        case 'q':
            switch(actionVal[1]) {
                case 'V':
                    //DEBUGS("query speed for loco "+String(dccLocoAddr) );
                    sendThrottleMsg(th,'A', iLocoAddr, String("V")+speed2int(CS.getLocoSpeed(slot)) );
                    break;
                case 'R':
                    //DEBUGS("query dir for loco "+String(dccLocoAddr) );
                    sendThrottleMsg(th,'A', iLocoAddr, String("R")+String(CS.getLocoDir(slot) ));
                    break;
            }
            CS.kickSlot(slot);
            break;
        case 'V':
            {
                //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
                int s = etl::to_arithmetic<int>(actionVal.substr(1)).value();
                CS.setLocoSpeed(slot, int2speed(s) );
            }
            break;
        case 'R':
            //DEBUGS("Sending dir to addr "+String(dccLocoAddr) );
            CS.setLocoDir(slot, etl::to_arithmetic<uint8_t>(actionVal.substr(1)).value() );
            break;
        case 'X': // EMGR stop
            CS.setLocoSpeed(slot, SPEED_EMGR);
            break;
        case 'I': // idle
            CS.setLocoSpeed(slot, SPEED_IDLE);
            break;
        case 'Q': // quit
            CS.setLocoSpeed(slot, SPEED_IDLE);
            cli->close();
            break;
    }

}

void WiThrottleServer::ClientData::checkHeartbeat() {
    if(! heartbeatEnabled) return;

    if (wdt.timedOut() && heartbeat==Heartbeat::Alive) {
        // stop loco
        LOGI("timeout exceeded: current %ds, last updated at %ds",  millis()/1000, wdt.getLastUpdate()/1000 );
        heartbeat = Heartbeat::SoftTimeout;
        for(const auto& throttle: slots)
            for(const auto& slot: throttle.second) {
                CS.setLocoSpeed(slot.second, SPEED_EMGR);
                sendThrottleMsg(throttle.first, 'A', slot.first, "V-1" );
            }
        sendMessage("Timeout exceeded, locos stopped");
    }
    if ((wdt.timedOut2() && heartbeat==Heartbeat::SoftTimeout)) {
        LOGI("timeout exceeded twice: closing connection" );
        heartbeat = Heartbeat::HardTimeout;
        cli->close();
    }

}

void WiThrottleServer::ClientData::sendMessage(String msg, bool alert) {
    wifiPrintln(cli, String("H")+(alert?'M':'m')+msg);
}

void WiThrottleServer::ClientData::sendThrottleMsg(char th, char cmd, LocoAddress iLocoAddr, String resp) {
    char tt[4] = "MTA";
    tt[1] = th;
    tt[2] = cmd;
    wifiPrintln(cli, String(tt)+addr2str(iLocoAddr)+DELIM + resp);
}

void WiThrottleServer::accessoryToggle(unsigned aAddr, char action, bool isNamed, ClientData &cc) {

    LOGI("Turnout addr=%d(named: %c) action=%c", aAddr, isNamed?'Y':'N', action );

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