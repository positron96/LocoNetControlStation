/*
 * Based on https://github.com/positron96/withrottle
 */

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <etl/map.h>
#include <etl/utility.h>

#include "CommandStation.h"
#include "debug.h"

/* Maximum WiFi clients that can be connected to WiThrottle */
#define MAX_CLIENTS 3

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED 2
#define TURNOUT_THROWN 4

#define LOG_WIFI 0

inline int invert(int value) {
    return value == 0 ? 1 : 0;
};

class WiThrottleServer {
public:

    WiThrottleServer(uint16_t port=44444) : port(port), server(port) {}

    void begin() {

        DEBUGS("Connected");

        server.begin();

        //MDNS.begin(hostString);
        MDNS.addService("withrottle","tcp", port);
        //MDNS.setInstanceName("DCC++ Network Interface");

    }


    void loop() {
        
        for (int iClient=0; iClient<MAX_CLIENTS; iClient++) {
            WiFiClient& cli = clients[iClient];
            if (!cli) {
                if(alreadyConnected[iClient])  throttleStop(iClient);
                cli = server.available();
            } else {
                if (!alreadyConnected[iClient]) {
                    throttleStart(iClient);
                }
            }
            if (cli.available()) {
                //bool changeSpeed[] = { };
                while(cli.available()>0) { 
                    String dataStr = cli.readStringUntil('\n'); 
                    if (LOG_WIFI) DEBUGS("WF>> "+dataStr);
                    if (dataStr.startsWith("*+")) {
                        heartbeatEnable[iClient] = true;
                    } else if (dataStr.startsWith("PPA")) {
                        turnPower(dataStr.charAt(3));
                        //notifyPowerChange();
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
                    } else if (dataStr.startsWith("N") 
                            || dataStr.startsWith("*")) {
                        wifiPrintln(iClient, "*" + String(heartbeatTimeout));
                    } else if (dataStr.startsWith("MT") 
                            || dataStr.startsWith("MS") 
                            || dataStr.startsWith("M0") 
                            || dataStr.startsWith("M1") ) {
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
                        clientData[iClient].heartbeat = millis();
                        
                    }
                }
                if (clientData[iClient].heartbeatEnabled) {
                    checkHeartbeat(iClient);
                }
            }
        }
    }

private:

    const uint16_t port;

    /* The interval of check connections between ESP & WiThrottle app */
    const unsigned int heartbeatTimeout = 10;
    bool heartbeatEnable[MAX_CLIENTS];
    unsigned long heartbeat[MAX_CLIENTS*2];

    //String locoAddesses[MAX_CLIENTS*2] = {""};
    //int locoStates[MAX_CLIENTS*2][31];

    boolean alreadyConnected[MAX_CLIENTS];

    /* Define WiThrottle Server */
    WiFiServer server;
    WiFiClient clients[MAX_CLIENTS];
    const static int MAX_THROTTLES_PER_CLIENT = 6;
    const static int MAX_LOCOS_PER_THROTTLE = 2;

    struct ClientData {
        bool connected;
        uint16_t heartbeatTimeout = 10;
        bool heartbeatEnabled;
        uint16_t heartbeat;
        // each client can have up to 6 multi throttles, each MT can have multiple locos (and slots)
        //std::map<char, std::vector<uint16_t> > slots;
        //uint16_t slots[MAX_THROTTLES_PER_CLIENT*MAX_LOCOS_PER_THROTTLE];
        etl::map< char, etl::map<uint16_t, uint8_t, MAX_LOCOS_PER_THROTTLE>, MAX_THROTTLES_PER_CLIENT> slots;
        uint16_t slot(char thr, uint16_t addr) { 
            auto it = slots.find(thr);
            if(it == slots.end()) return 0;
            auto iit = it->second.find(addr);
            if(iit == it->second.end() ) return 0;
            return iit->second;
        }

    };

    ClientData clientData[MAX_CLIENTS];

    char powerStatus;

    void notifyPowerStatus() {
        bool v = CS.getPowerState();
        powerStatus = v ? '1' : '0';
        for (int p=0; p<MAX_CLIENTS; p++) {
            if (alreadyConnected[p]) {
                clients[p].println("PPA"+powerStatus);
            }
        }
    }

    void turnPower(char v) {
        CS.turnPower(v=='1');
    }


    void wifiPrintln(int iClient, String v) {
        clients[iClient].println(v);
        if (LOG_WIFI) DEBUGS("WF<< "+v);
    }
    void wifiPrint(int iClient, String v) {
        clients[iClient].print(v);
        if (LOG_WIFI) DEBUGS("WF<< "+v);
    }

    void throttleStart(int iClient) {
        clients[iClient].flush();
        clients[iClient].setTimeout(500);
        DEBUGS("New client");

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
        wifiPrintln(iClient, "*"+String(heartbeatTimeout));
        alreadyConnected[iClient] = true;
    }

    void throttleStop(int iClient) {
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
        
        
        //locoStates[0+iClient*2][29] = 0;	 heartbeat[0+iClient*2] = 0;
        //locoStates[1+iClient*2][29] = 0;	 heartbeat[1+iClient*2] = 0;
    }

    void locoAdd(char th, String locoAddr, int iClient) {
        //locoAddesses[iThrottle] = locoAddr;
        uint16_t iLocoAddr = locoAddr.substring(1).toInt();
        wifiPrintln(iClient, String("M")+th+"+"+locoAddr+"<;>");
        for (int fKey=0; fKey<29; fKey++) {
            //locoStates[iThrottle][fKey] =0;
            wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>F0"+String(fKey));
        }
        //locoStates[iThrottle][29] =0;
        //locoStates[iThrottle][30] =1;
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>V0");
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>R1");
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>s1");
        DEBUGS("loco add thr="+String(th)+"; addr"+String(locoAddr) );
        uint16_t slot = CS.locateLocoSlot(iLocoAddr);
        clientData[iClient].slots[th][iLocoAddr] = slot;

    }

    void locoRelease(char th, String locoAddr, int iClient) {
        //String locoAddress = locoAddesses[iThrottle].substring(1);
        //heartbeat[iThrottle] = 0;
        //locoAddesses[iThrottle] = "";
        wifiPrintln(iClient, String("M")+th+"-"+locoAddr+"<;>");
        DEBUGS("loco release thr="+String(th)+"; addr"+String(locoAddr) );
        // stop now
        //sendDCCppCmd(String("t ")+String(iThrottle+1)+" "+locoAddress+" 0 "+String(locoStates[iThrottle][30]));
        ClientData &client = clientData[iClient];
        uint16_t iLocoAddr = locoAddr.substring(1).toInt();;
        CS.releaseLocoSlot( client.slots[th][iLocoAddr] );
        client.slots[th].erase(iLocoAddr);
    }

    void locoAction(char th, String locoAddr, String actionVal, int iClient) {
        ClientData &client = clientData[iClient];

        uint16_t iLocoAddr;
        if (locoAddr == "*") {
            //ocoAddr = locoAddesses[iThrottle];
            iLocoAddr = client.slots[th].begin()->first;
        } else iLocoAddr = locoAddr.substring(1).toInt();

        uint8_t slot = client.slots[th][iLocoAddr];

        //String dccLocoAddr = locoAddr.substring(1).toInt();
        DEBUGS("loco action thr="+String(th)+"; action="+actionVal+"; DCC "+String(iLocoAddr) );
        //int *locoState = locoStates[iThrottle];
        if (actionVal.startsWith("F1")) {
            int fKey = actionVal.substring(2).toInt();
            //locoState[fKey] = invert(locoState[fKey]);
            bool newVal = ! CS.getLocoFn(slot, fKey);
            CS.setLocoFn(slot, fKey, newVal );
            wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>" + (newVal?"F1":"F0")+String(fKey));
        }
        else if (actionVal.startsWith("qV")) {
            //DEBUGS("query speed for loco "+String(dccLocoAddr) );
            wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>" + "V"+String(CS.getLocoSpeed(slot)));							
        }
        else if (actionVal.startsWith("V")) {
            //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
            //locoState[29] = actionVal.substring(1).toInt();
            CS.setLocoSpeed(slot, actionVal.substring(1).toInt());
            //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));
            //response = loadResponse();
        }
        else if (actionVal.startsWith("qR")) {
            //DEBUGS("query dir for loco "+String(dccLocoAddr) );
            wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>" + "R"+String(CS.getLocoDir(slot) ));							
        }
        else if (actionVal.startsWith("R")) {
            //DEBUGS("Sending dir to addr "+String(dccLocoAddr) );
            //locoState[30] = actionVal.substring(1).toInt();
            CS.setLocoDir(slot, actionVal.substring(1).toInt() );
            //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" "+String(locoState[29])+"	"+String(locoState[30]));
            //response = loadResponse();
        }
        else if (actionVal.startsWith("X")) { // EMGR stop
            CS.setLocoSpeed(slot, 1);
            //locoState[29] = 0;
            //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
            //response = loadResponse();
        }
        else if (actionVal.startsWith("I")) { // idle
            //locoState[29] = 0;
            // sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
            //response = loadResponse();
            CS.setLocoSpeed(slot, 0);
        }
        else if (actionVal.startsWith("Q")) { // quit
            //locoState[29] = 0;
            //sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
            //response = loadResponse();
            CS.setLocoSpeed(slot, 0);
        }
    }

    void checkHeartbeat(int iClient) {
        ClientData &c = clientData[iClient];
        
        if (c.heartbeat > 0 && c.heartbeat + c.heartbeatTimeout * 1000 < millis()) {
            // stop loco
            //TODO: is it sent to track?
            //locoStates[ii][29] = 0;
            c.heartbeat = 0;
            for(const auto& throttle: c.slots)
                for(const auto& slot: throttle.second) {
                    CS.setLocoSpeed(slot.second, 1); // emgr
                    wifiPrintln(iClient, String("M")+throttle.first+"A"+slot.first+"<;>V0");
                }
            
        }
    }

    void accessoryToggle(int aAddr, char aStatus, bool namedTurnout) {
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
};