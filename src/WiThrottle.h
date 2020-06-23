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

#define LOG_WIFI 0

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED 2
#define TURNOUT_THROWN 4

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
            ClientData& cc = clientData[iClient];
            if (!cli) {
                if(cc.connected) throttleStop(iClient);
                cli = server.available();
                if(cli) throttleStart(iClient);
            } 
            if (cli) { // connected client
                while(cli.available()>0) { 
                    String dataStr = cli.readStringUntil('\n'); 
                    if (LOG_WIFI) DEBUGS("WF>> "+dataStr);

                    if (dataStr.startsWith("*")) {
                        if(dataStr.length()==1) 
                            cc.lastHeartbeat = millis();
                        else {
                            switch(dataStr.charAt(1) ) {
                                case '+' : cc.heartbeatEnabled = true; break;
                                case '-' : cc.heartbeatEnabled = false; break;
                            } 
                        }                       
                    } else if (dataStr.startsWith("PPA") ) {
                        turnPower(dataStr.charAt(3) );
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
                    } else if (dataStr.startsWith("N") ) {
                        wifiPrintln(iClient, "*" + String(cc.heartbeatTimeout));
                    } else if (dataStr.startsWith("M") ) {
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
                        clientData[iClient].lastHeartbeat = millis();
                        
                    }
                }

                if (cc.heartbeatEnabled) {
                    checkHeartbeat(iClient);
                }
            }
            
        }
    }

private:

    const uint16_t port;

    WiFiServer server;
    WiFiClient clients[MAX_CLIENTS];

    const static int MAX_THROTTLES_PER_CLIENT = 6;
    const static int MAX_LOCOS_PER_THROTTLE = 2;

    struct ClientData {
        bool connected;
        uint16_t heartbeatTimeout = 10;
        bool heartbeatEnabled;
        uint16_t lastHeartbeat;
        // each client can have up to 6 multi throttles, each MT can have multiple locos (and slots)
        etl::map< char, etl::map<LocoAddress, uint8_t, MAX_LOCOS_PER_THROTTLE>, MAX_THROTTLES_PER_CLIENT> slots;
        uint8_t slot(char thr, LocoAddress addr) { 
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
            if (clientData[p].connected) {
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

    void throttleStart(int iClient) ;

    void throttleStop(int iClient);

    void locoAdd(char th, String sLocoAddr, int iClient);

    void locoRelease(char th, String sLocoAddr, int iClient);

    void locoAction(char th, String sLocoAddr, String actionVal, int iClient);

    void checkHeartbeat(int iClient);

    void accessoryToggle(int aAddr, char aStatus, bool namedTurnout);
};