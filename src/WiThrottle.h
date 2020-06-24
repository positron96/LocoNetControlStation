/**
 * Based on https://github.com/positron96/withrottle
 * 
 * Also, see JMRI sources, start at java\src\jmri\jmrit\withrottle\DeviceServer.java
 */

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <etl/map.h>
#include <etl/utility.h>

#include "CommandStation.h"
//#include "debug.h"

#undef DEBUGS
#define DEBUGS(s) Serial.println(s);

/* Maximum WiFi clients that can be connected to WiThrottle */
#define MAX_CLIENTS 3

#define LOG_WIFI 1

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED 2
#define TURNOUT_THROWN 4

class WiThrottleServer {
public:

    WiThrottleServer(uint16_t port=44444) : port(port), server(port) {}

    void begin() {

        DEBUGS("WiThrottleServer::begin");

        server.begin();

        //MDNS.begin(hostString);
        MDNS.addService("withrottle","tcp", port);
        //MDNS.setInstanceName("DCC++ Network Interface");

    }

    void loop();

private:

    const uint16_t port;

    WiFiServer server;
    WiFiClient clients[MAX_CLIENTS];

    const static int MAX_THROTTLES_PER_CLIENT = 6;
    const static int MAX_LOCOS_PER_THROTTLE = 2;

    struct ClientData {
        bool connected;
        uint16_t heartbeatTimeout = 30;
        bool heartbeatEnabled;
        uint32_t lastHeartbeat;
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

    char powerStatus = '0';

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
    void locoRelease(char th, LocoAddress addr, int iClient);

    void locoAction(char th, String sLocoAddr, String actionVal, int iClient);
    void locoAction(char th, LocoAddress addr, String actionVal, int iClient);

    void checkHeartbeat(int iClient);

    void accessoryToggle(int aAddr, char aStatus, bool namedTurnout);
};