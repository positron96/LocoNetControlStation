/**
 * Based on https://github.com/positron96/withrottle
 * 
 * Also, see JMRI sources, start at java\src\jmri\jmrit\withrottle\DeviceServer.java
 */

#pragma once

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <etl/map.h>
#include <etl/utility.h>
#include <AsyncTCP.h>

#include "CommandStation.h"


#define WT_DEBUG_

#ifdef WT_DEBUG
#define WT_LOGI(format, ...)  log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#else
#define WT_LOGI(...)
#endif


class WiThrottleServer {
public:

    WiThrottleServer(uint16_t port=44444);

    void begin();

    void end() {
        server.end();
    }

    
    void notifyPowerStatus(AsyncClient *c=nullptr) {
        bool v = CS.getPowerState();
        powerStatus = v ? '1' : '0';
        if(c==nullptr) {
            for (auto p: clients) {
                wifiPrintln(p.first, String("PPA")+powerStatus);
            }
        } else wifiPrintln(c, String("PPA")+powerStatus);
    }

    void loop();

private:

    const uint16_t port;

    const static int MAX_CLIENTS = 3;
    const static int MAX_THROTTLES_PER_CLIENT = 6;
    const static int MAX_LOCOS_PER_THROTTLE = 2;

    AsyncServer server;
    //WiFiClient clients[MAX_CLIENTS];

    struct ClientData {
        bool connected;
        uint16_t heartbeatTimeout = 30;
        bool heartbeatEnabled;
        uint32_t lastHeartbeat;

        char cmdline[100];
        size_t cmdpos = 0;

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

    etl::map<AsyncClient*, ClientData, MAX_CLIENTS> clients;
    //ClientData clientData[MAX_CLIENTS];

    void processCmd(AsyncClient *c);

    char powerStatus = '0';

    void turnPower(char v) {
        CS.setPowerState(v=='1');
        notifyPowerStatus();
    }


    void wifiPrintln(AsyncClient *c, String v) {
        c->add(v.c_str(), v.length() );
        char lf = '\n';
        c->add(&lf, 1);
        c->send();
        //WT_LOGI("WTTX %s", v.c_str() );
    }
    void wifiPrint(AsyncClient *c, String v) {
        c->write(v.c_str(), v.length() );
        //clients[iClient].print(v);
        //WT_LOGI("WFTX %s", v.c_str() );
    }

    void clientStart(AsyncClient *c) ;

    void clientStop(AsyncClient *c);

    void locoAdd(char th, String sLocoAddr, AsyncClient *c);

    void locoRelease(char th, String sLocoAddr, AsyncClient *c);
    void locoRelease(char th, LocoAddress addr, AsyncClient *c);

    void locoAction(char th, String sLocoAddr, String actionVal, AsyncClient *c);
    void locoAction(char th, LocoAddress addr, String actionVal, AsyncClient *c);

    void checkHeartbeat(AsyncClient *c);

    void accessoryToggle(int aAddr, char aStatus, bool namedTurnout);
};