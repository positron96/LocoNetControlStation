/**
 *
 * Serverside implementation of WiThrottle protocol.
 *
 * Based on https://github.com/positron96/withrottle
 *
 * Also, see JMRI sources, start at https://github.com/JMRI/JMRI/blob/master/java/src/jmri/jmrit/withrottle/DeviceServer.java
 *
 * Official documentation: https://www.jmri.org/help/en/package/jmri/jmrit/withrottle/Protocol.shtml
 */

#pragma once

#include "CommandStation.h"
#include "Watchdog.h"
#include "dcc/power_event.hpp"
#include "FastClock.hpp"

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>

#include <esp_task_wdt.h>

#include <etl/map.h>
#include <etl/utility.h>
#include <etl/string_view.h>
#include <etl/enum_type.h>
#include <etl/chrono.h>

#define WT_DEBUG

#ifdef WT_DEBUG
#define WT_LOGI(format, ...)  do{ ets_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__);  } while(0)
#else
#define WT_LOGI(...)
#endif


class WiThrottleServer: public dcc::PowerObserver, public fast_clock::clock_observer {
public:

    constexpr static uint16_t DEF_PORT = 4444;

    WiThrottleServer(uint16_t port, const char* name=nullptr);

    void begin();

    void end();

    void notification(const fast_clock::ClockChangedEvent &evt) override {
        notifyFastClock(nullptr);
    }

    void notification(const dcc::PowerEvent &event) override {
        notifyPowerStatus();
    }

    void notifyPowerStatus(AsyncClient *c=nullptr) {
        bool v = CS.getPowerState();
        powerOn = v;
        String s = String("PPA") + (powerOn ? '1' : '0');
        if(c==nullptr) {
            for (auto p: clients) {
                wifiPrintln(p.first, s);
            }
        } else wifiPrintln(c, s);
    }

    void notifyFastClock(AsyncClient *c=nullptr) {
        uint32_t seconds = fast_clock::clock.getSeconds();
        unsigned rate = fast_clock::clock.getRate();
        String s = String("PFT")+ seconds + "<;>"+rate;
        if(c==nullptr) {
            for (auto p: clients) {
                wifiPrintln(p.first, s);
            }
        } else wifiPrintln(c, s);
    }

    void loop();

    String getInfo() const;

private:

    const uint16_t port;
    const char* name;

    constexpr static int MAX_CLIENTS = 3;
    constexpr static int MAX_THROTTLES_PER_CLIENT = 6;
    constexpr static int MAX_LOCOS_PER_THROTTLE = 2;

    constexpr static millis_t HEARTBEAT_INTL = 20; ///< in seconds

    AsyncServer server;

    struct ClientHealth {
        enum enum_type {
            Alive,
            SoftTimeout, ///< stop all locos after soft timeout
            HardTimeout,  ///< disconnect client after hard timeout
            Dead  ///< no need to check anything, it should be cleaned up soon
        };

        ETL_DECLARE_ENUM_TYPE(ClientHealth, unsigned)
        ETL_ENUM_TYPE(Alive, "OK")
        ETL_ENUM_TYPE(SoftTimeout, "Yellow")
        ETL_ENUM_TYPE(HardTimeout, "Red")
        ETL_ENUM_TYPE(Dead, "Dead")
        ETL_END_ENUM_TYPE
    };

    struct ClientData {
        AsyncClient *cli;
        bool heartbeatEnabled;
        ClientHealth health = ClientHealth::Alive;
        Watchdog<HEARTBEAT_INTL*1000+5000, 500, HEARTBEAT_INTL*1000*2+5000> wdt;

        constexpr static size_t RX_SIZE = 100;
        char rx[RX_SIZE];
        size_t rxpos = 0;

        String hwId;

        using AddrToSlotMap = etl::map<LocoAddress, uint8_t, MAX_LOCOS_PER_THROTTLE>;
        // each client can have up to 6 multi throttles, each MT can have multiple locos (and slots)
        etl::map< char, AddrToSlotMap, MAX_THROTTLES_PER_CLIENT> slots;
        uint8_t slot(char thr, LocoAddress addr) {
            auto it = slots.find(thr);
            if(it == slots.end()) return 0;
            auto iit = it->second.find(addr);
            if(iit == it->second.end() ) return 0;
            return iit->second;
        }

        void locoAdd(char th, etl::string_view sLocoAddr);

        void locosRelease(char th, etl::string_view  sLocoAddr);
        void locoRelease(char th, LocoAddress addr);

        void locosAction(char th, etl::string_view sLocoAddr, etl::string_view actionVal);
        void locoAction(char th, LocoAddress addr, etl::string_view actionVal);

        void checkHeartbeat();
        void updateHeartbeat() {
            wdt.kick();
            health = ClientHealth::Alive;
        }

        void sendMessage(String msg, bool alert=false);

        void sendThrottleMsg(char th, char a, LocoAddress iLocoAddr, String resp);

    };

    etl::map<AsyncClient*, ClientData, MAX_CLIENTS> clients;

    void onNewClient(AsyncClient* cli);

    void processCmd(ClientData &cc);

    void notifyHearbeatStatus(ClientData &c);

    bool powerOn = false;

    void turnPower(char v) {
        CS.setPowerState(v=='1');
        notifyPowerStatus();
    }


    static void wifiPrintln(AsyncClient *c, String v) {
        c->add(v.c_str(), v.length() );
        char lf = '\n';
        c->add(&lf, 1);
        c->send();
        //WT_LOGI("TX '%s'", v.c_str() );
    }
    static void wifiPrint(AsyncClient *c, String v) {
        c->write(v.c_str(), v.length() );
        //WT_LOGI("WFTX  %s", v.c_str() );
    }

    void clientStart(AsyncClient *cli) ;

    void clientStop(ClientData &c);

    void accessoryToggle(unsigned aAddr, char aStatus, bool namedTurnout, ClientData &cc);

};
