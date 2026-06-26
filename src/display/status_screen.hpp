#pragma once

#include "display.hpp"
#include "../CommandStation.h"
#include "../WiThrottle.h"
#include "../LocoNetTCPServer.h"

#include <Arduino.h>

namespace display {

    class StatusScreen {
    public:

        WiThrottleServer *wtServer;
        LbServer *lbServer;

        void draw() {
            U8G2 &u8g2 = Display::u8g2;

            String v = "";
            #ifdef USE_WIFI
            bool connected = WiFi.isConnected();
            if(connected) {
                v = "WF:" + WiFi.getRSSI() + "IP:"+ WiFi.localIP().toString();
            } else {
                v = "WF:---"
            }
            u8g2.drawStr(0, 0, v.c_str());

            if(wtServer!=nullptr) {
                v = "WT: "+String(wtServer->getInfo());
                u8g2.drawStr(0, 8, v.c_str());
            }
            if(lbServer!=nullptr) {
                v = "LB: "+String(lbServer->getInfo());
                u8g2.drawStr(0, 16, v.c_str());
            }
            #endif

            int i=0;
            for(const auto slot: CS.getAllocatedSlots()) {
                const auto &data = CS.getSlotData(slot);
                v = String("Slot")+slot+": "+String(data.addr)+" ";
                if(data.refreshing) {
                    v += (data.dir==1?"F ":"R ");
                    if(data.speed.isEmgr()) {
                        v += "EStp";
                    } else {
                        v += String(data.speed.get128()*100/128)+"%";
                    }
                }
                u8g2.drawStr(0, 16+i*8, v.c_str());
            }
        }
    };

}
