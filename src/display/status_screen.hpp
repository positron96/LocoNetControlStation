#pragma once

#include "../config.hpp"

#include "display.hpp"

#include "../CommandStation.h"
#include "../WiThrottleServer.h"
#include "../LocoNetTCPServer.h"

#include <WiFi.h>
#include <Arduino.h>

namespace display {

    class StatusScreen: public Screen, public dcc::PowerObserver {
    public:

        WiThrottleServer *wtServer;
        LbServer *lbServer;
        int cur_page{0};
        uint32_t last_page_change{0};

        static constexpr size_t N_PAGES = 5;

        void loop() override {
            if(millis()-last_page_change>4000) {

                last_page_change = millis();

                for(int i=1; i<N_PAGES; i++) {
                    int nextPage = (cur_page+i)%N_PAGES;
                    if(nextPage==2 && USE_WIFI==0) continue;
                    if(nextPage==3 && (USE_WIFI==0 || lbServer==nullptr)) continue;
                    if(nextPage==4 && (USE_WIFI==0 || wtServer==nullptr)) continue;
                    cur_page = nextPage;
                    break;
                }

                switch(cur_page) {
                    case 0: title="Track"; break;
                    case 1: title="Locos"; break;
                    case 2: title="WiFi"; break;
                    case 3: title="LnTCP"; break;
                    case 4: title="WiThrottle"; break;
                }

                setDirty();
            }
        }

        void notification(const dcc::PowerEvent &event) override {
            setDirty();
        }

    protected:

        void onShow() override { last_page_change = millis(); }

        void drawContents() override {
            U8G2 &u8g2 = Display::u8g2;
            int scroller_width = u8g2.getWidth() / N_PAGES;
            int x = cur_page*scroller_width;
            int y = Display::STATUS_BAR_HEIGHT;

            u8g2.drawHLine(x, y, scroller_width);

            //u8g2_font_8x13B_tr  u8g2_font_nokiafc22_tr
            const static uint8_t * mainFont = u8g2_font_9x6LED_tr;
            u8g2.setFont(mainFont);
            u8g2.setFontPosBottom();
            x = 0;
            y += u8g2.getMaxCharHeight()+1;

            switch(cur_page) {
                case 0:
                    drawTrackPower(u8g2, x, y);
                    break;
                case 1:
                    drawLocoStatus(u8g2, x, y);
                    break;
            #if USE_WIFI==1
                case 2:
                    drawWiFiStatus(u8g2, x, y);
                    break;
                case 3:
                    drawLbServerStatus(u8g2, x, y);
                    break;
                case 4:
                    drawWiThrottleStatus(u8g2, x, y);
                    break;
            #endif
            }
        }

        #ifdef USE_WIFI
        void drawWiFiStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            int dy = u8g2.getMaxCharHeight();
            String v;

            if((WiFi.getMode() & WIFI_MODE_AP) != 0) {
                v = "AP Name:" + String(WiFi.softAPSSID());
                u8g2.drawStr(x, y, v.c_str()); y += dy;

                v = "AP IP:" + WiFi.softAPIP().toString();
                u8g2.drawStr(x, y, v.c_str()); y += dy;

                v = WiFi.softAPgetStationNum() + " clients";
                u8g2.drawStr(x, y, v.c_str()); y += dy;
            }

            if((WiFi.getMode() & WIFI_MODE_STA) != 0) {
                bool connected = WiFi.isConnected();
                if(!connected) {
                    u8g2.drawStr(x, y, "STA Not connected");
                } else {
                    v = "STA Name: "+ String(WiFi.SSID());
                    unsigned t = u8g2.drawStr(x, y, v.c_str());

                    // drawWifiBars(u8g2, x+t+2, y-dy, WiFi.RSSI(), 4, 4, 12, 1);
                    y += dy;

                    v = "STA IP:" + WiFi.localIP().toString();
                    u8g2.drawStr(x, y, v.c_str());
                }
            }
        }

        void drawLbServerStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            String v;
            if(lbServer!=nullptr) {
                v = lbServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }

        void drawWiThrottleStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            String v;
            if(wtServer!=nullptr) {
                v = wtServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }
        #endif

        int drawOneTrackPower(U8G2 &u8g2, unsigned x, unsigned y, String name, const dcc::BaseChannel *track) {
            auto font = u8g2.getU8g2()->font;
            char v[20];
            int tx = x;
            snprintf(v, sizeof(v), "%s: %s",
                name.c_str(), track->getOvercurrentStatus() ? "OVP" : track->getPower()?"ON":"OFF");
            tx += u8g2.drawStr(tx, y, v);

            tx = x + 50;
            snprintf(v, sizeof(v), "%03d", track->getCurrent());
            u8g2.setFont(u8g2_font_profont17_tn); // big numbers
            int dy = u8g2.getMaxCharHeight();
            tx += u8g2.drawStr(tx, y+2, v);
            tx += 2;

            u8g2.setFont(font);
            u8g2.drawStr(tx, y, "mA"); y += dy;
            return y;
        }

        void drawTrackPower(U8G2 &u8g2, unsigned x, unsigned y) {
            x = 15;
            y += 3;

            int voltage = 15000;
            int tx = x;
            tx += u8g2.drawStr(tx, y, "Volts: ");

            auto font = u8g2.getU8g2()->font;
            tx = x + 50;
            char v[20];
            snprintf(v, sizeof(v), "%02d.%1d", voltage/1000, (voltage/100)%10);
            u8g2.setFont(u8g2_font_profont17_tn); // big numbers
            int dy = u8g2.getMaxCharHeight();
            tx += u8g2.drawStr(tx, y+2, v);
            tx += 2;
            u8g2.setFont(font);
            u8g2.drawStr(tx, y, "V"); y += dy;


            const dcc::BaseChannel *mainTrack = CS.getMainTrack();
            if(mainTrack!=nullptr) {
                y = drawOneTrackPower(u8g2, x, y, "Main", mainTrack);
            }

            const dcc::BaseChannel *progTrack = CS.getProgTrack();
            if(progTrack!=nullptr) {
                drawOneTrackPower(u8g2, x, y, "Prog", progTrack);
            }
        }

        void drawLocoStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            int dy = u8g2.getMaxCharHeight();

            String v;
            if(CS.getAllocatedSlotsCount() == 0) {
                u8g2.drawStr(x, y, "No locos");
            } else {
                for(const auto slot: CS.getAllocatedSlots()) {
                    const auto &data = CS.getSlotData(slot);
                    v = String(slot) + ": " + String(data.addr) + " ";
                    if(data.refreshing) {
                        v += (data.dir==1?"F ":"R ") + String(data.speed);
                    }
                    u8g2.drawStr(x, y, v.c_str());
                    y += dy;
                }
            }
        }

    };

}  // namespace display
