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
        bool overcurrent[2]{false};

        static constexpr size_t N_PAGES = 5;

        void loop() override {
            if(millis()-last_page_change>5000) {

                last_page_change = millis();

                for(int i=1; i<N_PAGES; i++) {
                    int nextPage = (cur_page+i)%N_PAGES;
                    if(nextPage==0 && USE_WIFI==0) continue;
                    if(nextPage==1 && (USE_WIFI==0 || lbServer==nullptr)) continue;
                    if(nextPage==2 && (USE_WIFI==0 || wtServer==nullptr)) continue;
                    cur_page = nextPage;
                    break;
                }

                setDirty();
            }
        }

        void notification(const dcc::PowerEvent &event) override {
            setDirty();
            size_t idx = (event.channel == CS.getMainTrack()) ? 0 : 1;
            if(!event.state && event.reason == dcc::PowerEvent::Reason::Overcurrent) {
                overcurrent[idx] = true;
            } else {
                overcurrent[idx] = false;
            }
        }

    protected:

        void onShow() override { last_page_change = millis(); }

        void drawContents() override {
            U8G2 &u8g2 = Display::u8g2;
            int x = 2;
            int y = Display::STATUS_BAR_HEIGHT;

            int scroller_width = u8g2.getWidth() / N_PAGES;
            u8g2.drawBox(cur_page*scroller_width, y-2, scroller_width, 2);

            //u8g2_font_8x13B_tr  u8g2_font_nokiafc22_tr
            const static uint8_t * mainFont = u8g2_font_12x6LED_tr;
            u8g2.setFont(mainFont);
            u8g2.setFontPosBottom();
            y += u8g2.getMaxCharHeight();

            switch(cur_page) {
            #if USE_WIFI==1
                case 0:
                    drawWiFiStatus(u8g2, x, y);
                    break;
                    [[fallthrough]];
                case 1:
                    if(lbServer!=nullptr) {
                        drawLbServerStatus(u8g2, x, y);
                        break;
                    }
                    [[fallthrough]];
                case 2:
                    if(wtServer!=nullptr) {
                        drawWiThrottleStatus(u8g2, x, y);
                        break;
                    }
                    [[fallthrough]];
            #endif
                case 3:
                    drawTrackPower(u8g2, x, y);
                    break;
                case 4:
                default:
                    drawLocoStatus(u8g2, x, y);
                    break;
            }
        }

        #ifdef USE_WIFI
        void drawWiFiStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            int dy = u8g2.getMaxCharHeight();
            String v;

            drawStrCentered(u8g2, y, "WIFI");
            y += dy;

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

                    drawWifiBars(u8g2, x+t+2, y, WiFi.RSSI(), 4, 4, 12, 1);
                    y += dy;

                    v = "STA IP:" + WiFi.localIP().toString();
                    u8g2.drawStr(x, y, v.c_str());
                }
            }
        }

        void drawLbServerStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            int dy = u8g2.getMaxCharHeight();

            String v;
            if(lbServer!=nullptr) {
                v = "LocoNet TCP";
                drawStrCentered(u8g2, y, v.c_str());
                y += dy;

                v = lbServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }

        void drawWiThrottleStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            int dy = u8g2.getMaxCharHeight();

            String v = "";
            if(wtServer!=nullptr) {
                v = "WiThrottle";
                drawStrCentered(u8g2, y, v.c_str());
                y += dy;

                v = wtServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }
        #endif

        void drawTrackPower(U8G2 &u8g2, unsigned x, unsigned y) {

            int dy = u8g2.getMaxCharHeight();
            auto font = u8g2.getU8g2()->font;

            String v = "";
            const dcc::BaseChannel *mainTrack = CS.getMainTrack();
            if(mainTrack!=nullptr) {
                int tx = x;
                v = "Main: ";
                v += overcurrent[0] ? "OVP" : mainTrack->getPower()?"ON":"OFF";
                tx += u8g2.drawStr(tx, y, v.c_str());

                v = String(mainTrack->getCurrent());
                u8g2.setFont(u8g2_font_profont17_tn); // big numbers
                tx += u8g2.drawStr(tx, y, v.c_str());

                u8g2.setFont(font);
                u8g2.drawStr(tx, y, "mA"); y += dy + 5;
            }

            const dcc::BaseChannel *progTrack = CS.getProgTrack();
            if(progTrack!=nullptr) {
                u8g2.setFont(font);
                v = "Prog: ";
                v += overcurrent[1] ? "OVP" : progTrack->getPower()?"ON":"OFF";
                v += " " + String(progTrack->getCurrent()) + "mA";
                u8g2.drawStr(x, y, v.c_str());
            }
        }

        void drawLocoStatus(U8G2 &u8g2, unsigned x, unsigned y) {
            u8g2.setFont(u8g2_font_8x13B_tr); // use smaller font
            int dy = u8g2.getMaxCharHeight();

            String v = "";
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

    };

}  // namespace display
