#pragma once

#include "../config.hpp"

#include "display.hpp"

#include "../CommandStation.h"
#include "../WiThrottleServer.h"
#include "../LocoNetTCPServer.h"

#include <etl/enum_type.h>

#include <WiFi.h>
#include <Arduino.h>

namespace ui {

    struct StatusPage {
        enum enum_type {
            Tracks,
            Locos,
            WiFi,
            LbServer,
            WiThrottle
        };
        static constexpr size_t N_PAGES = 5;
        ETL_DECLARE_ENUM_TYPE(StatusPage, uint8_t)
        ETL_ENUM_TYPE(Tracks, "Tracks")
        ETL_ENUM_TYPE(Locos, "Locos")
        ETL_ENUM_TYPE(WiFi,  "WiFi")
        ETL_ENUM_TYPE(LbServer,  "LnTCP")
        ETL_ENUM_TYPE(WiThrottle,  "WiThrottle")
        ETL_END_ENUM_TYPE
    };

    class StatusScreen: public Screen, public dcc::PowerObserver {
    public:

        WiThrottleServer *wtServer;
        LbServer *lbServer;
        StatusPage cur_page{StatusPage::Tracks};
        uint32_t last_page_change{0};

        void setPage(StatusPage page) {
            if(cur_page == page) return;
            cur_page = page;
            title = cur_page.c_str();
            setDirty();
        }

        void loop() override {
            if(millis()-last_page_change>4000) {

                last_page_change = millis();

                for(int i=1; i<StatusPage::N_PAGES; i++) {
                    uint8_t nextPage = (cur_page.get_value()+i) % StatusPage::N_PAGES;
                    if(nextPage==2 && USE_WIFI==0) continue;
                    if(nextPage==3 && (USE_WIFI==0 || lbServer==nullptr)) continue;
                    if(nextPage==4 && (USE_WIFI==0 || wtServer==nullptr)) continue;
                    setPage(StatusPage{nextPage});
                    break;
                }
            }
        }

        void notification(const dcc::PowerEvent &event) override {
            setDirty();
        }

    protected:

        void onShow() override { last_page_change = millis(); }

        void drawContents() override {
            U8G2 &u8g2 = Display::u8g2;
            int scroller_width = u8g2.getWidth() / StatusPage::N_PAGES;
            int x = cur_page*scroller_width;
            int y = Display::STATUS_BAR_HEIGHT;

            u8g2.drawHLine(x, y, scroller_width);

            //u8g2_font_6x13B_tr    u8g2_font_9x6LED_tr
            u8g2.setFont(u8g2_font_nokiafc22_tr);
            u8g2.setFontPosBottom();
            x = 0;
            y += u8g2.getMaxCharHeight()+1;

            switch(cur_page) {
                case StatusPage::Tracks: drawPowerPage(u8g2, x, y);  break;
                case StatusPage::Locos: drawLocoStatus(u8g2, x, y);  break;
            #if USE_WIFI==1
                case StatusPage::WiFi: drawWiFiStatus(u8g2, x, y);  break;
                case StatusPage::LbServer: drawLbServerStatus(u8g2, x, y); break;
                case StatusPage::WiThrottle: drawWiThrottleStatus(u8g2, x, y); break;
            #endif
            }
        }

        #ifdef USE_WIFI
        void drawWiFiStatus(U8G2 &u8g2, int x, int y) {
            int dy = u8g2.getMaxCharHeight();
            String v;

            if((WiFi.getMode() & WIFI_MODE_AP) != 0) {
                v = "AP: " + String(WiFi.softAPSSID());
                u8g2.drawStr(x, y, v.c_str()); y += dy;

                v = "AP IP: " + WiFi.softAPIP().toString();
                u8g2.drawStr(x, y, v.c_str()); y += dy;

                v = WiFi.softAPgetStationNum() + " clients";
                u8g2.drawStr(x, y, v.c_str()); y += dy;
            }

            if((WiFi.getMode() & WIFI_MODE_STA) != 0) {
                bool connected = WiFi.isConnected();
                if(!connected) {
                    u8g2.drawStr(x, y, "STA Not connected");
                } else {
                    v = "STA: "+ String(WiFi.SSID());
                    u8g2.drawStr(x, y, v.c_str());

                    // drawWifiBars(u8g2, x+t+2, y-dy, WiFi.RSSI(), 4, 4, 12, 1);
                    y += dy;

                    v = "STA IP: " + WiFi.localIP().toString();
                    u8g2.drawStr(x, y, v.c_str());
                }
            }
        }

        void drawLbServerStatus(U8G2 &u8g2, int x, int y) {
            String v;
            if(lbServer!=nullptr) {
                v = lbServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }

        void drawWiThrottleStatus(U8G2 &u8g2, int x, int y) {
            String v;
            if(wtServer!=nullptr) {
                v = wtServer->getInfo();
                u8g2.drawStr(x, y, v.c_str());
            }
        }
        #endif

        /** Draws a fixed point value.
         * Value is with 3 decimal places (i.e. float_val*1000)
         */
        int drawValue(U8G2 &u8g2, int x, int y, int value, const char* suffix) {
            auto font = u8g2.getU8g2()->font;
            char v[20];
            int tx = x;

            // value is fixed-point with 3 decimal places (real_value * 1000)
            // Convert to N.NN by rounding to the nearest hundredth.
            long roundedToHundredth = (value >= 0 ? value + 5 : value - 5) / 10;
            long absRounded = (roundedToHundredth >= 0) ? roundedToHundredth : -roundedToHundredth;
            long whole = absRounded / 100;
            long frac = absRounded % 100;
            snprintf(v, sizeof(v), "%s%ld.%02ld", roundedToHundredth < 0 ? "-" : "", whole, frac);
            u8g2.setFont(u8g2_font_profont17_tn); // big numbers
            int dy = u8g2.getMaxCharHeight();
            tx += u8g2.drawStr(tx, y+2, v);
            tx += 2;

            u8g2.setFont(font);
            u8g2.drawStr(tx, y, suffix); y += dy - 2;
            return y;
        }

        int drawTrack(U8G2 &u8g2, int x, int y, const char* name, const dcc::BaseChannel *track) {
            auto font = u8g2.getU8g2()->font;
            int tx = x;
            tx += u8g2.drawStr(tx, y, name) + 10;
            if(track->getOvercurrentStatus()) {
                u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
                u8g2.drawGlyph(tx, y+1, 0x43);
                y += u8g2.getMaxCharHeight();
                u8g2.setFont(font);
            } else if(track->getPower() == false) {
                u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
                u8g2.drawGlyph(tx, y+1, 0x4E);
                y += u8g2.getMaxCharHeight();
                u8g2.setFont(font);
            } else {
                y = drawValue(u8g2, tx, y, track->getCurrent(), "A");
            }
            return y;
        }


        void drawPowerPage(U8G2 &u8g2, int x, int y) {
            x = 5;
            y += 5;

            int voltage = 12345; // mV
            int tx = x;
            tx += u8g2.drawStr(x, y, "Input:");
            y = drawValue(u8g2, tx + 10, y, voltage, "V");

            String v;
            const dcc::BaseChannel *mainTrack = CS.getMainTrack();
            if(mainTrack!=nullptr) {
                y = drawTrack(u8g2, x, y, "Main:", mainTrack);
            }

            const dcc::BaseChannel *progTrack = CS.getProgTrack();
            if(progTrack!=nullptr) {
                drawTrack(u8g2, x, y, "Prog:", progTrack);
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

}  // namespace ui
