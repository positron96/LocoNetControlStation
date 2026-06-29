#include "display.hpp"

#include "../config.hpp"
#include "../CommandStation.h" // for statusbar stuff

#include "WiFi.h"

namespace display {

    Display *Display::inst{nullptr};

    void drawStatusbar(U8G2 &u8g2);

    void Display::loop() {
        //processInput();
        if(cScreen!=nullptr) cScreen->loop();
        draw();
    }

    void Display::setScreen(Screen *screen) {
        if(cScreen!=nullptr) {
            cScreen->onHide();
        }
        cScreen = screen;
        if(cScreen!=nullptr) {
            cScreen->onShow();
        }
        setDirty();
    }

    void Display::draw() {
        if(!dirty) return;

        u8g2.clearBuffer();

        if(cScreen != nullptr) {
            cScreen->draw();
        }
        drawStatusbar(u8g2);

        u8g2.sendBuffer();
        dirty = false;
    }

    u8g2_uint_t drawWifiBars(U8G2 &u8g2, int x, int y, int rssi, int maxBars, int barWidth, int maxHeight, int spacing) {
        int activeBars = map(rssi, -100, -50, 1, maxBars);
        activeBars = constrain(activeBars, 1, maxBars);

        // Step height size calculation for progressive bars
        float heightStep = (float)maxHeight / maxBars;
        int pitch = barWidth + spacing;

        for (int i = 0; i < maxBars; i++) {
            int currentHeight = (int)((i + 1) * heightStep);
            int currentX = x + (i * pitch);
            int currentY = y + (maxHeight - currentHeight);

            if (i < activeBars) {
                u8g2.drawBox(currentX, currentY, barWidth, currentHeight);
            } else {
                u8g2.drawFrame(currentX, currentY, barWidth, currentHeight);
            }
        }
        return pitch*maxBars;
    }

    u8g2_uint_t drawStrCentered(U8G2 &u8g2, int y, const char *str) {
        u8g2_uint_t width = u8g2.getStrWidth(str);
        u8g2_uint_t x = (u8g2.getWidth() - width) / 2;
        u8g2.drawStr(x, y, str);
        return width;
    }

    void drawStatusbar(U8G2 &u8g2) {

        unsigned x = 2; // small margin
        const unsigned y = 0;

        u8g2.drawLine(
            0, Display::STATUS_BAR_HEIGHT,
            u8g2.getWidth(), Display::STATUS_BAR_HEIGHT
        );

        // Main track power
        u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
        const dcc::BaseChannel *mainTrack = CS.getMainTrack();
        if(mainTrack!=nullptr) {
            x += u8g2.drawGlyph(x, y, mainTrack->getPower() ? 0x41 : 0x42);
            x += 2;
            uint16_t cur = mainTrack->getCurrent();
            String c = String(cur)+"mA";
            //u8g2.setFont(u8g2_font_nokiafc22_tr);
            u8g2.setFont(u8g2_font_helvB12_tr);
            x += u8g2.drawStr(x, y, c.c_str());
            x += 2;
        }

        // Prog track
        const dcc::BaseChannel *progTrack = CS.getProgTrack();
        x = 55;
        if(progTrack!=nullptr) {
            u8g2.setFont(u8g2_font_helvB12_tr);
            x += u8g2.drawStr(x, y, "P:");
            u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
            x += u8g2.drawGlyph(x, y, progTrack->getPower() ? 0x41 : 0x42);
            x += 2;
        }

#if USE_WIFI==1
        x = u8g2.getWidth() - 30;

        u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
        bool conn = WiFi.isConnected();
        x += u8g2.drawGlyph(x,y, 0x51); // wifi icon
        x += 2;
        if(!conn) {
            x += u8g2.drawGlyph(x,y, 0x4A); // stop sign
        } else {
            x += drawWifiBars(u8g2, x,y, WiFi.RSSI());
        }
        x += 2;
#endif

    }

}
