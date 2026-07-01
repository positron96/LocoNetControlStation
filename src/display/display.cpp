#include "display.hpp"

#include "../config.hpp"
#include "../CommandStation.h" // for statusbar stuff

#include "WiFi.h"

namespace display {

    Display *Display::inst{nullptr};

    void drawStatusbar(U8G2 &u8g2);

    void Display::begin() {
        dirty=true;
        u8g2.begin();
        u8g2.setFontPosTop();

        // TODO: move it somewhere else later
        u8g2.setFont(u8g2_font_9x6LED_tr);
        u8g2.clearBuffer();
        u8g2.drawStr(16,16, "INIT...");
        u8g2.sendBuffer();
    }

    /** Redraws the display if needed. */
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
            String title = cScreen->title;
            u8g2.setFont(u8g2_font_9x6LED_tr);
            u8g2.setFontPosBottom();
            u8g2.drawStr(0, STATUS_BAR_HEIGHT-3, title.c_str());
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

    u8g2_uint_t drawStrLeft(U8G2 &u8g2, int x, int y, const char *str) {
        u8g2_uint_t width = u8g2.getStrWidth(str);
        u8g2.drawStr(x - width, y, str);
        return width;
    }
    u8g2_uint_t drawGlyphLeft(U8G2 &u8g2, int x, int y, const uint16_t c) {
        u8g2_uint_t width = u8g2_GetGlyphWidth(u8g2.getU8g2(), c);
        u8g2.drawGlyph(x - width, y, c);
        return width;
    }

    int drawTrackStatus(U8G2 &u8g2, int x, int y, const char *name, const dcc::BaseChannel *track) {
        if(track == nullptr) return x;
        auto font = u8g2.getU8g2()->font;
        if(track->getOvercurrentStatus()) {
            u8g2.setFont(u8g2_font_open_iconic_thing_1x_t);
            x -= drawGlyphLeft(u8g2, x, y-1, 0x4E);
        } else {
            u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
            x -= drawGlyphLeft(u8g2, x, y-1, track->getPower() ? 0x40 : 0x44);
        }
        x -= 2;
        u8g2.setFont(font);
        x -= drawStrLeft(u8g2, x, y, name);
        x -= 4;
        return x;
    }

    void drawStatusbar(U8G2 &u8g2) {

        unsigned x = 2; // small margin
        unsigned y = Display::STATUS_BAR_HEIGHT - 3;

        u8g2.drawHLine(0, y, u8g2.getWidth());
        u8g2.setFontPosBottom();

        x = u8g2.getWidth();

        #if USE_WIFI==1
        x -= 15;
        bool conn = WiFi.isConnected();
        //x += u8g2.drawGlyph(x,y, 0x51); // wifi icon
        if(!conn) {
            u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
            u8g2.drawGlyph(x, y-1, 0x4A); // stop sign
        } else {
            drawWifiBars(u8g2, x, y-13, WiFi.RSSI());
        }
        x -= 2;
        #endif

        // Prog track
        x = drawTrackStatus(u8g2, x, y, "P", CS.getProgTrack());

        // Main track power
        drawTrackStatus(u8g2, x, y, "M", CS.getMainTrack());

    }

}
