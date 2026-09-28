#include "status_screen.hpp"

#include "../command_station.hpp"
#include "../loconet_tcp_server.hpp"
#include "../withrottle_server.hpp"

#include <dcc/esp32_current_meter.hpp>
#include <etl/string_utilities.h>

#include <Arduino.h>
#include <WiFi.h>

namespace ui {

constexpr unsigned value_dot_pos = 60;

bool isPageValid(StatusPage page) {
    if(page == StatusPage::WiFi && USE_WIFI == 0) return false;
    if(page == StatusPage::LbServer && USE_WIFI == 0) return false;
    if(page == StatusPage::WiThrottle && USE_WIFI == 0) return false;
    return true;
}

StatusPage StatusPage::advance(int8_t step) {
    step += StatusPage::N_PAGES; // in case it's negative
    auto nextPage = *this;
    for(int i = 0; i < StatusPage::N_PAGES - 1; i++) {
        nextPage = StatusPage{StatusPage::value_type((nextPage.get_value() + step) % StatusPage::N_PAGES)};
        if(isPageValid(nextPage)) return nextPage;
    }
    assert(false); // absolutely no valid pages is impossible
    return StatusPage{0};
}

void StatusScreen::setPage(StatusPage page, uint32_t duration) {
    if(curPage == page) return;
    curPage = page;
    title = curPage.c_str();
    lastPageChange = millis();
    pageDuration = duration;
    setDirty();
}

void StatusScreen::loop() {
    if(millis() - lastPageChange <= pageDuration) return;
    pageDuration = DEFAULT_PAGE_DURATION;
    lastPageChange = millis();

    setPage(curPage.advance(1));
}

void StatusScreen::notification(const dcc::PowerEvent &) {
    if(curPage == StatusPage::Tracks) setDirty();
}

void StatusScreen::onShow() {
    lastPageChange = millis();
}

void StatusScreen::drawContents() {
    U8G2 &u8g2 = Display::u8g2;
    int scroller_width = u8g2.getWidth() / StatusPage::N_PAGES;
    int x = curPage * scroller_width;
    int y = Display::STATUS_BAR_HEIGHT;

    u8g2.drawHLine(x, y, scroller_width);
    u8g2.setFont(u8g2_font_nokiafc22_tr);
    u8g2.setFontPosBottom();
    y += 1;

    switch(curPage) {
        case StatusPage::Tracks: drawPowerPage(u8g2, 0, y); break;
        case StatusPage::Locos: drawLocosPage(u8g2, 0, y); break;
#if USE_WIFI == 1
        case StatusPage::WiFi: drawWiFiPage(u8g2, 0, y); break;
        case StatusPage::LbServer: drawLbServerPage(u8g2, 0, y); break;
        case StatusPage::WiThrottle: drawWiThrottlePage(u8g2, 0, y); break;
#endif
    }
}

bool StatusScreen::onButtonEvent(unsigned button, bool pressed, bool held) {
    if(!pressed || held) return false;
    int8_t inc;
    switch(button) {
        case 0: inc = 1; break;
        case 1: inc = -1; break;
        default: return false;
    }
    setPage(curPage.advance(inc));
    return true;
}

#ifdef USE_WIFI
void StatusScreen::drawWiFiPage(U8G2 &u8g2, int x, int y) {
    y += u8g2.getMaxCharHeight();
    int h = u8g2.getMaxCharHeight() + 1;
    String v;

    if((WiFi.getMode() & WIFI_MODE_AP) != 0) {
        v = "AP: " + String(WiFi.softAPSSID());
        u8g2.drawStr(x, y, v.c_str()); y += h;
        v = "AP IP: " + WiFi.softAPIP().toString();
        u8g2.drawStr(x, y, v.c_str()); y += h;
        v = WiFi.softAPgetStationNum() + " clients";
        u8g2.drawStr(x, y, v.c_str()); y += h;
    }

    if((WiFi.getMode() & WIFI_MODE_STA) != 0) {
        if(!WiFi.isConnected()) {
            u8g2.drawStr(x, y, "STA Not connected");
        } else {
            v = "STA: " + String(WiFi.SSID());
            u8g2.drawStr(x, y, v.c_str()); y += h;
            v = "STA IP: " + WiFi.localIP().toString();
            u8g2.drawStr(x, y, v.c_str());
        }
    }
}

void StatusScreen::drawStrView(U8G2 &u8g2, int x, int y, const etl::string_view s) {
    for(char c: s) x += u8g2.drawGlyph(x, y, c);
}

void StatusScreen::drawMultiStr(U8G2 &u8g2, int x, int y, const etl::string_view s) {
    int h = u8g2.getMaxCharHeight() + 1;
    etl::optional<etl::string_view> token;
    while((token = etl::get_token(s, "\n\r", token, true))) {
        drawStrView(u8g2, x, y, token.value());
        y += h;
    }
}

void StatusScreen::drawLbServerPage(U8G2 &u8g2, int x, int y) {
    if(lbServer == nullptr) return;
    y += u8g2.getMaxCharHeight();
    String v = lbServer->getInfo();
    drawMultiStr(u8g2, x, y, {v.c_str(), v.length()});
}

void StatusScreen::drawWiThrottlePage(U8G2 &u8g2, int x, int y) {
    if(wtServer == nullptr) return;
    y += u8g2.getMaxCharHeight();
    String v = wtServer->getInfo();
    drawMultiStr(u8g2, x, y, {v.c_str(), v.length()});
}
#endif

int StatusScreen::drawValue(U8G2 &u8g2, int x, int y, int value, const char *suffix) {
    auto font = u8g2.getU8g2()->font;
    char v[20];
    int tx = x;
    long roundedToHundredth = (value >= 0 ? value + 5 : value - 5) / 10;
    long absRounded = roundedToHundredth >= 0 ? roundedToHundredth : -roundedToHundredth;
    long whole = absRounded / 100;
    long frac = absRounded % 100;
    u8g2.setFont(u8g2_font_profont17_tn);
    snprintf(v, sizeof(v), "%s%ld", roundedToHundredth < 0 ? "-" : "", whole);
    unsigned iw = u8g2.getStrWidth(v);
    snprintf(v, sizeof(v), "%s%ld.%02ld", roundedToHundredth < 0 ? "-" : "", whole, frac);
    int h = u8g2.getMaxCharHeight();
    tx = tx + u8g2.drawStr(tx - iw, y + 3, v) - iw + 2;
    u8g2.setFont(font);
    u8g2.drawStr(tx, y, suffix);
    return y + h - 2;
}

int StatusScreen::drawTrack(U8G2 &u8g2, int x, int y, const char *name, const dcc::BaseChannel *track) {
    auto font = u8g2.getU8g2()->font;
    int tx = x;
    u8g2.drawStr(tx, y, name);
    tx = x + value_dot_pos;
    if(track->getOvercurrentStatus()) {
        u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
        u8g2.drawGlyph(tx, y + 1, 0x43);
        y += u8g2.getMaxCharHeight() + 1;
        u8g2.setFont(font);
    } else if(!track->getPower()) {
        u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
        u8g2.drawGlyph(tx, y + 1, 0x4E);
        y += u8g2.getMaxCharHeight() + 1;
        u8g2.setFont(font);
    } else {
        y = drawValue(u8g2, tx, y, track->getCurrent(), "A");
    }
    return y;
}

void StatusScreen::drawPowerPage(U8G2 &u8g2, int x, int y) {
    x = 5;
    y += u8g2.getMaxCharHeight() + 8;
    while(dcc::ESP32CurrentMeter::isBusy()) delayMicroseconds(10);
    int voltage_mv = analogReadMilliVolts(PIN_VSENSE) * VSENSE_COEF;
    u8g2.drawStr(x, y, "Input:");
    y = drawValue(u8g2, x + value_dot_pos, y, voltage_mv, "V");

    const dcc::BaseChannel *mainTrack = CS.getMainTrack();
    if(mainTrack != nullptr) y = drawTrack(u8g2, x, y, "Main:", mainTrack);
    const dcc::BaseChannel *progTrack = CS.getProgTrack();
    if(progTrack != nullptr) drawTrack(u8g2, x, y, "Prog:", progTrack);
}

void StatusScreen::drawLocosPage(U8G2 &u8g2, unsigned x, unsigned y) {
    y += u8g2.getMaxCharHeight();
    int h = u8g2.getMaxCharHeight() + 1;
    if(CS.getAllocatedSlotsCount() == 0) {
        u8g2.drawStr(x, y, "No locos");
        return;
    }

    String v;
    for(const auto slot: CS.getAllocatedSlots()) {
        const auto &data = CS.getSlotData(slot);
        v = String(slot) + ": " + String(data.addr) + " ";
        if(data.refreshing) v += (data.dir == 1 ? "F " : "R ") + String(data.speed);
        if(data.hasOwner()) {
            const uintptr_t o = (const uintptr_t)data.owner;
            v += " h" + String(o & 0xFF, HEX);
        }
        int32_t t = (millis() - data.wdt.getLastUpdate()) / 1000;
        if(t > 15) v += " (" + String(t) + "s ago)";
        u8g2.drawStr(x, y, v.c_str());
        y += h;
    }
}

} // namespace ui
