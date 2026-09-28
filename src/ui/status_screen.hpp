#pragma once

#include "../config.hpp"
#include "display.hpp"

#include <dcc/power_event.hpp>

#include <etl/enum_type.h>
#include <etl/string_view.h>

class LbServer;
class WiThrottleServer;

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
        ETL_ENUM_TYPE(WiFi, "WiFi")
        ETL_ENUM_TYPE(LbServer, "LnTCP")
        ETL_ENUM_TYPE(WiThrottle, "WiThrottle")
        ETL_END_ENUM_TYPE
    };

    class StatusScreen: public Screen, public dcc::PowerObserver {
    public:

        constexpr static uint32_t DEFAULT_PAGE_DURATION = 4000;

        WiThrottleServer *wtServer;
        LbServer *lbServer;
        StatusPage cur_page{StatusPage::Tracks};
        uint32_t last_page_change{0};
        uint32_t page_duration{DEFAULT_PAGE_DURATION};

        void setPage(StatusPage page, uint32_t duration = DEFAULT_PAGE_DURATION);
        void loop() override;
        void notification(const dcc::PowerEvent &event) override;

    protected:
        void onShow() override;
        void drawContents() override;
        bool onButtonEvent(unsigned button, bool pressed, bool held) override;

        private:

        #ifdef USE_WIFI
        void drawWiFiPage(U8G2 &u8g2, int x, int y);
        void drawStrView(U8G2 &u8g2, int x, int y, const etl::string_view s);
        void drawMultiStr(U8G2 &u8g2, int x, int y, const etl::string_view s);
        void drawLbServerPage(U8G2 &u8g2, int x, int y);
        void drawWiThrottlePage(U8G2 &u8g2, int x, int y);
        #endif

        int drawValue(U8G2 &u8g2, int x, int y, int value, const char* suffix);
            int drawTrack(U8G2 &u8g2, int x, int y, const char* name, const dcc::BaseChannel *track);
        void drawPowerPage(U8G2 &u8g2, int x, int y);
        void drawLocosPage(U8G2 &u8g2, unsigned x, unsigned y);
    };

}  // namespace ui
