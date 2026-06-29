#pragma once

#include <U8g2lib.h>

namespace display {

    class Screen;

    class Display {
    public:
        static U8G2 &u8g2;
        static const int STATUS_BAR_HEIGHT = 16; // for dual color 128x64 OLEDs

        Display() {
            assert(inst==nullptr);
            inst=this;
        }

        void setDirty(bool fdirty=true) { dirty=fdirty; }

        void begin() { dirty=true; }

        void loop();

        void draw();

        void setScreen(Screen *screen) ;

        static Display *getDisplay() { return inst; }

    private:

        static Display *inst;

        Screen *cScreen;

        bool dirty;

        // void processInput() {};
    };


    class Screen {
    public:

        Screen() {}

        virtual ~Screen() = default;

        void setDirty(bool fdirty=true) { Display::getDisplay()->setDirty(fdirty); }

        // virtual void begin() { setDirty(true); }

        virtual void loop() {}

        void draw() {
            drawContents();
        }

    protected:

        virtual void drawContents() = 0;

        // virtual void onButtonPressed(Button bt, int8_t arg) {};

        // virtual void onPotValueChanged(int pot, int val) {};

        //virtual void onMenuItemSelected(MenuItem & item) {};

        virtual void onShow() {};
        virtual void onHide() {};

    private:

        friend class Display;

    };

    // util functions

    u8g2_uint_t drawWifiBars(U8G2 &u8g2, int x, int y, int rssi,
        int maxBars = 4, int barWidth = 3, int maxHeight = 12, int spacing = 1);

    u8g2_uint_t drawStrCentered(U8G2 &u8g2, int y, const char *str);

}
