#pragma once

#include <U8g2lib.h>

namespace display {


    class Display {
    public:
        static U8G2 &u8g2;
        static const int STATUS_BAR_HEIGHT = 9;

        Display() {
            assert(inst==nullptr);
            inst=this;
        }

        // void setDirty(bool fdirty=true) { dirty=fdirty; }

        // void begin() { dirty=true; }

        // void loop();

        // void draw();

        // void setScreen(Screen *screen) ;

        // static Display *getDisplay();

    private:

        static Display *inst;

        // Screen *cScreen;

        // bool dirty;

        // void processInput() {};
    };


    class Screen {
    public:

        Screen() {}

        // void setDirty(bool fdirty=true) { Display::getDisplay()->setDirty(fdirty); }

        // virtual void begin() { setDirty(true); }

        // virtual void loop() {}

        void draw();

    protected:

        virtual void drawContents() = 0;

        // virtual void onButtonPressed(Button bt, int8_t arg) {};

        // virtual void onPotValueChanged(int pot, int val) {};

        //virtual void onMenuItemSelected(MenuItem & item) {};

        // virtual void onShow() {};
        // virtual void onHide() {};

    private:

        friend class Display;

    };

}
