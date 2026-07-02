#include "config.hpp"

#include <DCC.h>
// #include <esp32_timer_channel.hpp>
// #include <esp32_timer.hpp>
#include <dcc/esp32_rmtcont_channel.hpp>
#include <dcc/esp32_current_meter.hpp>

#include "CommandStation.h"

#include "LocoNetManagers.h"

#include "LocoNetSerial.h"
#include "LocoNetTCPServer.h"

#include "WiThrottleServer.h"

#include <LocoNetStream.h>

#include "ui/display.hpp"
#include "ui/status_screen.hpp"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>

#include <Arduino.h>

#include <Wire.h>
#include <U8g2lib.h>

#include <etl/callback_timer_atomic.h>
#include <stdio.h>
#include <atomic>

LocoNetBus bus;

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17
#include <LocoNetStreamESP32.h>
//LocoNetStreamESP32 locoNetPhy(2, LOCONET_PIN_RX, LOCONET_PIN_TX, false, true, &bus); // UART2
LocoNetDispatcher parser(&bus);

LbServer lbServer(LBSERVER_DEFAULT_TCP_PORT, &bus);

//LocoNetSerial lSerial(&Serial, &bus);

#define DCC_MAIN_PIN 25
#define DCC_MAIN_PIN_EN 32
#define DCC_MAIN_PIN_SENSE 36
#define DCC_PROG_PIN 26
#define DCC_PROG_PIN_EN 33
#define DCC_PROG_PIN_SENSE 39

dcc::PacketList<10> dcc_packets_main;
dcc::PacketList<2> dcc_packets_prog;
// dcc::ESP32TimerChannel dccMain(DCC_MAIN_PIN, DCC_MAIN_PIN_EN, DCC_MAIN_PIN_SENSE, dcc_packets_main);
// dcc::ESP32TimerChannel dccProg(DCC_PROG_PIN, DCC_PROG_PIN_EN, DCC_PROG_PIN_SENSE, dcc_packets_prog);
// dcc::ESP32Timer dccTimer(1); //timer1
dcc::ESP32RMTChannel dccMain(DCC_MAIN_PIN, DCC_MAIN_PIN_EN, DCC_MAIN_PIN_SENSE, dcc_packets_main);
dcc::ESP32RMTChannel dccProg(DCC_PROG_PIN, DCC_PROG_PIN_EN, DCC_PROG_PIN_SENSE, dcc_packets_prog);
dcc::ESP32CurrentMeter currentMeter;

LocoNetSlotManager slotMan(&bus);

WiThrottleServer withrottleServer(WiThrottleServer::DEF_PORT, CS_FULL_NAME);

#if USE_DISPLAY==1
constexpr int PIN_DISP_SDA = 18;
constexpr int PIN_DISP_SCL = 19;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, PIN_DISP_SCL, PIN_DISP_SDA);
U8G2 &ui::Display::u8g2 = u8g2_;
ui::Display disp;
ui::StatusScreen statusScreen;
#endif

#define PIN_LED  22
#define PIN_BT 13
#define PIN_BT2 15

// constexpr int _debug_pin = 14;
// constexpr int _debug_pin2 = 12;

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 500;
constexpr int LED_INTL_CONFIG2 = 250;

uint8_t ledVal;

void ledStartBlinking(uint32_t ms=0, uint8_t val=1);
void ledStop();
void ledUpdate();


void tick1s();
void tick20ms();

using TimerType = etl::callback_timer_atomic<3, std::atomic_uint>;
TimerType timerController;
etl::timer::id::type ledTimer;
etl::timer::id::type timer20ms;
etl::timer::id::type timer1s;

class PowerStatusObserver: public dcc::PowerObserver {
    void notification(const dcc::PowerEvent &event) override {
        if(!event.state && event.reason == dcc::PowerEvent::Reason::Overcurrent) {
            if(event.channel == &dccMain) {
                ledStartBlinking(LED_INTL_CONFIG2, 1);
            Serial.println("Overcurrent on main");
            } else if(event.channel == &dccProg) {

                Serial.println("Overcurrent on prog");
            }

        }
    }
} powerStatusObserver;

void setup() {

    Serial.begin(115200);
    Serial.println(CS_FULL_NAME);

    pinMode(PIN_BT, INPUT_PULLUP);
    pinMode(PIN_BT2, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    // pinMode(_debug_pin, OUTPUT);
    // pinMode(_debug_pin2, OUTPUT);

    digitalWrite(PIN_LED, LOW);

    //locoNetPhy.start();
    //lSerial.begin();

    parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
        char tmp[100];
        formatMsg(*rxPacket, tmp, sizeof(tmp));
        Serial.printf("onPacket: %s\n", tmp);
    });


    dccMain.setVoltageToCurrentCoef(1.0f); // depends on schematic
    dccMain.setOvercurrentThreshold(2000);
    currentMeter.addChannel(dccMain);

    dccProg.setVoltageToCurrentCoef(1.0f);
    dccProg.setOvercurrentThreshold(500);
    currentMeter.addChannel(dccProg);

    dccMain.add_observer(powerStatusObserver);
    dccProg.add_observer(powerStatusObserver);

    CS.setDccMain(&dccMain);
    CS.setDccProg(&dccProg);
    CS.setLocoNetBus(&bus);

    // dccTimer.setMainChannel(&dccMain);
    // dccTimer.setProgChannel(&dccProg);
    // dccTimer.begin();

    dccMain.begin();
    dccProg.begin();
    dccMain.setPower(true);
    dccProg.setPower(true);
    currentMeter.begin();

    ledTimer = timerController.register_timer(
        TimerType::callback_type::create<ledUpdate>(),
        LED_INTL_NORMAL, true);
    timer20ms = timerController.register_timer(
        TimerType::callback_type::create<tick20ms>(),
        20, true);

    timerController.enable(true);
    timerController.start(timer20ms);
    //timerController.start(timer1s);

    #if USE_DISPLAY==1
    statusScreen.wtServer = &withrottleServer;
    statusScreen.lbServer = &lbServer;
    statusScreen.setPage(ui::StatusPage::WiFi);
    dccMain.add_observer(statusScreen);
    dccProg.add_observer(statusScreen);
    disp.begin();
    disp.setScreen(&statusScreen);
    disp.loop();
    #endif

#if USE_WIFI != 0
    WiFi.setSleep(WIFI_PS_NONE);
    bool bt = digitalRead(PIN_BT)==0;
    if(bt) {
        // start AP
        WiFi.persistent(false);
        //WiFi.softAPConfig(IPAddress{192,168,1,0}, IPAddress{192,168,1,1}, IPAddress{255,255,255,0});
        WiFi.softAP(CS_FULL_NAME " AP", "");
        Serial.println("");
        Serial.println("WiFi AP started.");
        Serial.println("IP address: ");
        Serial.println(WiFi.softAPIP());
        ledStartBlinking(LED_INTL_NORMAL/2);
    } else {
        WiFiManager wifiManager;
        wifiManager.setConfigPortalTimeout(300); // 5 min
        if ( !wifiManager.autoConnect(CS_FULL_NAME " AP") ) { // sometimes wifi connects during captive portal
            if(WiFi.status() != WL_CONNECTED) {
                Serial.print("Failed connection");
                delay(1000);
                ESP.restart();
            }
        }
        WiFi.setAutoReconnect(true);
        Serial.println("");
        Serial.println("WiFi connected.");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        ledStartBlinking();
    }

    MDNS.begin(CS_SHORT_NAME);
    MDNS.setInstanceName(CS_FULL_NAME);
    lbServer.begin();
    withrottleServer.begin();
    dccMain.add_observer(withrottleServer);  // withrottle doesn't need prog channel

#endif

}


void loop() {

#if USE_WIFI != 0
    lbServer.loop();
    withrottleServer.loop();
#endif
    CS.loop();
    //lSerial.loop();

    uint32_t ms = millis();
    static uint32_t lastMs = millis(); // don't start from 0 as connecting to wifi can take a lot
    if(ms!=lastMs && timerController.tick(ms - lastMs)) {
        lastMs = ms;
    }

    static unsigned long nextInRead = 0;
    static int inState = 0;
    static int inState2 = 0;
    if(millis()>nextInRead) {
        // Serial.println("CHECK");
        int v = 1-digitalRead(PIN_BT);
        if(v!=inState) {
            //CS.turnoutAction(6, false, v ? TurnoutAction::THROW : TurnoutAction::CLOSE);
            auto slot = CS.findOrAllocateLocoSlot(LocoAddress::shortAddr(16));
            if(v) {
                CS.setLocoSlotRefresh(slot, true);
                CS.setLocoSpeed(slot, v ? LocoSpeed::from128(64) : LocoSpeed::from128(0));
                //CS.setLocoFns(slot, 0xFFFFFFFF, 0xFFFFFFFF); // all on
            } else {
                //CS.setLocoFns(slot, 0xFFFFFFFF, 0);
                CS.releaseLocoSlot(slot);
            }

            // Serial.printf( "reporting sensor %d\n", v==HIGH) ;
            // reportSensor(&bus, 1, v==HIGH);
            // Serial.printf("errs: rx:%d,  tx:%d\n", locoNetPhy.getRxStats()->rxErrors, locoNetPhy.getTxStats()->txErrors );
        }
        inState = v;

        v = 1-digitalRead(PIN_BT2);
        if(v!=inState2) {
            auto slot = CS.findOrAllocateLocoSlot(LocoAddress::shortAddr(32));
            if(v) {
                CS.setLocoSlotRefresh(slot, true);
                CS.setLocoSpeed(slot, v ? LocoSpeed::from128(64) : LocoSpeed::from128(0));
                //CS.setLocoSpeed(slot, v ? LocoSpeed::from128(64) : LocoSpeed::from128(0));
                CS.setLocoFn(slot, 0, 1);
                CS.setLocoFn(slot, 5, 1);
                CS.setLocoFn(slot, 8, 1);

            } else {
                CS.releaseLocoSlot(slot);
            }
            // if(dccMain.getPower()) {
            //     dccMain.setPower(false);
            //     dccProg.setPower(false);
            // } else {
            //     dccMain.setPower(true);
            //     dccProg.setPower(true);
            // }
        }
        inState2 = v;

        nextInRead = millis() + 10;
    }

}


void tick20ms() {
#if USE_DISPLAY==1
    disp.loop();
#endif
    currentMeter.checkOvercurrent();
}


void tick1s() {
#if USE_DISPLAY==0 && USE_WIFI==1
    Serial.println(WiFi.isConnected() ? (String("RSSI:")+WiFi.RSSI()) : "No WIFI");
#endif
}


void ledStartBlinking(uint32_t ms, uint8_t val) {
    if(ms==0) ms = LED_INTL_NORMAL;
    ledVal = val;
    digitalWrite(PIN_LED, ledVal);
    //ledNextUpdate = millis()+ms;

    timerController.set_period(ledTimer, ms);
    timerController.start(ledTimer);
}

void ledStop() {
    //ledNextUpdate = 0; // turn off blink
    timerController.stop(ledTimer);
    digitalWrite(PIN_LED, LOW);
}
void ledUpdate() {
    //if(ledNextUpdate!=0 && millis()>ledNextUpdate) {
        ledVal = 1-ledVal;
        digitalWrite(PIN_LED, ledVal);

        //if(!configMode) {
        //ledNextUpdate = LED_INTL_NORMAL;
        /*} else {
        ledNextUpdate = configVar==0 ? LED_INTL_CONFIG1 : LED_INTL_CONFIG2;
        }*/
        //ledNextUpdate += millis();
    //}
}
