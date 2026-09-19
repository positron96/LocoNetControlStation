#include "config.hpp"

#include <DCC.h>
// #include <esp32_timer_channel.hpp>
// #include <esp32_timer.hpp>
#include <dcc/esp32_rmtcont_channel.hpp>
#include <dcc/esp32_current_meter.hpp>

#include "command_station.hpp"

#include "loconet_managers.hpp"

#include "loconet_serial.hpp"
#include "loconet_tcp_server.hpp"

#include "withrottle_server.hpp"
#include "led.hpp"

#include <LocoNetStream.h>

#include "dccpp_proto_decoder.hpp"

#if USE_DISPLAY==1
#include "ui/display.hpp"
#include "ui/status_screen.hpp"
#include <U8g2lib.h>
#include <Wire.h>  // move out from this ifdef if I2C is used elsewhere
#endif

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>

#include <Arduino.h>

#include <etl/callback_timer_atomic.h>
#include <stdio.h>
#include <atomic>

LocoNetBus bus;

#include <LocoNetStreamESP32.h>
//LocoNetStreamESP32 locoNetPhy(2, LOCONET_PIN_RX, LOCONET_PIN_TX, false, true, &bus); // UART2
LocoNetDispatcher parser(&bus);

LbServer lbServer(LBSERVER_DEFAULT_TCP_PORT, &bus);

//LocoNetSerial lSerial(&Serial, &bus);

dcc::PacketList<10> dcc_packets_main;
dcc::PacketList<2> dcc_packets_prog;
// dcc::ESP32TimerChannel dccMain(DCC_MAIN_PIN, DCC_MAIN_PIN_EN, DCC_MAIN_PIN_SENSE, dcc_packets_main);
// dcc::ESP32TimerChannel dccProg(DCC_PROG_PIN, DCC_PROG_PIN_EN, DCC_PROG_PIN_SENSE, dcc_packets_prog);
// dcc::ESP32Timer dccTimer(1); //timer1
dcc::ESP32RMTChannel dccMain(DCC_MAIN_PIN, DCC_MAIN_PIN_EN, DCC_MAIN_PIN_SENSE, dcc_packets_main);
dcc::ESP32RMTChannel dccProg(DCC_PROG_PIN, DCC_PROG_PIN_EN, DCC_PROG_PIN_SENSE, dcc_packets_prog);
dcc::ESP32CurrentMeter currentMeter;

LocoNetSlotManager lnSlotMan(&bus);
LocoNetTurnoutManager lnTurnoutMan(&bus);

WiThrottleServer withrottleServer(WiThrottleServer::DEF_PORT, CS_FULL_NAME);

dccpp::DccppStreamHandler dccpp{&Serial};

#if USE_DISPLAY==1
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, PIN_DISP_SCL, PIN_DISP_SDA);
U8G2 &ui::Display::u8g2 = u8g2_;
ui::Display disp;
ui::StatusScreen statusScreen;
#endif

led::Led statusLed(PIN_LED);

void tick1s();
void tick20ms();

using TimerType = etl::callback_timer_atomic<3, std::atomic_uint>;
TimerType timerController;
etl::timer::id::type timer20ms;
etl::timer::id::type timer1s;

class PowerStatusObserver: public dcc::PowerObserver {
    void notification(const dcc::PowerEvent &event) override {
        if(event.channel == &dccMain) {
            if(!event.state && event.reason == dcc::PowerEvent::Reason::Overcurrent) {
                statusLed.enable_state(led::State::error);
                Serial.println("Overcurrent on main");
            } else if (event.state) {
                statusLed.disable_state(led::State::error);
            }
        } else {
            // prog
            if(!event.state && event.reason == dcc::PowerEvent::Reason::Overcurrent) {
                Serial.println("Overcurrent on prog");
            }
        }
    }
} powerStatusObserver;

void setup() {

    Serial.begin(115200);
    Serial.println(CS_FULL_NAME);
    Serial.println("Config: ");
    Serial.printf(" PCB_NAME=%s\n", PCB_NAME);
    Serial.printf(" USE_DISPLAY=%d\n", USE_DISPLAY);
    Serial.printf(" USE_WIFI=%d\n", USE_WIFI);

    pinMode(PIN_BT, INPUT_PULLUP);
    pinMode(PIN_BT2, INPUT_PULLUP);

    // pinMode(_debug_pin, OUTPUT);
    // pinMode(_debug_pin2, OUTPUT);

    statusLed.begin();

    //locoNetPhy.start();
    //lSerial.begin();

    parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
        char tmp[100];
        formatMsg(*rxPacket, tmp, sizeof(tmp));
        Serial.printf("onPacket: %s\n", tmp);
    });


    dccMain.setVoltageToCurrentCoef(DCC_MAIN_MV_TO_MA_COEF);
    dccMain.setOvercurrentThreshold(2000);
    currentMeter.addChannel(dccMain);

    dccProg.setVoltageToCurrentCoef(DCC_PROG_MV_TO_MA_COEF);
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

    timer20ms = timerController.register_timer(
        TimerType::callback_type::create<tick20ms>(),
        20, true);
    timer1s = timerController.register_timer(
        TimerType::callback_type::create<tick1s>(),
        1000, true);

    timerController.enable(true);
    timerController.start(timer20ms);
    timerController.start(timer1s);

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
    WiFi.setSleep(WIFI_PS_NONE);  // ! makes WiFi MUCH more reliable.
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
        statusLed.enable_state(led::State::normal, 2); // 2 blinks: running as AP
    } else {
        WiFiManager wifiManager;
        wifiManager.setConfigPortalTimeout(300); // 5 min
        statusLed.enable_state(led::State::attention);
        if ( !wifiManager.autoConnect(CS_FULL_NAME " AP") ) { // sometimes wifi connects during captive portal
            if(WiFi.status() != WL_CONNECTED) {
                Serial.print("Failed connection");
                statusLed.enable_state(led::State::error);
                delay(1000);
                ESP.restart();
            }
        }
        WiFi.setAutoReconnect(true);
        statusLed.disable_state(led::State::attention);
        Serial.println("");
        Serial.println("WiFi connected.");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        statusLed.enable_state(led::State::normal, 1);
    }

    MDNS.begin(CS_SHORT_NAME);
    MDNS.setInstanceName(CS_FULL_NAME);
    lbServer.begin();
    withrottleServer.begin();
    dccMain.add_observer(withrottleServer);  // withrottle doesn't need prog channel
#else
    statusLed.enable_state(led::State::normal);
#endif

}


void loop() {

#if USE_WIFI != 0
    lbServer.loop();
    withrottleServer.loop();
#endif
    CS.loop();
    //lSerial.loop();
    dccpp.loop();

    uint32_t ms = millis();
    static uint32_t lastMs = millis(); // don't start from 0 as connecting to wifi can take a lot
    if(ms!=lastMs && timerController.tick(ms - lastMs)) {
        lastMs = ms;
    }

    static unsigned long nextInRead = 0;
    static int inState = 0;
    static int inState2 = 0;
    if(millis()>nextInRead) {
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
