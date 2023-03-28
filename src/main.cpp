#include <DCC.h>

#include "CommandStation.h"

#include "LocoNetSlotManager.h"

#include "LocoNetSerial.h"
#include "LbServer.h"

#include "WiThrottle.h"

#include <LocoNetESP32.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>

#include <Arduino.h>

#include <etl/callback_timer_atomic.h>
#include <stdio.h>
#include <atomic>

LocoNetBus bus;

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17
//#include <LocoNetESP32UART.h>
//LocoNetESP32Uart locoNetPhy(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 1, false, true, false );
//#include <LocoNetESP32Hybrid.h>
//LocoNetESP32Hybrid locoNetPhy(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 1, false, true, 0 );
//#include <LocoNetESP32.h>
LocoNetESP32 locoNetPhy(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 0);
LocoNetDispatcher parser(&bus);


#define LBSERVER_TCP_PORT  1234
LbServer lbServer(LBSERVER_TCP_PORT, &bus);

//LocoNetSerial lSerial(&Serial, &bus);

#define CS_NAME "ESP32CommandStation"

#define PIN_LED  22

#define DCC_MAIN_PIN 25
#define DCC_MAIN_PIN_EN 32
#define DCC_MAIN_PIN_SENSE 36
#define DCC_PROG_PIN 26
#define DCC_PROG_PIN_EN 33
#define DCC_PROG_PIN_SENSE 39

DCCESP32Channel<10> dccMain(DCC_MAIN_PIN, DCC_MAIN_PIN_EN, DCC_MAIN_PIN_SENSE);
DCCESP32Channel<2> dccProg(DCC_PROG_PIN, DCC_PROG_PIN_EN, DCC_PROG_PIN_SENSE);
DCCESP32SignalGenerator dccTimer(1); //timer1

LocoNetSlotManager slotMan(&bus);

WiThrottleServer withrottleServer(WiThrottleServer::DEF_PORT, CS_NAME);

#define PIN_BT 13
#define PIN_BT2 15

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 500;
constexpr int LED_INTL_CONFIG2 = 250;

uint8_t ledVal;

void ledStartBlinking(uint32_t ms=0, uint8_t val=1);
void ledStop();
void ledUpdate();

void checkCurrent();

using TimerType = etl::callback_timer_atomic<2, std::atomic_uint>;
TimerType timerController;
etl::timer::id::type ledTimer;
etl::timer::id::type checkCurrentTimer;

void setup() {

    Serial.begin(115200);
    Serial.println(CS_NAME);

    pinMode(PIN_BT, INPUT_PULLUP);
    pinMode(PIN_BT2, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    digitalWrite(PIN_LED, LOW);

    locoNetPhy.begin();
    //lSerial.begin();


    parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
        char tmp[100];
        formatMsg(*rxPacket, tmp, sizeof(tmp));
        Serial.printf("onPacket: %s\n", tmp);
    });


    parser.onSwitchRequest([](uint16_t address, bool output, bool direction) {
        Serial.print("Switch Request: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
    });
    parser.onSwitchReport([](uint16_t address, bool state, bool sensor) {
        Serial.print("Switch/Sensor Report: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(sensor ? "Switch" : "Aux");
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });
    parser.onSensorChange([](uint16_t address, bool state) {
        Serial.print("Sensor: ");
        Serial.print(address, DEC);
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });

    dccTimer.setMainChannel(&dccMain);
    dccTimer.setProgChannel(&dccProg);

    CS.setDccMain(&dccMain);
    CS.setDccProg(&dccProg);
    CS.setLocoNetBus(&bus);


    bool bt = digitalRead(PIN_BT)==0;
    if(bt) {
        // start AP
        WiFi.persistent(false);
        //WiFi.softAPConfig(IPAddress{192,168,1,0}, IPAddress{192,168,1,1}, IPAddress{255,255,255,0});
        WiFi.softAP(CS_NAME " AP", "");
        Serial.println("");
        Serial.println("WiFi AP started.");
        Serial.println("IP address: ");
        Serial.println(WiFi.softAPIP());
    } else {
        WiFiManager wifiManager;
        wifiManager.setConfigPortalTimeout(300); // 5 min
        if ( !wifiManager.autoConnect(CS_NAME " AP") ) {
            delay(1000);
            Serial.print("Failed connection");
            ESP.restart();
        }
        Serial.println("");
        Serial.println("WiFi connected.");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }

    MDNS.begin("ESP32Server");
	//MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName(CS_NAME);

    dccTimer.begin();

    dccMain.setPower(true);
    dccProg.setPower(true);

    lbServer.begin();
    withrottleServer.begin();

    ledTimer = timerController.register_timer(
        TimerType::callback_type::create<ledUpdate>(),
        LED_INTL_NORMAL, true);
    timerController.enable(true);

    checkCurrentTimer = timerController.register_timer(
        TimerType::callback_type::create<checkCurrent>(),
        1, true);
    timerController.start(checkCurrentTimer);

    ledStartBlinking();

}


void loop() {

    lbServer.loop();
    withrottleServer.loop();
    CS.loop();
    //lSerial.loop();

    /*
    static unsigned long nextDccMeter = 0;
    if(millis()>nextDccMeter) {
        uint16_t v = dccMain.readCurrentAdc() ;
        if(v > 15) dccMain.setPower(false);
        nextDccMeter = millis()+20;
    }*/
    uint32_t ms = millis();
    static uint32_t lastMs = 0;
    if(timerController.tick(ms - lastMs)) {
        lastMs = ms;
    }

    static unsigned long nextInRead = 0;
    static int inState = 0;
    static int inState2 = 0;
    if(millis()>nextInRead) {
        int v = 1-digitalRead(PIN_BT);
        if(v!=inState) {

            //bool r = dccMain.verifyCVByteProg(1, v==1 ? 13 : 14);
            //int r = dccMain.readCVProg(1);

            //Serial.printf("main(): readCVProg: %d\n", r);
            Serial.printf( "reporting sensor %d\n", v==HIGH) ;
            reportSensor(&bus, 1, v==HIGH);
            Serial.printf("errs: rx:%d,  tx:%d\n", locoNetPhy.getRxStats()->rxErrors, locoNetPhy.getTxStats()->txErrors );
        }
        inState = v;

        v = 1-digitalRead(PIN_BT2);
        if(v!=inState2) {
            if(dccMain.getPower()) {
                dccMain.setPower(false);
                dccProg.setPower(false);
            } else {
                dccMain.setPower(true);
                dccProg.setPower(true);
            }
        }
        inState2 = v;

        nextInRead = millis() + 10;
    }

    /*
    if(Serial.available()) {
        Serial.read();
        DCCESP32Channel<10>::RegisterList *r = dccMain.getReg();
        Packet *p = r->currentPacket;
        while(r->currentPacket == p) {
            dccMain.timerFunc();
            delay(1);
            if(r->currentBit==1) break;
        }
    }
    */

}

void checkCurrent() {
    bool oc = dccMain.checkOvercurrent();
    if(!oc) {
        withrottleServer.notifyPowerStatus();
        Serial.println("Overcurrent on main");
    }

    oc = dccProg.checkOvercurrent();
    if(!oc) {
        Serial.println("Overcurrent on prog");
    }

    //uint32_t v = dccMain.readCurrentAdc();
    //cur = cur*0.9 + v*0.1;
    //if(v!=0)Serial.printf("%d, %d\n", v, (int)cur );
    //dccMain.timerFunc();
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