#include <stdio.h>
#include <Arduino.h>

#include <DCC.h>

#include "CommandStation.h"

#include "LocoNetSlotManager.h"

#include "LocoNetSerial.h"
#include "LbServer.h"


#include <WiFi.h>
#include <ESPmDNS.h>

#include "WiThrottle.h"

LocoNetBus bus;

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17
//#include <LocoNetESP32UART.h>
//LocoNetESP32Uart locoNet(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 1, false, true, false );
//#include <LocoNetESP32Hybrid.h>
//LocoNetESP32Hybrid locoNet(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 1, false, true, 0 );
#include <LocoNetESP32.h>
LocoNetESP32 locoNet(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 0);
LocoNetDispatcher parser(&bus);


#define LBSERVER_TCP_PORT  1234
LbServer lbServer(LBSERVER_TCP_PORT, &bus);

//LocoNetSerial lSerial(&Serial, &bus);

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

WiThrottleServer withrottleServer;


#define IN_PIN 13

void setup() {

    Serial.begin(115200);
    Serial.println("Ultimate LocoNet Command Station");

    pinMode(IN_PIN, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    
    digitalWrite(PIN_LED, HIGH);

    locoNet.begin();
    //lSerial.begin();

    parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
        Serial.print("rx'd ");
        for(uint8_t x = 0; x < 4; x++) {
            uint8_t val = rxPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
            if(val < 16) {  Serial.print('0');  }

            Serial.print(val, HEX);
            Serial.print(' ');
        }
        Serial.print("\r\n");

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

    CS.setDccMain(&dccMain);
    CS.setDccProg(&dccProg);
    CS.setLocoNetBus(&bus);

    dccTimer.setMainChannel(&dccMain);
    dccTimer.setProgChannel(&dccProg);

    /*
    WifiManager wifiManager;
	wifiManager.setConfigPortalTimeout(300); // 5 min
	if ( !wifiManager.autoConnect("LocoNet WiFi") ) {
		delay(1000);
        Serial.print("Failed connection");
		ESP.restart();
	}
    */
    
    WiFi.begin("MelNet", "melnikov-network");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    MDNS.begin("ESP32Server");
	//MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName("OpenCommandStation");

    lbServer.begin();
    withrottleServer.begin(); 

    dccTimer.begin();
    //dccMain.begin();

    dccMain.setPower(true);

    digitalWrite(PIN_LED, HIGH);

}


void loop() {

    lbServer.loop();
    withrottleServer.loop();
    //lSerial.loop();
    
    /*
    static unsigned long nextDccMeter = 0;
    if(millis()>nextDccMeter) {
        uint16_t v = dccMain.readCurrent() ;
        if(v > 15) dccMain.setPower(false);
        nextDccMeter = millis()+20;
    }*/

    /*
    static unsigned long nextInRead = 0;
    static int inState = HIGH;
    if(millis()>nextInRead) {
        int v = digitalRead(IN_PIN);
        digitalWrite(PIN_LED, v);
        if(v!=inState) {
            Serial.printf( "reporting sensor %d\n", v==LOW) ;
            reportSensor(&bus, 10, v==LOW);// it's pulled up when idle.
            
            inState = v;
            //Serial.printf("errs: rx:%d,  tx:%d\n", locoNet.getRxStats()->rxErrors, locoNet.getTxStats()->txErrors );
            nextInRead = millis()+10;
        }
    }
    */

    /*
    if(Serial.available()) {
        Serial.read();
        DCCESP32Channel<10>::RegisterList *r = dccMain.getReg();
        Packet *p = r->currentSlot;
        while(r->currentSlot == p) {
            dccMain.timerFunc();
            delay(1);
            if(r->currentBit==1) break;
        }
    }
    */

    static long nextDump = millis();
    static float cur=0;
    if(millis()>nextDump) {
        nextDump = millis()+25;
        uint32_t v = dccMain.readCurrent();
        cur = cur*0.95 + v*0.05;
        Serial.printf("%d, %d\n", v, (int)cur );
        //dccMain.timerFunc();
    }

}