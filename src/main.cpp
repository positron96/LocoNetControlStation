#include <stdio.h>
#include <Arduino.h>

#include <DCC.h>

#include "CommandStation.h"

#include <LocoNetESP32.h>
#include "LocoNetBus.h"
#include "LocoNetSlotManager.h"

#include "LocoNetSerial.h"
#include "LbServer.h"

//#include "WiThrottle.h"

LocoNetBus bus;

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 15
//#include <LocoNetESP32UART.h>
//LocoNetESP32Uart locoNet(16, 15, 1, false, true, false, tskNO_AFFINITY);
LocoNetESP32 locoNet(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 0);

#define DCC_PIN 19
#define DCC_PIN_EN 23
#define DCC_PIN_SENSE 35


#define LBSERVER_TCP_PORT  1234
LbServer lbServer(1234, &bus);

LocoNetSerial lSerial(&Serial, &bus);

LocoNetSlotManager slotMan(&bus);

DCCESP32Channel dccMain(DCC_PIN, DCC_PIN_EN, DCC_PIN_SENSE, true);
DCCESP32SignalGenerator dcc(1); //timer1

//WiThrottleServer withrottleServer;

/*
#include <WiFi.h>
#include <ESPmDNS.h>


*/

#define IN_PIN 25
int inState = HIGH;
unsigned long nextInRead = 0;
unsigned long nextDccMeter = 0;


void setup() {
  
    pinMode(IN_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    Serial.println("LocoNet");

    locoNet.begin();
    lSerial.begin();

/*
    locoNet.onPacket(CALLBACK_FOR_ALL_OPCODES, [](lnMsg *rxPacket) {
        Serial.print("rx'd ");
        for(uint8_t x = 0; x < 4; x++) {
            uint8_t val = rxPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
            if(val < 16) {  Serial.print('0');  }

            Serial.print(val, HEX);
            Serial.print(' ');
        }
        Serial.print("\r\n");

        switch(rxPacket->data[0]) {
            case OPC_GPON: break;
            case OPC_GPOFF: break;
        }

        sendWifi(rxPacket);
    });*/

/*
    locoNet.onSwitchRequest([](uint16_t address, bool output, bool direction) {
        Serial.print("Switch Request: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
    });
    locoNet.onSwitchReport([](uint16_t address, bool state, bool sensor) {
        Serial.print("Switch/Sensor Report: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(sensor ? "Switch" : "Aux");
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });
    locoNet.onSensorChange([](uint16_t address, bool state) {
        Serial.print("Sensor: ");
        Serial.print(address, DEC);
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
    });*/

    CS.setDccMain(&dccMain);

    dcc.setMainChannel(&dccMain);
    
    dcc.begin();

    dccMain.setPower(true);

    /*
    WifiManager wifiManager;
	wifiManager.setConfigPortalTimeout(300); // 5 min
	if ( !wifiManager.autoConnect("LocoNet WiFi") ) {
		delay(1000);
        Serial.print("Failed connection");
		ESP.restart();
	}
    */

/*
    
    WiFi.begin("MelNet", "melnikov-network");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    lbServer.begin();

    MDNS.begin("ESP32Server");
	//MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName("OpenCommandStation");

    withrottleServer.begin();
    */

}


void loop() {

    lbServer.loop();
    lSerial.loop();
    
    if(millis()>nextDccMeter) {
        uint16_t v = dccMain.readCurrent() ;
        if(v > 15) dccMain.setPower(false);
        nextDccMeter = millis()+20;
    }
/*
    if(millis()>nextInRead) {
        int v = digitalRead(IN_PIN);
        if(v!=inState) {
            Serial.printf( "reporting sensor %d\n", v==LOW) ;
            locoNet.reportSensor(10, v==LOW);// it's pulled up when idle.
            
            inState = v;
            Serial.printf("errs: rx:%d,  tx:%d\n", locoNet.getRxStats()->rxErrors, locoNet.getTxStats()->txErrors );
            nextInRead = millis()+10;
            delay(1);
        }
    }*/

}