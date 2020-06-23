#include <stdio.h>
#include <Arduino.h>

#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 15
//#include <LocoNetESP32UART.h>
//LocoNetESP32Uart locoNet(16, 15, 1, false, true, false, tskNO_AFFINITY);
#include <LocoNetESP32.h>
LocoNetESP32 locoNet(LOCONET_PIN_RX, LOCONET_PIN_TX, 0);

#include <LocoNetSlotManager.h>
LocoNetSlotManager slotMan(&locoNet);

#define DCC_PIN 19
#define DCC_PIN_EN 23
#define DCC_PIN_SENSE 35
#include <DCC.h>
DCCESP32Channel dccMain(DCC_PIN, DCC_PIN_EN, DCC_PIN_SENSE, true);
DCCESP32SignalGenerator dcc(1);

#include "CommandStation.h"

#include "WiThrottle.h"
WiThrottleServer withrottleServer;

/*
#include <WiFi.h>
#include <ESPmDNS.h>

#define LBSERVER_TCP_PORT  1234
WiFiServer lbServer(LBSERVER_TCP_PORT);
WiFiClient lbClient;
*/
void sendWifi(lnMsg *msg);

#define IN_PIN 25
int inState = HIGH;
unsigned long nextInRead = 0;

void send(lnMsg *msg) {
    locoNet.send(msg);
    sendWifi(msg);
}

void setup() {
  
    pinMode(IN_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    Serial.println("LocoNet");

    // First initialize the LocoNet interface
    locoNet.begin();
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
    });

    slotMan.registerCallbacks();
    slotMan.setDccMainChannel(&dccMain);

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
    });

    dcc.setMainChannel(&dccMain);
    //dcc.begin();

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
	//MDNS.addService("withrottle","tcp", WTServer_Port);
	//MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName("Loconet Network Interface");
    */

}

void sendWifi(lnMsg *msg) {
    /*
    if(!lbClient) return;

    lbClient.print("RECEIVE ");
    uint8_t ln = lnPacketSize(msg);
    for(int j=0; j<ln; j++) {
        lbClient.print(msg->data[j], HEX);
        lbClient.print(" ");
    }
    lbClient.println();
    */
}

#define LB_BUF_SIZE 100
#define FROM_HEX(c) (   ((c)>'9') ? ((c) &~ 0x20)-'A'+0xA : ((c)-'0')   )

void loop() {
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
    }

/*
    static char lbStr[LB_BUF_SIZE];
    static int lbPos=0;
    static LocoNetMessageBuffer lbBuf;

    if (!lbClient) {
        lbClient = lbServer.available();
        if(lbClient) { lbClient.print("VERSION "); lbClient.println("ESP32 WiFi 0.1"); }
    }
	if (lbClient) {
		while(lbClient.available()>0) {
            char v = lbClient.read();
            if(v=='\r') continue;

            lbStr[lbPos] = v;
            if(v=='\n') {
                lbStr[lbPos] = ' '; lbStr[lbPos+1]=0;
                if(strncmp("SEND ", lbStr, 5)==0) {
                    for(uint8_t i=5; i<=lbPos; i++) {
                        if(lbStr[i]==' ') {
                            uint8_t val = FROM_HEX(lbStr[i-2])<<4 | FROM_HEX(lbStr[i-1]);
                            lnMsg *msg = lbBuf.addByte(val);
                            if(msg!=nullptr) {
                                sendWifi(msg); // echo
                                locoNet.parsePacket(msg); // decode callbacks
                                LN_STATUS ret = locoNet.send(msg); 
                                if(ret==LN_DONE) lbClient.println("SENT OK"); else
                                if(ret==LN_RETRY_ERROR) lbClient.println("SENT ERROR LN_RETRY_ERROR");
                            }
                        }
                    }
                } else {
                    Serial.println("got line but it's not SEND:");
                    for(int i=0; i<lbPos; i++) { Serial.print(lbStr[i], HEX); Serial.print(" "); }
                    Serial.println();
                }
                lbPos=0;
            } else {
                lbPos++;
            }

		}

	}
    */
}