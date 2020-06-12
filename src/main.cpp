// LocoNet Packet Monitor
// Demonstrates the use of the:
//
//   LocoNet.processSwitchSensorMessage(LnPacket)
//
//   Requires the ESP8266 and ESP32 SSD1306 OLED
//   library by Daniel Eichhorn (@squix78) and
//   Fabrice Weinberg (@FWeinb)
//   https://github.com/squix78/esp8266-oled-ssd1306
//
// function and examples of each of the notifyXXXXXXX user call-back functions
#include <stdio.h>
#include <Arduino.h>

//#include <LocoNetESP32UART.h>
//LocoNetESP32Uart locoNet(16, 15, 1, false, true, false, tskNO_AFFINITY);
#include <LocoNetESP32.h>
LocoNetESP32 locoNet;

#include <WiFi.h>
#include <ESPmDNS.h>

#define LOCONET_TCP_PORT  1234
WiFiServer lnServer(LOCONET_TCP_PORT);
WiFiClient lnClient;

#define IN_PIN 25
int inState = HIGH;
unsigned long nextInRead = 0;


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

        if(lnClient) {
            lnClient.write(rxPacket->data, lnPacketSize(rxPacket));
        }
    });
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


/*
    WifiManager wifiManager;
	wifiManager.setConfigPortalTimeout(300); // 5 min
	if ( !wifiManager.autoConnect("LocoNet WiFi") ) {
		delay(1000);
        Serial.print("Failed connection");
		ESP.restart();
	}*/

    WiFi.begin("MelNet", "melnikov-network");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    lnServer.begin();

    MDNS.begin("ESP32Server");
	//MDNS.addService("withrottle","tcp", WTServer_Port);
	//MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName("Loconet Network Interface");

}

void sendWifi(lnMsg *msg) {
    if(!lnClient) return;

    lnClient.print("RECEIVE ");
    uint8_t ln = lnPacketSize(msg);
    for(int j=0; j<ln; j++) {
        lnClient.print(msg->data[j], HEX);
        lnClient.print(" ");
    }
    lnClient.print("\r\n");
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

    static char lbStr[LB_BUF_SIZE];
    static int lbPos=0;
    static LocoNetMessageBuffer lbBuf;

    if (!lnClient) lnClient = lnServer.available();
	if (lnClient) {
		while(lnClient.available()>0) {
            char v = lnClient.read();
            if(v=='\r') continue;

            lbStr[lbPos] = v;
            if(v=='\n') {
                lbStr[lbPos] = ' '; lbStr[lbPos+1]=0;
                if(strncmp("SEND ", lbStr, 5)==0) {
                    for(uint8_t i=5; i<=lbPos; i++) {
                        if(lbStr[i]==' ') {
                            uint8_t val = FROM_HEX(lbStr[i-2])<<4 | FROM_HEX(lbStr[i-1]);
                            lnMsg * msg = lbBuf.addByte(val);
                            if(msg!=nullptr) {
                                sendWifi(msg); // echo
                                locoNet.processPacket(msg);
                                locoNet.send(msg); 
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
}