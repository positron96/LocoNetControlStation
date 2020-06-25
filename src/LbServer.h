#pragma once

#include <WiFi.h>
#include <ln_opc.h>
#include <LocoNet.h>

#include "LocoNetBus.h"

#define FROM_HEX(c) (   ((c)>'9') ? ((c) &~ 0x20)-'A'+0xA : ((c)-'0')   )

class LbServer: public LocoNetConsumer {

public:

    LbServer(const uint16_t port, LocoNetBus * const bus): bus(bus) {
        server = WiFiServer(port);
    }

    void begin() {
        server.begin();
    }

    void end() {
        server.end();
    }


    void loop() {

        if (!cli) {
            cli = server.available();
            if(cli) { cli.print("VERSION "); cli.println("ESP32 WiFi 0.1"); }
        }
        if (cli) {
            while(cli.available()>0) {
                char v = cli.read();
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
                                    
                                    onMessage(*msg); // echo
                                    
                                    LN_STATUS ret = bus->receive(*msg, this);

                                    if(ret==LN_DONE) cli.println("SENT OK"); else
                                    if(ret==LN_RETRY_ERROR) cli.println("SENT ERROR LN_RETRY_ERROR");
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

    virtual LN_STATUS onMessage(const lnMsg& msg) {
        if(!cli) return LN_DONE;

        cli.print("RECEIVE ");
        uint8_t ln = msg.length();
        for(int j=0; j<ln; j++) {
            cli.print(msg.data[j], HEX);
            cli.print(" ");
        }
        cli.println();

        return LN_DONE;
    }

private:

    LocoNetBus *bus;

    WiFiServer server;
    WiFiClient cli;

    LocoNetMessageBuffer lbBuf;
    const static int LB_BUF_SIZE = 100;
    char lbStr[LB_BUF_SIZE];
    int lbPos = 0;

};