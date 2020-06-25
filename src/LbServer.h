#pragma once

#include <WiFi.h>
#include <ln_opc.h>
#include <LocoNet.h>

#include "LocoNetBus.h"


class LbServer: public Consumer<lnMsg> {

    LbServer(const uint16_t port) {
        server = WiFiServer(port);
    }

    void begin() {
        server.begin();
    }

    void end() {
        server.end();
    }


    void loop() {
        static char lbStr[LB_BUF_SIZE];
        static int lbPos = 0;
        static LocoNetMessageBuffer lbBuf;

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
                                    sendWifi(msg); // echo
                                    locoNet.parsePacket(msg); // decode callbacks
                                    LN_STATUS ret = locoNet.send(msg); 
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

    virtual void onMessage(const lnMsg& msg) {
        if(!cli) return;

        cli.print("RECEIVE ");
        uint8_t ln = lnPacketSize(&msg);
        for(int j=0; j<ln; j++) {
            cli.print(msg.data[j], HEX);
            cli.print(" ");
        }
        cli.println();
    }

private:

    WiFiServer server;
    WiFiClient cli;

};