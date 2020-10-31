#pragma once

#include <WiFi.h>
#include <ln_opc.h>
#include <LocoNet.h>
#include <etl/queue.h>

#define LB_DEBUG

#ifdef LB_DEBUG
#define LB_DEBUGF(...)  do{ Serial.printf(__VA_ARGS__); }while(0)
#else
#define LB_DEBUGF(...)
#endif


#define FROM_HEX(c) (   ((c)>'9') ? ((c) &~ 0x20)-'A'+0xA : ((c)-'0')   )

class LbServer: public LocoNetConsumer {

public:

    LbServer(const uint16_t port, LocoNetBus * const bus): bus(bus) {
        server = WiFiServer(port);
        bus->addConsumer(this);
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
            size_t a = cli.available();
            while(a-- >0) {
                char v = cli.read();
                if(v=='\r') continue;

                lbStr[lbPos] = v;
                if(v=='\n') {
                    lbStr[lbPos] = ' '; lbStr[lbPos+1]=0;
                    if(strncmp("SEND ", lbStr, 5)==0) {
                        for(uint8_t i=5; i<=lbPos; i++) {
                            if(lbStr[i]==' ') {
                                uint8_t val = FROM_HEX(lbStr[i-2])<<4 | FROM_HEX(lbStr[i-1]);
                                //LB_DEBUGF("LbServer::loop adding byte %02x\n", val);
                                LnMsg *msg = lbBuf.addByte(val);
                                if(msg!=nullptr) {
                                    
                                    sendMessage(*msg); // echo

                                    LB_DEBUGF("LbServer::loop got message ");
                                    for(int i=0; i<msg->length(); i++) { LB_DEBUGF("%02x ", msg->data[i]); }
                                    LB_DEBUGF("\n");
                                    
                                    LN_STATUS ret = bus->broadcast(*msg, this);

                                    if(ret==LN_DONE) cli.println("SENT OK"); else
                                    if(ret==LN_RETRY_ERROR) cli.println("SENT ERROR LN_RETRY_ERROR");
                                }
                            }
                        }
                    } else {
                        LB_DEBUGF("LbServer::loop: Got line but it's not SEND:");
                        for(int i=0; i<lbPos; i++) { LB_DEBUGF("%02x ", lbStr[i]); }
                        LB_DEBUGF("\n");
                    }
                    lbPos=0;
                } else {
                    lbPos++;
                }

            }

            while(!txQueue.empty()) {
                sendMessage(txQueue.front());
                txQueue.pop();
            }

        }
    }

    LN_STATUS onMessage(const lnMsg& msg) override {
        txQueue.push(msg);
        return LN_DONE;
    }

private:

    LocoNetBus *bus;

    WiFiServer server;
    WiFiClient cli;

    etl::queue<LnMsg, 5> txQueue;

    LocoNetMessageBuffer lbBuf;
    const static int LB_BUF_SIZE = 100;
    char lbStr[LB_BUF_SIZE];
    int lbPos = 0;

    void sendMessage(const LnMsg &msg) {
        if(!cli) return;
        char ttt[100] = "RECEIVE";
        uint t = strlen(ttt);
        uint8_t ln = msg.length();
        for(int j=0; j<ln; j++) {
            t += sprintf(ttt+t, " %02X", msg.data[j]);
        }
        LB_DEBUGF("LbServer::sendMessage: Transmitting '%s'\n", ttt );
        cli.write(ttt);
        cli.println();

        /*LB_DEBUGF("LbServer::onMessage: Transmitting %d bytes\n", msg.length() );
        cli.print("RECEIVE ");
        uint8_t ln = msg.length();
        for(int j=0; j<ln; j++) {
            if(msg.data[j]<=0xF) cli.print('0');
            cli.print(msg.data[j], HEX);
            cli.print(" ");
        }
        cli.println();*/

    }

};