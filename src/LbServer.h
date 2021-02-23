#pragma once

#include <WiFi.h>
#include <ln_opc.h>
#include <LocoNet.h>
#include <etl/queue.h>

#define LB_DEBUG

#ifdef LB_DEBUG
#define LB_LOGI(format, ...)  do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__); }while(0)
#define LB_LOGD(format, ...)  //do{ log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__); }while(0)
#else
#define LB_LOGI(...)
#define LB_LOGD(...) 
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
            if(cli) { 
                LB_LOGI("New client: %s", cli.remoteIP().toString().c_str() );
                cli.print("VERSION "); cli.println("ESP32 WiFi 0.1"); 
            }
        }

        if (cli) {
            while(cli.available() > 0) {
                int v = cli.read();
                if(v==-1) return;
                if(v=='\r') continue;

                lbStr[lbPos] = v;
                if(v=='\n') {
                    lbStr[lbPos] = ' '; lbStr[lbPos+1]=0;
                    LB_LOGD("Processing string '%s'", lbStr);
                    if(strncmp("SEND ", lbStr, 5)==0) {
                        for(uint8_t i=5; i<=lbPos; i++) {
                            if(lbStr[i]==' ') {
                                uint8_t val = FROM_HEX(lbStr[i-2])<<4 | FROM_HEX(lbStr[i-1]);
                                LB_LOGD("LbServer::loop adding byte %02x", val);
                                LnMsg *msg = lbBuf.addByte(val);
                                if(msg!=nullptr) {
                                    
                                    sendMessage(*msg); // echo

                                    LN_STATUS ret = bus->broadcast(*msg, this);

                                    if(ret==LN_DONE) cli.println("SENT OK"); else
                                    if(ret==LN_RETRY_ERROR) cli.println("SENT ERROR LN_RETRY_ERROR");
                                    break;
                                }
                            }
                        }
                    } else {
                        LB_LOGI("Got line but it's not SEND: %s", lbStr);
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
        char ttt[LB_BUF_SIZE] = "RECEIVE";
        uint t = strlen(ttt);
        uint8_t ln = msg.length();
        for(int j=0; j<ln; j++) {
            t += sprintf(ttt+t, " %02X", msg.data[j]);
        }
        LB_LOGD("Transmitting '%s'", ttt );
        t += sprintf(ttt+t, "\n");
        cli.write(ttt);
    }

};