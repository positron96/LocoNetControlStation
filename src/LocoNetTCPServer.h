/**
 * @see http://loconetovertcp.sourceforge.net/Protocol/LoconetOverTcp.html
 */
#pragma once

#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>

#include <ln_opc.h>
#include <LocoNet2.h>
#include <etl/queue.h>
#include <etl/set.h>


#define LB_DEBUG

#ifdef LB_DEBUG
#define LB_LOGI(format, ...)  do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__); }while(0)
#define LB_LOGD(format, ...)  //do{ log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__); }while(0)
#else
#define LB_LOGI(...)
#define LB_LOGD(...)
#endif

#define FROM_HEX(c) (   ((c)>'9') ? ((c) & ~0x20)-'A'+0xA : ((c)-'0')   )

constexpr uint16_t LBSERVER_DEFAULT_TCP_PORT = 1234;

class LbServer: public LocoNetConsumer {

public:

    LbServer(const uint16_t port, LocoNetBus * const bus): bus(bus), port(port), server(port) {
        //server = Server(port);
        bus->addConsumer(this);

        server.onClient( [this](void*, AsyncClient* cli ) {
            if(clients.full()) {
                LB_LOGI("onConnect: Not accepting client: %s (full)", cli->remoteIP().toString().c_str() );
                cli->close();
                return;
            }
            cli->setKeepAlive(10000, 2);
            clients.insert(cli);
            LB_LOGI("onConnect: New client(%X): %s", (intptr_t)cli, cli->remoteIP().toString().c_str() );
            cli->write("VERSION ESP32 WiFi 0.1");

            cli->onDisconnect([this](void*, AsyncClient* cli) {
                LB_LOGI("onDisconnect: Client(%X) disconnected", (intptr_t)cli );
                clients.erase(cli);
            });

            cli->onData( [this](void*, AsyncClient* cli, void *data, size_t len) {
                for(size_t i=0; i<len; i++)
                    processRx( ((char*)data)[i], cli);
            });

            cli->onError([this](void*, AsyncClient* cli, int8_t err) {
                LB_LOGI("onError(%X): %d", (intptr_t)cli, err);
            });
            cli->onTimeout([this](void*, AsyncClient* cli, uint32_t time) {
                LB_LOGI("onTimeout(%X): %d", (intptr_t)cli, time);
                cli->close();
            });

        }, nullptr);
    }

    void begin() {
        MDNS.addService("lbserver","tcp", port);
        server.begin();
    }

    void end() {
        server.end();
    }


    void loop() {
        if (!clients.empty()) {
            while(!txQueue.empty()) {
                sendMessage(txQueue.front());
                txQueue.pop();
            }
        }
    }

    LN_STATUS onMessage(const lnMsg& msg) override {
        if( !txQueue.full() && !clients.empty() ) txQueue.push(msg);
        return LN_IDLE;
    }

    String getInfo() const {
        String v;
        if (clients.empty()) {
            v = "No clients";
        } else {
            for(const auto &cli: clients) {
                v += " " + cli->remoteIP().toString() + "\n";
            }
        }
        return v;
    }

private:
    constexpr static size_t MAX_CLIENTS = 5;

    LocoNetBus *bus;

    uint16_t port;

    AsyncServer server;
    etl::set<AsyncClient*, MAX_CLIENTS> clients;

    etl::queue<LnMsg, MAX_CLIENTS> txQueue;

    LocoNetMessageBuffer lbBuf;
    const static int LB_BUF_SIZE = 100;
    // FIXME: there is only one buffer, several clients writing at the same time will produce a mess!
    //  Realistically, only one PC connects to it.
    char lbStr[LB_BUF_SIZE];
    int lbPos = 0;

    void processRx(char v, AsyncClient *cli) {
        lbStr[lbPos] = v;
        if(v=='\n' || v=='\r') {
            if(lbPos==0) return; // deal with CRLF ending
            lbStr[lbPos] = ' '; lbStr[lbPos+1]=0;
            LB_LOGD("Processing string '%s'", lbStr);
            if(strncmp("SEND ", lbStr, 5)==0) {
                for(uint8_t i=5; i<=lbPos; i++) {
                    if(lbStr[i]==' ') {
                        uint8_t val = FROM_HEX(lbStr[i-2])<<4 | FROM_HEX(lbStr[i-1]);
                        LB_LOGD("LbServer::loop adding byte %02x from chars '%c' '%c' (pos %d)", val, lbStr[i-2], lbStr[i-1], i);
                        LnMsg *msg = lbBuf.addByte(val);
                        if(msg!=nullptr) {

                            sendMessage(*msg); // echo
                            LN_STATUS ret = bus->broadcast(*msg, this);

                            if(ret==LN_IDLE) cli->write("SENT OK\n"); else
                            if(ret==LN_RETRY_ERROR) cli->write("SENT ERROR LN_RETRY_ERROR\n"); else
                            cli->write("SENT ERROR generic\n");
                            break;
                        }
                    }
                }
            } else {
                LB_LOGI("Got line but it's not SEND: '%s'", lbStr);
            }
            lbPos=0;
        } else {
            lbPos++;
        }
    }

    void sendMessage(const LnMsg &msg) {

        char buf[LB_BUF_SIZE] = "RECEIVE";
        uint len = strlen(buf);
        uint8_t nBytes = msg.length();
        for(int j=0; j<nBytes; j++) {
            len += snprintf(buf+len, LB_BUF_SIZE-len, " %02X", msg.data[j]);
        }
        LB_LOGD("Transmitting '%s'", buf );
        len += snprintf(buf+len, LB_BUF_SIZE-len, "\n");
        for (auto cli: clients) {
            size_t written_len = cli->write(buf);
            if(len != written_len) {
                LB_LOGI("cli(%x) tx length mismatch: expected %d, actual %d", (intptr_t)cli, len, len);
            }
        }
    }

};
