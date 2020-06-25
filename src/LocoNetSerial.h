#pragma once

#include "LocoNetBus.h"

#include <LocoNet.h>
#include <Stream.h>

class LocoNetSerial: public Consumer<lnMsg> {

public:
    LocoNetSerial(Stream * const str) : stream(str) {}

    void begin() {}

    void end() {}

    void loop() {
        if(stream->available() ) {
            uint8_t inByte = stream->read();
            lnMsg *msg = buf.addByte(inByte);
            if(msg != nullptr) {
                // broadcast msg
            }
        }
    }

    virtual void onMessage(const lnMsg& msg) {
        uint8_t ln = lnPacketSize(&msg);
        for(int j=0; j<ln; j++) {
            stream->write(msg.data[j]);
        }
    }

private:
    Stream *stream;

    LocoNetMessageBuffer buf;

    

};