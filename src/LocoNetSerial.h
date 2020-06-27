#pragma once

#include <LocoNet.h>
#include <Stream.h>

class LocoNetSerial: public LocoNetConsumer {

public:
    LocoNetSerial(Stream * const str, LocoNetBus * const bus) : stream(str), bus(bus) {
        bus->addConsumer(this);
    }

    void begin() {}

    void end() {}

    void loop() {
        if(stream->available() ) {
            uint8_t inByte = stream->read();
            lnMsg *msg = buf.addByte(inByte);
            if(msg != nullptr) {
                bus->broadcast(*msg, this);
            }
        }
    }

    virtual LN_STATUS onMessage(const lnMsg& msg) {
        uint8_t ln = msg.length();
        for(int j=0; j<ln; j++) {
            stream->write(msg.data[j]);
        }
        return LN_DONE;
    }

private:
    Stream *stream;
    LocoNetBus * bus;

    LocoNetMessageBuffer buf;

};