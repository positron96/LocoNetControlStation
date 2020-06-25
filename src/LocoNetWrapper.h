#pragma once

#include <LocoNet.h>

#include "LocoNetBus.h"

class LocoNetWrapper: public LocoNetConsumer {
public:
    LocoNetWrapper(LocoNet * const ln, LocoNetBus * const bus) : ln(ln), bus(bus) {

        ln->onPacket(CALLBACK_FOR_ALL_OPCODES, [this](lnMsg *rxPacket) {
            bus->receive(*rxPacket, this);
        } );

    }

    void begin() { ln->begin(); }
    void end() { ln->end(); }

    virtual LN_STATUS onMessage(const lnMsg& msg) {
        lnMsg copy = msg;
        return ln->send(&copy);
    }

private:
    LocoNet * ln;
    LocoNetBus * bus;

};