#pragma once

#include <etl/message_bus.h>

enum {
    PowerEventId = 0
};

class PowerEventMsg: public etl::message<PowerEventId> {
public:
    dcc::PowerEvent event;
    PowerEventMsg(const dcc::PowerEvent &e): event{e} {}
};

/** Main event bus for application.
 * Passes through all messages like power changes, users connecting/disconnecting, etc.
 * Interested parties can subscribe to relevant events.
 */
inline  etl::message_bus<20> event_bus;


inline etl::imessage_bus &get_event_bus() {
    return event_bus;
}
