#pragma once

#include <etl/observer.h>

namespace dcc {

    class BaseChannel; // fwd declaration

    struct PowerEvent {

        enum class Reason: uint8_t {
            Normal,
            Overcurrent
        };

        bool state;
        Reason reason;
        BaseChannel *channel;
    };

    using PowerObserver = etl::observer<const PowerEvent&>;
}
