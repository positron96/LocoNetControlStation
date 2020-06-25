/**
 * Takes LocoNet messages from several sources and resends them to 
 * many destinations. Used for resending messages between LocoNet, 
 * Loconet over USB-Serial and LbServer.
 * */

#pragma once

#include <etl/vector.h>

#include <LocoNet.h>

template < class Msg, class Ret >
class Consumer {
    virtual Ret onMessage(const Msg& msg) = 0;
};

template <class Msg, class Ret, Ret okVal, const size_t MAX_CONSUMERS>
class Bus {
public:
    using MsgConsumer = Consumer<Msg>;

    Ret receive(const Msg &msg, MsgConsumer* sender = nullptr) {
        Ret ret = okVal;
        for(const auto & c: consumers) {
            if(c!=sender) {
                Ret v = c->onMessage(msg);
                if(v!=okVal) ret = v;
            }
        }
        return ret;
    }

    void addConsumer(MsgConsumer * c) {
        consumers.push_back(c);
    }

    void removeConsumer(MsgConsumer * c) {
        consumers.erase( std::remove(consumers.begin(), consumers.end(), c), consumers.end() );
    }

private:
    etl::vector<MsgConsumer*, MAX_CONSUMERS> consumers;
};


using LocoNetBus = Bus<lnMsg, LN_STATUS, LN_STATUS::LN_DONE, 5>;

using LocoNetConsumer = Consumer<lnMsg, LN_STATUS>;