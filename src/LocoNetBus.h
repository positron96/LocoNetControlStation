/**
 * Takes LocoNet messages from several sources and resends them to 
 * many destinations. Used for resending messages between LocoNet, 
 * Loconet over USB-Serial and LbServer.
 * */

#pragma once

#include <etl/vector.h>


template < class Msg >
class Consumer {
    virtual void onMessage(const Msg& msg) = 0;
};

template <class Msg, const size_t MAX_CONSUMERS>
class LocoNetBus {
public:
    using MsgConsumer = Consumer<Msg>;

    void receive(const Msg &msg, MsgConsumer* sender = nullptr) {
        for(const auto & c: consumers) {
            if(c!=sender) c->onMessage(msg);
        }
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