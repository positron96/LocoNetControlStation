#include "command_station.hpp"

CommandStation CS;

uint8_t CommandStation::LocoData::dccSpeedByte() const {
    uint8_t ret = speed.getDCCByte(speedMode);
    if(speedMode==SpeedMode::S14) ret |= this->fn[0] << 4;
    return ret;
}


uint8_t CommandStation::findLocoSlot(LocoAddress addr) {
    auto it = locoSlot.find(addr);
    if(it != locoSlot.end() ) {
        return it->second;
    }
    return 0;
}

uint8_t CommandStation::locateFreeSlot() {
    if(!locoSlot.full()) {
        for(int i=0; i<MAX_SLOTS; i++) {
            if(!slots[i].allocated() ) {
                return i+1;
            }
        }
    }
    return 0;
}

void CommandStation::initLocoSlot(uint8_t slot, LocoAddress addr) {
    LocoData &_slot = getSlot(slot);
    _slot.addr = addr;
    _slot.dir = 1;
    _slot.fn = LocoData::Fns();
    _slot.refreshing = false;
    _slot.speed = LocoSpeed{};
    _slot.speedMode = SpeedMode::S128;
    _slot.owner = nullptr;
    _slot.resetWatchdog();
    locoSlot[addr] = slot;
}

/** @returns 0 if slot wasn't created (no space) */
uint8_t CommandStation::findOrAllocateLocoSlot(LocoAddress addr) {
    uint8_t slot = findLocoSlot(addr);
    if(slot==0) {
        slot = locateFreeSlot();
        if(slot!=0) initLocoSlot(slot, addr);
    }
    return slot;
}

void CommandStation::releaseLocoSlot(uint8_t slot) {
    auto it = locoSlot.find(getSlot(slot).addr);
    if(it == locoSlot.end()) {
        CS_DEBUGF("invalid slot");
        return;
    }
    releaseLocoSlot(it);
}

CommandStation::LocoSlotMap::iterator CommandStation::releaseLocoSlot(CommandStation::LocoSlotMap::iterator it) {
    uint8_t slot = it->second;
    CS_DEBUGF("Releasing slot %d", slot);
    setLocoSlotRefresh(slot, false);
    slots[slot-1].deallocate();
    return locoSlot.erase(it);
}

void CommandStation::setLocoSlotRefresh(uint8_t slot, bool refresh) {
    if(!isSlotSupported(slot)) { CS_DEBUGF("invalid slot"); return; }
    LocoData &dd = getSlot(slot);
    if(!dd.allocated()) { CS_DEBUGF("slot not allocated"); return; }
    if(dd.refreshing == refresh) return;
    CS_DEBUGF("slot %d refresh %c", slot, refresh?'Y':'N');
    dd.refreshing = refresh;

    dd.resetWatchdog();
    if(refresh) {
        // no need to do anything, DCC will start on setLocoSpeed/setLocoFn
    } else {
        // TODO: somehow send 0 speed to track
        dccMain->unloadSlot(dd.addr);
    }
}

void CommandStation::setSlotOwner(uint8_t slot, void* o) {
    LocoData &dd = getSlot(slot);
    dd.owner = o;
    dd.resetWatchdog();
}



void CommandStation::setLocoDir(uint8_t slot, uint8_t dir) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    if(dd.dir==dir) return;
    dd.dir = dir;
    if(dd.refreshing)
        dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir);
}

void CommandStation::setLocoSpeed(uint8_t slot, LocoSpeed spd) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    if(dd.speed == spd) return;
    dd.speed = spd;
    if(dd.refreshing)
        dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir);
}

void CommandStation::setLocoSpeedMode(uint8_t slot, SpeedMode mode) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    if(dd.speedMode == mode) return;
    dd.speedMode = mode;
    if(dd.refreshing)
        dccMain->sendThrottle(dd.addr, dd.speed, dd.speedMode, dd.dir > 0);
}



void CommandStation::setLocoFn(uint8_t slot, uint8_t fn, bool val) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    if(dd.fn[fn] == val) return;
    // CS_DEBUGF("slot %d FN%d=%d", slot, fn, val);

    dd.fn[fn] = val;
    using dcc::fn_group;
    fn_group fg = dcc::fn_to_group(fn);
    uint32_t ifn = dd.fn.value<uint32_t>();

    dccMain->sendFunctionGroup(dd.addr, fg, ifn);
}

void CommandStation::setLocoFns(uint8_t slot, dcc::fn_group fg, uint32_t vals) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    uint32_t current = dd.fn.value<uint32_t>();
    uint32_t mask = dcc::fn_group_mask(fg);
    vals = (current & ~mask) | (vals & mask);
    if(vals == current) return;
    // CS_DEBUGF("slot %d FN G%d = %d", slot, (int)fg, vals);

    dccMain->sendFunctionGroup(dd.addr, fg, vals);
    dd.fn = LocoData::Fns( vals );
}

void CommandStation::setLocoFns(uint8_t slot, uint32_t mask, uint32_t vals ) {
    LocoData &dd = getSlot(slot);
    dd.resetWatchdog();
    vals = vals & mask; // only take bits in mask, ignore others
    uint32_t current = dd.fn.value<uint32_t>();
    vals = (current & ~mask) | vals; // updated value for all bits
    uint32_t changed = current ^ vals;

    for(size_t g=0; g<dcc::FN_NUMBER; g++) {
        dcc::fn_group fg = static_cast<dcc::fn_group>(g);
        uint32_t gm = dcc::fn_group_mask(fg);
        // if required mask intersects function group mask
        //  and these bits differ from current value,
        // update bits (v=) and send function group
        if((mask & gm) != 0 && (changed & gm) != 0) {
            dccMain->sendFunctionGroup(dd.addr, fg, vals);
        }
    }

    dd.fn = LocoData::Fns( vals );
}




/**
 * Updates slots that have not been used for a long time (PURGE_TIMEOUT)
 */
void CommandStation::loop() {
    millis_t ms = millis();
    auto it = locoSlot.begin();
    while(it != locoSlot.end()) {
        bool slotRemoved = false;
        uint8_t slot = it->second;
        LocoData &dd = getSlot(slot);
        // Slots that are refreshed will stop refreshing after a timeout.
        // Those that have no owner expire faster.
        // Slots that aren't refreshed get removed after a second timeout.
        if(dd.refreshing) {
            if(( dd.hasOwner() && dd.wdt.timedOut()) ||
                (!dd.hasOwner() && dd.wdt.timedOut2())
            ) {
                CS_DEBUGF("slot %d %s stopping after %lds", slot,
                    !dd.hasOwner() ? "(without owner)" : "",
                    (ms - dd.wdt.getLastUpdate())/1000 );
                setLocoSlotRefresh(slot, false);
                dd.resetWatchdog();
            }
        } else {
            // non-refreshing slots get removed
            if(dd.wdt.timedOut()) {
                CS_DEBUGF("slot %d clearing after %lds", slot,
                    (ms - dd.wdt.getLastUpdate())/1000 );
                it = releaseLocoSlot(it);
                slotRemoved = true;
            }
        }
        if(!slotRemoved) it++;
    }
}




void CommandStation::loadTurnouts() {
    addTurnout({ dcc::AccessoryAddress::from9bit(1, 0), 0, TurnoutState::CLOSED });
    addTurnout({ dcc::AccessoryAddress::from9bit(1, 1), 1, TurnoutState::CLOSED });
    addTurnout({ dcc::AccessoryAddress::from9bit(1, 2), 2, TurnoutState::UNKNOWN });
    addTurnout({ dcc::AccessoryAddress::from9bit(1, 3), 3, TurnoutState::THROWN });
}

TurnoutState CommandStation::turnoutToggle(const dcc::AccessoryAddress addr, bool fromRoster) {
    return turnoutAction(addr, fromRoster, TurnoutAction::TOGGLE);
}

TurnoutState CommandStation::getTurnoutState(const dcc::AccessoryAddress addr) {
    auto t = turnoutData.find(addr);
    if(t != turnoutData.end() ) {
        return t->second.state;
    }
    return TurnoutState::UNKNOWN;
}

TurnoutState CommandStation::turnoutAction(dcc::AccessoryAddress addr, bool fromRoster, TurnoutAction action) {
    CS_DEBUGF("addr11=%d named=%d action=%d", addr.longAddr(), fromRoster, (int)action );

    TurnoutState newState = TurnoutState::THROWN;

    if(fromRoster) {
        auto t = turnoutData.find(addr);
        if(t != turnoutData.end() ) {
            if (action==TurnoutAction::TOGGLE) {
                newState = toggleTurnout(t->second.state);
            } else {  // throw or close
                newState = actionToState(action);
            }

            t->second.state = newState;
            addr = t->second.addr;
        } else {
            CS_DEBUGF("Did not find turnout in roster");
            return TurnoutState::UNKNOWN;
        }
    } else {
        if (action==TurnoutAction::TOGGLE) {
            CS_DEBUGF("Trying to toggle numeric turnout");
            newState = TurnoutState::THROWN;
        } else {  // throw or close
            newState = actionToState(action);
        }

        if(!turnoutData.full()) {
            // add turnout to roster
            addTurnout({addr, int(turnoutData.size()+1), newState});
            CS_DEBUGF("Added new turnout to roster: ID=%d, addr=%d", addr.get11bitAddr() );
        }
    }

    // send to DCC
    dccMain->sendAccessory(addr, newState==TurnoutState::THROWN);
    // send to LocoNet
    // FIXME: this is a dirty hack.
    // If LocoNet calls this function, it will be bounced back to bus.
    // Fortunately, right now, accessory commands from LocoNet do not get propagated to DCC
    // and this command is only called from WiThrottle code.
    if(locoNet!=nullptr) {
        LnMsg ttt = makeSwRec(addr.longAddr(), true, newState==TurnoutState::THROWN);
        locoNet->broadcast(ttt);
    }

    //sendDCCppCmd("a "+String(addr)+" "+sub+" "+int(newStat) );

    return newState;
}
