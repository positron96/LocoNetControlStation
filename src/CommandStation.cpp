#include "CommandStation.h"

CommandStation CS;

uint8_t CommandStation::LocoData::dccSpeedByte() {
    uint8_t ret = speed.getDCCByte(speedMode);
    if(speedMode==SpeedMode::S14) ret |= this->fn[0] << 4;
    return ret;
}