#include "CommandStation.h"

CommandStation CS;

uint8_t CommandStation::LocoData::dccSpeedByte() {
    uint8_t ret;
    switch(speedMode) {
        case SpeedMode::S14: 
            ret = this->fn[0] << 4;
            if(speed128<2) ret |= speed128;
            else ret |= ((speed128-2)/9 ) + 2;
            break;
        case SpeedMode::S128:
            ret = speed128;
            break;
        case SpeedMode::S28:
            /// @see https://www.nmra.org/sites/default/files/s-92-2004-07.pdf table at line 57
            if(speed128<2) ret = speed128;
            else {
                ret = ((speed128-2)*2/9 ) + 4;
                ret = (ret & 1)<<4 | ( (ret>>1) & 0b1111 );
            }
            break;
        default: ret = 0;
    }
    return ret;
}