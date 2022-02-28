#include "LocoSpeed.h"


    uint8_t LocoSpeed::dccSpeedByte(SpeedMode speedMode) const {
        uint8_t ret;
        switch(speedMode) {
            case SpeedMode::S14: 
                if(speed128<2) ret = speed128;
                else ret = ((speed128-2)/9 ) + 2;
                break;
            case SpeedMode::S128:
                ret = speed128;
                break;
            case SpeedMode::S28:
                /// @see https://www.nmra.org/sites/default/files/s-92-2004-07.pdf table at line 57
                if(speed128<=1) ret = speed128;
                else {
                    ret = ((speed128-2)*2/9 ) + 4;
                    ret = (ret & 1)<<4 | (ret & 0b11110)>>1; // bit swap
                }
                break;
            default: ret = 0;
        }
        return ret;
    }

    uint8_t LocoSpeed::speedModeTo128(uint8_t s, SpeedMode speedMode) {
        if(s<=1) return s; // 0 and 1 are always IDLE and EMGR
        switch(speedMode) {
            case SpeedMode::S128: return s;
            case SpeedMode::S14:  return (s-2)*9 + 2;
            case SpeedMode::S28:  return (s-4)*9/2 + 2;
            default: return 0;
        }
    }


uint8_t getMaxSpeedVal(SpeedMode s) {
    switch(s) {
        case SpeedMode::S14: return 14;
        case SpeedMode::S28: return 28;
        case SpeedMode::S128: return LocoSpeed::MAX_SPEED;
        default: return 0;
    }
}
