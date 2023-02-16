#include "LocoSpeed.h"

#include <cmath>


    LocoSpeed LocoSpeed::fromFloat(float speed) {
        if(speed<0) return SPEED_EMGR; 
        else {
            int s = ceil(speed*MAX_SPEED_VAL);
            if(s==0) return SPEED_IDLE;
            else return LocoSpeed::from128(s+1);
        }
    }

    float LocoSpeed::getFloat() const {
        if(speed128==DCC_SPEED_EMGR ) return -1.0;
        if(speed128==DCC_SPEED_IDLE ) return 0.0;
        return (float)(speed128-1)/MAX_SPEED_VAL;
    }

    uint8_t LocoSpeed::getDCCByte(SpeedMode speedMode) const {
        uint8_t ret = getDCC(speedMode);
        if(speedMode == SpeedMode::S28)
            ret = (ret & 1)<<4 | (ret & 0b11110)>>1; // bit swap
        return ret;
    }

    uint8_t LocoSpeed::getDCC(SpeedMode speedMode) const {
        if(speed128<=1) return speed128;
        switch(speedMode) {
            case SpeedMode::S14: return ((speed128-1)/9 ) + 1;
            case SpeedMode::S128: return speed128;
            case SpeedMode::S28:
                /// @see https://www.nmra.org/sites/default/files/s-92-2004-07.pdf table at line 57
                return ((speed128-1)*2 + 4)/9 + 3; // first +4 is an attempt to properly round
            default: return 0;
        }
    }

    uint8_t LocoSpeed::dccTo128(uint8_t s, SpeedMode speedMode) {
        if(s<=1) return s; // 0 and 1 are always IDLE and EMGR
        switch(speedMode) {
            case SpeedMode::S128: return s;
            case SpeedMode::S14: if(s>=15) return 127; return (s-1)*9 + 1;
            case SpeedMode::S28:  
                if(s<=3) return s-2; // 2 and 3 are IDLE and EMGR
                if(s>=31) return 127;
                return (s-3)*9/2 + 1;
            default: return 0;
        }
    }


constexpr uint8_t getMaxSpeedVal(SpeedMode s) {
    switch(s) {
        case SpeedMode::S14: return 14;
        case SpeedMode::S28: return 28;
        case SpeedMode::S128: return 126;
        default: return 0;
    }
}
