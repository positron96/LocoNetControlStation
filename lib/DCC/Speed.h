#pragma once
#include <cstdint>

enum class SpeedMode { S14, S28, S128 };

class Speed {
public:
    Speed(): Speed(0) {}
    Speed(uint8_t speed128): speed128{speed128} {}

    bool operator==(const Speed& rhs) const {
        return speed128 == rhs.speed128;
    }

    uint8_t dccSpeedByte(SpeedMode speedMode) {
        uint8_t ret;
        switch(speedMode) {
            case SpeedMode::S14: 
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

    uint8_t getSpeed128() const { return speed128; }

private:
    uint8_t speed128;
};