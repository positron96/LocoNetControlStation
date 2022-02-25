#pragma once
#include <cstdint>
#include <cmath>

enum class SpeedMode { S14, S28, S128 };

constexpr uint8_t DCC_SPEED_IDLE = 0;
constexpr uint8_t DCC_SPEED_EMGR = 1;

class LocoSpeed {
public:
    LocoSpeed(): LocoSpeed(0) {}    

    static constexpr LocoSpeed from128(uint8_t speed128) {
        return LocoSpeed{speed128};
    }

    static LocoSpeed fromFloat(float speed) {
        if(speed<0) return LocoSpeed::from128(DCC_SPEED_EMGR); 
        else {
            int s = ceil(speed*MAX_SPEED);
            if(s==0) return LocoSpeed::from128(DCC_SPEED_IDLE);
            else return LocoSpeed::from128(s+1);
        }
    }

    uint8_t dccSpeedByte(SpeedMode speedMode) const;

    constexpr uint8_t get128() const { return speed128; }
    constexpr float getFloat() const {
        if(speed128==DCC_SPEED_EMGR ) return -1.0;
        if(speed128==DCC_SPEED_IDLE ) return 0.0;
        return (float)(speed128-1)/LocoSpeed::MAX_SPEED;
    }

    bool operator==(const LocoSpeed& rhs) const { return speed128 == rhs.speed128;  }
    bool operator< (const LocoSpeed& rhs) const { return speed128 < rhs.speed128; }

    constexpr static uint8_t MAX_SPEED = 126;

private:
    uint8_t speed128;
    constexpr LocoSpeed(uint8_t speed128): speed128{speed128} {}
};

uint8_t getMaxSpeedVal(SpeedMode s);

constexpr LocoSpeed SPEED_IDLE = LocoSpeed::from128(DCC_SPEED_IDLE);
constexpr LocoSpeed SPEED_EMGR = LocoSpeed::from128(DCC_SPEED_EMGR);

