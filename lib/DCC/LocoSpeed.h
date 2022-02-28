#pragma once
#include <cstdint>
#include <cmath>

enum class SpeedMode { S14, S28, S128 };

constexpr uint8_t DCC_SPEED_IDLE = 0;
constexpr uint8_t DCC_SPEED_EMGR = 1;

class LocoSpeed {
public:
    constexpr LocoSpeed(): LocoSpeed(0) {} 

    LocoSpeed(uint8_t speed, SpeedMode mode): LocoSpeed{speedModeTo128(speed, mode)} {}

    static constexpr LocoSpeed from128(uint8_t speed128) {
        return LocoSpeed{speed128};
    }

    static LocoSpeed fromFloat(float speed) ;

    float getFloat() const ;

    uint8_t dccSpeedByte(SpeedMode speedMode) const;

    constexpr uint8_t get128() const { return speed128; }

    uint8_t getSpeedMode(SpeedMode mode) const;

    bool operator==(const LocoSpeed& rhs) const { return speed128 == rhs.speed128;  }
    bool operator< (const LocoSpeed& rhs) const { return speed128 < rhs.speed128; }

    static uint8_t speedModeTo128(uint8_t s, SpeedMode mode);

private:
    constexpr static uint8_t MAX_SPEED_VAL = 126;
    uint8_t speed128;
    constexpr LocoSpeed(uint8_t speed128): speed128{speed128} {}
};

constexpr uint8_t getMaxSpeedVal(SpeedMode s);

constexpr LocoSpeed SPEED_IDLE = LocoSpeed::from128(DCC_SPEED_IDLE);
constexpr LocoSpeed SPEED_EMGR = LocoSpeed::from128(DCC_SPEED_EMGR);

