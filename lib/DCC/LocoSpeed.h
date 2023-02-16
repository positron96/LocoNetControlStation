#pragma once
#include <cstdint>

/** DCC speed mode. */
enum class SpeedMode { S14, S28, S128 };

constexpr uint8_t DCC_SPEED_IDLE = 0; ///< DCC speed value for idle (stop)
constexpr uint8_t DCC_SPEED_EMGR = 1; ///< DCC speed value for emergency stop

/** 
 * Stores locomotive speed.
 * 
 * Has utility functions to convert to/from different DCC speed modes (14, 28, 128) and float (0..1) values.
 * Internally, stores speed as DCC 128 speed steps.
 * It's a lightweight object, so can be passed by value cheapliy.
 */
class LocoSpeed {
public:
    constexpr LocoSpeed(): LocoSpeed{0} {} 

    LocoSpeed(uint8_t speed, SpeedMode mode): LocoSpeed{dccTo128(speed, mode)} {}

    static constexpr LocoSpeed from128(uint8_t speed128) {  return LocoSpeed{speed128};  }
    static LocoSpeed fromFloat(float speed) ;
    static LocoSpeed fromDCC(uint8_t speed, SpeedMode sm) { return LocoSpeed{speed, sm}; }

    constexpr uint8_t get128() const { return speed128; }
    uint8_t getDCC(SpeedMode mode) const;
    uint8_t getDCCByte(SpeedMode speedMode) const; ///< In addition to getDCC also swaps bits for S28.
    float getFloat() const ;

    bool operator==(const LocoSpeed& rhs) const { return speed128 == rhs.speed128;  }
    bool operator< (const LocoSpeed& rhs) const { return speed128 < rhs.speed128; }

    bool isEmgr() const { return speed128==DCC_SPEED_EMGR; }

private:
    constexpr static uint8_t MAX_SPEED_VAL = 126;
    uint8_t speed128;
    constexpr LocoSpeed(uint8_t speed128): speed128{speed128} {}

    static uint8_t dccTo128(uint8_t s, SpeedMode mode);
};

constexpr uint8_t getMaxSpeedVal(SpeedMode s);

constexpr LocoSpeed SPEED_IDLE = LocoSpeed::from128(DCC_SPEED_IDLE);
constexpr LocoSpeed SPEED_EMGR = LocoSpeed::from128(DCC_SPEED_EMGR);

