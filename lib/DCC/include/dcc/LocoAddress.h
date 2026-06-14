#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

/**
 * Represents a DCC address.
 *
 * Can be either short (or basic or primary) or long (or extended).
 * By DCC standard, short addresses are in range 1--127, long addresses are in range 1--10239.
 * Other systems can limit address range
 * (e.g. LocoNet has extended address range of 128--9983, i.e. a numeric value uniquely identifies address type).
 * Use static creation methods to create appropriate address.
 * Address 0 is treated as invalid.
 *
 * Internally, the address is stored as int16_t, the signum specifies short(positive)/long(negative).
 * This class is small and can be cheaply passed by value.
 */
class LocoAddress {
public:
    LocoAddress() : num(0) {}
    /** Creates short address. */
    static LocoAddress shortAddr(uint8_t addr) {  return LocoAddress{addr}; }
    /** Creates long address. */
    static LocoAddress longAddr(uint16_t addr) {  return LocoAddress{-addr}; }
    bool isShort() const { return num>=0; }
    bool isLong() const { return num<=0; }
    /** Returns numeric value of this address. */
    uint16_t addr() const { return abs(num); }
    bool isValid() const { return num!=0; }
    bool operator < (const LocoAddress& a) const {
        // long address must be "larger" than short one
        size_t key = (isShort() ? num : -num + 128);
        size_t aKey = (a.isShort() ? a.num : -a.num + 128);
        return (key < aKey);
    }
    operator String() const {  return String( (isShort() ? 'S' : 'L') )+addr(); }
private:
    int16_t num;
    LocoAddress(int16_t num): num{num} { }
};
