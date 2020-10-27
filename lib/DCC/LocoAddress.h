#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <math.h>


class LocoAddress {
public:
    LocoAddress() : num(0) {}
    static LocoAddress shortAddr(uint8_t addr) {  return LocoAddress(addr); }
    static LocoAddress longAddr(uint16_t addr) {  return LocoAddress(-addr); }
    bool isShort() const { return num>=0; }
    bool isLong() const { return num<=0; }
    uint16_t addr() const { return abs(num); }
    bool isValid() const { return num!=0; }
    bool operator < (const LocoAddress& a) const { return (num < a.num); }
    operator String() const {  return String( (isShort() ? 'S' : 'L') )+addr(); }
private:
    LocoAddress(int16_t num): num(num) { }
    int16_t num;
};