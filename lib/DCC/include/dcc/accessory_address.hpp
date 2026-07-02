#pragma once

#include <cstdint>

namespace dcc {

    /**
     * According to DCC, accessory decoder output can be addressed via
     *   11 bit address or 9 bit decoder address and 2 bit output index.
     **/
    class AccessoryAddress {
    public:
        AccessoryAddress(): addr11{0} {};
        static AccessoryAddress from11bit(uint16_t addr) {
            return AccessoryAddress{addr};
        }
        static AccessoryAddress from9bit(uint16_t addr9, uint8_t output) {
            return AccessoryAddress{static_cast<uint16_t>((addr9<<2) | (output & 0b11))};
        }

        uint16_t get9bitAddr() const { return addr11 >> 2; }

        uint8_t getOutputIdx() const { return addr11 & 0b11; }

        uint16_t get11bitAddr() const { return addr11; }
        uint16_t longAddr() const { return addr11; }

        bool operator < (const AccessoryAddress& a) const {
            return addr11 < a.addr11;
        }
        bool operator == (const AccessoryAddress& a) const {
            return addr11 == a.addr11;
        }
    private:
        AccessoryAddress(uint16_t a): addr11{a} {};
        uint16_t addr11;
    };

}
