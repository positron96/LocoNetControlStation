#pragma once

#include "LocoAddress.h"
#include "LocoSpeed.h"

#include <etl/vector.h>
#include <etl/array.h>
#include <etl/bit_stream.h>

#include <cstdint>
#include <cstdlib>

namespace dcc {

    enum class fn_group {
        F0_4, F5_8, F9_12, F13_20, F21_28
    };


    constexpr size_t ACCESSORY_PACKET_REPEATS = 4; // repeat accessory packets this number of times
    constexpr size_t FN_PACKET_REPEATS = 4; // repeat function packets this number of times


    constexpr size_t fn_group_index(const fn_group fg) {
        switch(fg) {
            case fn_group::F0_4: return 0;
            case fn_group::F5_8: return 1;
            case fn_group::F9_12: return 2;
            case fn_group::F13_20: return 3;
            case fn_group::F21_28: return 4;
            default: return 0;
        }
    }

    inline size_t encode_dcc(const etl::span<uint8_t> src, etl::span<uint8_t> dst, size_t preamble_bits) {
        uint8_t crc = src[0];
        size_t len = src.size();
        for(int i=1; i<len; i++)
            crc ^= src[i];

        // don't care about endianness, we don't write multi-bytes
        etl::bit_stream_writer s(dst.data(), dst.size(), etl::endian::native);
        if(preamble_bits != 0) s.write(0xFFFFFFFF, preamble_bits);  // preamble (max 32 bits)
        for(size_t i=0; i<len; i++) {
            s.write(0, 1); // data start bit
            s.write(src[i], 8);
        }
        s.write(0, 1); // data start bit
        s.write(crc, 8);
        return s.size_bits();
    }

    constexpr size_t MAX_RAW_PACKET_BYTES = 10;

    struct PacketBits {
        etl::array<uint8_t, MAX_RAW_PACKET_BYTES> buf;
        uint8_t len;
        static PacketBits from_bytes(const etl::span<uint8_t> src, size_t preamble_len = 0) {
            PacketBits p;
            p.len = encode_dcc(src, p.buf, preamble_len);
            return p;
        }
    };

    struct PacketWithRepeats {
        PacketBits packet;
        uint8_t nRepeats;
    };


    template <typename It>
    constexpr inline It encode_address(const LocoAddress addr, It out) {
        uint16_t iAddr = addr.addr();
        if ( addr.isLong() ) {
            *out++ = highByte(iAddr) | 0xC0;  // convert train number into a two-byte address
        }

        *out++ = lowByte(iAddr);
        return out;
    }

    inline etl::array<uint8_t, 4> make_speed_dir_packet(LocoAddress addr, uint8_t tSpeed, SpeedMode sm, uint8_t tDirection) {
        etl::array<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());

        if(sm==SpeedMode::S128) {
            // Advanced Operations Instruction: https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf #200
            *it++ = 0b0011'1111;
            *it++ = (tSpeed & 0x7F) | ( (tDirection & 0x1) << 7);
        } else {
            // basic packet: https://www.nmra.org/sites/default/files/s-92-2004-07.pdf #35
            *it = 0b0100'0000 | (tSpeed & 0b0001'1111);
            if(tDirection==1) *it |= 0b0010'0000;
            // 14 speed steps unsupported
        }

        // DCC_LOGI("addr %d, speed=%d(mode %d) %c", addr, tSpeed, (int)sm, (tDirection==1)?'F':'B');

        return data;
    }

    inline etl::array<uint8_t, 4> make_speed_dir_packet(LocoAddress addr, LocoSpeed speed, SpeedMode mode, bool fwd) {
        return make_speed_dir_packet(addr, speed.getDCCByte(mode), mode, fwd ? 1 : 0);
    }

    inline auto make_fn_packet(LocoAddress addr, uint8_t fByte, uint8_t eByte) {
        etl::array<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());

        if ( (fByte & 0b1100'0000) == 0b1000'0000) {// this is a request for functions FL,F1-F12
            *it = (fByte | 0x80) & 0xBF; // for safety this guarantees that first nibble of function byte will always be of binary form 10XX which should always be the case for FL,F1-F12
        } else {                             // this is a request for functions F13-F28
            *it++ = (fByte | 0xDE) & 0xDF; // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
            *it = eByte;
        }

        // DCC_LOGI("iReg %d, addr %d, fByte=%02x eByte=%02x", iReg, addr, fByte, eByte);
        return data;

    }

    inline auto make_f0_f4_packet(LocoAddress addr, uint32_t fns) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        *it = 0b1000'0000u | (fns & 0b1u) << 4u | (fns & 0x1Fu) >> 1u;
        return data;
    }

    inline auto make_f5_f8_packet(LocoAddress addr, uint32_t state) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        state >>= 5;
        *it = 0b1011'0000u | (state & 0b1111);
        return data;
    }

    inline auto make_f9_f12_packet(LocoAddress addr, uint32_t fns) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        fns >>= 9;
        *it = 0b1010'0000u | (fns & 0b1111);
        return data;
    }

    inline auto make_f13_f20_packet(LocoAddress addr, uint32_t fns) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        fns >>= 13;
        *it++ = 0b1101'1110u;
        *it++ = fns & 0xFF;
        return data;
    }

    inline auto make_f21_f28_packet(LocoAddress addr, uint32_t fns) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        fns >>= 21;
        *it++ = 0b1101'1111u;
        *it++ = fns;
        return data;
    }

    inline auto make_f29_f36_packet(LocoAddress addr, uint32_t fns) {
        etl::vector<uint8_t, 4> data;
        auto it = encode_address(addr, data.begin());
        fns >>= 29;
        *it++ = 0b1101'1000u;
        *it++ = fns & 0b1111;  //!!! we have only 4 bits left in uint32_t
        return data;
    }

    inline auto make_fn_packet(LocoAddress addr, fn_group fg, uint32_t fns) {
        switch(fg) {
            case fn_group::F0_4:   return make_f0_f4_packet(addr, fns);
            case fn_group::F5_8:   return make_f5_f8_packet(addr, fns);
            case fn_group::F9_12:  return make_f9_f12_packet(addr, fns);
            case fn_group::F13_20: return make_f13_f20_packet(addr, fns);
            case fn_group::F21_28: return make_f21_f28_packet(addr, fns);
            default:                return make_f0_f4_packet(addr, 0); // should not happen, return something valid
        }
    }

    inline auto make_accessory_packet(uint16_t addr9, uint8_t ch, bool thrown) {
        // DCC_LOGI("addr9=%d, ch=%d, %c", addr9, ch, thrown?'T':'C');

        etl::array<uint8_t, 2> b;

        /*
        first byte is of the form 10AAAAAA, where AAAAAA represent
        6 least significant bits of accessory address (9-bit. Here we have 14-bit address, so take bits 2-7) */
        b[0] = ( addr9 & 0x3F) | 0x80;
        /*
        "The most significant bits of the 9-bit address are bits 4-6 of the second data byte.
        By convention these bits (bits 4-6 of the second data byte) are in ones complement. "
        https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
        */
        // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represents throw/close
        b[1] = ( ((addr9>>6 & 0x7) << 4 ) ^ 0b0111'0000 )
            | (ch & 0x3) << 1
            | (thrown?0x1:0)
            | 0b1000'0000   ;

        return b;
    }

    inline auto make_accessory_packet(uint16_t addr11, bool thrown) {
        return make_accessory_packet((addr11>>2) + 1U, addr11 & 0x3, thrown);
    }

}
