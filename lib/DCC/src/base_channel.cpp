/**
 * @see On basic packets: https://www.nmra.org/sites/default/files/s-92-2004-07.pdf
 * @see On extended packets: https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
 */

#include "dcc/base_channel.hpp"

namespace dcc {

Packet idlePacket{0xFF, 0x00};
Packet resetPacket{0x00, 0x00};

PacketBits idle_packet_bits = PacketBits::from_packet(idlePacket);

#define  ACK_BASE_COUNT            100      /**< Number of analogRead samples to take before each CV verify to establish a baseline current.*/
#define  ACK_SAMPLE_MILLIS         10       ///< analogReads are taken for this number of milliseconds
/** The mA difference for CV acknowledgement.
 *  NMRA 9.2.3 mandates +60mA for 6ms (+-1ms), but provide some legroom.
 */
#define  ACK_SAMPLE_THRESHOLD      35

void BaseChannel::sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd) {

    DCC_LOGI("addr %d, speed=%d(%s) %c", addr.addr(), sp.get128(), sm.c_str(), fwd?'F':'R');
    packets.put_loco_speed_dir_packet(addr, sp, sm, fwd);
}

void BaseChannel::sendThrottleOnce(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd) {
    DCC_LOGI("addr %d, speed=%d(%s) %c", addr.addr(), sp.get128(), sm.c_str(), fwd?'F':'R');

    auto bytes = make_speed_dir_packet(addr, sp, sm, fwd);
    packets.put_generic_packet(bytes, sp.isEmgr() ? -100 : 0);

    DCC_LOGI("Addr:%d, spd:%d(%s) %c, %s",
        addr.addr(), sp.get128(), sm.c_str(), fwd?'F':'R',
        fmt_span(bytes));
}

void BaseChannel::sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn) {
    DCC_LOGI("addr %d, group=%d fn=0x%08x", addr.addr(), (uint8_t)group, fn);

    packets.put_loco_fn_packet(addr, group, fn);

}

void BaseChannel::sendAccessory(const AccessoryAddress &addr, bool thr) {
    DCC_LOGI("addr11=%d, %c", addr.get11bitAddr(), thr?'T':'C');

    packets.put_accessory_packet(addr, thr);
}

unsigned BaseChannel::getBaselineCurrent() {
    unsigned baseline = 0;

    // collect baseline current
    for (int j = 0; j < ACK_BASE_COUNT; j++) {
        //updateCurrent(); // seems it's not reentrant
        uint16_t v = getCurrent();
        baseline += v;
        delayMicroseconds(500);
    }
    baseline /= ACK_BASE_COUNT;
    DCC_LOGD("Baseline %d", baseline);
    return baseline;
}

// https://www.nmra.org/sites/default/files/s-9.2.3_2012_07.pdf
bool BaseChannel::checkCurrentResponse(unsigned baseline) const {
    delay(ACK_SAMPLE_MILLIS);
    int max = getMaxCurrent();
    bool ret = max - (int)baseline > ACK_SAMPLE_THRESHOLD;
    DCC_LOGI("result is %d, max: %d, baseline: %d", ret?1:0, max, baseline);
    return ret;
}


/**
 * Enqueues a DCC packet and waits until pending queue is empty.
 *
 * In other words, it returns when packet starts transmitting.
 */
bool BaseChannel::sendPacketFully(const etl::span<uint8_t> packet, size_t nRepeat, size_t timeout_ms) {
    size_t remaining_ms = timeout_ms;

    // First phase: try to enqueue within the timeout budget.
    while (!packets.put_generic_packet(packet, nRepeat)) {
        if (remaining_ms == 0) return false;
        delay(1);
        remaining_ms--;
    }

    // Second phase: wait for queue drain using the remaining budget.
    while (!packets.is_queue_empty()) {
        if (remaining_ms == 0) return false;
        delay(1);
        remaining_ms--;
    }

    return true;
}

// #define PIN_DBG1 12
// #define PIN_DBG2 14

constexpr uint8_t B1_OPS_PREFIX     = 0b0111'0000;
constexpr uint8_t B1_MAIN_LONGFORM  = 0b1110'0000;
constexpr uint8_t B1_MAIN_SHORTFORM = 0b0110'0000;
constexpr uint8_t B1_BIT_MANIP = 0b1000;
constexpr uint8_t B1_VERIFY_BYTE = 0b0100;
constexpr uint8_t B1_WRITE_BYTE = 0b1100;

constexpr uint8_t B2_BIT_WRITE = 0b1'0000;
constexpr uint8_t B2_BIT_VERIFY = 0;

constexpr uint8_t B2_BIT_MANIP = 0b1110'0000;

constexpr size_t PRE_PACKET_REPEATS = 3; // NMRA recommends starting with min 3 reset packets
constexpr size_t READ_REPEATS = 5;  // NMRA recommends min 5 verify packets
constexpr size_t WRITE_REPEATS = 5; // NMRA recommends min 5 verify packets
constexpr size_t POST_READ_REPEATS = 1; // "followed by 1 or more Reset Packets, if an acknowledgement is detected"
constexpr size_t POST_WRITE_REPEATS = 6; // "6 or more Identical Write or Reset packets (DecoderRecovery-Time)"

constexpr uint8_t cvHighBits(uint16_t cv) {
    return highByte(cv) & 0b11;
}

constexpr uint8_t cvLowBits(uint16_t cv) {
    return lowByte(cv);
}

etl::expected<uint8_t, CvCommError> BaseChannel::readCVProg(uint16_t cv) {

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError::InvalidState);
    }

    uint8_t packet[3];
    int ret;

    cv--;     // actual CV addresses are cv-1 (0-1023)
    packet[0] = B1_OPS_PREFIX | B1_BIT_MANIP | cvHighBits(cv);
    packet[1] = cvLowBits(cv);

    ret = 0;

    unsigned baseline = getBaselineCurrent();

    // TODO: implement logic: verify bit==1; if no ack received, verify bit==0; if no ack received, abort as NO_RESP

    constexpr uint8_t B2_BIT_ONE = 0b1000;
    for (uint8_t pos=0; pos<8; pos++) {
        packet[2] = B2_BIT_MANIP | B2_BIT_VERIFY | B2_BIT_ONE | pos;

        if(!sendPacketFully(resetPacket, PRE_PACKET_REPEATS)) return etl::unexpected(CvCommError::Timeout);
        resetMaxCurrent(); // start reading current here
        if(!sendPacketFully(packet, READ_REPEATS)) return etl::unexpected(CvCommError::Timeout);
        if(!sendPacketFully(resetPacket, POST_READ_REPEATS)) return etl::unexpected(CvCommError::Timeout);

        bool bitVal = checkCurrentResponse(baseline);
        if(bitVal) bitSet(ret, pos);

        DCC_LOGI("Reading bit %d, value is %d", pos, bitVal?1:0);
    }

    auto verified = verifyCVByteProg(cv+1, ret);
    if(!verified) return etl::unexpected(verified.error());
    if(!verified.value()) return etl::unexpected(CvCommError::NoResponse);
    return ret;
}

etl::expected<bool, CvCommError> BaseChannel::verifyCVByteProg(uint16_t cv, uint8_t value){

    DCC_LOGI("Verifying cv%d==%d", cv, value);

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError::InvalidState);
    }

    uint8_t packet[3];

    cv--;

    packet[0] = B1_OPS_PREFIX | B1_VERIFY_BYTE | cvHighBits(cv);
    packet[1] = cvLowBits(cv);
    packet[2] = value;

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError::InvalidState);
    }

    unsigned baseline = getBaselineCurrent();
    if(!sendPacketFully(resetPacket, PRE_PACKET_REPEATS)) return etl::unexpected(CvCommError::Timeout);    // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    if(!sendPacketFully(packet, READ_REPEATS)) return etl::unexpected(CvCommError::Timeout);         // NMRA recommends 5 verify packets
    if(!sendPacketFully(resetPacket, POST_READ_REPEATS)) return etl::unexpected(CvCommError::Timeout);

    return checkCurrentResponse(baseline);

}

etl::expected<void, CvCommError> BaseChannel::writeCVByteProg(uint16_t cv, uint8_t value) {

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError{CvCommError::InvalidState});
    }

    uint8_t packet[3];

    cv--;  // actual CV addresses are cv-1 (0-1023)

    packet[0] = B1_OPS_PREFIX | B1_WRITE_BYTE | cvHighBits(cv);
    packet[1] = cvLowBits(cv);
    packet[2] = value;

    sendPacketFully(resetPacket, PRE_PACKET_REPEATS);
    sendPacketFully(packet, WRITE_REPEATS);
    sendPacketFully(resetPacket, POST_WRITE_REPEATS);

    // turn into "verify byte" packet
    unsigned baseline = getBaselineCurrent();
    packet[0] = B1_OPS_PREFIX | B1_VERIFY_BYTE | cvHighBits(cv);

    sendPacketFully(resetPacket, PRE_PACKET_REPEATS);
    resetMaxCurrent();
    sendPacketFully(packet, READ_REPEATS);
    sendPacketFully(resetPacket, POST_READ_REPEATS);

    if (!checkCurrentResponse(baseline)) return etl::unexpected(CvCommError{CvCommError::NoResponse});
    return {};

}

etl::expected<void, CvCommError> BaseChannel::writeCVBitProg(uint16_t cv, uint8_t bit_num, uint8_t value){

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError{CvCommError::InvalidState});
    }

    uint8_t packet[3];

    cv--;  // actual CV addresses are cv-1 (0-1023)
    value &= 0x1;
    bit_num &= 0x7;

    packet[0] = B1_OPS_PREFIX | B1_BIT_MANIP | cvHighBits(cv);
    packet[1] = cvLowBits(cv);
    packet[2] = B2_BIT_MANIP | B2_BIT_WRITE | (value<<3) | bit_num;

    sendPacketFully(resetPacket, PRE_PACKET_REPEATS);
    sendPacketFully(packet, WRITE_REPEATS);
    sendPacketFully(resetPacket, POST_WRITE_REPEATS);

    unsigned baseline = getBaselineCurrent();

    // Turn it into a "verify bit" packet
    packet[2] = B2_BIT_MANIP | B2_BIT_VERIFY | (value<<3) | bit_num;

    sendPacketFully(resetPacket, PRE_PACKET_REPEATS);
    resetMaxCurrent();
    sendPacketFully(packet, READ_REPEATS);
    sendPacketFully(resetPacket, POST_READ_REPEATS);

    if(!checkCurrentResponse(baseline)) return etl::unexpected(CvCommError{CvCommError::NoResponse});
    return {};

}

void BaseChannel::writeCVByteMain(LocoAddress addr, uint16_t cv, uint8_t value) {
    DCC_LOGI("addr=%s cv%hu=%d", String(addr).c_str(), cv, value);

    uint8_t packet[5];

    cv--;

    auto it = encode_address(addr, packet);

    *it++ = B1_MAIN_LONGFORM | B1_WRITE_BYTE | cvHighBits(cv);
    *it++ = cvLowBits(cv);
    *it++ = value;

    sendPacketFully(etl::span{packet, it}, WRITE_REPEATS);

}

void BaseChannel::writeCVBitMain(LocoAddress addr, uint16_t cv, uint8_t bit_num, uint8_t value) {
    DCC_LOGI("addr=%s cv%hu[%u]=%d", String(addr).c_str(), cv, bit_num, value);
    uint8_t packet[5];

    cv--;
    value &= 0x1;
    bit_num &= 0x3;

    auto it = encode_address(addr, packet);

    *it++ = B1_MAIN_LONGFORM | B1_BIT_MANIP | cvHighBits(cv);
    *it++ = cvLowBits(cv);
    *it++ = B2_BIT_MANIP | B2_BIT_WRITE | (value<<3) | bit_num;

    sendPacketFully(etl::span{packet, it}, WRITE_REPEATS);

}

}
