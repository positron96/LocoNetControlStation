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
#define  ACK_SAMPLE_THRESHOLD      40

void BaseChannel::sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd) {

    DCC_LOGI("addr %d, speed=%d(%s) %c", addr.addr(), sp.get128(), sm.c_str(), fwd?'F':'R');
    packets.put_loco_speed_dir_packet(addr, sp, sm, fwd);
}

void BaseChannel::sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn) {
    DCC_LOGI("addr %d, group=%d fn=%08x", addr.addr(), (uint8_t)group, fn);

    packets.put_loco_fn_packet(addr, group, fn);

}

void BaseChannel::sendAccessory(const AccessoryAddress &addr, bool thr) {
    DCC_LOGI("addr11=%d, %c", addr.get11bitAddr(), thr?'T':'C');

    packets.put_accessory_packet(addr, thr);
}

uint BaseChannel::getBaselineCurrent() {
    uint baseline = 0;

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
bool BaseChannel::checkCurrentResponse(uint baseline) const {
    delay(ACK_SAMPLE_MILLIS);
    int max = getMaxCurrent();
    bool ret = max - baseline > ACK_SAMPLE_THRESHOLD;
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

constexpr uint8_t CV_LONGFORM = 0b0111'0000;
constexpr uint8_t CV_BIT_MANIP = 0b1000;
constexpr uint8_t CV_VERIFY_BYTE = 0b0100;
constexpr uint8_t CV_WRITE_BYTE = 0b1100;

constexpr uint8_t BIT_WRITE = 0b10000;
constexpr uint8_t BIT_VERIFY = 0;



etl::expected<uint8_t, CvCommError> BaseChannel::readCVProg(uint16_t cv) {

    uint8_t packet[3];
    int ret;

    cv--;     // actual CV addresses are cv-1 (0-1023)
    packet[0] = CV_LONGFORM | CV_BIT_MANIP | (highByte(cv) & 0x03);
    packet[1] = lowByte(cv);

    ret = 0;

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError::InvalidState);
    }

    int baseline = getBaselineCurrent();

    // TODO: implement logic: verify bit==1; if no ack received, verify bit==0; if no ack received, abort as NO_RESP

    constexpr uint8_t BIT_ONE = 0b1000;
    for (uint8_t pos=0; pos<8; pos++) {
        packet[2] = 0b1110'0000 | BIT_VERIFY | BIT_ONE | pos;

        if(!sendPacketFully(resetPacket, 3)) return etl::unexpected(CvCommError::Timeout);          // NMRA recommends starting with 3 reset packets
        resetMaxCurrent(); // start reading current here
        if(!sendPacketFully(packet, 5)) return etl::unexpected(CvCommError::Timeout);               // NMRA recommends 5 verify packets
        if(!sendPacketFully(resetPacket, 1)) return etl::unexpected(CvCommError::Timeout);          // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

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
    uint8_t packet[3];

    cv--;

    packet[0] = CV_LONGFORM | CV_VERIFY_BYTE | (highByte(cv) & 0x03);
    packet[1] = lowByte(cv);
    packet[2] = value;

    if(packets.used_loco_slots() != 0) {
        // sending packets reliably needs no locos.
        return etl::unexpected(CvCommError::InvalidState);
    }

    uint baseline = getBaselineCurrent();
    if(!sendPacketFully(resetPacket, 3)) return etl::unexpected(CvCommError::Timeout);    // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    if(!sendPacketFully(packet, 5)) return etl::unexpected(CvCommError::Timeout);         // NMRA recommends 5 verify packets
    //TODO: implement according to NMRA quote:
    // "followed by 1 or more Reset Packets, if an acknowledgement is detected"
    if(!sendPacketFully(resetPacket, 1)) return etl::unexpected(CvCommError::Timeout);

    return checkCurrentResponse(baseline);

}

bool BaseChannel::writeCVByteProg(uint16_t cv, uint8_t value) {
    uint8_t packet[3];
    uint baseline;

    cv--;  // actual CV addresses are cv-1 (0-1023)

    packet[0] = CV_LONGFORM | CV_WRITE_BYTE | (highByte(cv)&0x03);
    packet[1] = lowByte(cv);
    packet[2] = value;

    sendPacketFully(resetPacket,3);
    sendPacketFully(packet,5);
    sendPacketFully(resetPacket,1);

    // set-up to re-verify entire byte; same code as verifyCVByte
    baseline = getBaselineCurrent();
    packet[0] = CV_LONGFORM | CV_VERIFY_BYTE | (highByte(cv)&0x03);

    sendPacketFully(resetPacket,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    sendPacketFully(packet,5);               // NMRA recommends 5 verfy packets
    sendPacketFully(resetPacket,1);

    return checkCurrentResponse(baseline);

}

bool BaseChannel::writeCVBitProg(uint16_t cv, uint8_t bit_num, uint8_t value){
    uint8_t packet[4];
    uint baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)
    value &= 0x1;
    bit_num &= 0x7;

    packet[0] = 0x78 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1] = lowByte(cv);
    packet[2] = 0xF0 | value<<3 | bit_num;

    sendPacketFully(resetPacket,2,1);
    sendPacketFully(packet,3,4);
    sendPacketFully(resetPacket,2,1);
    sendPacketFully(idlePacket,2,10);

    baseline = getBaselineCurrent();

    bitClear(packet[2],4);              // change instruction code from Write Bit to Verify Bit

    sendPacketFully(resetPacket,2,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    sendPacketFully(packet,3,5);               // NMRA recommends 5 verfy packets
    sendPacketFully(resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

void BaseChannel::writeCVByteMain(LocoAddress addr, uint16_t cv, uint8_t value) {
    uint8_t packet[6];   // save space for checksum byte

    byte nB=0;

    cv--;

    uint16_t iAddr = addr.addr();
    if( addr.isLong() )
        packet[nB++]=highByte(iAddr) | 0xC0;      // convert train number into a two-byte address

    packet[nB++] = lowByte(iAddr);
    packet[nB++] = 0xEC | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[nB++] = lowByte(cv);
    packet[nB++] = value;

    sendPacketFully(packet,nB,4);

}

void BaseChannel::writeCVBitMain(LocoAddress addr, uint16_t cv, uint8_t bit_num, uint8_t value) {
    uint8_t b[6];                      // save space for checksum byte

    byte nB=0;

    cv--;

    value &= 0x1;
    bit_num &= 0x3;

    uint16_t iAddr = addr.addr();
    if( addr.isLong() )
        b[nB++] = highByte(iAddr) | 0xC0;      // convert train number into a two-byte address

    b[nB++]=lowByte(iAddr);
    b[nB++]=0xE8 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    b[nB++]=lowByte(cv);
    b[nB++]=0xF0 | value<<3 | bit_num;

    sendPacketFully(b,nB,4);

}

}
