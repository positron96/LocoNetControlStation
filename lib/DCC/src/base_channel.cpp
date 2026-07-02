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
#define  ACK_SAMPLE_MILLIS         50       ///< analogReads are taken for this number of milliseconds
#define  ACK_SAMPLE_SMOOTHING      0.3      /**< Exponential smoothing to use in processing the analogRead samples after a CV verify (bit or byte) has been sent.*/
#define  ACK_SAMPLE_THRESHOLD      2       /**< The threshold that the exponentially-smoothed analogRead samples (after subtracting the baseline current) must cross to establish ACKNOWLEDGEMENT.*/


void BaseChannel::sendThrottle(LocoAddress addr, LocoSpeed sp, SpeedMode sm, bool fwd) {

    DCC_LOGI("addr %d, speed=%d(mode %d) %c", addr.addr(), sp.get128(), (int)sm, fwd?'F':'B');
    packets.put_loco_speed_dir_packet(addr, sp, sm, fwd);
}

void BaseChannel::sendFunctionGroup(LocoAddress addr, fn_group group, uint32_t fn) {
    DCC_LOGI("addr %d, group=%d fn=%08x", addr.addr(), (uint8_t)group, fn);

    packets.put_loco_fn_packet(addr, group, fn);

}

void BaseChannel::sendAccessory(uint16_t addr11, bool thrown) {
    DCC_LOGI("addr11=%d, %c", addr11, thrown?'T':'C');

    packets.put_accessory_packet(AccessoryAddress::from11bit(addr11), thrown);
}

void BaseChannel::sendAccessory(const AccessoryAddress &addr, bool thr) {
    DCC_LOGI("addr11=%d, %c", addr.get11bitAddr(), thr?'T':'C');

    packets.put_accessory_packet(addr, thr);
}

uint BaseChannel::getBaselineCurrent() const {
    uint baseline = 0;

    // collect baseline current
    for (int j = 0; j < ACK_BASE_COUNT; j++) {
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
    bool ret = false;
    int max = 0;
    delay(ACK_SAMPLE_MILLIS);
    max = getMaxCurrent();
    ret = max - baseline > ACK_SAMPLE_THRESHOLD;
    DCC_LOGD("result is %d, max: %d, baseline: %d", ret?1:0, max, baseline);
    return ret;
}

int16_t BaseChannel::readCVProg(int cv) {
	uint8_t packet[4];
	int ret;

	cv--;                              // actual CV addresses are cv-1 (0-1023)

	packet[0] = 0x78 | (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
	packet[1] = lowByte(cv);

	ret = 0;

    int baseline = getBaselineCurrent();

	for (uint8_t i = 0; i<8; i++) {
		packet[2] = 0xE8 | i;

		loadPacket(resetPacket, 2, 3);          // NMRA recommends starting with 3 reset packets
        resetMaxCurrent();
		loadPacket(packet, 3, 5);               // NMRA recommends 5 verify packets
		loadPacket(resetPacket, 2, 1);          // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

        bool bitVal = checkCurrentResponse(baseline);
        if(bitVal) bitSet(ret, i);

        DCC_LOGD("Reading bit %d, value is %d", i, bitVal?1:0);
	}

    return verifyCVByteProg(cv+1, ret) ? ret : -1;

}

bool BaseChannel::verifyCVByteProg(uint16_t cv, uint8_t bValue) {
    DCC_LOGI("Verifying cv%d==%d", cv, bValue);
    uint8_t packet[4];

    cv--;

    packet[0] = 0x74 | (highByte(cv) & 0x03);
    packet[1] = lowByte(cv);
	packet[2] = bValue;

    loadPacket(resetPacket, 2, 1);    // NMRA recommends starting with 3 reset packets
    loadPacket(resetPacket, 2, 3);
    uint baseline = getBaselineCurrent();
    resetMaxCurrent();
	loadPacket(packet, 3, 5);         // NMRA recommends 5 verify packets
	loadPacket(resetPacket, 2, 1);    // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

bool BaseChannel::writeCVByteProg(int cv, uint8_t bValue) {
    uint8_t packet[4];
    uint baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)

    packet[0]=0x7C | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1]=lowByte(cv);
    packet[2]=bValue;

    loadPacket(resetPacket,2,1);
    loadPacket(packet,3,4);
    loadPacket(resetPacket,2,1);
    loadPacket(idlePacket,2,10);

    baseline = getBaselineCurrent();

    packet[0]=0x74 | (highByte(cv)&0x03);   // set-up to re-verify entire byte

    loadPacket(resetPacket,2,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    loadPacket(packet,3,5);               // NMRA recommends 5 verfy packets
    loadPacket(resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

bool BaseChannel::writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue){
    uint8_t packet[4];
    uint baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)
    bValue &= 0x1;
    bNum &= 0x7;

    packet[0] = 0x78 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1] = lowByte(cv);
    packet[2] = 0xF0 | bValue<<3 | bNum;

    loadPacket(resetPacket,2,1);
    loadPacket(packet,3,4);
    loadPacket(resetPacket,2,1);
    loadPacket(idlePacket,2,10);

    baseline = getBaselineCurrent();

    bitClear(packet[2],4);              // change instruction code from Write Bit to Verify Bit

    loadPacket(resetPacket,2,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    loadPacket(packet,3,5);               // NMRA recommends 5 verfy packets
    loadPacket(resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

void BaseChannel::writeCVByteMain(LocoAddress addr, int cv, uint8_t bValue) {
    uint8_t packet[6];   // save space for checksum byte

    byte nB=0;

    cv--;

    uint16_t iAddr = addr.addr();
    if( addr.isLong() )
        packet[nB++]=highByte(iAddr) | 0xC0;      // convert train number into a two-byte address

    packet[nB++] = lowByte(iAddr);
    packet[nB++] = 0xEC | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[nB++] = lowByte(cv);
    packet[nB++] = bValue;

    loadPacket(packet,nB,4);

}

void BaseChannel::writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue) {
    uint8_t b[6];                      // save space for checksum byte

    byte nB=0;

    cv--;

    bValue &= 0x1;
    bNum &= 0x3;

    uint16_t iAddr = addr.addr();
    if( addr.isLong() )
        b[nB++] = highByte(iAddr) | 0xC0;      // convert train number into a two-byte address

    b[nB++]=lowByte(iAddr);
    b[nB++]=0xE8 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    b[nB++]=lowByte(cv);
    b[nB++]=0xF0 | bValue<<3 | bNum;

    loadPacket(b,nB,4);

}

}
