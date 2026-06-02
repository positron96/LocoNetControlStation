/**
 * @see On basic packets: https://www.nmra.org/sites/default/files/s-92-2004-07.pdf
 * @see On extended packets: https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
 */

#include "DCC.h"

uint8_t idlePacket[3] = {0xFF, 0x00, 0};
uint8_t resetPacket[3] = {0x00, 0x00, 0};

#define  ACK_BASE_COUNT            100      /**< Number of analogRead samples to take before each CV verify to establish a baseline current.*/
#define  ACK_SAMPLE_MILLIS         50       ///< analogReads are taken for this number of milliseconds
#define  ACK_SAMPLE_SMOOTHING      0.3      /**< Exponential smoothing to use in processing the analogRead samples after a CV verify (bit or byte) has been sent.*/
#define  ACK_SAMPLE_THRESHOLD      2       /**< The threshold that the exponentially-smoothed analogRead samples (after subtracting the baseline current) must cross to establish ACKNOWLEDGEMENT.*/


void IDCCChannel::sendThrottle(int iReg, LocoAddress addr, uint8_t tSpeed, SpeedMode sm, uint8_t tDirection) {
    auto t = make_speed_dir_packet(addr, tSpeed, sm, tDirection);

    DCC_LOGI("iReg %d, addr %d, speed=%d(mode %d) %c", iReg, addr, tSpeed, (int)sm, (tDirection==1)?'F':'B');

    loadPacket(iReg, t.data(), t.size(), 0);
}

void IDCCChannel::sendFunctionGroup(int iReg, LocoAddress addr, DCCFnGroup group, uint32_t fn) {
    DCC_LOGI("iReg %d, addr %d, group=%d fn=%08x", iReg, addr, (uint8_t)group, fn);
    switch(group) {
        case DCCFnGroup::F0_4:
            // move FL(F0) to 5th bit
            fn = (fn & 0x1)<<4 | (fn & 0b1'1110)>>1;
            sendFunction(iReg, addr,  0b1000'0000 | (fn & 0b0001'1111) );
            break;
        case DCCFnGroup::F5_8:
            fn >>= 5;
            sendFunction(iReg, addr,  0b1011'0000 | (fn & 0b0000'1111) );
            break;
        case DCCFnGroup::F9_12:
            fn >>= 9;
            sendFunction(iReg, addr,  0b1010'0000 | (fn & 0b0000'1111) );
            break;
        case DCCFnGroup::F13_20:
            fn >>= 13;
            sendFunction(iReg, addr,  0b1101'1110, (uint8_t)fn );
            break;
        case DCCFnGroup::F21_28:
            fn >>= 21;
            sendFunction(iReg, addr,  0b1101'1111, (uint8_t)fn );
            break;
        default:
            break;
    }

}
void IDCCChannel::sendFunction(int iReg, LocoAddress addr, uint8_t fByte, uint8_t eByte) {
    auto b = make_fn_packet(addr, fByte, eByte);

    DCC_LOGI("iReg %d, addr %d, fByte=%02x eByte=%02x", iReg, addr, fByte, eByte);

    /*
    NMRA DCC norm ask for two DCC packets instead of only one:
    "Command Stations that generate these packets, and which are not periodically refreshing these functions,
    must send at least two repetitions of these commands when any function state is changed."
    https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
    */
    loadPacket(0, b.data(), b.size(), 4);

}

void IDCCChannel::sendAccessory(uint16_t addr11, bool thrown) {
    if(addr11>0) {
        addr11--;
    }
    sendAccessory( (addr11>>2) + 1U, addr11 & 0x3, thrown);
}

void IDCCChannel::sendAccessory(uint16_t addr9, uint8_t ch, bool thrown) {
    DCC_LOGI("addr9=%d, ch=%d, %c", addr9, ch, thrown?'T':'C');

    auto b = make_accessory_packet(addr9, ch, thrown);

    loadPacket(0, b.data(), b.size(), 4);
}

uint IDCCChannel::getBaselineCurrent() const {
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
bool IDCCChannel::checkCurrentResponse(uint baseline) const {
    bool ret = false;
    int max = 0;
    delay(ACK_SAMPLE_MILLIS);
    max = getMaxCurrent();
    ret = max - baseline > ACK_SAMPLE_THRESHOLD;
    DCC_LOGD("result is %d, max: %d, baseline: %d", ret?1:0, max, baseline);
    return ret;
}

int16_t IDCCChannel::readCVProg(int cv) {
	uint8_t packet[4];
	int ret;

	cv--;                              // actual CV addresses are cv-1 (0-1023)

	packet[0] = 0x78 | (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
	packet[1] = lowByte(cv);

	ret = 0;

    int baseline = getBaselineCurrent();

	for (uint8_t i = 0; i<8; i++) {
		packet[2] = 0xE8 | i;

		loadPacket(0, resetPacket, 2, 3);          // NMRA recommends starting with 3 reset packets
        resetMaxCurrent();
		loadPacket(0, packet, 3, 5);               // NMRA recommends 5 verify packets
		loadPacket(0, resetPacket, 2, 1);          // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

        bool bitVal = checkCurrentResponse(baseline);
        if(bitVal) bitSet(ret, i);

        DCC_LOGD("Reading bit %d, value is %d", i, bitVal?1:0);
	}

    return verifyCVByteProg(cv+1, ret) ? ret : -1;

}

bool IDCCChannel::verifyCVByteProg(uint16_t cv, uint8_t bValue) {
    DCC_LOGI("Verifying cv%d==%d", cv, bValue);
    uint8_t packet[4];

    cv--;

    packet[0] = 0x74 | (highByte(cv) & 0x03);
    packet[1] = lowByte(cv);
	packet[2] = bValue;

    loadPacket(0, resetPacket, 2, 1);    // NMRA recommends starting with 3 reset packets
    loadPacket(0, resetPacket, 2, 3);
    uint baseline = getBaselineCurrent();
    resetMaxCurrent();
	loadPacket(0, packet, 3, 5);         // NMRA recommends 5 verify packets
	loadPacket(0, resetPacket, 2, 1);    // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

bool IDCCChannel::writeCVByteProg(int cv, uint8_t bValue) {
    uint8_t packet[4];
    uint baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)

    packet[0]=0x7C | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1]=lowByte(cv);
    packet[2]=bValue;

    loadPacket(0,resetPacket,2,1);
    loadPacket(0,packet,3,4);
    loadPacket(0,resetPacket,2,1);
    loadPacket(0,idlePacket,2,10);

    baseline = getBaselineCurrent();

    packet[0]=0x74 | (highByte(cv)&0x03);   // set-up to re-verify entire byte

    loadPacket(0,resetPacket,2,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    loadPacket(0,packet,3,5);               // NMRA recommends 5 verfy packets
    loadPacket(0,resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

bool IDCCChannel::writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue){
    uint8_t packet[4];
    uint baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)
    bValue &= 0x1;
    bNum &= 0x7;

    packet[0] = 0x78 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1] = lowByte(cv);
    packet[2] = 0xF0 | bValue<<3 | bNum;

    loadPacket(0,resetPacket,2,1);
    loadPacket(0,packet,3,4);
    loadPacket(0,resetPacket,2,1);
    loadPacket(0,idlePacket,2,10);

    baseline = getBaselineCurrent();

    bitClear(packet[2],4);              // change instruction code from Write Bit to Verify Bit

    loadPacket(0,resetPacket,2,3);          // NMRA recommends starting with 3 reset packets
    resetMaxCurrent();
    loadPacket(0,packet,3,5);               // NMRA recommends 5 verfy packets
    loadPacket(0,resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

void IDCCChannel::writeCVByteMain(LocoAddress addr, int cv, uint8_t bValue) {
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

    loadPacket(0,packet,nB,4);

}

void IDCCChannel::writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue) {
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

    loadPacket(0,b,nB,4);

}





//char _msg[1024];
//char _buf[100];
/*
void DCCESP32Channel::PacketSlot::initPackets(){
    activePacket = packet;
    updatePacket = packet+1;
} */


static DCCESP32SignalGenerator * _inst = nullptr;


void IRAM_ATTR timerCallback() {
    _inst->timerFunc();
}

void adcTimerCallback(void* arg) {
    ((DCCESP32SignalGenerator*)arg)->adcTimerFunc();
}


DCCESP32SignalGenerator::DCCESP32SignalGenerator(uint8_t timerNum)
    : _timerNum(timerNum)
{
    _inst = this;
}

void DCCESP32SignalGenerator::begin() {
    if (main!=nullptr) main->begin();
    if (prog!=nullptr) prog->begin();

    (void)_timerNum;
    _timer = timerBegin(1000000 / 58); // 58us
    timerAttachInterrupt(_timer, timerCallback);
    timerAlarm(_timer, 10, true, 0);
    timerStart(_timer);

    esp_timer_create_args_t cfg{adcTimerCallback, this, ESP_TIMER_TASK, "adc"};
    esp_timer_create(&cfg, &_adcTimer);
    esp_timer_start_periodic(_adcTimer, 1000);  // 1ms
}

void DCCESP32SignalGenerator::end() {
    if(_timer!=nullptr) {
        timerStop(_timer);
        timerEnd(_timer);
        _timer = nullptr;
    }
    esp_timer_stop(_adcTimer);
    esp_timer_delete(_adcTimer);
    if (main!=nullptr) main->end();
    if (prog!=nullptr) prog->end();
}

IRAM_ATTR void DCCESP32SignalGenerator::timerFunc() {

    if (main!=nullptr) main->timerFunc();
    if (prog!=nullptr) prog->timerFunc();

}

void DCCESP32SignalGenerator::adcTimerFunc() {
    if (main!=nullptr) main->updateCurrent();
    if (prog!=nullptr) prog->updateCurrent();
}
