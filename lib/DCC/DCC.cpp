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
#define  ACK_SAMPLE_THRESHOLD      500      /**< The threshold that the exponentially-smoothed analogRead samples (after subtracting the baseline current) must cross to establish ACKNOWLEDGEMENT.*/


void IDCCChannel::copyPacket(uint8_t *src, uint8_t nBytes, int nRepeat, Packet *dst) {
    uint8_t *buf = dst->buf;

    // copy first byte into what will become the checksum byte 
    // XOR remaining bytes into checksum byte 
    src[nBytes] = src[0];                        
    for(int i=1; i<nBytes; i++)              
        src[nBytes]^=src[i];
    nBytes++;  // increment number of bytes in packet to include checksum byte
        
    buf[0] = 0xFF;                        // first 8 bits of 22-bit preamble
    buf[1] = 0xFF;                        // second 8 bits of 22-bit preamble
    buf[2] = 0xFC | bitRead(src[0],7);      // last 6 bits of 22-bit preamble + data start bit + src[0], bit 7
    buf[3] = src[0]<<1;                     // src[0], bits 6-0 + data start bit
    buf[4] = src[1];                        // src[1], all bits
    buf[5] = src[2]>>1;                     // start bit + src[2], bits 7-1
    buf[6] = src[2]<<7;                     // src[2], bit 0
    
    if(nBytes == 3) {
        dst->nBits = 49;
    } else {
        buf[6] |= src[3]>>2;    // src[3], bits 7-2
        buf[7] =  src[3]<<6;    // src[3], bit 1-0
        if(nBytes==4) {
            dst->nBits = 58;
        } else {
            buf[7] |= src[4]>>3;  // src[4], bits 7-3
            buf[8] =  src[4]<<5;   // src[4], bits 2-0
            if(nBytes==5) {
                dst->nBits = 67;
            } else {
                buf[8] |= src[5]>>4;   // src[5], bits 7-4
                buf[9] =  src[5]<<4;   // src[5], bits 3-0
                dst->nBits = 76;
            } 
        } 
    }
    dst->nRepeat = nRepeat; 
    dst->debugPrint();  
}

void IDCCChannel::sendThrottle(int iReg, LocoAddress addr, uint8_t tSpeed, SpeedMode sm, uint8_t tDirection){
    uint8_t b[5];                         // save space for checksum byte
    uint8_t nB = 0;

    uint16_t iAddr = addr.addr();
    if ( addr.isLong() ) {
        b[nB++] = highByte(iAddr) | 0xC0;  // convert train number into a two-byte address
    }

    b[nB++] = lowByte(iAddr);
    if(sm==SpeedMode::S128) {
        // Advanced Operations Instruction: https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf #200
        b[nB++] = 0b00111111; 
        b[nB++] = (tSpeed & 0x7F) | ( (tDirection & 0x1) << 7); 
    } else {
        // basic packet: https://www.nmra.org/sites/default/files/s-92-2004-07.pdf #35
        uint8_t t=nB;
        b[nB++] = 0b01000000;
        if(tDirection==1) b[t] |= 0b00100000;
        if(sm==SpeedMode::S14) b[t] |= tSpeed & 0b00001111;
        if(sm==SpeedMode::S28) b[t] |= (tSpeed & 1)<<4 | (tSpeed & 0b11110)>>1;
    }
    
    DCC_LOGI("iReg %d, addr %d, speed=%d(mode %d) %c", iReg, addr, tSpeed, (int)sm, (tDirection==1)?'F':'B');
    
    loadPacket(iReg, b, nB, 0);
}

void IDCCChannel::sendFunctionGroup(int iReg, LocoAddress addr, DCCFnGroup group, uint32_t fn) {
    DCC_LOGI("iReg %d, addr %d, group=%d fn=%08x", iReg, addr, (uint8_t)group, fn);
    switch(group) {
        case DCCFnGroup::F0_4: 
            // move FL(F0) to 5th bit
            fn = (fn & 0x1)<<4 | (fn & 0b11110)>>1;
            sendFunction(iReg, addr,  0b10000000 | (fn & 0b00011111) );
            break;
        case DCCFnGroup::F5_8:
            fn >>= 5;
            sendFunction(iReg, addr,  0b10110000 | (fn & 0b00001111) );
            break;
        case DCCFnGroup::F9_12:
            fn >>= 9;
            sendFunction(iReg, addr,  0b10100000 | (fn & 0b00001111) );
            break;
        case DCCFnGroup::F13_20:
            fn >>= 13; 
            sendFunction(iReg, addr,  0b11011110, (uint8_t)fn );
            break;
        case DCCFnGroup::F21_28:
            fn >>= 21; 
            sendFunction(iReg, addr,  0b11011111, (uint8_t)fn );
            break;
        default:
            break;
    }     

}
void IDCCChannel::sendFunction(int iReg, LocoAddress addr, uint8_t fByte, uint8_t eByte) {
    // save space for checksum byte
    uint8_t b[5]; 
    uint8_t nB = 0;
    uint16_t iAddr = addr.addr();

    if (addr.isLong()) {
        b[nB++] = highByte(iAddr) | 0xC0;  // convert train number into a two-byte address
    }

    b[nB++] = lowByte(iAddr);

    if ( (fByte & 0b11000000) == 0b10000000) {// this is a request for functions FL,F1-F12  
        b[nB++] = (fByte | 0x80) & 0xBF; // for safety this guarantees that first nibble of function byte will always be of binary form 10XX which should always be the case for FL,F1-F12  
    } else {                             // this is a request for functions F13-F28
        b[nB++] = (fByte | 0xDE) & 0xDF; // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
        b[nB++] = eByte;
    }

    DCC_LOGI("iReg %d, addr %d, fByte=%02x eByte=%02x", iReg, addr, fByte, eByte);

    /* 
    NMRA DCC norm ask for two DCC packets instead of only one:
    "Command Stations that generate these packets, and which are not periodically refreshing these functions,
    must send at least two repetitions of these commands when any function state is changed."
    https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
    */
    loadPacket(0, b, nB, 4);

}

void IDCCChannel::sendAccessory(uint16_t addr11, bool thrown) {
    if(addr11>0) {
        addr11--;
    }
    sendAccessory( (addr11>>2) + 1U, addr11 & 0x3, thrown);
}

void IDCCChannel::sendAccessory(uint16_t addr9, uint8_t ch, bool thrown) {
    DCC_LOGI("addr9=%d, ch=%d, %c", addr9, ch, thrown?'T':'C');

    uint8_t b[3];     // save space for checksum byte

    /*
    first byte is of the form 10AAAAAA, where AAAAAA represent
    6 least significant bits of accessory address (9-bit. Here we have 14-bit address, so take bits 2-7) */
    b[0] = ( addr9 & 0x3F) | 0x80;      
    /*
    "The most significant bits of the 9-bit address are bits 4-6 of the second data byte. 
    By convention these bits (bits 4-6 of the second data byte) are in ones complement. "
    https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
    */
    // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent throw/close
    b[1] = ( ((addr9>>6 & 0x7) << 4 ) ^ 0b01110000 )
        | (ch & 0x3) << 1 
        | (thrown?0x1:0) 
        | 0b10000000   ;

    loadPacket(0, b, 2, 4);
}

uint IDCCChannel::getBaselineCurrent() {
    uint baseline = 0;

    // collect baseline current
    for (int j = 0; j < ACK_BASE_COUNT; j++) {
        uint16_t v = readCurrentAdc();
        baseline += v;
    }
    baseline /= ACK_BASE_COUNT;
    return baseline;
}

// https://www.nmra.org/sites/default/files/s-9.2.3_2012_07.pdf
bool IDCCChannel::checkCurrentResponse(uint baseline) {
    bool ret = false;
    float c = 0;
    int max = 0;
    uint32_t to = millis()+ACK_SAMPLE_MILLIS;
    while(millis()<to) {    
        int v = readCurrentAdc();
        v-= baseline;
        c = v*ACK_SAMPLE_SMOOTHING + c*(1.0 - ACK_SAMPLE_SMOOTHING);
        if(c>max) { max=(int)c; }
        if (c>ACK_SAMPLE_THRESHOLD) {
            ret = true;
        }
    }
    DCC_LOGD("result is %d, last value:%d, max: %d, baseline: %d", ret?1:0, max, baseline);
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
    int baseline = getBaselineCurrent();
	loadPacket(0, packet, 3, 5);         // NMRA recommends 5 verify packets
	loadPacket(0, resetPacket, 2, 1);    // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

}

bool IDCCChannel::writeCVByteProg(int cv, uint8_t bValue) {
    uint8_t packet[4];
    int baseline;

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
    loadPacket(0,packet,3,5);               // NMRA recommends 5 verfy packets
    loadPacket(0,resetPacket,2,1);          // forces code to wait until all repeats of bRead are completed (and decoder begins to respond)

    return checkCurrentResponse(baseline);

} 

bool IDCCChannel::writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue){
    uint8_t packet[4];
    int baseline;
    
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


DCCESP32SignalGenerator::DCCESP32SignalGenerator(uint8_t timerNum) 
    : _timer(nullptr),  _timerNum(timerNum)
{
    _inst = this;
}

void DCCESP32SignalGenerator::begin() {
    if (main!=nullptr) main->begin();
    if (prog!=nullptr) prog->begin();

    _timer = timerBegin(_timerNum, 464, true);
    timerAttachInterrupt(_timer, timerCallback, true);
    timerAlarmWrite(_timer, 10, true);
    timerAlarmEnable(_timer);
    timerStart(_timer);
}

void DCCESP32SignalGenerator::end() {
    if(_timer!=nullptr) {
        if(timerStarted(_timer) ) { timerStop(_timer); }
        timerEnd(_timer);
        _timer = nullptr;
    }
    if (main!=nullptr) main->end();
    if (prog!=nullptr) prog->end();
}

void DCCESP32SignalGenerator::timerFunc() {
    
    if (main!=nullptr) main->timerFunc();
    if (prog!=nullptr) prog->timerFunc();

}
