#include "DCC.h"

uint8_t idlePacket[3] = {0xFF, 0x00, 0}; 
uint8_t resetPacket[3] = {0x00, 0x00, 0};

#define  ACK_BASE_COUNT            100      /**< Number of analogRead samples to take before each CV verify to establish a baseline current.*/
#define  ACK_SAMPLE_COUNT          500      /**< Number of analogRead samples to take when monitoring current after a CV verify (bit or byte) has been sent.*/ 
#define  ACK_SAMPLE_SMOOTHING      0.8      /**< Exponential smoothing to use in processing the analogRead samples after a CV verify (bit or byte) has been sent.*/
#define  ACK_SAMPLE_THRESHOLD       10      /**< The threshold that the exponentially-smoothed analogRead samples (after subtracting the baseline current) must cross to establish ACKNOWLEDGEMENT.*/
#define  ACK_MUL   1


void IDCCChannel::setThrottle(int iReg, int addr, uint8_t tSpeed, uint8_t tDirection){
    uint8_t b[5];                         // save space for checksum byte
    uint8_t nB = 0;

    if (addr>127)
        b[nB++] = highByte(addr) | 0xC0;  // convert train number into a two-byte address

    b[nB++] = lowByte(addr);
    b[nB++] = B00111111;  // 128-step speed control byte (0x3F)
    b[nB++] = (tSpeed & B01111111) | ( (tDirection & 1) << 7); 
    
    DCC_LOGI("DCC::setThrottle iReg %d, addr %d, speed=%d %c", addr, addr, tSpeed, tDirection==1?'F':'B');
    
    loadPacket(iReg, b, nB, 0);
}
void IDCCChannel::setFunctionGroup(int iReg, int addr, DCCFnGroup group, uint32_t fn) {
    DCC_LOGI("DCC::setFunctionGroup iReg %d, addr %d, group=%d fn=%08x", iReg, addr, (uint8_t)group, fn);
    switch(group) {
        case DCCFnGroup::F0_4: 
            // move FL(F0) to 5th bit
            fn = (fn & 1)<<4 | (fn & 0x1E)<<1;
            setFunction(iReg, addr,  B10000000 | (fn & B00011111) );
            break;
        case DCCFnGroup::F5_8:
            fn >>= 5;
            setFunction(iReg, addr,  B10110000 | (fn & B00001111) );
            break;
        case DCCFnGroup::F9_12:
            fn >>= 9;
            setFunction(iReg, addr,  B10100000 | (fn & B00001111) );
            break;
        case DCCFnGroup::F13_20:
            fn >>= 13; 
            setFunction(iReg, addr, B11011110, (uint8_t)fn );
            break;
        case DCCFnGroup::F21_28:
            fn >>= 21; 
            setFunction(iReg, addr, B11011111, (uint8_t)fn );
            break;
    }     

}
void IDCCChannel::setFunction(int iReg, int addr, uint8_t fByte, uint8_t eByte) {
    // save space for checksum byte
    uint8_t b[5]; 
    uint8_t nB = 0;

    if (addr>127)
        b[nB++] = highByte(addr) | 0xC0;  // convert train number into a two-byte address

    b[nB++] = lowByte(addr);

    if ( (fByte & B11000000) == B10000000) {// this is a request for functions FL,F1-F12  
        b[nB++] = (fByte | 0x80) & 0xBF; // for safety this guarantees that first nibble of function byte will always be of binary form 10XX which should always be the case for FL,F1-F12  
    } else {                             // this is a request for functions F13-F28
        b[nB++] = (fByte | 0xDE) & 0xDF; // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
        b[nB++] = eByte;
    }

    DCC_LOGI("DCC::setFunction iReg %d, addr %d, fByte=%02x eByte=%02x", iReg, addr, fByte, eByte);

    /* 
    NMRA DCC norm ask for two DCC packets instead of only one:
    "Command Stations that generate these packets, and which are not periodically refreshing these functions,
    must send at least two repetitions of these commands when any function state is changed."
    https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
    */
    loadPacket(0, b, nB, 4);

}
void IDCCChannel::setAccessory(int aAdd, int aNum, int activate) {
    DCC_LOGI("DCC::setAccessory addr=%d; ch=%d; state=%d", aAdd, aNum, activate);

    uint8_t b[3];                      // save space for checksum byte

    b[0] = aAdd % 64 + 128;            // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
    b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

    loadPacket(0, b, 2, 4);
}

uint IDCCChannel::getBaselineCurrent() {
    uint baseline = 0;    
    // collect baseline current
    for (int j = 0; j < ACK_BASE_COUNT; j++) baseline += readCurrent();
    baseline /= ACK_BASE_COUNT;
    return baseline;
}

bool IDCCChannel::checkCurrentResponse(uint baseline) {
    bool ret = false;
    int c = 0, v;
    for (int j = 0; j<ACK_SAMPLE_COUNT; j++) {
        v = readCurrent();
        c = (int)(ACK_MUL*(v - baseline)*ACK_SAMPLE_SMOOTHING + c*(1.0 - ACK_SAMPLE_SMOOTHING));
        if (c>ACK_SAMPLE_THRESHOLD*ACK_MUL) {
            ret = true;
        }
    }
    return ret;
}

int16_t IDCCChannel::readCVProg(int cv) {
	uint8_t packet[4];
	int ret;
	int baseline;
    bool bitVal;

	cv--;                              // actual CV addresses are cv-1 (0-1023)

	packet[0] = 0x78 | (highByte(cv) & 0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
	packet[1] = lowByte(cv);

	ret = 0;

	for (int i = 0; i<8; i++) {
        //Serial.print("bit "+String(i) );

        baseline = getBaselineCurrent();
		
		packet[2] = 0xE8 + i;

		loadPacket(0, resetPacket, 2, 3);          // NMRA recommends starting with 3 reset packets
		loadPacket(0, packet, 3, 5);                // NMRA recommends 5 verify packets
		loadPacket(0, resetPacket, 2, 1);          // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

        bitVal = checkCurrentResponse(baseline);
		bitWrite(ret, i, bitVal?1:0);
		//Serial.println("");
	}

	bitVal = 0;
	baseline = getBaselineCurrent();

	packet[0] = 0x74 | highByte(cv) & 0x03;   // set-up to re-verify entire byte
	packet[2] = ret;

	loadPacket(0, resetPacket, 2, 3);          // NMRA recommends starting with 3 reset packets
	loadPacket(0, packet, 3, 5);                // NMRA recommends 5 verify packets
	loadPacket(0, resetPacket, 2, 1);          // forces code to wait until all repeats of packet are completed (and decoder begins to respond)

	//Serial.print("verify "+String(ret)+": " );
    bitVal = checkCurrentResponse(baseline);

	//Serial.println("");

	if (!bitVal)    // verify unsuccessful
		ret = -1;

	return ret;
}

bool IDCCChannel::writeCVByteProg(int cv, uint8_t bValue) {
    uint8_t packet[4];
    int baseline;

    cv--;                              // actual CV addresses are cv-1 (0-1023)

    packet[0]=0x7C | highByte(cv)&0x03;   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1]=lowByte(cv);
    packet[2]=bValue;

    loadPacket(0,resetPacket,2,1);
    loadPacket(0,packet,3,4);
    loadPacket(0,resetPacket,2,1);
    loadPacket(0,idlePacket,2,10);

    baseline = getBaselineCurrent();

    packet[0]=0x74+(highByte(cv)&0x03);   // set-up to re-verify entire byte

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
    bNum &= 0x3;
    
    packet[0]=0x78 | (highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[1]=lowByte(cv);  
    packet[2]=0xF0 | bValue<<3 | bNum;

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

    uint16_t adr = addr.addr();
    if( addr.isLong() )    
        packet[nB++]=highByte(adr) | 0xC0;      // convert train number into a two-byte address

    packet[nB++]=lowByte(adr);
    packet[nB++]=0xEC+(highByte(cv)&0x03);   // any CV>1023 will become modulus(1024) due to bit-mask of 0x03
    packet[nB++]=lowByte(cv);
    packet[nB++]=bValue;

    loadPacket(0,packet,nB,4);

}

void IDCCChannel::writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue) {
    uint8_t b[6];                      // save space for checksum byte

    byte nB=0;
    
    cv--;
    
    bValue &= 0x1;
    bNum &= 0x3;

    uint16_t adr = addr.addr();
    if( addr.isLong() )  
        b[nB++] = highByte(adr) | 0xC0;      // convert train number into a two-byte address
  
    b[nB++]=lowByte(adr);
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
    : _timerNum(timerNum) 
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
        if(timerStarted(_timer) ) timerStop(_timer);
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
