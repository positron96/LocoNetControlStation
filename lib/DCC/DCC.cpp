#include "DCC.h"

uint8_t idlePacket[3] = {0xFF, 0x00, 0}; 
uint8_t resetPacket[3] = {0x00, 0x00, 0};


void IDCCChannel::setThrottle(int slot, int addr, uint8_t tSpeed, uint8_t tDirection){
    uint8_t b[5];                         // save space for checksum byte
    uint8_t nB = 0;

    if (addr>127)
        b[nB++] = highByte(addr) | 0xC0;  // convert train number into a two-byte address

    b[nB++] = lowByte(addr);
    b[nB++] = B00111111;  // 128-step speed control byte (0x3F)
    b[nB++] = (tSpeed & B01111111) | ( (tDirection & 1) << 7); 
    
    DCC_DEBUGF("DCC::setThrottle slot %d, addr %d, speed=%d %c\n", addr, addr, tSpeed, tDirection==1?'F':'B');
    
    loadPacket(slot, b, nB, 0);
}
void IDCCChannel::setFunctionGroup(int slot, int addr, DCCFnGroup group, uint32_t fn){
    DCC_DEBUGF("DCC::setFunctionGroup slot %d, addr %d, group=%d fn=%08x\n", slot, addr, (uint8_t)group, fn);
    switch(group) {
        case DCCFnGroup::F0_4: 
            setFunction(slot, addr,  B10000000 | (fn & B00011111) );
            break;
        case DCCFnGroup::F5_8:
            fn >>= 5;
            setFunction(slot, addr,  B10110000 | (fn & B00001111) );
            break;
        case DCCFnGroup::F9_12:
            fn >>= 9;
            setFunction(slot, addr,  B10100000 | (fn & B00001111) );
            break;
        case DCCFnGroup::F13_20:
            fn >>= 13; 
            setFunction(slot, addr, B11011110, (uint8_t)fn );
            break;
        case DCCFnGroup::F21_28:
            fn >>= 21; 
            setFunction(slot, addr, B11011111, (uint8_t)fn );
            break;
    }     

}
void IDCCChannel::setFunction(int slot, int addr, uint8_t fByte, uint8_t eByte) {
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

    DCC_DEBUGF("DCC::setFunction slot %d, addr %d, fByte=%02x eByte=%02x\n", slot, addr, fByte, eByte);

    /* 
    NMRA DCC norm ask for two DCC packets instead of only one:
    "Command Stations that generate these packets, and which are not periodically refreshing these functions,
    must send at least two repetitions of these commands when any function state is changed."
    https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
    */
    loadPacket(slot, b, nB, 4);

}
void IDCCChannel::setAccessory(int aAdd, int aNum, int activate) {
    DCC_DEBUGF("DCC::setAccessory addr=%d; ch=%d; state=%d\n", aAdd, aNum, activate);

    uint8_t b[3];                      // save space for checksum byte

    b[0] = aAdd % 64 + 128;            // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
    b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

    loadPacket(0, b, 2, 4);
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
