#include "DCC.h"

DCCESP32SignalGenerator * DCCESP32SignalGenerator::_inst = nullptr;

uint8_t DCCESP32Channel::RegisterList::idlePacket[3] = {0xFF, 0x00, 0}; 
uint8_t DCCESP32Channel::RegisterList::resetPacket[3] = {0x00, 0x00, 0};


void DCCESP32Channel::Register::initPackets(){
    activePacket = packet;
    updatePacket = packet+1;
} 

DCCESP32Channel::RegisterList::RegisterList(int maxNumRegs) {
    this->maxNumRegs = maxNumRegs;
    reg = (Register *)calloc((maxNumRegs+1),sizeof(Register));
    for (int i=0; i<=maxNumRegs; i++) reg[i].initPackets();
    regMap = (Register **)calloc((maxNumRegs+1),sizeof(Register *));
    currentReg = reg;
    regMap[0] = reg;
    maxLoadedReg = reg;
    nextReg = nullptr;
    currentBit = 0;
    nRepeat = 0;
} 

DCCESP32Channel::RegisterList::~RegisterList() {

}

void DCCESP32Channel::RegisterList::loadPacket(int nReg, uint8_t *b, uint8_t nBytes, int nRepeat) {
    // force nReg to be between 0 and maxNumRegs, inclusive
    nReg = nReg % (maxNumRegs+1);        

    // pause while there is a Register already waiting to be updated -- nextReg will be reset to NULL by timer when prior Register updated fully processed
    while(nextReg != nullptr) delay(1);             
    
    // first time this Register Number has been called
    // set Register Pointer for this Register Number to next available Register
    if(regMap[nReg] == nullptr) {        
        regMap[nReg] = maxLoadedReg + 1;   
        Serial.printf("loadPacket:: Allocating new reg %d\n", nReg);
    }
    
    Register *r = regMap[nReg];
    Packet *p = r->updatePacket;
    uint8_t *buf = p->buf;
            
    // copy first byte into what will become the checksum byte 
    // XOR remaining bytes into checksum byte 
    b[nBytes] = b[0];                        
    for(int i=1;i<nBytes;i++)              
        b[nBytes]^=b[i];
    nBytes++;  // increment number of bytes in packet to include checksum byte
        
    buf[0] = 0xFF;                        // first 8 bits of 22-bit preamble
    buf[1] = 0xFF;                        // second 8 bits of 22-bit preamble
    buf[2] = 0xFC | bitRead(b[0],7);      // last 8 bits of 22-bit preamble + data start bit + b[0], bit 7
    buf[3] = b[0]<<1;                     // b[0], bits 6-0 + data start bit
    buf[4] = b[1];                        // b[1], all bits
    buf[5] = b[2]>>1;                     // b[2], bits 7-1
    buf[6] = b[2]<<7;                     // b[2], bit 0
    
    if(nBytes == 3) {
        p->nBits = 49;
    } else {
        buf[6] |= b[3]>>2;    // b[3], bits 7-2
        buf[7] =  b[3]<<6;    // b[3], bit 1-0
        if(nBytes==4) {
            p->nBits = 58;
        } else {
            buf[7] |= b[4]>>3;  // b[4], bits 7-3
            buf[8] =  b[4]<<5;   // b[4], bits 2-0
            if(nBytes==5) {
                p->nBits = 67;
            } else {
                buf[8] |= b[5]>>4;   // b[5], bits 7-4
                buf[9] =  b[5]<<4;   // b[5], bits 3-0
                p->nBits = 76;
            } 
        } 
    } 
    
    nextReg = r;
    this->nRepeat = nRepeat;
    maxLoadedReg = max(maxLoadedReg, nextReg);
}


void DCCESP32Channel::nextBit() {
    const uint8_t bitMask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

    // IF no more bits in this DCC Packet, reset current bit pointer and determine which Register and Packet to process next  
	if(R.currentBit==R.currentReg->activePacket->nBits) {
		R.currentBit = 0;
        // IF current Register is first Register AND should be repeated, decrement repeat count; result is this same Packet will be repeated                             
		if (R.nRepeat>0 && R.currentReg == &R.reg[0]){        
			R.nRepeat--;                                  
		}  else {
            // IF another Register has been updated, update currentReg to nextReg and reset nextReg to NULL 
            if (R.nextReg != NULL){                      
                R.currentReg = R.nextReg;                     
                R.nextReg = NULL;         
                // flip active and update Packets
                Packet *tmp = R.currentReg->activePacket;    
                R.currentReg->activePacket = R.currentReg->updatePacket; 
                R.currentReg->updatePacket = tmp; 
            } else {    
                // ELSE simply move to next Register    
                // BUT IF this is last Register loaded, first reset currentReg to base Register, THEN       
                // increment current Register (note this logic causes Register[0] to be skipped when simply cycling through all Registers)                         
                if (R.currentReg == R.maxLoadedReg)
                    R.currentReg = &R.reg[0];
                R.currentReg++; 
            }      
        }                                        
	} // currentReg, activePacket, and currentBit should now be properly set to point to next DCC bit

	if(R.currentReg->activePacket->buf[R.currentBit/8] & bitMask[R.currentBit%8] ) {  
		/* For "1" bit, we need 1 periods of 58us timer ticks for each signal level */ 
		R.timerPeriods = 1; 
		R.timerPeriodsLeft = 2; 
	} else {  /* ELSE it is a ZERO bit */ 
		/* For "0" bit, we need 2 period of 58us timer ticks for each signal level */ 
		R.timerPeriods = 2; 
		R.timerPeriodsLeft = 4; 
	} 

	R.currentBit++; 
}


void DCCESP32Channel::timerFunc() {
    R.timerPeriodsLeft--;                          
	if(R.timerPeriodsLeft == R.timerPeriods) {
        digitalWrite(_outputPin, 1-digitalRead(_outputPin) );
	}                                              
	if(R.timerPeriodsLeft == 0) {                  
		digitalWrite(_outputPin, 1-digitalRead(_outputPin) );
		nextBit();                           
	}
    //current = readCurrent(); 
}