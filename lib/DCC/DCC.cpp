#include "DCC.h"

uint8_t DCCESP32Channel::RegisterList::idlePacket[3] = {0xFF, 0x00, 0}; 
uint8_t DCCESP32Channel::RegisterList::resetPacket[3] = {0x00, 0x00, 0};

//char _msg[1024];
//char _buf[100];

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

    DCC_DEBUGF("DCCESP32Channel::loadPacket reg=%d len=%d, repeat=%d\n", nReg, nBytes, nRepeat);

    // force nReg to be between 0 and maxNumRegs, inclusive
    nReg = nReg % (maxNumRegs+1);

    // pause while there is a Register already waiting to be updated -- nextReg will be reset to NULL by timer when prior Register updated fully processed
    while(nextReg != nullptr) delay(1);             
    
    // first time this Register Number has been called
    // set Register Pointer for this Register Number to next available Register
    if(regMap[nReg] == nullptr) {        
        regMap[nReg] = maxLoadedReg + 1;   
        DCC_DEBUGF("loadPacket:: Allocating new reg %d\n", nReg);
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
    buf[2] = 0xFC | bitRead(b[0],7);      // last 6 bits of 22-bit preamble + data start bit + b[0], bit 7
    buf[3] = b[0]<<1;                     // b[0], bits 6-0 + data start bit
    buf[4] = b[1];                        // b[1], all bits
    buf[5] = b[2]>>1;                     // start bit + b[2], bits 7-1
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
    this->nRepeat = nRepeat; // bug: if timer uses this var to send another packet, that packet will be sent more times.
    maxLoadedReg = max(maxLoadedReg, nextReg);

    char ttt[50] = {0};
    for(int i=0; i<p->nBits/8+1; i++) {
        snprintf(ttt, 50, "%s %02x", ttt, buf[i]);
    }
    DCC_DEBUGF("loadPacket: %s into Reg %d, packet %d\n", ttt, nReg, p-&r->packet[0] );

}

#define ARR_BIT(arr, bit)   ( (arr)[(bit)/8] & (1<<(7-(bit)%8) ) )

void DCCESP32Channel::nextBit() {
    //const uint8_t bitMask[] = {  0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01  };
    DCC_DEBUGF_ISR("nextBit: currentReg=%d, activePacket=%d, cbit=%d, len=%d \n", (R.currentReg-&R.reg[0]), 
        (R.currentReg->activePacket-&R.currentReg->packet[0]), R.currentBit, R.currentReg->activePacket->nBits );

    // IF no more bits in this DCC Packet, reset current bit pointer and determine which Register and Packet to process next  
	if(R.currentBit==R.currentReg->activePacket->nBits) {
		R.currentBit = 0;
        // IF current Register is first Register AND should be repeated, decrement repeat count; result is this same Packet will be repeated                             
		if (R.nRepeat>0 && R.currentReg == &R.reg[0]) {        
			R.nRepeat--;    
            DCC_DEBUGF_ISR("nextBit: repeat packet = %d\n", R.nRepeat);                              
		}  else {
            // IF another Register has been updated, update currentReg to nextReg and reset nextReg to NULL 
            if (R.nextReg != NULL){                      
                R.currentReg = R.nextReg;                     
                R.nextReg = NULL;         
                // flip active and update Packets
                Packet *tmp = R.currentReg->activePacket;    
                R.currentReg->activePacket = R.currentReg->updatePacket; 
                R.currentReg->updatePacket = tmp; 
                tmp = R.currentReg->activePacket;                
                DCC_DEBUGF_ISR("nextBit: advance to nextReg %d, packet = (%d bits) %02x %02x %02x...\n", 
                    (R.currentReg-&R.reg[0]),    tmp->nBits, tmp->buf[2], tmp->buf[3], tmp->buf[4] );
            } else {    
                // ELSE simply move to next Register    
                // BUT IF this is last Register loaded, first reset currentReg to base Register, THEN       
                // increment current Register (note this logic causes Register[0] to be skipped when simply cycling through all Registers)  
                
                if (R.currentReg == R.maxLoadedReg)
                    R.currentReg = &R.reg[0];
                R.currentReg++;                        
                    
                DCC_DEBUGF_ISR("nextBit: advance currentReg=%d\n", (R.currentReg-&R.reg[0]) );
            }      
        }                                        
	} // currentReg, activePacket, and currentBit should now be properly set to point to next DCC bit

	if( ARR_BIT(R.currentReg->activePacket->buf, R.currentBit) ) {  
		/* For "1" bit, we need 1 periods of 58us timer ticks for each signal level */ 
        DCC_DEBUGF_ISR("nextBit: bit %d = 1\n", R.currentBit );
		R.timerPeriodsHalf = 1; 
		R.timerPeriodsLeft = 2; 
	} else {  /* ELSE it is a ZERO bit */ 
		/* For "0" bit, we need 2 period of 58us timer ticks for each signal level */ 
        DCC_DEBUGF_ISR("nextBit: bit %d = 0\n", R.currentBit );
		R.timerPeriodsHalf = 2; 
		R.timerPeriodsLeft = 4; 
	} 

	R.currentBit++; 
}


void DCCESP32Channel::timerFunc() {
    R.timerPeriodsLeft--;
    //DCC_DEBUGF_ISR("DCCESP32Channel::timerFunc, periods left: %d, total: %d\n", R.timerPeriodsLeft, R.timerPeriodsHalf*2);                    
	if(R.timerPeriodsLeft == R.timerPeriodsHalf) {
        digitalWrite(_outputPin, HIGH );
	}                                              
	if(R.timerPeriodsLeft == 0) {                  
		digitalWrite(_outputPin, LOW );
		nextBit();                           
	}

    //current = readCurrent(); 
}



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
