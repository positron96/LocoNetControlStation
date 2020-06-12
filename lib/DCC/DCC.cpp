#include "DCC.h"


void DCCESP32Channel::nextBit() {
    if(_cBit==_packetLen*8) {
        _cBit=0;
        if(R.nRepeat>0 && R.currentReg==R.reg) { 
            R.nRepeat--;   
        } else if(R.nextReg!=NULL){    
            R.currentReg=R.nextReg;  
            R.nextReg=NULL;   
            R.tempPacket=R.currentReg->activePacket;  
            R.currentReg->activePacket=R.currentReg->updatePacket;
            R.currentReg->updatePacket=R.tempPacket;
        } else { 
            if(R.currentReg==R.maxLoadedReg)   
            R.currentReg=R.reg;     
            R.currentReg++;   
        }                                                    
    }                                                       

    if(R.currentReg->activePacket->buf[R.currentBit/8] & R.bitMask[R.currentBit%8]) {  
        _ticksBitFlip = 1; 
        _ticksLeft = 2; 
    } else {     
        _ticksBitFlip = 2; 
        _ticksLeft = 4; 
    }   

    _cBit++;
}


void DCCESP32Channel::timerFunc() {
    _ticksLeft--;  
    if(_ticksLeft == _ticksBitFlip) { 
        *DCC_SIGNAL_PORTIN_ ## N = DCC_SIGNAL_PORTMASK_ ## N; /* toggle pin */  
    }  
    if(_ticksLeft == 0) {  
        *DCC_SIGNAL_PORTIN_ ## N = DCC_SIGNAL_PORTMASK_ ## N;
        nextBit();
    }  
}