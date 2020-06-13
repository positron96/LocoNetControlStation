#pragma once

#include <Arduino.h>
#include <esp32-hal-timer.h>

#define DCCPP_DEBUG

class DCCESP32Channel {
public:

    DCCESP32Channel(uint8_t outputPin=10, uint8_t sensePin=12, bool isOps=true, uint8_t timerId): 
        _outputPin(outputPin), _sensePin(sensePin)
    {
        _inst = this;
    }

    void begin() {
        
        _timer = timerBegin(_timerId, 480, true);

        timerAttachInterrupt(_timer, timerCallback, true);

        timerAlarmWrite(_timer, 10, true);
        
        pinMode(_outputPin, OUTPUT);
        //pinMode(_progOutputPin, OUTPUT);
    }

    void end() {

    }

    struct Packet{
        byte buf[10];
        byte nBits;
    }; // Packet

    struct Register{
        Packet packet[2];
        Packet *activePacket;
        Packet *updatePacket;
        void initPackets();
    }; // Register

    void setThrottle(int nReg, int addr, int tSpeed, int tDirection) volatile {
        uint8_t b[5];                         // save space for checksum byte
        uint8_t nB = 0;

        if (addr>127)
            b[nB++] = highByte(addr) | 0xC0;   // convert train number into a two-byte address

        b[nB++] = lowByte(addr);
        b[nB++] = 0x3F;                       // 128-step speed control byte
        if (tSpeed >= 0)
            b[nB++] = tSpeed + (tSpeed>0) + tDirection * 128;   // max speed is 126, but speed codes range from 2-127 (0=stop, 1=emergency stop)
        else {
            b[nB++] = 1;
            tSpeed = 0;
        }

        loadPacket(nReg, b, nB, 0);

    } 

    void setFunction(int nReg, int addr, int fByte, int eByte)  {
        uint8_t b[5];                        // save space for checksum byte
        uint8_t nB = 0;

        if (addr>127)
            b[nB++] = highByte(addr) | 0xC0;  // convert train number into a two-byte address

        b[nB++] = lowByte(addr);

        if (eByte < 0) {                     // this is a request for functions FL,F1-F12  
            b[nB++] = (fByte | 0x80) & 0xBF; // for safety this guarantees that first nibble of function byte will always be of binary form 10XX which should always be the case for FL,F1-F12  
        } else {                             // this is a request for functions F13-F28
            b[nB++] = (fByte | 0xDE) & 0xDF; // for safety this guarantees that first byte will either be 0xDE (for F13-F20) or 0xDF (for F21-F28)
            b[nB++] = eByte;
        }

        /* 
        NMRA DCC norm ask for two DCC packets instead of only one:
        "Command Stations that generate these packets, and which are not periodically refreshing these functions,
        must send at least two repetitions of these commands when any function state is changed."
        https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
        */
        loadPacket(nReg, b, nB, 4);
    } 

    void setAccessory(int aAdd, int aNum, int activate) {

        #ifdef DCCPP_DEBUG
            Serial.print("{RL::setAccessory addr="); Serial.print(aAdd);Serial.print("; ch=");
            Serial.print(aNum);Serial.print("; state=");Serial.print(activate);Serial.println("}");
        #endif

        uint8_t b[3];                      // save space for checksum byte

        b[0] = aAdd % 64 + 128;            // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
        b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

        loadPacket(0, b, 2, 4);

    } 

private:
    uint8_t _timerId;
    hw_timer_t * _timer;
    uint8_t _outputPin;    
    uint8_t _sensePin;

    uint8_t _cBit;
    uint8_t _cByte;
    uint8_t *_cPacket;
    uint8_t _packetLen;

    /** How manu timer periods are left for this bit. */
    uint8_t _ticksLeft;
    /** Timer tick value when to flip pin. Half of total bit length. */
    uint8_t _ticksBitFlip;

    static DCCESP32Channel * _inst;
    static void timerCallback() {
        _inst->timerFunc();
    }

    void IRAM_ATTR nextBit();
    void IRAM_ATTR timerFunc();

    void loadPacket(int nReg, uint8_t *b, int nBytes, int nRepeat) {
        nReg=nReg%((maxNumRegs+1));        // force nReg to be between 0 and maxNumRegs, inclusive

        while(nextReg!=NULL);              // pause while there is a Register already waiting to be updated -- nextReg will be reset to NULL by interrupt when prior Register updated fully processed
        
        if(regMap[nReg]==NULL)             // first time this Register Number has been called
            regMap[nReg]=maxLoadedReg+1;   // set Register Pointer for this Register Number to next available Register
        
        Register *r=regMap[nReg];          // set Register to be updated
        Packet *p=r->updatePacket;         // set Packet in the Register to be updated
        byte *buf=p->buf;                  // set byte buffer in the Packet to be updated
                
        b[nBytes]=b[0];                        // copy first byte into what will become the checksum byte  
        for(int i=1;i<nBytes;i++)              // XOR remaining bytes into checksum byte
            b[nBytes]^=b[i];
        nBytes++;                              // increment number of bytes in packet to include checksum byte
            
        buf[0]=0xFF;                        // first 8 bytes of 22-byte preamble
        buf[1]=0xFF;                        // second 8 bytes of 22-byte preamble
        buf[2]=0xFC + bitRead(b[0],7);      // last 6 bytes of 22-byte preamble + data start bit + b[0], bit 7
        buf[3]=b[0]<<1;                     // b[0], bits 6-0 + data start bit
        buf[4]=b[1];                        // b[1], all bits
        buf[5]=b[2]>>1;                     // b[2], bits 7-1
        buf[6]=b[2]<<7;                     // b[2], bit 0
        
        if(nBytes==3){
            p->nBits=49;
        } else{
            buf[6]+=b[3]>>2;                  // b[3], bits 7-2
            buf[7]=b[3]<<6;                   // b[3], bit 1-0
            if(nBytes==4){
            p->nBits=58;
            } else{
            buf[7]+=b[4]>>3;                // b[4], bits 7-3
            buf[8]=b[4]<<5;                 // b[4], bits 2-0
            if(nBytes==5){
                p->nBits=67;
            } else{
                buf[8]+=b[5]>>4;              // b[5], bits 7-4
                buf[9]=b[5]<<4;               // b[5], bits 3-0
                p->nBits=76;
            } // >5 bytes
            } // >4 bytes
        } // >3 bytes
        
        nextReg=r;
        this->nRepeat=nRepeat;
        maxLoadedReg=max(maxLoadedReg,nextReg);
        
        #ifdef DCCPP_DEBUG  
                printPacket(nReg,b,nBytes,nRepeat);  
        #endif
    }

}