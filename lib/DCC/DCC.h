#pragma once

#include <Arduino.h>
#include <esp32-hal-timer.h>

#define DCC_DEBUG_

#ifdef DCC_DEBUG
#define DCC_DEBUGF(...)  do{ Serial.printf(__VA_ARGS__); }while(0)
#define DCC_DEBUGF_ISR(...)  do{ Serial.printf(__VA_ARGS__); }while(0)
//extern char _msg[1024];
//extern char _buf[100];
//#define DCC_DEBUG_ISR(...)  do{ snprintf(_buf, 100, __VA_ARGS__); snprintf(_msg, 1024, "%s%s\n", _msg, _buf ); } while(0)
//#define DCC_DEBUG_ISR_DUMP()  do{ Serial.print(_msg); _msg[0]=0; } while(0);
#else
#define DCC_DEBUGF(...)
#define DCC_DEBUGF_ISR(...) 
#endif

enum class DCCFnGroup {
    F0_4, F5_8, F9_12, F13_20, F21_28
};

class DCCESP32Channel {
public:

    DCCESP32Channel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin, bool isOps=true): 
        _outputPin(outputPin), _enPin(enPin), _sensePin(sensePin), R(isOps?3:12)
    {
        R.timerPeriodsLeft=1; // first thing a timerfunc does is decrement this, so make it not underflow
        R.timerPeriodsHalf=2; // some nonzero sane value
    }

    void begin() {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_enPin, HIGH);

        R.loadPacket(1, RegisterList::idlePacket, 2, 0);
    }

    void end() {
        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v) {
        DCC_DEBUGF("DCC::setPower(%d)\n", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() {
        return digitalRead(_enPin) == HIGH;
    }

    struct Packet {
        uint8_t buf[10];
        uint8_t nBits;
    };

    struct Register {
        Packet packet[2];
        Packet *activePacket;
        Packet *updatePacket;
        void initPackets();
    };

    /** Define a series of registers that can be sequentially accessed over a loop to generate a repeating series of DCC Packets. */
    struct RegisterList {
        int maxNumRegs;
        Register *reg;
        Register **regMap;
        Register *currentReg;
        Register *maxLoadedReg;
        Register *nextReg;
        /* how many 58us periods needed for half-cycle (1 for "1", 2 for "0") */
        uint8_t timerPeriodsHalf;
        /* how many 58us periods are left (at start, 2 for "1", 4 for "0"). */
        uint8_t timerPeriodsLeft;

        uint8_t currentBit;
        uint8_t nRepeat;
        
        static uint8_t idlePacket[];
        static uint8_t resetPacket[];

        RegisterList(int);
        ~RegisterList();
        void loadPacket(int, uint8_t*, uint8_t, int);

    };

    void unload(uint8_t nReg) {
        //for()
    }


    void setThrottle(int nReg, int addr, uint8_t tSpeed, uint8_t tDirection) {
        uint8_t b[5];                         // save space for checksum byte
        uint8_t nB = 0;

        if (addr>127)
            b[nB++] = highByte(addr) | 0xC0;  // convert train number into a two-byte address

        b[nB++] = lowByte(addr);
        b[nB++] = B00111111;  // 128-step speed control byte (0x3F)
        b[nB++] = (tSpeed & B01111111) | ( (tDirection & 1) << 7); 
        
        DCC_DEBUGF("DCC::setThrottle slot %d, addr %d, speed=%d %c\n", addr, addr, tSpeed, tDirection==1?'F':'B');
        
        R.loadPacket(nReg, b, nB, 0);

    } 

    void setFunctionGroup(int nReg, int addr, DCCFnGroup group, uint32_t fn) {
        DCC_DEBUGF("DCC::setFunctionGroup slot %d, addr %d, group=%d fn=%08x\n", nReg, addr, (uint8_t)group, fn);
        switch(group) {
            case DCCFnGroup::F0_4: 
                setFunction(nReg, addr,  B10000000 | (fn & B00011111) );
                break;
            case DCCFnGroup::F5_8:
                fn >>= 5;
                setFunction(nReg, addr,  B10110000 | (fn & B00001111) );
                break;
            case DCCFnGroup::F9_12:
                fn >>= 9;
                setFunction(nReg, addr,  B10100000 | (fn & B00001111) );
                break;
            case DCCFnGroup::F13_20:
                fn >>= 13; 
                setFunction(nReg, addr, B11011110, (uint8_t)fn );
                break;
            case DCCFnGroup::F21_28:
                fn >>= 21; 
                setFunction(nReg, addr, B11011111, (uint8_t)fn );
                break;
        }        
    }


    void setFunction(int nReg, int addr, uint8_t fByte, uint8_t eByte=0)  {
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

        DCC_DEBUGF("DCC::setFunction slot %d, addr %d, fByte=%02x eByte=%02x\n", nReg, addr, fByte, eByte);

        /* 
        NMRA DCC norm ask for two DCC packets instead of only one:
        "Command Stations that generate these packets, and which are not periodically refreshing these functions,
        must send at least two repetitions of these commands when any function state is changed."
        https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
        */
        R.loadPacket(nReg, b, nB, 4);
    } 

    void setAccessory(int aAdd, int aNum, int activate) {

        DCC_DEBUGF("DCC::setAccessory addr=%d; ch=%d; state=%d\n", aAdd, aNum, activate);

        uint8_t b[3];                      // save space for checksum byte

        b[0] = aAdd % 64 + 128;            // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
        b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

        R.loadPacket(0, b, 2, 4);

    } 

    uint16_t readCurrent() {
        return analogRead(_sensePin);
    }

    void IRAM_ATTR timerFunc();

private:

    uint8_t _outputPin;    
    uint8_t _enPin;
    uint8_t _sensePin;

    uint16_t current;

    RegisterList R;

    void IRAM_ATTR nextBit();
    //void IRAM_ATTR timerFunc();

    friend class DCCESP32SignalGenerator;

};


class DCCESP32SignalGenerator {

public:
    DCCESP32SignalGenerator(uint8_t timerNum = 1);

    void setProgChannel(DCCESP32Channel * ch) { prog = ch;}
    void setMainChannel(DCCESP32Channel * ch) { main = ch;}

    /**
     * Starts half-bit timer.
     * To get 58us tick we need divisor of 58us/0.0125us(80mhz) = 4640,
     * separate this into 464 prescaler and 10 timer alarm.
     */
    void begin();

    void end();

private:
    hw_timer_t * _timer;
    volatile uint8_t _timerNum;
    DCCESP32Channel *main = nullptr;
    DCCESP32Channel *prog = nullptr;

    friend void timerCallback();

    void IRAM_ATTR timerFunc();
};
