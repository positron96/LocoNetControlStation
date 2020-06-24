#pragma once

#include <Arduino.h>
#include <esp32-hal-timer.h>

#define DCC_DEBUG

class DCCESP32Channel {
public:

    DCCESP32Channel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin, bool isOps=true): 
        _outputPin(outputPin), _enPin(enPin), _sensePin(sensePin), R(isOps?3:12)
    {
        
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
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() {
        return digitalRead(_enPin) == HIGH;
    }

    struct Packet {
        uint8_t buf[10];
        uint8_t nBits;
    };

    struct Register{
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
        uint8_t timerPeriods;
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
        #ifdef DCC_DEBUG
            Serial.printf("setThrottle %d speed=%d %c\n", addr, tSpeed, tDirection==1?'F':'B');
        #endif
        R.loadPacket(nReg, b, nB, 0);

    } 

    void setFunctionGroup1(int nReg, int addr, uint32_t fn) {
        setFunction(nReg, addr,  B10000000 | (fn & 0x1F) );
    }
    void setFunctionGroup2(int nReg, int addr, uint32_t fn) {
        fn >>= 5;
        setFunction(nReg, addr,  B10100000 | (fn & 0x1F) );
    }
    void setFunctionGroup3(int nReg, int addr, uint32_t fn) {
        fn >>= 13; 
        setFunction(nReg, addr, B11011110, (uint8_t)fn );
    }
    void setFunctionGroup4(int nReg, int addr, uint32_t fn) {
        fn >>= 21; 
        setFunction(nReg, addr, B11011111, (uint8_t)fn );
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

        #ifdef DCC_DEBUG
            Serial.printf("setFunction %d fByte=%d eByte=%d\n", addr, fByte, eByte);
        #endif

        /* 
        NMRA DCC norm ask for two DCC packets instead of only one:
        "Command Stations that generate these packets, and which are not periodically refreshing these functions,
        must send at least two repetitions of these commands when any function state is changed."
        https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
        */
        R.loadPacket(nReg, b, nB, 4);
    } 

    void setAccessory(int aAdd, int aNum, int activate) {

        #ifdef DCCPP_DEBUG
            Serial.print("{RL::setAccessory addr="); Serial.print(aAdd);Serial.print("; ch=");
            Serial.print(aNum);Serial.print("; state=");Serial.print(activate);Serial.println("}");
        #endif

        uint8_t b[3];                      // save space for checksum byte

        b[0] = aAdd % 64 + 128;            // first byte is of the form 10AAAAAA, where AAAAAA represent 6 least significant bits of accessory address  
        b[1] = ((((aAdd / 64) % 8) << 4) + (aNum % 4 << 1) + activate % 2) ^ 0xF8;      // second byte is of the form 1AAACDDD, where C should be 1, and the least significant D represent activate/deactivate

        R.loadPacket(0, b, 2, 4);

    } 

    uint16_t readCurrent() {
        return analogRead(_sensePin);
    }

private:

    uint8_t _outputPin;    
    uint8_t _enPin;
    uint8_t _sensePin;

    uint16_t current;

    RegisterList R;

    void IRAM_ATTR nextBit();
    void IRAM_ATTR timerFunc();

    friend class DCCESP32SignalGenerator;

};


class DCCESP32SignalGenerator {

public:
    DCCESP32SignalGenerator(uint8_t timerNum = 1) : _timerNum(timerNum) {
        _inst = this;
    }

    void setProgChannel(DCCESP32Channel * ch) { prog = ch;}
    void setMainChannel(DCCESP32Channel * ch) { main = ch;}

    /**
     * Starts half-bit timer.
     * To get 58us tick we need divisor of 58us/0.0125us(80mhz) = 4640,
     * separate this into 464 prescaler and 10 timer alarm.
     */
    void begin() {
        if (main!=nullptr) main->begin();
        if (prog!=nullptr) prog->begin();

        _timer = timerBegin(_timerNum, 464, true);
        timerAttachInterrupt(_timer, timerCallback, true);
        timerAlarmWrite(_timer, 10, true);
        timerAlarmEnable(_timer);
        timerStart(_timer);
    }

    void end() {
        if(_timer!=nullptr) {
            if(timerStarted(_timer) ) timerStop(_timer);
            timerEnd(_timer);
            _timer = nullptr;
        }
        if (main!=nullptr) main->end();
        if (prog!=nullptr) prog->end();
    }



private:
    hw_timer_t * _timer;
    uint8_t _timerNum;
    DCCESP32Channel *prog;
    DCCESP32Channel *main;

    static DCCESP32SignalGenerator * _inst;
    static void timerCallback() {
        _inst->timerFunc();
    }

    void IRAM_ATTR timerFunc() {
        if (main!=nullptr) main->timerFunc();
        if (prog!=nullptr) prog->timerFunc();
    }
};
