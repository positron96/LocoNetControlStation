#pragma once

#include <Arduino.h>
#include <esp32-hal-timer.h>

#define DCC_DEBUG

#ifdef DCC_DEBUG
#define DCC_DEBUGF(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#define DCC_DEBUGF_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
//#define DCC_DEBUGF_ISR(...) 
//#define DCC_DEBUGF(...)  do{ Serial.printf(__VA_ARGS__); }while(0)
//#define DCC_DEBUGF_ISR(...)  do{ Serial.printf(__VA_ARGS__); }while(0)
//extern char _msg[1024];
//extern char _buf[100];
//#define DCC_DEBUG_ISR(...)  do{ snprintf(_buf, 100, __VA_ARGS__); snprintf(_msg, 1024, "%s%s\n", _msg, _buf ); } while(0)
//#define DCC_DEBUG_ISR_DUMP()  do{ Serial.print(_msg); _msg[0]=0; } while(0);
#else
#define DCC_DEBUGF(...)
#define DCC_DEBUGF_ISR(...) 
#endif


extern uint8_t idlePacket[3];
extern uint8_t resetPacket[3];

enum class DCCFnGroup {
    F0_4, F5_8, F9_12, F13_20, F21_28
};

class IDCCChannel {

public:
    virtual void begin()=0;

    virtual void end()=0;

    virtual void setPower(bool v)=0;

    virtual bool getPower()=0;

    void setThrottle(int slot, int addr, uint8_t tSpeed, uint8_t tDirection);
    void setFunctionGroup(int slot, int addr, DCCFnGroup group, uint32_t fn);
    void setFunction(int slot, int addr, uint8_t fByte, uint8_t eByte=0);
    void setAccessory(int aAdd, int aNum, int activate);
    virtual uint16_t readCurrent()=0;

    virtual void unload(uint8_t slot) {
        //for()
    }
protected:
    virtual void timerFunc()=0;
    virtual void loadPacket(int, uint8_t*, uint8_t, int)=0;
private:
    friend class DCCESP32SignalGenerator;

};

struct Packet {
    uint8_t buf[10];
    uint8_t nBits;
    int8_t nRepeat;
    void debugPrint() {
        /*    
        char ttt[50]; ttt[0]='\0'; 
        char *pos=ttt;
        uint8_t nBytes = nBits/8; if (nBytes*8!=nBits) nBytes++;
        for(int i=0; i<nBytes; i++) {
            pos += sprintf(pos, "%02x ", buf[i]);
        }
        DCC_DEBUGF_ISR("packet (%d): '%s'", nBits, ttt );
        */
    }
};
/*
struct PacketSlot {
    Packet *activePacket;
    Packet *updatePacket;
    PacketSlot(): activePacket(&packet[0]), updatePacket(&packet[1]) { }
    inline Packet * flip() {
        Packet *tmp = activePacket;
        activePacket = updatePacket;
        updatePacket = tmp;
        return activePacket;
    }
    inline uint8_t activeIdx() { return activePacket-&packet[0]; }

private:
    Packet packet[2];

};*/

template<uint8_t SLOT_COUNT>
class DCCESP32Channel: public IDCCChannel {
public:

    DCCESP32Channel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin): 
        _outputPin(outputPin), _enPin(enPin), _sensePin(sensePin)
    {
        R.timerPeriodsLeft=1; // first thing a timerfunc does is decrement this, so make it not underflow
        R.timerPeriodsHalf=2; // some nonzero sane value
    }


    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        //digitalWrite(_enPin, HIGH);

        loadPacket(1, idlePacket, 2, 0);
    }

    void end() override {
        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v) override {
        DCC_DEBUGF("setPower(%d)", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() override {
        return digitalRead(_enPin) == HIGH;
    }

    /** Define a series of registers that can be sequentially accessed over a loop to generate a repeating series of DCC Packets. */
    struct RegisterList {
        Packet slots[SLOT_COUNT+1];
        Packet *slotMap[SLOT_COUNT+1];
        Packet *currentSlot;
        Packet *maxLoadedSlot;
        Packet *urgentSlot;
        /* how many 58us periods needed for half-cycle (1 for "1", 2 for "0") */
        uint8_t timerPeriodsHalf;
        /* how many 58us periods are left (at start, 2 for "1", 4 for "0"). */
        uint8_t timerPeriodsLeft;
        Packet newPacket;
        //int8_t newSlot;

        uint8_t currentBit;
        
        RegisterList() {
            //reg = (PacketSlot *)calloc((maxNumRegs+1),sizeof(PacketSlot));
            //for (int i=0; i<=SLOT_COUNT; i++) slots[i] = PacketSlot();
            //slotMap = (PacketSlot **)calloc((SLOT_COUNT+1),sizeof(PacketSlot *));
            currentSlot = &slots[0];
            slotMap[0] = currentSlot;
            maxLoadedSlot = currentSlot;
            urgentSlot = nullptr;
            currentBit = 0;
            
        } 
        ~RegisterList() {}

        inline bool newSlotVacant() { return urgentSlot==nullptr;}

        IRAM_ATTR inline bool currentBitValue() {
            return (currentSlot->buf[currentBit/8] & 1<<(7-currentBit%8) )!= 0;
        } 

        inline uint8_t currentIdx() { return currentSlot-&slots[0]; }
        inline void advanceSlot() {
            if (urgentSlot != nullptr) {                      
                currentSlot = urgentSlot; 
                *currentSlot = newPacket; 
                urgentSlot = nullptr;                
                // flip active and update Packets
                //Packet * p = currentSlot->flip();
                DCC_DEBUGF_ISR("advance to urgentSlot %d",  currentIdx() );
                //currentSlot->debugPrint();
            } else {    
                // ELSE simply move to next Register    
                // BUT IF this is last Register loaded, first reset currentSlot to base Register, THEN       
                // increment current Register (note this logic causes Register[0] to be skipped when simply cycling through all Registers)  
                
                if (currentSlot == maxLoadedSlot)
                    currentSlot = &slots[0];
                currentSlot++;                        
                    
                //DCC_DEBUGF_ISR("advance to next slot=%d", currentIdx() );
                //currentSlot->debugPrint();
            }
        }
        inline void setBitTimings() {
            if ( currentBitValue() ) {  
                /* For "1" bit, we need 1 periods of 58us timer ticks for each signal level */ 
                //DCC_DEBUGF_ISR("bit %d (0x%02x) = 1", currentBit, currentSlot->buf[currentBit/8] );
                timerPeriodsHalf = 1; 
                timerPeriodsLeft = 2; 
            } else {  /* ELSE it is a ZERO bit */ 
                /* For "0" bit, we need 2 period of 58us timer ticks for each signal level */ 
                //DCC_DEBUGF_ISR("bit %d (0x%02x) = 0", currentBit, currentSlot->buf[currentBit/8] );
                timerPeriodsHalf = 2; 
                timerPeriodsLeft = 4; 
            } 
        }
    };

    uint16_t readCurrent() override {
        return analogRead(_sensePin);
    }

    void IRAM_ATTR timerFunc() override {
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

    RegisterList * getReg() { return &R; }

protected:

    void loadPacket(int iSlot, uint8_t *b, uint8_t nBytes, int nRepeat) override {

        DCC_DEBUGF("reg=%d len=%d, repeat=%d", iSlot, nBytes, nRepeat);

        // force slot to be between 0 and maxNumRegs, inclusive
        iSlot = iSlot % (SLOT_COUNT+1);

        // pause while there is a Register already waiting to be updated -- urgentSlot will be reset to NULL by timer when prior Register updated fully processed
        int t=1000;
        while(!R.newSlotVacant() && --t >0) delay(1);
        if(t==0) {
            DCC_DEBUGF("timeout for slot %d", iSlot );
            return;
        }

        DCC_DEBUGF("Loading into slot %d, took %d ms", iSlot, 1000-t );
        
        // first time this Register Number has been called
        // set Register Pointer for this Register Number to next available Register
        if(R.slotMap[iSlot] == nullptr) {        
            R.slotMap[iSlot] = R.maxLoadedSlot + 1;   
            DCC_DEBUGF("Allocating new reg %d", iSlot);
        }
        
        Packet *slot = R.slotMap[iSlot];
        Packet *p = &R.newPacket;
        uint8_t *buf = p->buf;

        // copy first byte into what will become the checksum byte 
        // XOR remaining bytes into checksum byte 
        b[nBytes] = b[0];                        
        for(int i=1; i<nBytes; i++)              
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
        
        p->nRepeat = nRepeat; 
        R.urgentSlot = slot;
        R.maxLoadedSlot = max(R.maxLoadedSlot, slot);

        p->debugPrint();  

    }

private:

    uint8_t _outputPin;    
    uint8_t _enPin;
    uint8_t _sensePin;

    uint16_t current;

    RegisterList R;

    void IRAM_ATTR nextBit() {
        //const uint8_t bitMask[] = {  0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01  };
        Packet *p = R.currentSlot;
        //DCC_DEBUGF_ISR("nextBit: currentSlot=%d, activePacket=%d, cbit=%d, bits=%d", R.currentIdx(),  R.currentSlot->activeIdx(), R.currentBit, p->nBits );

        // IF no more bits in this DCC Packet, reset current bit pointer and determine which Register and Packet to process next  
        if (R.currentBit == p->nBits) {
            R.currentBit = 0;
            // IF current Register is first Register AND should be repeated, decrement repeat count; result is this same Packet will be repeated                             
            if (p->nRepeat>0 && R.currentSlot == &R.slots[0]) {        
                p->nRepeat--;    
                DCC_DEBUGF_ISR("repeat packet = %d", p->nRepeat);
            } else {
                // IF another slot has been updated, update currentSlot to urgentSlot and reset urgentSlot to NULL 
                R.advanceSlot();
            }                                        
        } // currentSlot, activePacket, and currentBit should now be properly set to point to next DCC bit

        R.setBitTimings();

        R.currentBit++; 
    }

    //void IRAM_ATTR timerFunc();

};


class DCCESP32SignalGenerator {

public:
    DCCESP32SignalGenerator(uint8_t timerNum = 1);

    void setProgChannel(IDCCChannel * ch) { prog = ch;}
    void setMainChannel(IDCCChannel * ch) { main = ch;}

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
    IDCCChannel *main = nullptr;
    IDCCChannel *prog = nullptr;

    friend void timerCallback();

    void IRAM_ATTR timerFunc();
};
