#pragma once

#include <Arduino.h>
#include <esp32-hal-timer.h>
//#include <esp_adc_cal.h>

#include <etl/map.h>
#include <etl/bitset.h>

#include "LocoAddress.h"

constexpr float ADC_RESISTANCE = 0.1;
constexpr float ADC_TO_MV = 3300.0/4096;
constexpr float ADC_TO_MA = ADC_TO_MV / ADC_RESISTANCE;
constexpr uint16_t MAX_CURRENT = 2000;

#define DCC_DEBUG

#ifdef DCC_DEBUG
#define DCC_LOGD(...) 
#define DCC_LOGD_ISR(...)
#define DCC_LOGI(format, ...) log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define DCC_LOGI_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define DCC_LOGW(format, ...) log_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
//extern char _msg[1024];
//extern char _buf[100];
//#define DCC_DEBUG_ISR(...)  do{ snprintf(_buf, 100, __VA_ARGS__); snprintf(_msg, 1024, "%s%s\n", _msg, _buf ); } while(0)
//#define DCC_DEBUG_ISR_DUMP()  do{ Serial.print(_msg); _msg[0]=0; } while(0);
#else
#define DCC_LOGD(...) 
#define DCC_LOGD_ISR(...)
#define DCC_LOGI(...) 
#define DCC_LOGI_ISR( ...) 
#define DCC_LOGW( ...) 
#endif


extern uint8_t idlePacket[3];
extern uint8_t resetPacket[3];

enum class DCCFnGroup {
    F0_4, F5_8, F9_12, F13_20, F21_28
};

enum class SpeedMode { S14, S28, S128 };

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

class IDCCChannel {

public:
    virtual void begin()=0;

    virtual void end()=0;

    virtual void unloadSlot(uint8_t ) = 0;

    virtual void setPower(bool v)=0;

    virtual bool getPower()=0;

    void sendThrottle(int slot, LocoAddress addr, uint8_t tSpeed,  SpeedMode sm, uint8_t tDirection);
    void sendFunctionGroup(int slot, LocoAddress addr, DCCFnGroup group, uint32_t fn);
    void sendFunction(int slot, LocoAddress addr, uint8_t fByte, uint8_t eByte=0);
    /**
     * @param addr11 is 1-based.
     */
    void sendAccessory(uint16_t addr11, bool thr);
    /** 
     * @param addr9 is 1-based
     * @param ch is 0-based.
     */
    void sendAccessory(uint16_t addr9, uint8_t ch, bool);
    virtual uint16_t readCurrentAdc()=0;

    int16_t readCVProg(int cv);
    bool verifyCVByteProg(uint16_t cv, uint8_t bValue);
    bool writeCVByteProg(int cv, uint8_t bValue);
    bool writeCVBitProg(int cv, uint8_t bNum, uint8_t bValue);
    void writeCVByteMain(LocoAddress addr, int cv, uint8_t bValue);
    void writeCVBitMain(LocoAddress addr, int cv, uint8_t bNum, uint8_t bValue);

    bool checkOvercurrent() {
        uint16_t v = readCurrentAdc();
        float mA = v * ADC_TO_MA;
        //if(v!=0) DCC_LOGI("%d, %d", v, (int)mA);
        if(mA>MAX_CURRENT) {
            setPower(0);
            return false;
        }
        else return true;
    }

protected:
    virtual void timerFunc()=0;
    virtual bool loadPacket(int, uint8_t*, uint8_t, int)=0;
    static void copyPacket(uint8_t *src, uint8_t len, int repeat, Packet *dst);
private:
    friend class DCCESP32SignalGenerator;

    uint getBaselineCurrent();
    bool checkCurrentResponse(uint baseline);

};

template<uint8_t SLOT_COUNT>
class DCCESP32Channel: public IDCCChannel {
public:

    DCCESP32Channel(uint8_t outputPin, uint8_t enPin, uint8_t sensePin): 
        _outputPin(outputPin), _enPin(enPin), _sensePin(sensePin)
    {
        R.timerPeriodsLeft=1; // first thing a timerfunc does is decrement this, so make it not underflow
        R.timerPeriodsHalf=2; // some sane nonzero value
    }


    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_enPin, LOW);

        //DCC_LOGI("DCCESP32Channel(enPin=%d)::begin", _enPin);

        //analogSetCycles(16);
        //analogSetWidth(11);
        analogSetPinAttenuation(_sensePin, ADC_0db); 
        /*esp_adc_cal_value_t ar = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_chars);
        if (ar == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            DCC_LOGI("eFuse Vref");
        } else {
            DCC_LOGI("Default vref");
        }*/

        // during loadPacket there is time when new index is enabled for refresh, but urgentPacket is not yet loaded.
        for(size_t i=1; i<=SLOT_COUNT; i++) {
            copyPacket(idlePacket, 2, 0, &R.packets[i]);
        }
        loadPacket(1, idlePacket, 2, 0);
    }

    void end() override {
        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v) override {
        DCC_LOGI("setPower(%d)", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() override {
        return digitalRead(_enPin) == HIGH;
    }

    /** Define a series of registers that can be sequentially accessed over a loop to generate a repeating series of DCC Packets. */
    struct RegisterList {
        Packet packets[SLOT_COUNT+1];
        etl::bitset<SLOT_COUNT+1> indicesTaken;
        etl::map<uint, size_t, SLOT_COUNT> regToIdxMap;
        volatile Packet *currentPacket;
        size_t maxIdx;
        volatile Packet *urgentPacket;
        /* how many 58us periods needed for half-cycle (1 for "1", 2 for "0") */
        volatile uint8_t timerPeriodsHalf;
        /* how many 58us periods are left (at start, 2 for "1", 4 for "0"). */
        volatile uint8_t timerPeriodsLeft;
        Packet newPacket;
        //int8_t newSlot;

        volatile uint8_t currentBit;
        
        RegisterList() {
            currentPacket = &packets[0];
            maxIdx = 0;
            urgentPacket = nullptr;
            currentBit = 0;
        } 

        ~RegisterList() {}

        inline bool newPacketVacant() { return urgentPacket==nullptr;}

        inline bool currentBitValue() {
            return (currentPacket->buf[currentBit/8] & 1<<(7-currentBit%8) )!= 0;
        } 

        inline uint8_t currentIdx() { return currentPacket-&packets[0]; }

        inline void advanceCurrentPacket() {
            if (urgentPacket != nullptr) {                      
                currentPacket = urgentPacket; 
                *(Packet*)currentPacket = newPacket;  // this copies packet into packet array
                urgentPacket = nullptr;                
                // flip active and update Packets
                //Packet * p = currentPacket->flip();
                DCC_LOGD_ISR("advance to urgentPacket %d",  currentIdx() );
                //currentPacket->debugPrint();
            } else {    
                // ELSE simply move to next Register    
                // BUT IF this is last Register loaded, first reset currentPacket to base Register, THEN       
                // increment current Register (note this logic causes Register[0] to be skipped when simply cycling through all Registers)  
                
                size_t i = currentIdx();
                do {
                    if (i == maxIdx) { i = 0; }
                    i++;
                    //DCC_DEBUGF_ISR("indicesTaken[%d]=%d  (max=%d)", i, indicesTaken[i]?1:0, maxIdx);
                } while( !indicesTaken[i] );
                
                currentPacket = &packets[i];
                    
                DCC_LOGD_ISR("advance to next slot=%d", i );
                //currentPacket->debugPrint();
            }
        }
        inline void setBitTimings() {
            if ( currentBitValue() ) {  
                /* For "1" bit, we need 1 58us timer tick for each signal level */ 
                DCC_LOGD_ISR("bit %d (0x%02x) = 1", currentBit, currentPacket->buf[currentBit/8] );
                timerPeriodsHalf = 1; 
                timerPeriodsLeft = 2; 
            } else {  /* ELSE it is a ZERO bit */ 
                /* For "0" bit, we need 2 58us timer ticks for each signal level */ 
                DCC_LOGD_ISR("bit %d (0x%02x) = 0", currentBit, currentPacket->buf[currentBit/8] );
                timerPeriodsHalf = 2; 
                timerPeriodsLeft = 4; 
            } 
        }

        size_t findEmptyIdx() {
            if(regToIdxMap.available()==0) return 0;

            for(int  i=1; i<=SLOT_COUNT; i++) 
                if (!indicesTaken[i]) return i;
            return 0;
        }

        size_t findOrAllocateIdx(size_t iReg) {
            size_t arrIdx;
            const auto it = regToIdxMap.find(iReg);
            if(it == regToIdxMap.end() ) {
                arrIdx = findEmptyIdx();
                if(arrIdx==0) return 0;
                //DCC_DEBUGF("Allocating new slot %d for reg %d", arrIdx,  iReg);
                regToIdxMap[iReg] = arrIdx;
                indicesTaken[arrIdx] = true;
                maxIdx = max(maxIdx, arrIdx);
                //DCC_DEBUGF("maxIdx=%d", R.maxIdx);
            } else {
                arrIdx = it->second;
            }
            return arrIdx;
        }

        void bindNewPacket(size_t idx) {
            urgentPacket = &packets[idx];
        }
    };

    uint16_t readCurrentAdc() override {        
        //return esp_adc_cal_raw_to_voltage(analogRead(_sensePin), &adc_chars);
        return analogRead(_sensePin);//*1093.0/4096;
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

        //current = readCurrentAdc(); 
    }

    RegisterList * getReg() { return &R; }

protected:

    bool loadPacket(int iReg, uint8_t *b, uint8_t nBytes, int nRepeat) override {

        //DCC_DEBUGF("reg=%d len=%d, repeat=%d", iReg, nBytes, nRepeat);

        // force slot to be between 0 and maxNumRegs, inclusive
        //iReg = iReg % (SLOT_COUNT+1);

        // pause while there is a Register already waiting to be updated -- urgentPacket will be reset to NULL by timer when prior Register updated fully processed
        int t=1000;
        while(!R.newPacketVacant() && --t>0) delay(1);
        if(t==0) {
            DCC_LOGW("timeout for slot %d", iReg );
            return false;
        }
        //DCC_DEBUGF("Loading into slot %d, took %d ms", iReg, 1000-t );

        size_t arrIdx;
        if(iReg==0) {
            arrIdx = 0;
        } else {
            arrIdx = R.findOrAllocateIdx(iReg);
            if(arrIdx==0) return false;
        }

        Packet *p = &R.newPacket;
        copyPacket(b, nBytes, nRepeat, p);
        R.bindNewPacket(arrIdx);

        return true;

    }

    void unloadSlot(uint8_t iReg) override {
        const auto it = R.regToIdxMap.find(iReg);
        if(it==R.regToIdxMap.end()) {
            DCC_LOGW("Did not find slot for reg %d", iReg);
            return;
        }

        size_t slot = it->second;
        DCC_LOGI("unloading slot %d for reg %d", slot, iReg);
        if(R.regToIdxMap.size()==1) {
            // if it's last last slot, remove it and load Idle packet into slot 1
            loadPacket(1, idlePacket, 2, 0);
            if(slot==1) return; // don't unload slot 1 if it's the only one left
        }
        
        R.regToIdxMap.erase(iReg);
        R.indicesTaken[slot] = false;
        for(int i=R.regToIdxMap.capacity()-1; i>0; i--) {
            if (R.indicesTaken[i]) { R.maxIdx = i; break; }
        }
    }

private:

    uint8_t _outputPin;    
    uint8_t _enPin;
    uint8_t _sensePin;

    uint16_t current;

    //esp_adc_cal_characteristics_t adc_chars;

    RegisterList R;

    void IRAM_ATTR nextBit() {
        auto p = R.currentPacket;
        //DCC_DEBUGF_ISR("nextBit: currentPacket=%d, activePacket=%d, cbit=%d, bits=%d", R.currentIdx(),  R.currentPacket->activeIdx(), R.currentBit, p->nBits );

        // IF no more bits in this DCC Packet, reset current bit pointer and determine which Register and Packet to process next  
        if (R.currentBit == p->nBits) {
            R.currentBit = 0;
            // IF current Register is first Register AND should be repeated, decrement repeat count; result is this same Packet will be repeated                             
            if (p->nRepeat>0 && R.currentPacket == &R.packets[0]) {        
                p->nRepeat--;    
                DCC_LOGD_ISR("repeat packet = %d", p->nRepeat);
            } else {
                // IF another slot has been updated, update currentPacket to urgentPacket and reset urgentPacket to NULL 
                R.advanceCurrentPacket();
            }                                        
        } // currentPacket, activePacket, and currentBit should now be properly set to point to next DCC bit

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
