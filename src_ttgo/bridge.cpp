#include "bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h" 
#include "lilygo.h" 
#include "CRC16.h"
#include "CRC.h"
//#include <string>

#ifdef TEMBED_CC1101
extern "C" {
    void TEMBED_displaySetBtState(bool connected);
    void TEMBED_displaySetFrequencyHz(uint32_t freqHz);
    void TEMBED_displaySetRssi(float rssi);
    void TEMBED_displaySetScanner(bool active);
}
#endif

extern BLECharacteristic *pRxCharacteristic;
float freq,rssi;
double lat, lon, alt;
char id[10],type[10];

LilyGo myLilyGoBoard;
extern "C" {

    void ttgo_setMsgQueue(QueueHandle_t queue) {
       myLilyGoBoard.setMsgQueue(queue);
    }; 

    void ttgo_setup() {
        myLilyGoBoard.setup();
#ifdef TEMBED_CC1101
        // Prime the live TFT state before TEMBED_displaySetup() draws it.
        TEMBED_displaySetFrequencyHz(myLilyGoBoard.EEPROM_getFrequency());
#endif
    };

    void ttgo_100msTask(){
        myLilyGoBoard.a100msTask();
    };

    void ttgo_setBtState(bool state) {
        myLilyGoBoard.setBtState(state);
#ifdef TEMBED_CC1101
        TEMBED_displaySetBtState(state);
#endif
    };

    uint32_t ttgo_getSerialNo() {
        return myLilyGoBoard.getSerialNo(); 
    };

    float ttgo_getBatVoltage() {
        return myLilyGoBoard.getBatVoltage();
    };

    void ttgo_setDisplayData(double lat, double lon, double alt, float freq, char *id, float rssi, uint32_t frameCounter)
    {
        myLilyGoBoard.setDisplayData(lat, lon, alt, freq, id, rssi, frameCounter);
#ifdef TEMBED_CC1101
        // The legacy display path passes sonde frequency in MHz here.
        uint32_t freqHz = (freq > 1000000.0f)
            ? (uint32_t)freq
            : (uint32_t)(freq * 1000000.0f + 0.5f);
        TEMBED_displaySetFrequencyHz(freqHz);
        TEMBED_displaySetRssi(rssi);
#endif
    };

    void ttgo_toggleDebugScreen() {
        myLilyGoBoard.toggleDebugScreen();
    };

    void ttgo_toggleScannerScreen(int enable) {
        myLilyGoBoard.toggleScannerScreen(enable);
#ifdef TEMBED_CC1101
        TEMBED_displaySetScanner(enable == 2);
#endif
    };

    void ttgo_debug(int eCrcCntr, int blockCntr)
    {
        myLilyGoBoard.setDebugCrc(eCrcCntr, blockCntr);
    }

    void ttgo_setDisplayFreq(float freqHz)
    {
        myLilyGoBoard.setDisplayFreq(freqHz);
#ifdef TEMBED_CC1101
        TEMBED_displaySetFrequencyHz((uint32_t)freqHz);
#endif
    };

    void ttgo_sendBtMessage( char* msg)
    {
       //myLilyGoBoard.getInfosFromMsg(msg);
        if(strncmp(msg,"#3,3",4) != 0)
        {
            //ESP_LOGE("HP","BT: %s",msg);
            Serial.println(msg);
        }

        if (pRxCharacteristic != nullptr) {
            int16_t l = strlen(msg);
            char *mPtr = &msg[0];
            do{
                if(l>20){
                    pRxCharacteristic->setValue((uint8_t*)mPtr,20);
                    mPtr+=20;
                    l-=20;
                }
                else{
                    pRxCharacteristic->setValue((uint8_t*)mPtr,l);
                    l=0;
                }
                pRxCharacteristic->notify();
                vTaskDelay(5/portTICK_PERIOD_MS); 
            }while(l>0);
        }
    };

    void SX1278_ioctl(const SX1278_Config config[]) {
        myLilyGoBoard.SX1278_ioctl(config);
    };

    void SX1278_setBitRate(uint16_t bitrate)
    {
        myLilyGoBoard.SX1278_setBitRate(bitrate);
    };

    float SX1278_setRadioFrequencyHz(uint32_t freqHz, bool readRssi)
    {
        float level = myLilyGoBoard.SX1278_setRadioFrequencyHz(freqHz, readRssi);
#ifdef TEMBED_CC1101
        // Scanner bins use readRssi=true. Do not make the TFT chase all 600
        // scan frequencies; only normal receive tuning updates the live QRG.
        if (!readRssi) {
            TEMBED_displaySetFrequencyHz(freqHz);
        }
#endif
        return level;
    };

    void SX1278_readRSSI(float* newLevel)
    {
        myLilyGoBoard.SX1278_readRSSI(newLevel);
#ifdef TEMBED_CC1101
        if (newLevel != nullptr) {
            TEMBED_displaySetRssi(*newLevel);
        }
#endif
    };

    void ttgo_writeFrequency2Eeprom(uint32_t frequency)
    {
        myLilyGoBoard.EEPROM_writeCfg(frequency);
    };  

    void ttgo_writeDetector2Eeprom(uint8_t detector)
    {
        myLilyGoBoard.EEPROM_writeCfg(detector);
    };

    uint32_t ttgo_getFrequency()
    {
        return myLilyGoBoard.EEPROM_getFrequency();
    };

    uint8_t ttgo_getDetector()
    {
        return myLilyGoBoard.EEPROM_getDetector();
    };

    uint16_t getCRC(const uint8_t* buffer, size_t length) 
    {
        CRC16 crc(CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL);
        crc.add(buffer, length);
        return(crc.calc());
    }

    uint16_t getCRC2(const uint8_t* buffer, size_t length, uint16_t initialValue ) 
    {
        CRC16 crc(CRC16_CCITT_FALSE_POLYNOME, initialValue);
   //     crc.setInitial(initialValue);
        crc.add(buffer, length);
        return(crc.calc());
    }

}