#include <lilygo.h>
#include <sys.h>
#include <EEPROM.h>
#include <esp_mac.h>
#include <SPI.h>
#include "SSD1306Wire.h"
#include "images.h"
#include "bridge.h"
#include "espPorting.h"
#include "esp_task_wdt.h"
#include <esp_log.h>
#include "esp_sleep.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "freertos/stream_buffer.h"

#ifdef TEMBED_CC1101
#include <RadioLib.h>

// T-Embed CC1101
static constexpr int TEMBED_CC1101_CS   = 12;
static constexpr int TEMBED_CC1101_GDO0 = 3;
static constexpr int TEMBED_CC1101_GDO2 = 38;

static constexpr int TEMBED_PWR_EN = 15;
static constexpr int TEMBED_RF_SW1 = 47;
static constexpr int TEMBED_RF_SW0 = 48;

// Display und SD teilen sich den SPI-Bus mit dem CC1101
static constexpr int TEMBED_DISPLAY_CS = 41;
static constexpr int TEMBED_SD_CS      = 13;

static SPISettings cc1101SpiSettings(2000000, MSBFIRST, SPI_MODE0);

static CC1101 cc1101 = new Module(
    TEMBED_CC1101_CS,
    TEMBED_CC1101_GDO0,
    RADIOLIB_NC,
    TEMBED_CC1101_GDO2,
    SPI,
    cc1101SpiSettings
);

// The CC1101 can not reproduce the very narrow SX1278 filters used by the
// original Ra receiver. 58 kHz is its narrowest supported RX bandwidth.
static float cc1101SupportedBandwidth(float requestedKhz)
{
    static const float supported[] = {
        58.0f, 68.0f, 81.0f, 102.0f, 116.0f, 135.0f, 162.0f, 203.0f,
        232.0f, 270.0f, 325.0f, 406.0f, 464.0f, 541.0f, 650.0f, 812.0f
    };

    for (float bw : supported) {
        if (requestedKhz <= bw) {
            return bw;
        }
    }
    return 812.0f;
}

// Demodulator loop settings taken from a CC1101 configuration proven to
// receive RS41 at 4800 bit/s. RadioLib's generic direct mode sets the GDO
// routing correctly, but leaves FOC/bit-sync/AGC at generic reset defaults.
static int16_t cc1101ApplyRadiosondeProfile()
{
    int16_t state = cc1101.standby();
    if (state != RADIOLIB_ERR_NONE) {
        return state;
    }

    struct RegValue {
        uint8_t reg;
        uint8_t value;
    };

    static const RegValue profile[] = {
        {RADIOLIB_CC1101_REG_FSCTRL1,  0x06}, // IF about 152 kHz
        {RADIOLIB_CC1101_REG_FOCCFG,   0x1D}, // frequency-offset compensation
        {RADIOLIB_CC1101_REG_BSCFG,    0x1C}, // bit synchronizer loop
        {RADIOLIB_CC1101_REG_AGCCTRL2, 0xC7}, // radiosonde-friendly AGC
        {RADIOLIB_CC1101_REG_AGCCTRL1, 0x00},
        {RADIOLIB_CC1101_REG_AGCCTRL0, 0xB2},
        {RADIOLIB_CC1101_REG_FREND1,   0xB6}, // RX front-end configuration
    };

    for (const RegValue &entry : profile) {
        state = cc1101.SPIsetRegValue(entry.reg, entry.value);
        if (state != RADIOLIB_ERR_NONE) {
            return state;
        }
    }

    return RADIOLIB_ERR_NONE;
}
#endif

#define SX127x_FREQUENCY_STEP_SIZE   61.03515625 // in Hz (32 MHz / 2^19)

SSD1306Wire* display = nullptr;
static const char* TAG = "HP";
int taskCalled_Cntr = 0;

uint64_t rxedBits,pattern;

#ifdef TEMBED_CC1101
volatile uint32_t cc1101DroppedBits = 0;
volatile uint32_t cc1101ClockEdges = 0;
#endif

uint8_t PIN_DIO1,dtstate;
extern StreamBufferHandle_t xBitBuffer;

#ifdef TEMBED_CC1101
extern "C" {
    extern volatile uint32_t rs41RealSyncHits;
}
#endif
BoardPins espBoard;

void handleConsole(const char *cmd);
volatile uint8_t DRAM_ATTR isr_lora_dio2_pin = 32; // Default to TTGO DIO2 pin, will be updated in detectBoard()

IRAM_ATTR void onDIO1Edge() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint8_t bit;

    if (isr_lora_dio2_pin >= 32) {
        bit = (GPIO.in1.val >> (isr_lora_dio2_pin - 32)) & 0x01;
    } else {
        bit = (GPIO.in >> isr_lora_dio2_pin) & 0x01;
    }

#ifdef TEMBED_CC1101
    cc1101ClockEdges++;
#endif
   

    BaseType_t sent =
    xStreamBufferSendFromISR(xBitBuffer, &bit, 1, &xHigherPriorityTaskWoken);

#ifdef TEMBED_CC1101
if (sent != 1) {
    cc1101DroppedBits++;
}
#endif
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void screenSaverCallback(TimerHandle_t xTimer) {
#ifdef TEMBED_CC1101
    (void)xTimer;
#else
    display->displayOff();
#endif
}

LilyGo::LilyGo() {
#ifndef TEMBED_CC1101
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    rssi = -128;
}

void LilyGo::setup() {
    uint8_t baseMac[6];
    
    detectBoard();
    #ifdef TEMBED_CC1101
    // Seitliche Taste: lang halten = Neustart zum Launcher
    pinMode(6, INPUT_PULLUP);
#endif
    if(isBoardTTGO) {
//        pinMode(14, OUTPUT);
        pinMode(espBoard.bat_adc, INPUT);
    }else if(isBoardHELTEC) {
        pinMode(espBoard.oled_rst, OUTPUT);
    }
    #ifndef TEMBED_CC1101
    display = new SSD1306Wire(
        OLED_I2C_ADDRESS,
        espBoard.oled_sda,
        espBoard.oled_scl,
        GEOMETRY_128_64,
        I2C_TWO,
        500000
    );
#else
    display = nullptr;
#endif
    BTisConnected = false;
    BLE_setup(true);   
    esp_base_mac_addr_get(baseMac);   
    SerialNoEsp = baseMac[3]<<16 |baseMac[4]<<8 |baseMac[5];
    EEPROM_setup();
    SX1278_setup();
    OLED_setup();
    #ifndef TEMBED_CC1101

    analogReadResolution(12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db);
    adc1_config_width(ADC_WIDTH_12Bit);
    vBattOnStart = getBatVoltage();

#else

    // T-Embed nutzt einen eigenen Battery-Fuel-Gauge.
    // Kommt später sauber über I2C.
    vBatt = 0.0f;
    vBattOnStart = 0.0f;

#endif
    if(ESP_OK != esp_task_wdt_init(5,true)) {
        ESP_LOGE(TAG, "Failed to initialize task watchdog");
    }
}   

void LilyGo::detectBoard() {

#ifdef TEMBED_CC1101

    espBoard = {
        "LILYGO T-Embed CC1101",

        // CC1101:
        // SCK, MISO, MOSI, CS, RST, GDO0, CLOCK, DATA
        11, 10, 9, 12, -1, 3, 3, 38,

        // T-Embed hat kein SSD1306-OLED.
        // Diese Werte werden im nächsten Schritt ersetzt.
        8, 18, -1, -1
    };

    isBoardTEMBED = true;

#else

    uint32_t flashSize = ESP.getFlashChipSize();

    if (flashSize >= 8 * 1024 * 1024) {

        espBoard = {
            "Heltec WiFi LoRa 32 V2",
            5, 19, 27, 18, 14, 26, 35, 34,
            4, 15, 16, 13
        };

        isBoardHELTEC = true;
    }

    if (flashSize == 4 * 1024 * 1024) {

        espBoard = {
            "TTGO LoRa32 V2.1 (1.6)",
            5, 19, 27, 18, 23, 26, 33, 32,
            21, 22, 16, 35
        };

        isBoardTTGO = true;
    }

#endif

    PIN_DIO1 = espBoard.lora_dio1;
    isr_lora_dio2_pin = espBoard.lora_dio2;

    Serial.printf("Detected %s\n", espBoard.boardName);
}

void LilyGo::setMsgQueue(QueueHandle_t q) { 
    BLE_setMsgQueue(q);
}

void LilyGo::setBtState(bool state) {
    BTisConnected = state;
#ifdef TEMBED_CC1101
    return;
#endif
    display->setColor(BTisConnected ? WHITE : BLACK);          
    switch(activeScreen)
    {
        case SCREEN_STARTUP:
            display->setFont(ArialMT_Plain_16);
            display->setTextAlignment(TEXT_ALIGN_LEFT);
            display->drawString(0, 37, "B  T");
            break;
        case SCREEN_SONDEDATA:
            if(BTisConnected){
              display->drawIco16x16(0,0, &BTon[0]);
            }
            else{
              display->fillRect(0,0,16,16);
            }
            break;
    }
    
    display->display();
    display->displayOn();
    display->setColor(WHITE);
    xTimerReset( screenSaverTimer, 0);
}

void LilyGo::OLED_setup(){
    OLED_drawScreen(SCREEN_STARTUP); 
 }


 void LilyGo::OLED_show(bool state){
    screenIsOff = !state;

#ifdef TEMBED_CC1101
    return;
#else
    state ? display->displayOn() : display->displayOff();
#endif
}


uint32_t LilyGo::getSerialNo() { 
    return SerialNoEsp;
}


float LilyGo::getBatVoltage()
{
#ifdef TEMBED_CC1101
    return (float)cc1101DroppedBits;
#endif
    float vBattOld = vBatt;
    // if(isBoardTTGO) {
    //     digitalWrite(14, HIGH);
    // }
    // delay(1);
    vBatt = (analogRead(espBoard.bat_adc) / 4095.0 * 2 * 3.3 * voltageCalibrationFactor); 
                       // voltage divider 100k/100k, ADC ref 3.3V, calibration;
    // if(isBoardTTGO) {
    //     digitalWrite(14, LOW);
    // }
    if(vBatt > 4.17)   // Simple threshold to detect charging state, adjust as needed
        isCharging = true;  
    else if(vBatt > vBattOld + 0.01)
        isCharging = true;
    else if(vBatt < vBattOld)
        isCharging = false;
 
    OLED_updateVoltage(vBatt);

    return vBatt;
}

void LilyGo::EEPROM_setup() {
    EEPROM.begin(5);
    frequencyInEeprom = EEPROM.readLong(0);
    detectorInEeprom  = EEPROM.readByte(4);
    if((frequencyInEeprom < 400e6)||(frequencyInEeprom > 406e6)||detectorInEeprom > 15)  //SONDE_DETECTOR_LMS6
    {
        frequencyInEeprom = 405100000;
        detectorInEeprom  = 0;  // SONDE_DETECTOR_RS41_RS92
        EEPROM.writeLong(0,frequencyInEeprom);
        EEPROM.writeByte(4,detectorInEeprom);  
        EEPROM.commit();
        ESP_LOGE("HP", "Eeprom empty, wrote defaults");
    }
    else
    {
       EEPROM.end();
    }
    freqMhz = frequencyInEeprom/1e6;
}

void LilyGo::EEPROM_writeCfg(uint32_t frequency)
{
    EEPROM.begin(5);
    EEPROM.writeLong(0, frequency);
    EEPROM.commit();
}

void LilyGo::EEPROM_writeCfg(uint8_t detector)
{
    EEPROM.begin(5);
    EEPROM.writeByte(4, detector);
    EEPROM.commit();
}

uint8_t sx1278ReadRegister(uint8_t reg) {
  digitalWrite( espBoard.lora_ss, LOW);
  SPI.transfer(reg & 0x7F); // read command
  uint8_t value = SPI.transfer(0x00);
  digitalWrite( espBoard.lora_ss, HIGH);
  return value;
}

void sx1278WriteRegister0(uint8_t reg, uint8_t value) {
  digitalWrite(espBoard.lora_ss, LOW);
  SPI.transfer(reg | 0x80); // write command
  SPI.transfer(value);
  digitalWrite(espBoard.lora_ss, HIGH);
}

void LilyGo::SX1278_readRSSI(float* newLevel)
{
#ifdef TEMBED_CC1101

    *newLevel = cc1101.getRSSI();

#else

    *newLevel = -sx1278ReadRegister(0x11) / 2.0f;

#endif
}

void LilyGo::SX1278_setBitRate(uint16_t bitrate) {

#ifdef TEMBED_CC1101

    int16_t state = cc1101.setBitRate((float)bitrate / 1000.0f);
    Serial.printf("CC1101 bitrate: %u bps, state: %d\n", bitrate, state);

#else

    uint16_t divisor = 32000000UL / bitrate;

    sx1278WriteRegister0(0x02, (uint8_t)(divisor >> 8));
    sx1278WriteRegister0(0x03, (uint8_t)(divisor & 0xFF));

#endif
}

void LilyGo::SX1278_setup() {

#ifdef TEMBED_CC1101

    // Alle Teilnehmer des gemeinsamen SPI-Busses abwählen
    pinMode(TEMBED_DISPLAY_CS, OUTPUT);
    digitalWrite(TEMBED_DISPLAY_CS, HIGH);

    pinMode(TEMBED_SD_CS, OUTPUT);
    digitalWrite(TEMBED_SD_CS, HIGH);

    pinMode(TEMBED_CC1101_CS, OUTPUT);
    digitalWrite(TEMBED_CC1101_CS, HIGH);

    // T-Embed Peripherie einschalten
    pinMode(TEMBED_PWR_EN, OUTPUT);
    digitalWrite(TEMBED_PWR_EN, HIGH);
    delay(10);

    // Antenne auf 387–464 MHz stellen
    pinMode(TEMBED_RF_SW1, OUTPUT);
    pinMode(TEMBED_RF_SW0, OUTPUT);
    digitalWrite(TEMBED_RF_SW1, HIGH);
    digitalWrite(TEMBED_RF_SW0, HIGH);

    pinMode(TEMBED_CC1101_GDO0, INPUT);
    pinMode(TEMBED_CC1101_GDO2, INPUT);

    // Gemeinsamen SPI-Bus starten
    SPI.begin(11, 10, 9);

    // Timer wird vom bisherigen Ra-Code noch benötigt
    screenSaverTimer = xTimerCreate(
        "SCREENSAVER-Timer",
        pdMS_TO_TICKS(60000),
        pdFALSE,
        (void *)NULL,
        screenSaverCallback
    );
    xTimerStart(screenSaverTimer, 0);

    // CC1101 zunächst für RS41 vorbereiten
    int16_t state = cc1101.begin(
        405.1,  // MHz
        4.8,    // kbit/s
        2.4,    // kHz Frequenzhub
        58.0,   // kHz Empfangsbandbreite
        10,     // dBm - für Empfang praktisch irrelevant
        16
    );

    if (state == RADIOLIB_ERR_NONE) {
        state = cc1101ApplyRadiosondeProfile();
    }

    Serial.printf("CC1101 init/profile state: %d\n", state);

#else

    pinMode(espBoard.lora_dio1, INPUT);
    pinMode(espBoard.lora_dio2, INPUT);
    pinMode(espBoard.lora_ss, OUTPUT);
    pinMode(espBoard.lora_rst, OUTPUT);
    digitalWrite(espBoard.lora_ss, HIGH);

    SPI.begin(
        espBoard.lora_sck,
        espBoard.lora_miso,
        espBoard.lora_mosi,
        espBoard.lora_ss
    );

    digitalWrite(espBoard.lora_rst, LOW);
    delay(100);
    digitalWrite(espBoard.lora_rst, HIGH);
    delay(100);

    screenSaverTimer = xTimerCreate(
        "SCREENSAVER-Timer",
        pdMS_TO_TICKS(60000),
        pdFALSE,
        (void *)NULL,
        screenSaverCallback
    );
    xTimerStart(screenSaverTimer, 0);

    sx1278WriteRegister0(0x01, 0x01);
    sx1278WriteRegister0(0x0C, 0b00100011);
    sx1278WriteRegister0(0x0D, 0b11111110);
    sx1278WriteRegister0(0x0E, 0b00000100);
    sx1278WriteRegister0(0x14, 0x28);
    sx1278WriteRegister0(0x1E, 0b00000001);
    sx1278WriteRegister0(0x1F, 0xAA);
    sx1278WriteRegister0(0x30, 0x00);
    sx1278WriteRegister0(0x31, 0x00);
    sx1278WriteRegister0(0x40, 0x00);

#endif
}

float LilyGo::SX1278_setRadioFrequencyHz(uint32_t freqInHz, bool needRssi) {

#ifdef TEMBED_CC1101

    float rssi = 0.0f;
    float freqMHz = (float)freqInHz / 1000000.0f;

    int16_t state = cc1101.standby();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 standby failed: %d\n", state);
        return -128.0f;
    }

    state = cc1101.setFrequency(freqMHz);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 setFrequency %.4f MHz failed: %d\n", freqMHz, state);
        return -128.0f;
    }

    state = cc1101.receiveDirect();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 receiveDirect failed: %d\n", state);
        return -128.0f;
    }

    if (needRssi) {
        delay(2);
        rssi = cc1101.getRSSI();
        updateTopSignals(freqInHz, rssi);
    }

    return rssi;

#else

    float rssi = 0.0f;
    uint8_t spiBuff[32];
    int32_t freq = (uint32_t)(freqInHz / SX127x_FREQUENCY_STEP_SIZE);

    sx1278WriteRegister0(0x01, 0x01);
    delay(2);

    spiBuff[0] = 0x80 | 0x06;
    spiBuff[3] = freq & 0xFF; freq >>= 8;
    spiBuff[2] = freq & 0xFF; freq >>= 8;
    spiBuff[1] = freq & 0xFF;

    digitalWrite(espBoard.lora_ss, LOW);
    SPI.transfer(spiBuff, 4);
    digitalWrite(espBoard.lora_ss, HIGH);

    sx1278WriteRegister0(0x01, 0x04);
    delay(2);
    sx1278WriteRegister0(0x01, 0x05);
    delay(2);

    if (needRssi) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        rssi = -sx1278ReadRegister(0x11) / 2.0f;
        updateTopSignals(freqInHz, rssi);
    }

    return rssi;

#endif
}




void LilyGo::SX1278_ioctl(const SX1278_Config config[]) {

#ifdef TEMBED_CC1101

    // Translate the relevant SX1278 FSK profile parameters into their CC1101
    // equivalents. Previously this function was a no-op on T-Embed, so all
    // detector-specific demodulator settings were silently lost.
    bool haveBitrateMsb = false;
    bool haveBitrateLsb = false;
    bool haveFdevMsb = false;
    bool haveFdevLsb = false;
    bool haveRxBw = false;
    uint8_t bitrateMsb = 0;
    uint8_t bitrateLsb = 0;
    uint8_t fdevMsb = 0;
    uint8_t fdevLsb = 0;
    uint8_t rxBwReg = 0;

    for (int i = 0; config[i].reg != 0xFF; i++) {
        switch (config[i].reg) {
            case 0x02:
                bitrateMsb = config[i].value;
                haveBitrateMsb = true;
                break;
            case 0x03:
                bitrateLsb = config[i].value;
                haveBitrateLsb = true;
                break;
            case 0x04:
                fdevMsb = config[i].value;
                haveFdevMsb = true;
                break;
            case 0x05:
                fdevLsb = config[i].value;
                haveFdevLsb = true;
                break;
            case 0x12:
                rxBwReg = config[i].value;
                haveRxBw = true;
                break;
            default:
                break;
        }
    }

    int16_t state = cc1101.standby();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 profile standby failed: %d\n", state);
        return;
    }

    float configuredBitrateKbps = 0.0f;
    float configuredFdevKhz = 0.0f;
    float configuredRxBwKhz = 0.0f;

    if (haveBitrateMsb && haveBitrateLsb) {
        uint16_t divisor = ((uint16_t)bitrateMsb << 8) | bitrateLsb;
        if (divisor != 0) {
            configuredBitrateKbps = 32000.0f / (float)divisor;
            state = cc1101.setBitRate(configuredBitrateKbps);
            if (state != RADIOLIB_ERR_NONE) {
                Serial.printf("CC1101 setBitRate %.3f failed: %d\n", configuredBitrateKbps, state);
                return;
            }
        }
    }

    if (haveFdevMsb && haveFdevLsb) {
        uint16_t fdevRaw = ((uint16_t)fdevMsb << 8) | fdevLsb;
        configuredFdevKhz = ((float)fdevRaw * (float)SX127x_FREQUENCY_STEP_SIZE) / 1000.0f;
        state = cc1101.setFrequencyDeviation(configuredFdevKhz);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("CC1101 setFdev %.3f failed: %d\n", configuredFdevKhz, state);
            return;
        }
    }

    if (haveRxBw) {
        uint8_t mantCode = (rxBwReg >> 3) & 0x03;
        uint8_t exp = rxBwReg & 0x07;
        uint16_t mant = 16;
        if (mantCode == 1) {
            mant = 20;
        }
        else if (mantCode == 2) {
            mant = 24;
        }

        float requestedRxBwKhz = 32000.0f / ((float)mant * (float)(1UL << (exp + 2)));
        configuredRxBwKhz = cc1101SupportedBandwidth(requestedRxBwKhz);
        state = cc1101.setRxBandwidth(configuredRxBwKhz);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("CC1101 setRxBw %.1f failed: %d\n", configuredRxBwKhz, state);
            return;
        }
    }

    state = cc1101ApplyRadiosondeProfile();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 radiosonde profile failed: %d\n", state);
        return;
    }

    Serial.printf(
        "CC1101 profile: %.3f kbit/s, fdev %.3f kHz, RXBW %.1f kHz\n",
        configuredBitrateKbps,
        configuredFdevKhz,
        configuredRxBwKhz
    );

#else

    for (int i = 0; config[i].reg != 0xFF; i++) {
        uint8_t regAddr = config[i].reg;
        uint8_t targetValue = config[i].value;
        sx1278WriteRegister0(regAddr, targetValue);
    }

    delay(2);

#endif
}

void LilyGo::a100msTask()
{
    taskCalled_Cntr++;
#ifdef TEMBED_CC1101
    static uint32_t launcherButtonSince = 0;

    if (digitalRead(6) == LOW) {
        if (launcherButtonSince == 0) {
            launcherButtonSince = millis();
        }
        else if (millis() - launcherButtonSince >= 2500) {
    Serial.println("Returning to Launcher...");

    // Kurzer Deep-Sleep erzeugt einen DEEPSLEEP_RESET.
    // Launcher 2.9.1 fängt diesen Start wieder ab.
    esp_sleep_enable_timer_wakeup(1000000ULL); // 1 Sekunde
    delay(50);
    esp_deep_sleep_start();
}
    }
    else {
        launcherButtonSince = 0;
    }
#endif
    if (Serial.available()) {
        uint8_t key = Serial.read();
        if (key != 10) {
            guiCmd[guiCmdIdx++] = key;
        } else {
            guiCmd[guiCmdIdx] = 0;
            handleConsole(&guiCmd[0]);
            guiCmdIdx = 0;
        }
    }

    if (taskCalled_Cntr % 10 == 0){   // Every second, increase age of data  
            debug_age = (millis() - latestDebugMsg)/1000;
            if(activeScreen == SCREEN_DEBUG)
                OLED_drawScreen(SCREEN_DEBUG,false); 
   
            if(activeScreen == SCREEN_SCANNER)
               OLED_drawScreen(SCREEN_SCANNER,false);
#ifdef TEMBED_CC1101
            static uint32_t previousClockEdges = 0;
            uint32_t currentClockEdges = cc1101ClockEdges;
            Serial.printf(
                "CC1101 RX: clock=%lu/s sync=%lu dropped=%lu\n",
                (unsigned long)(currentClockEdges - previousClockEdges),
                (unsigned long)rs41RealSyncHits,
                (unsigned long)cc1101DroppedBits
            );
            previousClockEdges = currentClockEdges;
#endif
    // uint64_t rxedTmp = rxedBits;
    // ESP_LOGE("HP", "rxed = 0x%llx, pattern = 0x%llx, dtstate = %d", rxedTmp, pattern, dtstate);
    }

   
    if(taskCalled_Cntr == 30)         // After showing startup screen for 3 seconds, switch to main screen
            OLED_drawScreen(SCREEN_SONDEDATA); 

    #ifndef TEMBED_CC1101

if (taskCalled_Cntr % 100 == 0)
    digitalWrite(LED_BUILTIN, HIGH);
else if(taskCalled_Cntr % 100 == 1)
    digitalWrite(LED_BUILTIN, LOW);

#endif
}

void LilyGo::setDisplayData(double lat_in, double lon_in, double alt_in, float freq_in,char *id_in, float rssi_in, uint32_t frameCounter)
{
   xTimerStart(screenSaverTimer, 0);

   lat  = lat_in;
   lon  = lon_in;
   freqMhz = freq_in;
   alt  = alt_in;
   rssi = rssi_in;
   debug_RS41frameNr = frameCounter;
   strcpy(id,id_in);
   OLED_drawScreen(SCREEN_CURRENT,true);  
}

void LilyGo::setDisplayFreq(float freqHz)
{
   xTimerReset( screenSaverTimer, 0);

   freqMhz = freqHz/1e6;
   lat  = 0;
   lon  = 0;
   alt  = 0;
   rssi = -128.0f;
   debug_RS41frameNr = 0;
   id[0] = 0;
   OLED_drawScreen(SCREEN_CURRENT,true);
}


void LilyGo::OLED_drawScreen(uint8_t screen, bool disableScreenSaver)
{
  #ifdef TEMBED_CC1101
    (void)screen;
    (void)disableScreenSaver;
    return;
#endif
    
    char s[40];
    if(disableScreenSaver){
        OLED_show(true);
        if(screen != SCREEN_SCANNER)
            xTimerReset( screenSaverTimer, 0);
    }

    if(screen != SCREEN_CURRENT)
        activeScreen = screen;  

    switch (activeScreen) {
        case SCREEN_STARTUP:
            if(isBoardHELTEC) {
                digitalWrite(espBoard.oled_rst, LOW);
                delay(50);
                digitalWrite(espBoard.oled_rst, HIGH);
            }
            display->init();
            display->flipScreenVertically();
            display->clear();
            display->displayOn();
            display->setColor(WHITE);
            display->drawXbm(0, 0, image_width, image_height, image_bits);
            display->setFont(ArialMT_Plain_24);
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawStringf(128,42,s,"V%d.%d",FIRMWARE_VERSION_MAJOR,FIRMWARE_VERSION_MINOR);
            break;
        case SCREEN_SONDEDATA:
            xTimerReset(screenSaverTimer, 0);
            display->clear();
            display->setColor(WHITE);
            display->setFont(ArialMT_Plain_16);
            display->setTextAlignment(TEXT_ALIGN_CENTER);
            display->drawStringf(63,0,s,"%6.3f",freqMhz);
            display->setTextAlignment(TEXT_ALIGN_LEFT);
            display->drawStringf(0,32,s,"%7.5f",lat);
            display->drawStringf(0,48,s,"%7.5f",lon);
            display->drawStringf(0,16,s,"%s",id);
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawStringf(127,32,s,"%.0f  ",alt);
            display->setFont(ArialMT_Plain_10);
            display->drawString(127,37,"m");
            OLED_drawBat();
            OLED_drawRSSI();
            if(BTisConnected )
                display->drawIco16x16(0,0, &BTon[0]);

            break;
        case SCREEN_DEBUG: 
            char dbgMsg[20];
            display->clear();
            display->setColor(WHITE);
            display->setFont(ArialMT_Plain_16);
            display->setTextAlignment(TEXT_ALIGN_LEFT);
            //display->drawString(0, 0,dbgMsg);
            if(debug_RS41BlockCntr > 0)
            {
                display->drawStringf(0,16,dbgMsg,"C:%d/%d",debug_RS41CrcCntr,debug_RS41BlockCntr);
                display->drawStringf(0,32,dbgMsg,"#%d",debug_RS41frameNr);
            }
            display->drawStringf(0,48,dbgMsg,"R:%s",rereMsg[esp_reset_reason()]);
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            if(debug_RS41BlockCntr > 0)
                display->drawStringf(127,16,dbgMsg,"%ds",debug_age);
            display->drawStringf(127,32,dbgMsg,"%.1fdB",rssi);
            break;
        case SCREEN_SCANNER: 
            display->clear();
            display->setColor(WHITE);
            display->setFont(ArialMT_Plain_16);    
            display->setTextAlignment(TEXT_ALIGN_LEFT);
            display->drawStringf(0,0 ,s,"%6.2f",topSignals[0].freq/100.0);
            display->drawStringf(0,16,s,"%6.2f",topSignals[1].freq/100.0);
            display->drawStringf(0,32,s,"%6.2f",topSignals[2].freq/100.0);
            display->drawStringf(0,48,s,"%6.2f",topSignals[3].freq/100.0);
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawStringf(127,0 ,s,"%.1fdB",topSignals[0].rssi);
            display->drawStringf(127,16,s,"%.1fdB",topSignals[1].rssi);
            display->drawStringf(127,32,s,"%.1fdB",topSignals[2].rssi);
            display->drawStringf(127,48,s,"%.1fdB",topSignals[3].rssi);
            break;
        case SCREEN_SHUTDOWN:
            display->clear();
            display->setColor(WHITE);
            display->setFont(ArialMT_Plain_24);
            display->setTextAlignment(TEXT_ALIGN_CENTER);
            display->drawString(63,24,"Sleeping...");
            display->display();
            vTaskDelay(3000/portTICK_PERIOD_MS);
            display->displayOff();
            break;
        default:
            break;  
    }

    display->display();
}

void LilyGo::toggleDebugScreen()
{
    OLED_drawScreen((activeScreen == SCREEN_DEBUG) ? SCREEN_SONDEDATA : SCREEN_DEBUG, true);
}

void LilyGo::toggleScannerScreen(int enable)
{
    if(enable == 2){
        xTimerStop(screenSaverTimer, 0);
        OLED_drawScreen(SCREEN_SCANNER, true);
    }
    else{
        OLED_drawScreen(SCREEN_SONDEDATA, true);
        resetTopList();
    }
}

void LilyGo::switchOffScreen()
{
    OLED_show(false);
}

void LilyGo::setDebugCrc(int eCrcCntr, int blockCntr)
{
    xTimerReset( screenSaverTimer, 0);
    debug_RS41CrcCntr   = eCrcCntr;
    debug_RS41BlockCntr = blockCntr;
    latestDebugMsg  = millis();
    if(activeScreen == SCREEN_DEBUG)
        OLED_drawScreen(SCREEN_DEBUG, true);
}   

void LilyGo::OLED_updateVoltage(float vBatt_in)
{
    #ifdef TEMBED_CC1101
    vBatt = vBatt_in;
    vBattLast = vBatt_in;
    return;
#endif
    if(vBatt_in != vBattLast){
        vBatt     = vBatt_in;
        vBattLast = vBatt_in;
        if(activeScreen == SCREEN_SONDEDATA){
            OLED_drawBat();
            display->display();
        }   
    }
}

void LilyGo::OLED_drawBat()
{
    #ifdef TEMBED_CC1101
    return;
#endif
//ESP_LOGE("HP","vBatt = %f", vBatt);
    //4.14 voll ohne laden, 3.0V leer, 3.9V ca. 50% Ladung
    //4.19 voll mit laden
    //3.8 nach 4h
    
    //display->drawProgressBar(104, 2, 24, 12, 50/*(uint8_t)(vBatt*100/4.2)*/);

    display->drawRect(104, 2, 24, 12);
    display->fillRect(102, 6, 2, 4);

    // if(isCharging)
    // {
    //     display->drawXbm(112, 3, 8, 10, bolt_tiny);
    // }
    // else
    {  
        int empty = (int)((4.0 - vBatt)*18.3);  
        if(empty < 0) 
          empty = 0;
        display->fillRect(106+empty, 4, 20-empty, 8);
    }
}

void LilyGo::OLED_drawRSSI()
{
    #ifdef TEMBED_CC1101
    return;
#endif
//   uint8_t n;
//   if(rssi >= -65) n = 5;
//   else if((rssi < -65) && (rssi >= -80)) n = 4;
//   else if((rssi < -80) && (rssi >= -95)) n = 3;
//   else if((rssi < -95) && (rssi >= -110)) n = 2;
//   else n = 1;  
//   //   = (rssi+155)/27;
//   for(int i = 0; i < 5; i++)
//   {
//     if(i==n)
//       display->setColor(BLACK);
//     display->fillRect(103+i*5, 53, 4, 10);
//   }
//   display->setColor(WHITE);
    char s[20];
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->setFont(ArialMT_Plain_16);
    display->drawStringf(127,48,s,"%.0fdB",rssi);
}

void LilyGo::handleConsole(const char *cmd)
{
    //static uint8_t debug(1);
    //const char *onOffState[] = {"off","on"};
    static bool isDisplayOn(true);

    Serial.print("cmd> ");
    Serial.write(*cmd);
    Serial.println();
    switch(*cmd)
    {
      case 'h':
      {
          Serial.println("h:  help");    
          Serial.println("d:  toggle Display");
          Serial.println("r:  register dump");
          Serial.println("x:  reboot");
          Serial.println("s:  Screen");
          break;
      }      
      case 'd':
      {
          isDisplayOn = !isDisplayOn;
          OLED_show(isDisplayOn);
          break;
      }    
      case 'r':
{
#ifdef TEMBED_CC1101
    Serial.printf("CC1101 counters: clock=%lu sync=%lu dropped=%lu\n",
                  (unsigned long)cc1101ClockEdges,
                  (unsigned long)rs41RealSyncHits,
                  (unsigned long)cc1101DroppedBits);
    for (int i = 0; i <= 0x2E; i++) {
        int16_t value = cc1101.SPIgetRegValue((uint8_t)i);
        Serial.printf("CC1101[0x%02X] = 0x%02X\n", i, value & 0xFF);
    }
#else
    for(int i=0;i<0x80;i++)
    {
       Serial.printf("Reg[0x%02x] = 0x%02x\n",i,sx1278ReadRegister(i));
    }
#endif
    break;
}
      case 'x':
      {
          esp_restart();
          break;
      } 

      case 's':
      {        
          uint8_t snr = cmd[1]-48;
          if(snr < SCREEN_MAX)
          {
             OLED_drawScreen(snr,true);
          }
          break;
      }       
       


      default:
        Serial.printf("key was %d\n",*cmd);
        break;  
    }
}


void LilyGo::updateTopSignals(uint32_t newFreq, float newRssi) {
    int existingIdx = -1;
    newFreq /= 10000; // Convert to 10 kHz steps for easier comparison and display

    // 1. Check if the frequency is already present in the top 4
    for (int i = 0; i < 4; i++) {
        if (topSignals[i].freq == newFreq) {
            existingIdx = i;
            break;
        }
    }

    if (existingIdx != -1) {
        // Frequency exists: Update RSSI only if the new signal is stronger
        if (newRssi > topSignals[existingIdx].rssi) {
            topSignals[existingIdx].rssi = newRssi;
        } else {
            return; // Existing entry is already stronger
        }
    } else {
        // New frequency: Only process if it's stronger than the current 4th place
        if (newRssi <= topSignals[3].rssi) return;
        
        // Replace the weakest signal (rank 4)
        topSignals[3].freq = newFreq;
        topSignals[3].rssi = newRssi;
    }

    // 2. Mini insertion sort to move the updated/new signal to its correct rank
    // Since only one element is out of order, max 3 swaps are needed
    for (int i = 3; i > 0; i--) {
        if (topSignals[i].rssi > topSignals[i-1].rssi) {
            Signal temp     = topSignals[i];
            topSignals[i]   = topSignals[i-1];
            topSignals[i-1] = temp;
        } else {
            break; // Correct position reached
        }
    }
}

void LilyGo::resetTopList() {
    for (int i = 0; i < 4; i++) {
        topSignals[i].freq = 0;
        topSignals[i].rssi = -128;
    }
}