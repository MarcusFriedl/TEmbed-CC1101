#include <lilygo.h>
#include <SPI.h>
#include "freertos/stream_buffer.h"
#include "bridge.h"
#include "scanner.h"
#include "sys.h"

extern "C" {
    void PIN_INT3_IRQHandler2(unsigned int bit);
    void MAILBOX_IRQHandler(uint32_t requests);
#ifdef TEMBED_CC1101
    void TEMBED_displaySetup();
    void TEMBED_displayService();
    void TEMBED_gpsSetup();
    void TEMBED_gpsService();
#endif
}

extern uint8_t PIN_DIO1;

IRAM_ATTR void onDIO1Edge();

SYS_Handle sys;
SCANNER_Handle scanner;
SONDE_Handle sonde;

TaskHandle_t xTaskScanner = NULL;
TaskHandle_t xTaskSyncDet = NULL;
StreamBufferHandle_t xBitBuffer;
const size_t xStreamBufferSizeBytes = 1024;
const size_t xTriggerLevel = 1;

int i_cntr = 0,*hc=0;

void SYNCDET_thread (void *param)
{
    uint8_t receivedBit;

    while (1)
    {
        if (xStreamBufferReceive(xBitBuffer, &receivedBit, 1, portMAX_DELAY) > 0) {
            do
            {
                PIN_INT3_IRQHandler2(receivedBit);
            } while (xStreamBufferReceive(xBitBuffer, &receivedBit, 1, 0) > 0);
        }
    }
}

#ifdef TEMBED_CC1101
void LAUNCHER_ESCAPE_thread(void *param)
{
    uint32_t pressedSince = 0;

    pinMode(6, INPUT_PULLUP);

    while (true) {
        if (digitalRead(6) == LOW) {
            if (pressedSince == 0) {
                pressedSince = millis();
            }
            else if (millis() - pressedSince >= 2500) {
                Serial.println("Emergency return to Launcher...");
                esp_sleep_enable_timer_wakeup(1000000ULL);
                delay(50);
                esp_deep_sleep_start();
            }
        }
        else {
            pressedSince = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Weak-signal fix confirmed on-air with RS41 T4550632.
// The original Flipper-derived profile used AGCCTRL2=0xC7, which blocks the
// three highest DVGA stages. Re-enable them after receiver initialization.
void CC1101_FULL_GAIN_thread(void *param)
{
    (void)param;
    vTaskDelay(pdMS_TO_TICKS(3000));

    constexpr int CC1101_CS = 12;
    constexpr int CC1101_MISO = 10;
    SPISettings settings(2000000, MSBFIRST, SPI_MODE0);

    SPI.beginTransaction(settings);
    digitalWrite(CC1101_CS, LOW);

    uint32_t start = micros();
    while (digitalRead(CC1101_MISO) == HIGH && (uint32_t)(micros() - start) < 2000U) {
        ;
    }

    SPI.transfer(0x1B);
    SPI.transfer(0x07);
    digitalWrite(CC1101_CS, HIGH);
    SPI.endTransaction();

    Serial.println("CC1101 weak-signal gain enabled: AGCCTRL2=0x07");
    vTaskDelete(NULL);
}
#endif

void setup() {
   Serial.begin(115200);

   ttgo_setup();
#ifdef TEMBED_CC1101
   TEMBED_gpsSetup();
   // TFT and CC1101 share the same hardware SPI pins. Initial drawing happens
   // before scanner/decoder tasks start; later updates are throttled.
   TEMBED_displaySetup();
#endif
#ifdef TEMBED_CC1101
   xTaskCreate(
       LAUNCHER_ESCAPE_thread,
       "LauncherEscape",
       2048,
       NULL,
       20,
       NULL
   );
#endif
   SYS_open(&sys);

   SCANNER_open(&scanner);
   SONDE_open(&sonde);

   xBitBuffer = xStreamBufferCreate(xStreamBufferSizeBytes, xTriggerLevel);
   xTaskCreate(SYNCDET_thread, "SyncDet",  2000, (void *)sys,     12, &xTaskSyncDet);
   xTaskCreate(SYS_thread,     "System",  50000, (void *)sys,     8, NULL);
   xTaskCreate(SCANNER_thread, "Scanner", 20000, (void *)scanner, 4, &xTaskScanner);

#ifdef TEMBED_CC1101
   xTaskCreate(
       CC1101_FULL_GAIN_thread,
       "CC1101FullGain",
       2048,
       NULL,
       6,
       NULL
   );
#endif

   attachInterrupt(PIN_DIO1, onDIO1Edge, RISING);
}

void loop()
{
  vTaskDelay(100/portTICK_PERIOD_MS);
  ttgo_100msTask();
#ifdef TEMBED_CC1101
  TEMBED_gpsService();
  TEMBED_displayService();
#endif
}
