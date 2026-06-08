#include <lilygo.h>
#include "freertos/stream_buffer.h"
#include "bridge.h"
#include "scanner.h"
#include "sys.h"

extern "C" {
    void PIN_INT3_IRQHandler2(unsigned int bit);
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
// IRAM_ATTR void onDIO1Edge() {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    // uint8_t bit = (GPIO.in1.val /*>> (PIN_DIO2 - 32)*/) & 0x01;
//     uint8_t bit = digitalRead(PIN_DIO2);
//     xStreamBufferSendFromISR(xBitBuffer, &bit, 1, &xHigherPriorityTaskWoken);
//     if (xHigherPriorityTaskWoken == pdTRUE) { portYIELD_FROM_ISR(); }
// }

void SYNCDET_thread (void *param)
{ 
    uint8_t receivedBit;

    while (1) 
    {
        if (xStreamBufferReceive(xBitBuffer, &receivedBit, 1, portMAX_DELAY) > 0) {
            do
            {
                PIN_INT3_IRQHandler2(receivedBit);  // Call the IRQ handler to process the bit
            } while (xStreamBufferReceive(xBitBuffer, &receivedBit, 1, 0) > 0);      
        }
    }
} 

void setup() {
   Serial.begin(115200);

   ttgo_setup();
   SYS_open(&sys);
   SCANNER_open(&scanner);
   SONDE_open(&sonde);

   xBitBuffer = xStreamBufferCreate(xStreamBufferSizeBytes, xTriggerLevel);
   xTaskCreate(SYNCDET_thread, "SyncDet",  2000, (void *)sys,     12, &xTaskSyncDet);
   xTaskCreate(SYS_thread,     "System",  50000, (void *)sys,     8, NULL);
   xTaskCreate(SCANNER_thread, "Scanner", 20000, (void *)scanner, 4, &xTaskScanner);

   attachInterrupt(PIN_DIO1, onDIO1Edge, RISING);
  }


void loop() 
{
  vTaskDelay(100/portTICK_PERIOD_MS);
  ttgo_100msTask();
}


