//#include "ble.h"
//#include <BLEDevice.h>
//#include <BLE2902.h>
#include <Arduino.h>
#include "lpclib_types.h"
#include "lilygo.h"

/* keep in sync with sys.c !! */
#define SYS_OPCODE_BLE_MESSAGE  3  
#define COMMAND_LINE_SIZE   400
#define MAX_BLE_MESSAGES    3
typedef struct {
    uint8_t opcode;
    union {
        LPCLIB_Event event;
        int bufferIndex;
    };
} publicSYS_Message;
/* keep in sync with sys.c !! */

char bleCommandLine[MAX_BLE_MESSAGES][COMMAND_LINE_SIZE];
NimBLECharacteristic *pRxCharacteristic;
NimBLECharacteristic *pTxCharacteristic;
NimBLECharacteristic *pRxCharacteristic2;
NimBLECharacteristic *pTxCharacteristic2;
QueueHandle_t msgQueue = nullptr;

void BLE_setMsgQueue(QueueHandle_t q) { 
    msgQueue = q; 
}

void BLE_setup(bool isSpecialDevice)
{
  NimBLEDevice::init("RaOnTTGO");
  //NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // optional: increase power if necessary 
  //NimBLEDevice::setMTU(512); 

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  NimBLEService *pService = pServer->createService(SERVICE_UUID);  
  pTxCharacteristic = pService->createCharacteristic(
                        UUID_VSP_CHAR_TX, 
                        NIMBLE_PROPERTY::WRITE_NR);  // | NIMBLE_PROPERTY::WRITE);
  pTxCharacteristic->setCallbacks(new MyCallbacks());
  pRxCharacteristic = pService->createCharacteristic(
                        UUID_VSP_CHAR_RX,
                        NIMBLE_PROPERTY::NOTIFY);
  pRxCharacteristic->addDescriptor(new NimBLE2904());
  //pRxCharacteristic->setValue("xy");
  pService->start();

//   NimBLEService *pService2 = pServer->createService(SERVICE_UUID2);  
//   pTxCharacteristic2 = pService2->createCharacteristic(
//                         UUID_VSP_CHAR_TX2, 
//                         NIMBLE_PROPERTY::WRITE_NR);  // | NIMBLE_PROPERTY::WRITE);
//   pTxCharacteristic2->setCallbacks(new MyCallbacksApp2());
//   pRxCharacteristic2 = pService2->createCharacteristic(
//                         UUID_VSP_CHAR_RX2,
//                         NIMBLE_PROPERTY::NOTIFY);
//   pRxCharacteristic2->addDescriptor(new NimBLE2904());
//   //pRxCharacteristic2->setValue("xy");
//   pService2->start();



  // Advertising Configuration
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->addServiceUUID(SERVICE_UUID2);
  pAdvertising->setScanResponse(true);
//  pAdvertising->setAppearance(0x00);  // Standard Generic Phone, for findability (Appearance & Flags)
  pAdvertising->setMinPreferred(0x06);  // function that helps with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);  // function that helps with iPhone connections issue
  pAdvertising->start();
}

void MyCallbacks::onWrite (NimBLECharacteristic *pCharacteristic) 
{ 
    //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    publicSYS_Message aMessage;
    static int nextBufferIdx = 0;

    strncpy(bleCommandLine[nextBufferIdx], pCharacteristic->getValue().c_str(), COMMAND_LINE_SIZE);
    aMessage.bufferIndex = nextBufferIdx; 
    nextBufferIdx   = ++nextBufferIdx % MAX_BLE_MESSAGES;
    aMessage.opcode = SYS_OPCODE_BLE_MESSAGE;    
    xQueueSend/*FromISR*/(msgQueue, &aMessage,0/*,&xHigherPriorityTaskWoken*/ );  
    //if( xHigherPriorityTaskWoken == pdTRUE ){portYIELD_FROM_ISR(); }
}

// void MyCallbacksApp2::onWrite (NimBLECharacteristic *pCharacteristic) 
// {
//     // uint16_t currentMTU = NimBLEDevice::getMTU();
//     // Serial.printf("onWrite: MTU = %d",currentMTU);
//     std::string val = pCharacteristic->getValue();
//     char* d_ptr = (char*)malloc(val.length() + 1); 
//     if (d_ptr) {
//         strcpy(d_ptr, val.c_str());
//         if (xQueueSend(bleQueue, &d_ptr, 0) != pdPASS) {
//             Serial.println("Queue voll!");
//         }
//     }
// }