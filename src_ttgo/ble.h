#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
// #include <BLEUtils.h>
// #include <BLEDevice.h>
// #include <ble.h>
#include <NimBLEDevice.h>
#include "lilygo.h"
#include "bridge.h"

#define SERVICE_UUID      "569a1101-b87f-490c-92cb-11ba5ea5167c"
#define UUID_VSP_CHAR_RX  "569a2000-b87f-490c-92cb-11ba5ea5167c"
#define UUID_VSP_CHAR_TX  "569a2001-b87f-490c-92cb-11ba5ea5167c"
#define UUID_VSP_CHAR_CTS "569a2002-b87f-490c-92cb-11ba5ea5167c"
#define UUID_VSP_CHAR_RTS "569a2003-b87f-490c-92cb-11ba5ea5167c"

#define SERVICE_UUID2     "569a3000-b87f-490c-92cb-11ba5ea5167c"
#define UUID_VSP_CHAR_RX2 "569a3001-b87f-490c-92cb-11ba5ea5167c" /* Notify */
#define UUID_VSP_CHAR_TX2 "569a3002-b87f-490c-92cb-11ba5ea5167c" /* Write */

void BLE_setup(bool);
void BLE_setMsgQueue(QueueHandle_t q);

class MyCallbacks: public NimBLECharacteristicCallbacks 
{
  public:
    void onWrite (NimBLECharacteristic *pCharacteristic); 
};

// class MyCallbacksApp2: public NimBLECharacteristicCallbacks 
// {
//   public:
//     void onWrite (NimBLECharacteristic *pCharacteristic) override; 
// };


//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public NimBLEServerCallbacks {
  
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
     ttgo_setBtState(true);
     NimBLEDevice::startAdvertising();
  };

  void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    ttgo_setBtState(false);
    NimBLEDevice::startAdvertising();
  }
};

#endif