#ifndef RA_UTILS_H
#define RA_UTILS_H

#include <Arduino.h>


#ifdef __cplusplus
extern "C" {
#endif

bool utilsRS41_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);
bool utilsRS92_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);
bool utilsGTH3_checkCRC(uint8_t *buffer, int length, uint16_t receivedCRC);
bool utilsCF06_checkCRCOuter(uint8_t *buffer, int length, uint16_t receivedCRC);
bool utilsCF06_checkCRCInner(uint8_t *buffer, int length, uint16_t receivedCRC);
bool utilsWINDSOND_checkCRC(uint8_t *buffer, int length);
uint32_t __RBIT(uint32_t v);

#ifdef __cplusplus
}
#endif

#endif // RA_UTILS_H