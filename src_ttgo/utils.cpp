#include "utils.h"
#include "CRC16.h"
 
 

bool utilsRS41_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC)
{
    CRC16 crc(CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL);
    crc.add(buffer, length);
    return(receivedCRC == crc.calc());
}

bool utilsRS92_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC)
{
    CRC16 crc(0x1021, 0xFFFF, 0x0000, false, false);
    crc.add(buffer, length);
    return(receivedCRC == crc.calc());
}

bool utilsGTH3_checkCRC(uint8_t *buffer, int length, uint16_t receivedCRC) {
    CRC16 crc(0x1021, 0x0000, 0x0000, true, true);
    crc.add(buffer, length);
    return (receivedCRC == crc.calc());
}

bool utilsCF06_checkCRCOuter(uint8_t *buffer, int length, uint16_t receivedCRC) {
    CRC16 crc(0x1021, 0x0000, 0x0000, false, false);
    crc.add(buffer, length);
    return (receivedCRC == crc.calc());
}

bool utilsCF06_checkCRCInner(uint8_t *buffer, int length, uint16_t receivedCRC) {
    CRC16 crc(0x1021, 0x0000, 0x0000, false, false);
    crc.add(buffer, length);
    return (receivedCRC == crc.calc());
}

bool utilsWINDSOND_checkCRC (uint8_t *buffer, int length)
{
    CRC16 crc(0x1021, 0xFFFF, 0x0000, true, true);  
    bool result = false;

    for (int i = 0; i < length - 2; i++) {
        crc.add(buffer[i]);
        uint16_t receivedCRC = (uint16_t)buffer[i + 1] * 256 + buffer[i + 2];
        if (receivedCRC == crc.calc()) {
            result = true;
            break;
        }
    }
    return result;
}


// uint16_t getCRC(const uint8_t* buffer, size_t length) // Vaisala
// {
//     CRC16 crc(CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL);
//     crc.add(buffer, length);
//     return(crc.calc());
// }

// uint16_t getCRC2(const uint8_t* buffer, size_t length, uint16_t initialValue ) 
// {
//     CRC16 crc(CRC16_CCITT_FALSE_POLYNOME, initialValue);
// //     crc.setInitial(initialValue);
//     crc.add(buffer, length);
//     return(crc.calc());
// }

uint32_t __RBIT(uint32_t v) {  //esp32_rbit
v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
v = ( v >> 16             ) | ( v               << 16);
return v;
}