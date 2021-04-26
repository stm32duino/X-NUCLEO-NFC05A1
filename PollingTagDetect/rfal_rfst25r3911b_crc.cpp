#include "rfal_rfst25r3911b.h"

/* Function */

uint16_t rfal_rfst25r3911b::rfalCrcCalculateCcitt(uint16_t preloadValue, const uint8_t* buf, uint16_t length)
{
    uint16_t crc = preloadValue;
    uint16_t index;

    for (index = 0; index < length; index++)
    {
        crc = rfalCrcUpdateCcitt(crc, buf[index]);
    }

    return crc;
}

uint16_t rfal_rfst25r3911b::rfalCrcUpdateCcitt(uint16_t crcSeed, uint8_t dataByte)
{
    uint16_t crc = crcSeed;
    uint8_t  dat = dataByte;
    
    dat ^= (uint8_t)(crc & 0xFFU);
    dat ^= (dat << 4);

    crc = (crc >> 8)^(((uint16_t) dat) << 8)^(((uint16_t) dat) << 3)^(((uint16_t) dat) >> 4);

    return crc;
}
