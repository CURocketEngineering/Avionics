/* ------------------------------------------------------------ *
 *  Low‑level SPI helpers – ESP32‑Arduino (v2.x) compatible
 * ------------------------------------------------------------ */
#include <SPI.h>          // make sure this stays above the driver
#include "ArduinoHAL.h"
#include "CC1125.h"

 void CC1125::cc1125spi_TX_FIFO(uint8_t * data, size_t length)
 {
     uint8_t header = CC1125_FIFO | 0x7F;          // burst‑write
     digitalWrite(_cs, LOW);
     _spi->transfer(header);                       // send header
     _spi->writeBytes(data, length);               // write payload
     digitalWrite(_cs, HIGH);
 }
 
 void CC1125::cc1125spi_RX_FIFO(uint8_t * data, size_t length)
 {
     uint8_t header = CC1125_FIFO | 0xFF;          // burst‑read
     digitalWrite(_cs, LOW);
     _spi->transfer(header);                       // send header
     _spi->transferBytes(nullptr, data, length);   // read payload
     digitalWrite(_cs, HIGH);
 }
 
 void CC1125::cc1125spi_write(uint16_t addr,
                              uint8_t  * data,
                              size_t     length,
                              bool       TX)
 {
     uint8_t a   = addr & 0xFF;
     uint8_t hdr = 0;
 
     digitalWrite(_cs, LOW);
 
     if ((addr >> 8) == 0x2F) {                    // extended space
         hdr = 0x2F | 0x40;                        // write‑burst
         _spi->transfer(hdr);
         _spi->transfer(a);
     } else if (a == 0x3E) {                       // FIFO access
         hdr = 0x3E | 0x40;                        // write‑burst
         _spi->transfer(hdr);
         _spi->transfer(TX ? 0x00 : 0x80);         // TX/RX FIFO select
     } else {
         hdr = a | 0x40;                           // normal write
         _spi->transfer(hdr);
     }
 
     _spi->writeBytes(data, length);               // payload
     digitalWrite(_cs, HIGH);
 }
 
 void CC1125::cc1125spi_read(uint16_t addr,
                             uint8_t  * data,
                             size_t     length,
                             bool       TX)
 {
     uint8_t a   = addr & 0xFF;
     uint8_t hdr = 0;
 
     digitalWrite(_cs, LOW);
 
     if ((addr >> 8) == 0x2F) {                    // extended space
         hdr = 0x2F | 0xC0;                        // read‑burst
         _spi->transfer(hdr);
         _spi->transfer(a);
     } else if (a == 0x3E) {                       // FIFO access
         hdr = 0x3E | 0xC0;                        // read‑burst
         _spi->transfer(hdr);
         _spi->transfer(TX ? 0x00 : 0x80);         // TX/RX FIFO select
     } else {
         hdr = a | 0x80;                           // normal read
         _spi->transfer(hdr);
     }
 
     _spi->transferBytes(nullptr, data, length);   // read payload
     digitalWrite(_cs, HIGH);
 }
 