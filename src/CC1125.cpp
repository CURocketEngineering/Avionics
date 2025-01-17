#include "CC1125.h"

CC1125::CC1125(uint32_t debugLedPin, uint32_t cs) : _debugLedPin(debugLedPin), _cs(cs) {}
CC1125::~CC1125() {}

CC1125Status CC1125::initCC1125() 
{
   CC1125Status status = CC1125_INVALID;
   uint8_t marcstate = 0;
   uint8_t rx_buf = 0; 
   uint8_t partNumber = 0;  

   _spi->begin();

   // Reset radio
   //ARE strobe commands actaully working ????
   pinMode(_debugLedPin, OUTPUT);
   digitalWrite(_debugLedPin, LOW);
   digitalWrite(_debugLedPin, HIGH);
   // cc1125spi_read(CC1125_SRES, &rx_buf, 1);


   do
   {
      cc1125spi_read(CC1125_PARTNUMBER, &rx_buf, 1);
   }
   while(rx_buf != CC1125_ID);
   
   partNumber = rx_buf;

   status = registerConfig();
   if(status == CC1125_FAILURE || status == CC1125_INVALID)
      return status;

   cc1125spi_read(CC1125_SCAL, &rx_buf, 1);

   do 
   {
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   } 
   while (marcstate != 0x41);
   cc1125spi_read(CC1125_SFTX, &rx_buf, 1);
   cc1125spi_read(CC1125_SFRX, &rx_buf, 1);

   cc1125spi_read(CC1125_NUM_TXBYTES, &rx_buf, 1);

   if(partNumber == CC1125_ID)
      status = CC1125_SUCCESS;
   else 
      status = CC1125_FAILURE;
   
   return status;
}

void CC1125::runTX()
{

   uint8_t txBuffer[55] = {0};
   uint8_t txBuffer1[55] = {0};
   uint8_t rxBuffer[110] = {0};
   uint8_t statusBits = 0;
   uint8_t len = 0;
   uint8_t marcstate = 0;
   uint8_t element = 0;
   txBuffer[0] = 55;
   txBuffer1[0]= 55;


   for(int i = 1; i < 55; i++) 
   {
      txBuffer[i] = (uint8_t)rand();
   }

      for(int i = 1; i < 55; i++) 
   {
      txBuffer1[i] = (uint8_t)rand();
   }


   // Write packet to TX FIFO
   //Max is 0x80
   cc1125spi_TX_FIFO(txBuffer, sizeof(txBuffer));
   // cc1125spi_TX_FIFO(txBuffer1, sizeof(txBuffer1));
   //not working
   // cc1125spi_write(CC1125_FIFO_DIRECT,  txBuffer, sizeof(txBuffer));
   cc1125spi_read(CC1125_FIFO_DIRECT,  rxBuffer, sizeof(rxBuffer));
   cc1125spi_read(CC1125_NUM_TXBYTES, &len, 1);

   bool dataMatches = true;
   // for (size_t i = 0; i < 65; i++) 
   // {
   //    if (txBuffer[i] != rxBuffer[i]) 
   //    {
   //       dataMatches = false;
   //       element = i;
   //       break;
   //    }
   // }


   // Strobe TX to send packet
   if(dataMatches == true && len > 0)
   {

      cc1125spi_read(CC1125_STX, &statusBits, 1);
      delay(200);
      while(marcstate != 0x41 )
      { 
         while(marcstate < 19){cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);};
         cc1125spi_read(CC1125_MODEM_STATUS0, &statusBits, 1);
         if(statusBits == 1)
         {
            cc1125spi_read(CC1125_NUM_TXBYTES, &len, 1);
            cc1125spi_read(CC1125_SFTX, &statusBits, 1);
         }
         cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
      };
   }

   cc1125spi_read(CC1125_SFTX, &statusBits, 1);

   delay(100);

}

void CC1125::runRX()
{
   uint8_t statusBits = 0;
   uint8_t rxBuffer[111] = {0};
   uint8_t txBuffer[111] = {0};
   uint8_t marcstate = 0;
   int count = 0;

   cc1125spi_read(CC1125_SRX, &statusBits, 1);
   cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   while(marcstate < 13 ){ cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);};
   cc1125spi_read(CC1125_MODEM_STATUS1, &statusBits, 1); 

   while((marcstate & 0x1F) == 0xD)
   {
      count++;
      Serial1.println("RX State");
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }
   cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   txBuffer[0] = 110; txBuffer[1] = 0xAE; txBuffer[2] = 0x8D;
   // cc1125spi_write(CC1125_FIFO_DIRECT, txBuffer, sizeof(txBuffer), false);
   cc1125spi_read(CC1125_NUM_RXBYTES, &statusBits, 1);
   if(statusBits > 0)
   {
      cc1125spi_RX_FIFO(rxBuffer, sizeof(rxBuffer));
      //cc1125spi_read(CC1125_FIFO_DIRECT, rxBuffer, sizeof(rxBuffer), false);
   }
   while(marcstate == 17)
   {
      cc1125spi_read(CC1125_SFRX, &statusBits, 1);
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }
   delay(1);
}

CC1125Status CC1125::registerConfig() 
{
   CC1125Status status = CC1125_INVALID;
   uint8_t writeByte;

   // Write registers to radio
   for(uint16_t i = 0;
   i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
      writeByte = preferredSettings[i].data;
      cc1125spi_write(preferredSettings[i].addr, &writeByte, 1);
   }

   for(uint16_t i = 0;
   i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
      writeByte = preferredSettings[i].data;
      cc1125spi_write(preferredSettings[i].addr, &writeByte, 1);
      if(preferredSettings[i].data != writeByte)
      {
         status = CC1125_FAILURE;
         break;
      }
      status = CC1125_SUCCESS;
   }

   return status;
}

void CC1125::createPacket(uint8_t *txBuffer) 
{

    txBuffer[0] = 120;                       // Length byte

    // Fill rest of buffer with random bytes
    for(uint8_t i = 1; i < 120; i++) 
    {
        txBuffer[i] = (uint8_t)rand();
    }
}

void CC1125::cc1125spi_TX_FIFO(uint8_t *data, size_t length)
{
   uint8_t address = CC1125_FIFO;
   address |= 0x7F;
   digitalWrite(_cs, LOW);
   _spi->transfer(address);
   _spi->transfer(data, length, 1);
   digitalWrite(_cs, HIGH);
}

void CC1125::cc1125spi_RX_FIFO(uint8_t *data, size_t length)
{
   uint8_t address = CC1125_FIFO;
   address |= 0xFF;
   digitalWrite(_cs, LOW);
   _spi->transfer(&address, data, length);
   digitalWrite(_cs, HIGH);
}

void CC1125::cc1125spi_write(uint16_t addr, uint8_t *data, size_t length, bool TX)
{

   volatile uint8_t address_temp = 0x00;
   uint8_t address = (uint8_t)(addr & 0xFF);
   digitalWrite(_cs, LOW);
   if((addr >> 8) == 0x2F)
   {
      address_temp = 0x2F;
      address_temp |= 0x40;
      address = (address & 0xFF);
      _spi->transfer(address_temp);
      _spi->transfer(address);
      _spi->transfer(data, length, 1);
   }
   else if(address >= 0x30 && address <= 0x3D)
   {
      _spi->transfer(address);
      _spi->transfer(data, length, 1);
   }
   else if(address == 0x3E)
   {
      address_temp = 0x3E;
      address_temp |= 0x40;
      _spi->transfer(address_temp);
      if(TX == false)
         address = 0x80;
      else
         address = 0x00;
      _spi->transfer(address);
      _spi->transfer(data, length, 1);
   }
   else 
   {
      address |= 0x40;
      _spi->transfer(address);
      _spi->transfer(data, length, 1);
   }
   
   digitalWrite(_cs, HIGH);
}

void CC1125::cc1125spi_read(uint16_t addr, uint8_t *data, size_t length, bool TX)
{

   volatile uint8_t address_temp = 0x00;
   uint8_t address = (uint8_t)(addr & 0xFF);

   digitalWrite(_cs, LOW);
   if((addr >> 8) == 0x2F)
   {
      address_temp = 0x2F;
      address_temp |= 0xC0;
      _spi->transfer(address_temp);
      _spi->transfer(address);
      _spi->transfer(0x00, data, length);
   }
   else if(address >= 0x30 && address <= 0x3D)
   {
      address |= 0x80;
      _spi->transfer(&address, data, 1);
   }
   else if(address == 0x3E)
   {
      address_temp = 0x3E;
      address_temp |= 0xC0;
      _spi->transfer(address_temp);
      if(TX == false)
         address = 0x80;
      else
         address = 0x00;
      _spi->transfer(address);
      _spi->transfer(&address, data, length);
   }
   else
   {
      address |= 0xC0;
      _spi->transfer(&address, data, length);
   }
    digitalWrite(_cs, HIGH);

}