#include "CC1125.h"

CC1125::CC1125(uint8_t resetPin,
               uint8_t cs,
               Adafruit_LSM6DSOX *SOX, 
               Adafruit_LIS3MDL *MAG,
               Adafruit_BMP3XX *BMP): 
               _resetPin(resetPin), _cs(cs), sox(SOX), mag(MAG), bmp(BMP) {}

CC1125::~CC1125() {}

CC1125Status CC1125::init() 
{
   CC1125Status status = CC1125_INVALID;
   uint8_t marcstate = 0;
   uint8_t rx_buf = 0; 
   uint8_t partNumber = 0;  

   _spi->begin();

   // Reset radio
   pinMode(_cs, OUTPUT);
   pinMode(_resetPin, OUTPUT);
   digitalWrite(_resetPin, LOW);
   digitalWrite(_resetPin, HIGH);
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
   delay(100);
   for(uint16_t i = 0;
   i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
      writeByte = preferredSettings[i].data;
      cc1125spi_read(preferredSettings[i].addr, &writeByte, 1);
      if(preferredSettings[i].data != writeByte)
      {
         status = CC1125_FAILURE;
         break;
      }
      status = CC1125_SUCCESS;
   }

   return status;
}

void CC1125::runTX(uint8_t* data, size_t len)
{

   uint8_t txBuffer[len+1];
   uint8_t rxBuffer[len+1];

   uint8_t statusBits = 0;
   uint8_t txbytes = 0;
   uint8_t marcstate = 0;
   uint8_t element = 0;
   bool match = true;

   // createPacket(txBuffer, data, len);
   memcpy(txBuffer, data, len);

   // Write packet to TX FIFO
   //Max is 0x80
   cc1125spi_TX_FIFO(txBuffer, len);
   cc1125spi_read(CC1125_FIFO_DIRECT,  rxBuffer, len);
   cc1125spi_read(CC1125_NUM_TXBYTES, &txbytes, 1);

   Serial.flush(); 
   for(int i = 0; i < len; i++)
   {
      if(rxBuffer[i] != txBuffer[i])
      {
         match = false;
         element = i;

         break;
      }
   }


   // Strobe TX to send packet
   if((match == true) && (txbytes > 0))
   {

      cc1125spi_read(CC1125_STX, &statusBits, 1);
      //idle
      while(marcstate != 0x41 )
      { 
         while(marcstate < 19){cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);};
         cc1125spi_read(CC1125_MODEM_STATUS0, &statusBits, 1);
         if(statusBits == 1)
         {
            cc1125spi_read(CC1125_SFTX, &statusBits, 1);
         }
         cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
      };
   }

   cc1125spi_read(CC1125_SFTX, &statusBits, 1);


   delay(100);

}

void CC1125::runRX(uint8_t *rxBuffer)
{
   uint8_t rxbytes = 0;
   uint8_t marcstate = 0;
   uint8_t statusBits = 0;

   cc1125spi_read(CC1125_SRX, &statusBits, 1);
   //no clue what this is doing need to define these magic number
   cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   while(marcstate < 13 ){ cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);};

   while((marcstate & 0x1F) == 0xD)
   {
      DEBUG_PRINTLN("RX State");
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }
   
   cc1125spi_read(CC1125_NUM_RXBYTES, &rxbytes, 1);
   if(rxbytes > 0)
   {
      cc1125spi_RX_FIFO(rxBuffer, rxbytes);
      cc1125spi_read(CC1125_SFRX, &statusBits, 1);

      Serial.println("RX Buffer Contents:");

    for (size_t i = 0; i < rxbytes; i++) {
        // Print each byte in hexadecimal format
        Serial.print("0x");
        if (rxBuffer[i] < 0x10) {
            Serial.print("0"); // Add leading zero for single-digit hex numbers
        }
        Serial.print(rxBuffer[i], HEX);
        Serial.print(" ");

        // Add a newline after every 8 bytes
        if ((i + 1) % 8 == 0) {
            Serial.println();
        }
    }
   }

   while(marcstate == 17)
   {
      //error handling
      cc1125spi_read(CC1125_SFRX, &statusBits, 1);
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }

}

// For Command Response
 void CC1125::telemetryGroundStation()
 {
   uint8_t command[] = {
    (CC1125_TELEMTRY_GS >> 16) & 0xFF,  // Most significant byte (0x54)
    (CC1125_TELEMTRY_GS >> 8) & 0xFF,   // Middle byte (0x45)
    CC1125_TELEMTRY_GS & 0xFF           // Least significant byte (0x4C)
   };
   uint8_t received[0x80];
   uint32_t commandreceived;
   DataPoints_t data;

   // should send until we recieve, add a time out
   // nneed to add if 'q' then stop, flush everything before quitting

   unsigned long startTime = millis();
   while (true) {
         runTX(command, sizeof(command));
         runRX(received);

         if ((millis() - startTime) >= TIMEOUT_MS) {
            Serial.println("Timeout waiting for acknowledgment from rocket.");
            return;
         }
         
         commandreceived = received[0] << 16 |
                           received[1] << 8 |
                           received[2];
         if (commandreceived == CC1125_ACKNOWLEDGE) {
            break;
         }
    }


   while (true /*cmd line doesn't equal q */) {
      runRX(received);

      // Check for timeout or quit condition
      if (0/* Check for inactivity timeout or 'q' command */) {
            command[0] = (CC1125_QUIT >> 16) & 0xFF; 
            command[1] = (CC1125_QUIT >> 8) & 0xFF;   
            command[2] = CC1125_QUIT & 0xFF;  
            runTX(command, sizeof(command));
            break;
      } else {
            command[0] = (CC1125_ACKNOWLEDGE >> 16) & 0xFF; 
            command[1] = (CC1125_ACKNOWLEDGE >> 8) & 0xFF;   
            command[2] = CC1125_ACKNOWLEDGE & 0xFF;  
            runTX(command, sizeof(command));
      }

      // Cast received buffer to DataPoints_t
      memcpy(&data, received, sizeof(DataPoints_t));
      // Process the received data here, if necessary
      Serial.print("BMP390 DATA:\r\n");
      Serial.print("Pressure: "); Serial.print(data.altitude); Serial.print(" Pa\t");
      Serial.print("Altitude: "); Serial.print(data.pressure); Serial.println(" m\n");
      Serial.print("Temperature: "); Serial.print(data.temp_bmp); Serial.println(" d/s\n");

      Serial.print("LSM6DSOX DATA:\r\n");
      Serial.print("X: "); Serial.print(data.acceleration_x); Serial.print(" m/s^2\t");
      Serial.print("Y: "); Serial.print(data.acceleration_y); Serial.print(" m/s^2\t");
      Serial.print("Z: "); Serial.print(data.acceleration_z); Serial.println(" m/s^2");
      Serial.print("X: "); Serial.print(data.gyro_x); Serial.print(" d/s\t");
      Serial.print("Y: "); Serial.print(data.gyro_y); Serial.print(" d/s\t");
      Serial.print("Z: "); Serial.print(data.gyro_z); Serial.println(" d/s\n");

      Serial.print("LIS3MDL DATA:\r\n");
      Serial.print("X: "); Serial.print(data.magnetic_x); Serial.print(" µT\t");
      Serial.print("Y: "); Serial.print(data.magnetic_y); Serial.print(" µT\t");
      Serial.print("Z: "); Serial.print(data.magnetic_z); Serial.println(" µT\n");

    }


 }

// For Command Response
 void CC1125::telemetryRocket()
 {
   uint8_t command[] = {
    (CC1125_ACKNOWLEDGE >> 16) & 0xFF,  // Most significant byte (0x54)
    (CC1125_ACKNOWLEDGE >> 8) & 0xFF,   // Middle byte (0x45)
    CC1125_ACKNOWLEDGE & 0xFF           // Least significant byte (0x4C)
   };
   uint32_t commandreceived;
   uint8_t received[0x80];
   DataPoints_t data;


   // need to recieve untill we get the TELEMTRY command
   unsigned long startTime = millis();
   while (true) {
      runRX(received);

      if ((millis() - startTime) >= TIMEOUT_MS) {
         Serial.println("Timeout waiting for telemetry command.");
         return;
      }
               
      commandreceived = received[0] << 16 |
                        received[1] << 8 |
                        received[2];
      if (commandreceived == CC1125_TELEMTRY_GS) {
         break;
      }
   }
   

   // Acknowledge telemetry command
   runTX(command, sizeof(command));

   // Start transmitting data 
   while (true) {
      retriveData(&data); // Populate data
      runTX(reinterpret_cast<uint8_t*>(&data), sizeof(DataPoints_t));
      runRX(received);

      if (/* Check for acknowledgment timeout or quit command */0) {
         break;
      }
   }

 }

 void CC1125::retriveData(DataPoints_t *data)
 {
   //  sensors_event_t accel;
   //  sensors_event_t gyro;
   //  sensors_event_t temp;
   //  sensors_event_t mag_event; 

   // mag->getEvent(&mag_event);
   // data->magnetic_x = mag_event.magnetic.x;
   // data->magnetic_y = mag_event.magnetic.y;
   // data->magnetic_z = mag_event.magnetic.z;
   // // data->magnetic_x = 0x10;
   // // data->magnetic_y = 0x20; 
   // // data->magnetic_z = 0x30;
   // sox->getEvent(&accel, &gyro, &temp);
   // data->acceleration_x = accel.acceleration.x;
   // data->acceleration_y = accel.acceleration.y;
   // data->acceleration_z = accel.acceleration.z;
   // // data->acceleration_x = 0x40;
   // // data->acceleration_y = 0x50;
   // // data->acceleration_z = 0x60;
   // data->gyro_x = gyro.gyro.x;
   // data->gyro_y = gyro.gyro.y;
   // data->gyro_z = gyro.gyro.z;
   // // data->gyro_x = 0x70;
   // // data->gyro_y = 0x80;
   // // data->gyro_z = 0x90;


   // bmp->performReading();
   // data->altitude = bmp->readAltitude(1013.25);
   // data->pressure = bmp->pressure;
   // data->temp_bmp = bmp->temperature;

   // // data->altitude = 0xAE;
   // // data->pressure = 0x99;
   // // data->temp_bmp = 0x24;

   data->acceleration_x = 1.1f;
   data->acceleration_y = 2.2f;
   data->acceleration_z = 3.3f;
   data->gyro_x = 4.4f;
   data->gyro_y = 5.5f;
   data->gyro_z = 6.6f;
   data->magnetic_x = 7.7f;
   data->magnetic_y = 8.8f;
   data->magnetic_z = 9.9f;
   data->altitude = 10.1f;
   data->pressure = 1013.25f;
   data->temp_bmp = 25.5;  // Double value
   data->reserved = 0.0f;
 }

void CC1125::createPacket(uint8_t *txBuffer, uint8_t *data, size_t len) 
{

    txBuffer[0] = len+5;
    memcpy(&txBuffer[1], data, len);
}

void CC1125::cc1125spi_TX_FIFO(uint8_t *data, size_t length)
{
   uint8_t address = CC1125_FIFO;
   address |= 0x7F;
   digitalWrite(_cs, LOW);
   _spi->transfer(address);
   _spi->transfer(data, nullptr, length);
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
      _spi->transfer(data, nullptr, length);
   }
   else if(address >= 0x30 && address <= 0x3D)
   {
      _spi->transfer(address);
      _spi->transfer(data, nullptr, length);
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
      _spi->transfer(data, nullptr, length);
   }
   else 
   {
      address |= 0x40;
      _spi->transfer(address);
      _spi->transfer(data, nullptr, length);
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
      uint8_t dummy = 0x00;
      _spi->transfer(&dummy, data, length);
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
      address |= 0x80;
      _spi->transfer(address);
      _spi->transfer(&address, data, length);
   }
    digitalWrite(_cs, HIGH);

}