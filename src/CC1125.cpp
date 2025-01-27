#include "CC1125.h"

CC1125::CC1125(Adafruit_LSM6DSOX *SOX, Adafruit_LIS3MDL *MAG, Adafruit_BMP3XX *BMP): 
sox(SOX), mag(MAG), bmp(BMP) {}

CC1125::~CC1125() {}

CC1125Status CC1125::initCC1125() 
{
   CC1125Status status = CC1125_INVALID;
   uint8_t marcstate = 0;
   uint8_t rx_buf = 0; 
   uint8_t partNumber = 0;  

   _spi->begin();

   // Reset radio
   pinMode(PB15, OUTPUT);
   digitalWrite(PB15, LOW);
   digitalWrite(PB15, HIGH);
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

void CC1125::runTX(uint8_t* data, size_t len)
{

   uint8_t* txBuffer;
   uint8_t rxBuffer[len];
   uint8_t statusBits = 0;
   uint8_t txbytes = 0;
   uint8_t marcstate = 0;
   uint8_t element = 0;
   bool match = true;

   txBuffer = createPacket(data, len);

   // Write packet to TX FIFO
   //Max is 0x80
   cc1125spi_TX_FIFO(txBuffer, len);
   cc1125spi_read(CC1125_FIFO_DIRECT,  rxBuffer, sizeof(rxBuffer));
   cc1125spi_read(CC1125_NUM_TXBYTES, &txbytes, 1);

   for(int i = 0; i < len; i++)
   {
      if(rxBuffer[i] != txBuffer[i])
      {
         match = false;
      }
   }


   // Strobe TX to send packet
   if((match == true) && (txbytes > 0))
   {

      cc1125spi_read(CC1125_STX, &statusBits, 1);
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
   uint8_t rxBuffer[111] = {0};
   uint8_t rxbytes = 0;
   uint8_t marcstate = 0;
   uint8_t statusBits = 0;

   cc1125spi_read(CC1125_SRX, &statusBits, 1);
   //no clue what this is doing need to define these magic number
   cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   while(marcstate < 13 ){ cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);};

   while((marcstate & 0x1F) == 0xD)
   {
      Serial1.println("RX State");
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }
   
   cc1125spi_read(CC1125_NUM_RXBYTES, &rxbytes, 1);
   if(rxbytes > 0)
   {
      cc1125spi_RX_FIFO(rxBuffer, rxbytes);
   }

   while(marcstate == 17)
   {
      cc1125spi_read(CC1125_SFRX, &statusBits, 1);
      cc1125spi_read(CC1125_MARCSTATE, &marcstate, 1);
   }

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

 void CC1125::telemetryGroundStation()
 {
   uint8_t command = {CC1125_TELEMTRY_GS};
   uint8_t received[0x80];
   DataPoints_t data;

   // should send until we recieve, add a time out
   // nneed to add if 'q' then stop, flush everything before quitting

   unsigned long startTime = millis();
   while (true) {
        runTX(&command, sizeof(command));
        runRX(received);

        if ((millis() - startTime) >= TIMEOUT_MS) {
            Serial1.println("Timeout waiting for acknowledgment from rocket.");
            return;
        }

        if (received[0] == CC1125_ACKNOWLEDGE) {
            break;
        }
    }


   while (true /*cmd line doesn't equal q */) {
      runRX(received);

      // Check for timeout or quit condition
      if (0/* Check for inactivity timeout or 'q' command */) {
            command = CC1125_QUIT;
            runTX(&command, sizeof(command));
            break;
      } else {
            command = CC1125_ACKNOWLEDGE;
            runTX(&command, sizeof(command));
      }

      // Cast received buffer to DataPoints_t
      memcpy(&data, received, sizeof(DataPoints_t));
      // Process the received data here, if necessary
      Serial1.print("BMP390 DATA:\r\n");
      Serial1.print("Pressure: "); Serial1.print(data.altitude); Serial1.print(" Pa\t");
      Serial1.print("Altitude: "); Serial1.print(data.pressure); Serial1.println(" m\n");
      Serial1.print("Temperature: "); Serial1.print(data.temp_bmp); Serial1.println(" d/s\n");

      Serial1.print("LSM6DSOX DATA:\r\n");
      Serial1.print("X: "); Serial1.print(data.acceleration_x); Serial1.print(" m/s^2\t");
      Serial1.print("Y: "); Serial1.print(data.acceleration_y); Serial1.print(" m/s^2\t");
      Serial1.print("Z: "); Serial1.print(data.acceleration_z); Serial1.println(" m/s^2");
      Serial1.print("X: "); Serial1.print(data.gyro_x); Serial1.print(" d/s\t");
      Serial1.print("Y: "); Serial1.print(data.gyro_y); Serial1.print(" d/s\t");
      Serial1.print("Z: "); Serial1.print(data.gyro_z); Serial1.println(" d/s\n");

      Serial1.print("LIS3MDL DATA:\r\n");
      Serial1.print("X: "); Serial1.print(data.magnetic_x); Serial1.print(" µT\t");
      Serial1.print("Y: "); Serial1.print(data.magnetic_y); Serial1.print(" µT\t");
      Serial1.print("Z: "); Serial1.print(data.magnetic_z); Serial1.println(" µT\n");

    }


 }


 void CC1125::telemetryRocket()
 {
   uint8_t command = CC1125_ACKNOWLEDGE;
   uint8_t received[0x80];
   DataPoints_t data;


   // need to recieve untill we get the TELEMTRY command
   unsigned long startTime = millis();
   while (true) {
      runRX(received);

      if ((millis() - startTime) >= TIMEOUT_MS) {
         Serial1.println("Timeout waiting for telemetry command.");
         return;
      }

      if (received[0] == CC1125_TELEMTRY_GS) {
         break;
      }
   }
   

   // Acknowledge telemetry command
   runTX(&command, sizeof(command));

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
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sensors_event_t mag_event; 

    mag->getEvent(&mag_event);
    data->magnetic_x = mag_event.magnetic.x;
    data->magnetic_y = mag_event.magnetic.y;
    data->magnetic_z = mag_event.magnetic.z;

    sox->getEvent(&accel, &gyro, &temp);
    data->acceleration_x = accel.acceleration.x;
    data->acceleration_y = accel.acceleration.y;
    data->acceleration_z = accel.acceleration.z;
    data->gyro_x = gyro.gyro.x;
    data->gyro_y = gyro.gyro.y;
    data->gyro_z = gyro.gyro.z;

    bmp->performReading();
    data->altitude = bmp->readAltitude(1013.25);
    data->pressure = bmp->pressure;
    data->temp_bmp = bmp->temperature;
 }

uint8_t* CC1125::createPacket(uint8_t *data, size_t len) 
{
   uint8_t txBuffer[len];
   txBuffer[0] = len;                       

   return txBuffer;
}

void CC1125::cc1125spi_TX_FIFO(uint8_t *data, size_t length)
{
   uint8_t address = CC1125_FIFO;
   address |= 0x7F;
   digitalWrite(PB12, LOW);
   _spi->transfer(address);
   _spi->transfer(data, length, 1);
   digitalWrite(PB12, HIGH);
}

void CC1125::cc1125spi_RX_FIFO(uint8_t *data, size_t length)
{
   uint8_t address = CC1125_FIFO;
   address |= 0xFF;
   digitalWrite(PB12, LOW);
   _spi->transfer(&address, data, length);
   digitalWrite(PB12, HIGH);
}

void CC1125::cc1125spi_write(uint16_t addr, uint8_t *data, size_t length, bool TX)
{

   volatile uint8_t address_temp = 0x00;
   uint8_t address = (uint8_t)(addr & 0xFF);
   digitalWrite(PB12, LOW);
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
   
   digitalWrite(PB12, HIGH);
}

void CC1125::cc1125spi_read(uint16_t addr, uint8_t *data, size_t length, bool TX)
{

   volatile uint8_t address_temp = 0x00;
   uint8_t address = (uint8_t)(addr & 0xFF);

   digitalWrite(PB12, LOW);
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
    digitalWrite(PB12, HIGH);

}