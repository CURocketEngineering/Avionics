#ifndef CC1125_H
#define CC1125_H

#include <SPI.h>
#include <Adafruit_BMP3XX.h>
#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"

using namespace std;

#define CC1125_ID                       0x58 

/* Command strobe registers */
#define CC1125_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC1125_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC1125_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC1125_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC1125_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC1125_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC1125_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC1125_SAFC                     0x37      /*  AFC     - Automatic Frequency Correction */    
#define CC1125_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1125_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC1125_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC1125_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC1125_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC1125_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */

#define CC1125_FIFO_DIRECT              0x3E
#define CC1125_FIFO                     0x3F


/* Chip states returned in status byte */
#define CC1125_STATE_IDLE               0x00
#define CC1125_STATE_RX                 0x10
#define CC1125_STATE_TX                 0x20
#define CC1125_STATE_FSTXON             0x30
#define CC1125_STATE_CALIBRATE          0x40
#define CC1125_STATE_SETTLING           0x50
#define CC1125_STATE_RXFIFO_ERROR       0x60
#define CC1125_STATE_TXFIFO_ERROR       0x70

// Setting Registers
/* configuration registers */
#define CC1125_IOCFG3                   0x0000
#define CC1125_IOCFG2                   0x0001
#define CC1125_IOCFG1                   0x0002
#define CC1125_IOCFG0                   0x0003
#define CC1125_SYNC3                    0x0004
#define CC1125_SYNC2                    0x0005
#define CC1125_SYNC1                    0x0006
#define CC1125_SYNC0                    0x0007
#define CC1125_SYNC_CFG1                0x0008
#define CC1125_SYNC_CFG0                0x0009
#define CC1125_DEVIATION_M              0x000A
#define CC1125_MODCFG_DEV_E             0x000B
#define CC1125_DCFILT_CFG               0x000C
#define CC1125_PREAMBLE_CFG1            0x000D
#define CC1125_PREAMBLE_CFG0            0x000E
#define CC1125_FREQ_IF_CFG              0x000F
#define CC1125_IQIC                     0x0010
#define CC1125_CHAN_BW                  0x0011
#define CC1125_MDMCFG1                  0x0012
#define CC1125_MDMCFG0                  0x0013
#define CC1125_SYMBOL_RATE2             0x0014
#define CC1125_SYMBOL_RATE1             0x0015
#define CC1125_SYMBOL_RATE0             0x0016
#define CC1125_AGC_REF                  0x0017
#define CC1125_AGC_CS_THR               0x0018
#define CC1125_AGC_GAIN_ADJUST          0x0019
#define CC1125_AGC_CFG3                 0x001A
#define CC1125_AGC_CFG2                 0x001B
#define CC1125_AGC_CFG1                 0x001C
#define CC1125_AGC_CFG0                 0x001D
#define CC1125_FIFO_CFG                 0x001E
#define CC1125_DEV_ADDR                 0x001F
#define CC1125_SETTLING_CFG             0x0020
#define CC1125_FS_CFG                   0x0021
#define CC1125_WOR_CFG1                 0x0022
#define CC1125_WOR_CFG0                 0x0023
#define CC1125_WOR_EVENT0_MSB           0x0024
#define CC1125_WOR_EVENT0_LSB           0x0025
#define CC1125_PKT_CFG2                 0x0026
#define CC1125_PKT_CFG1                 0x0027
#define CC1125_PKT_CFG0                 0x0028
#define CC1125_RFEND_CFG1               0x0029
#define CC1125_RFEND_CFG0               0x002A
#define CC1125_PA_CFG2                  0x002B
#define CC1125_PA_CFG1                  0x002C
#define CC1125_PA_CFG0                  0x002D
#define CC1125_PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define CC1125_IF_MIX_CFG               0x2F00
#define CC1125_FREQOFF_CFG              0x2F01
#define CC1125_TOC_CFG                  0x2F02
#define CC1125_MARC_SPARE               0x2F03
#define CC1125_ECG_CFG                  0x2F04
#define CC1125_CFM_DATA_CFG             0x2F05
#define CC1125_EXT_CTRL                 0x2F06
#define CC1125_RCCAL_FINE               0x2F07
#define CC1125_RCCAL_COARSE             0x2F08
#define CC1125_RCCAL_OFFSET             0x2F09
#define CC1125_FREQOFF1                 0x2F0A
#define CC1125_FREQOFF0                 0x2F0B
#define CC1125_FREQ2                    0x2F0C
#define CC1125_FREQ1                    0x2F0D
#define CC1125_FREQ0                    0x2F0E
#define CC1125_IF_ADC2                  0x2F0F
#define CC1125_IF_ADC1                  0x2F10
#define CC1125_IF_ADC0                  0x2F11
#define CC1125_FS_DIG1                  0x2F12
#define CC1125_FS_DIG0                  0x2F13
#define CC1125_FS_CAL3                  0x2F14
#define CC1125_FS_CAL2                  0x2F15
#define CC1125_FS_CAL1                  0x2F16
#define CC1125_FS_CAL0                  0x2F17
#define CC1125_FS_CHP                   0x2F18
#define CC1125_FS_DIVTWO                0x2F19
#define CC1125_FS_DSM1                  0x2F1A
#define CC1125_FS_DSM0                  0x2F1B
#define CC1125_FS_DVC1                  0x2F1C
#define CC1125_FS_DVC0                  0x2F1D
#define CC1125_FS_LBI                   0x2F1E
#define CC1125_FS_PFD                   0x2F1F
#define CC1125_FS_PRE                   0x2F20
#define CC1125_FS_REG_DIV_CML           0x2F21
#define CC1125_FS_SPARE                 0x2F22
#define CC1125_FS_VCO4                  0x2F23
#define CC1125_FS_VCO3                  0x2F24
#define CC1125_FS_VCO2                  0x2F25
#define CC1125_FS_VCO1                  0x2F26
#define CC1125_FS_VCO0                  0x2F27
#define CC1125_GBIAS6                   0x2F28
#define CC1125_GBIAS5                   0x2F29
#define CC1125_GBIAS4                   0x2F2A
#define CC1125_GBIAS3                   0x2F2B
#define CC1125_GBIAS2                   0x2F2C
#define CC1125_GBIAS1                   0x2F2D
#define CC1125_GBIAS0                   0x2F2E
#define CC1125_IFAMP                    0x2F2F
#define CC1125_LNA                      0x2F30
#define CC1125_RXMIX                    0x2F31
#define CC1125_XOSC5                    0x2F32
#define CC1125_XOSC4                    0x2F33
#define CC1125_XOSC3                    0x2F34
#define CC1125_XOSC2                    0x2F35
#define CC1125_XOSC1                    0x2F36
#define CC1125_XOSC0                    0x2F37
#define CC1125_ANALOG_SPARE             0x2F38
#define CC1125_PA_CFG3                  0x2F39
#define CC1125_IRQ0M                    0x2F3F
#define CC1125_IRQ0F                    0x2F40 

/* Status Registers */
#define CC1125_WOR_TIME1                0x2F64
#define CC1125_WOR_TIME0                0x2F65
#define CC1125_WOR_CAPTURE1             0x2F66
#define CC1125_WOR_CAPTURE0             0x2F67
#define CC1125_BIST                     0x2F68
#define CC1125_DCFILTOFFSET_I1          0x2F69
#define CC1125_DCFILTOFFSET_I0          0x2F6A
#define CC1125_DCFILTOFFSET_Q1          0x2F6B
#define CC1125_DCFILTOFFSET_Q0          0x2F6C
#define CC1125_IQIE_I1                  0x2F6D
#define CC1125_IQIE_I0                  0x2F6E
#define CC1125_IQIE_Q1                  0x2F6F
#define CC1125_IQIE_Q0                  0x2F70
#define CC1125_RSSI1                    0x2F71
#define CC1125_RSSI0                    0x2F72
#define CC1125_MARCSTATE                0x2F73
#define CC1125_LQI_VAL                  0x2F74
#define CC1125_PQT_SYNC_ERR             0x2F75
#define CC1125_DEM_STATUS               0x2F76
#define CC1125_FREQOFF_EST1             0x2F77
#define CC1125_FREQOFF_EST0             0x2F78
#define CC1125_AGC_GAIN3                0x2F79
#define CC1125_AGC_GAIN2                0x2F7A
#define CC1125_AGC_GAIN1                0x2F7B
#define CC1125_AGC_GAIN0                0x2F7C
#define CC1125_CFM_RX_DATA_OUT          0x2F7D
#define CC1125_CFM_TX_DATA_IN           0x2F7E
#define CC1125_ASK_SOFT_RX_DATA         0x2F7F
#define CC1125_RNDGEN                   0x2F80
#define CC1125_MAGN2                    0x2F81
#define CC1125_MAGN1                    0x2F82
#define CC1125_MAGN0                    0x2F83
#define CC1125_ANG1                     0x2F84
#define CC1125_ANG0                     0x2F85
#define CC1125_CHFILT_I2                0x2F86
#define CC1125_CHFILT_I1                0x2F87
#define CC1125_CHFILT_I0                0x2F88
#define CC1125_CHFILT_Q2                0x2F89
#define CC1125_CHFILT_Q1                0x2F8A
#define CC1125_CHFILT_Q0                0x2F8B
#define CC1125_GPIO_STATUS              0x2F8C
#define CC1125_FSCAL_CTRL               0x2F8D
#define CC1125_PHASE_ADJUST             0x2F8E
#define CC1125_PARTNUMBER               0x2F8F
#define CC1125_PARTVERSION              0x2F90
#define CC1125_SERIAL_STATUS            0x2F91
#define CC1125_MODEM_STATUS1            0x2F92
#define CC1125_MODEM_STATUS0            0x2F93
#define CC1125_MARC_STATUS1             0x2F94
#define CC1125_MARC_STATUS0             0x2F95
#define CC1125_PA_IFAMP_TEST            0x2F96
#define CC1125_FSRF_TEST                0x2F97
#define CC1125_PRE_TEST                 0x2F98
#define CC1125_PRE_OVR                  0x2F99
#define CC1125_ADC_TEST                 0x2F9A
#define CC1125_DVC_TEST                 0x2F9B
#define CC1125_ATEST                    0x2F9C
#define CC1125_ATEST_LVDS               0x2F9D
#define CC1125_ATEST_MODE               0x2F9E
#define CC1125_XOSC_TEST1               0x2F9F
#define CC1125_XOSC_TEST0               0x2FA0  
                                        
#define CC1125_RXFIRST                  0x2FD2   
#define CC1125_TXFIRST                  0x2FD3   
#define CC1125_RXLAST                   0x2FD4 
#define CC1125_TXLAST                   0x2FD5 
#define CC1125_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */ 
#define CC1125_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC1125_FIFO_NUM_TXBYTES         0x2FD8  
#define CC1125_FIFO_NUM_RXBYTES         0x2FD9 

#define CC1125_TELEMTRY_GS              0x54454C   // TEL
#define CC1125_ACKNOWLEDGE              0x41434B   // ACK
#define CC1125_QUIT                     0x515554   // QUT
#define TIMEOUT_MS                      100

typedef struct
{
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float magnetic_x;
    float magnetic_y;
    float magnetic_z;
    float altitude;
    double pressure;
    float temp_bmp;
} DataPoints_t;


// Address Config = No address check 
// Bit Rate = 1.2 
// Carrier Frequency = 910.000000 (0x5B0000) / 915 (0x5B8000)
// Deviation = 3.995895 
// Device Address = 0 
// Manchester Enable = false 
// Modulation Format = 2-GFSK 
// PA Ramping = true 
// Packet Bit Length = 0 
// Packet Length = 255 
// Packet Length Mode = Variable 
// Performance Mode = High Performance 
// RX Filter BW = 15.625000 
// Symbol rate = 1.2 
// TX Power = 15 
// Whitening = false 
typedef struct
{
    uint16_t addr;
    uint8_t data;
} registerSetting_t;

static const registerSetting_t preferredSettings[]= 
{
  {CC1125_IOCFG3,            0xB0},   //GPIO3 IO Pin Configuration
  {CC1125_IOCFG2,            0x06},   //GPIO2 IO Pin Configuration
  {CC1125_IOCFG1,            0xB0},   //GPIO1 IO Pin Configuration
  {CC1125_IOCFG0,            0x40},   //GPIO0 IO Pin Configuration
  {CC1125_SYNC_CFG1,         0x0B},   //Sync Word Detection Configuration
  {CC1125_DEVIATION_M,       0xA3},   //Frequency Deviation Configuration
  {CC1125_MODCFG_DEV_E,      0x0A},   //Modulation Format and Frequency Deviation Configuration
  {CC1125_DCFILT_CFG,        0x1C},   //Digital DC Removal Configuration
  {CC1125_FREQ_IF_CFG,       0x33},   //RX Mixer Frequency Configuration
  {CC1125_IQIC,              0xC6},   //Digital Image Channel Compensation Configuration
  {CC1125_CHAN_BW,           0x10},   //Channel Filter Configuration
  {CC1125_MDMCFG0,           0x05},   //General Modem Parameter Configuration
  {CC1125_SYMBOL_RATE2,      0x3F},   //Symbol Rate Configuration Exponent and Mantissa
  {CC1125_SYMBOL_RATE1,      0x75},   //Symbol Rate Configuration Mantissa [15:8]
  {CC1125_SYMBOL_RATE0,      0x10},   //Symbol Rate Configuration Mantissa [7:0]
  {CC1125_AGC_REF,           0x20},   //AGC Reference Level Configuration
  {CC1125_AGC_CS_THR,        0x19},   //Carrier Sense Threshold Configuration
  {CC1125_AGC_CFG1,          0xA9},   //Automatic Gain Control Configuration Reg. 1
  {CC1125_AGC_CFG0,          0xCF},   //Automatic Gain Control Configuration Reg. 0
  {CC1125_FIFO_CFG,          0x00},   //FIFO Configuration
  {CC1125_SETTLING_CFG,      0x03},   //Frequency Synthesizer Calibration and Settling Con.
  {CC1125_FS_CFG,            0x12},   //Frequency Synthesizer Configuration
  {CC1125_PKT_CFG0,          0x20},   //Packet Configuration Reg. 0
  {CC1125_RFEND_CFG0,        0x00},   //0x20 to go back to TX
  {CC1125_PKT_LEN,           0xFF},   //Packet Length Configuration
  {CC1125_IF_MIX_CFG,        0x00},   //IF Mix Configuration
  {CC1125_FREQOFF_CFG,       0x22},   //Frequency Offset Correction Configuration
  {CC1125_FREQ2,             0x5B},   //Frequency Configuration [23:16]
  {CC1125_FREQ1,             0x80},   //Frequency Configuration [15:8]
  {CC1125_FREQ0,             0x00},   //Frequency Configuration [7:0]
  {CC1125_IF_ADC0,           0x05},   //Analog to Digital Converter Configuration Reg. 0
  {CC1125_FS_DIG1,           0x00},   //Frequency Synthesizer Digital Reg. 1
  {CC1125_FS_DIG0,           0x5F},   //Frequency Synthesizer Digital Reg. 0
  {CC1125_FS_CAL0,           0x0E},   //Frequency Synthesizer Calibration Reg. 0
  {CC1125_FS_DIVTWO,         0x03},   //Frequency Synthesizer Divide by 2
  {CC1125_FS_DSM0,           0x33},   //FS Digital Synthesizer Module Configuration Reg. 0
  {CC1125_FS_DVC0,           0x17},   //Frequency Synthesizer Divider Chain Configuration
  {CC1125_FS_PFD,            0x50},   //Frequency Synthesizer Phase Frequency Detector Configuration
  {CC1125_FS_PRE,            0x6E},   //Frequency Synthesizer Prescaler Configuration
  {CC1125_FS_REG_DIV_CML,    0x14},   //Frequency Synthesizer Divider Regulator Configuration
  {CC1125_FS_SPARE,          0xAC},   //Frequency Synthesizer Spare
  {CC1125_XOSC5,             0x0E},   //Crystal Oscillator Configuration Reg. 5
  {CC1125_XOSC3,             0xC7},   //Crystal Oscillator Configuration Reg. 3
  {CC1125_XOSC1,             0x07},   //Crystal Oscillator Configuration Reg. 1
};

enum CC1125Status {
    CC1125_SUCCESS,
    CC1125_FAILURE,
    CC1125_INVALID
};

class CC1125 {

public:
    CC1125(uint8_t resetPin, 
           uint8_t cs, 
           Adafruit_LSM6DSOX *SOX, 
           Adafruit_LIS3MDL *MAG, 
           Adafruit_BMP3XX *BMP);  
    ~CC1125(); // Destructor

    CC1125Status init();
    void runTX(uint8_t* data, size_t len);
    void runRX(uint8_t *rxBuffer);

    void telemetryGroundStation();
    void telemetryRocket();


    
private:
    Adafruit_LSM6DSOX *sox; 
    Adafruit_LIS3MDL *mag;
    Adafruit_BMP3XX *bmp;
    SPIClass *_spi = &SPI;
    uint8_t packetCounter = 0;
    uint32_t _resetPin;
    uint32_t _cs;
    
    CC1125Status registerConfig(void);
    void createPacket(uint8_t *txBuffer, uint8_t *data, size_t len);
    void retriveData(DataPoints_t *data);
    void cc1125spi_TX_FIFO(uint8_t *data, size_t length);
    void cc1125spi_RX_FIFO(uint8_t *data, size_t length);
    void cc1125spi_write(uint16_t addr, uint8_t *data, size_t length, bool TX = true);
    void cc1125spi_read(uint16_t addr, uint8_t *data, size_t length, bool TX = true);
};

#endif // FLASH_DRIVER_H