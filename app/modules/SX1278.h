#ifndef __SUBLOGIC_SX1278LR__
#define __SUBLOGIC_SX1278LR__

#include "platform.h"
#include "c_stdlib.h"

#define FREQ_STEP 61.03515625
#define RF_BUFFER_SIZE 8
#define LoRa_FREQENCY 915000000
#define LoRa_POWER 20
#define LoRa_BW 6
#define LoRa_SF 8
#define LoRa_EC 1
#define LoRa_CRCON true
#define LoRa_IMPLICTHEADON false
#define LoRa_PREAMBLELEN 9
#define LoRa_PAYLOADLEN 8
#define LoRa_SYNCWORD 0x34

typedef enum {
    RF_IDLE,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_LEN_ERROR,
    RF_CHANNEL_EMPTY,
    RF_CHANNEL_ACTIVITY_DETECTED,
} tRFProcessReturnCodes;

typedef struct sLoRaSettings {
    uint32_t RFFrequency;
    int8_t Power;
    uint8_t SignalBw;        // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8
    // kHz, 4: 31.2 kHz, 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz,
    // 8: 250 kHz, 9: 500 kHz, other: Reserved]
    uint8_t SpreadingFactor; // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024,
    // 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;     // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;              // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;   // [0: OFF, 1: ON]
    uint8_t PreambleLen;     // [Preamble Length]
    uint8_t PayloadLength;
    uint8_t SyncWord; // Sync Word
} tLoRaSettings;

typedef struct sSX1278 {
    uint8_t RegFifo; // 0x00
    // Common settings
    uint8_t RegOpMode; // 0x01
    uint8_t RegRes02;  // 0x02
    uint8_t RegRes03;  // 0x03
    uint8_t RegRes04;  // 0x04
    uint8_t RegRes05;  // 0x05
    uint8_t RegFrfMsb; // 0x06
    uint8_t RegFrfMid; // 0x07
    uint8_t RegFrfLsb; // 0x08
    // Tx settings
    uint8_t RegPaConfig; // 0x09
    uint8_t RegPaRamp;   // 0x0A
    uint8_t RegOcp;      // 0x0B
    // Rx settings
    uint8_t RegLna; // 0x0C
    // LoRa registers
    uint8_t RegFifoAddrPtr;                 // 0x0D
    uint8_t RegFifoTxBaseAddr;              // 0x0E
    uint8_t RegFifoRxBaseAddr;              // 0x0F
    uint8_t RegFifoRxCurrentAddr;           // 0x10
    uint8_t RegIrqFlagsMask;                // 0x11
    uint8_t RegIrqFlags;                    // 0x12
    uint8_t RegNbRxBytes;                   // 0x13
    uint8_t RegRxHeaderCntValueMsb;         // 0x14
    uint8_t RegRxHeaderCntValueLsb;         // 0x15
    uint8_t RegRxPacketCntValueMsb;         // 0x16
    uint8_t RegRxPacketCntValueLsb;         // 0x17
    uint8_t RegModemStat;                   // 0x18
    uint8_t RegPktSnrValue;                 // 0x19
    uint8_t RegPktRssiValue;                // 0x1A
    uint8_t RegRssiValue;                   // 0x1B
    uint8_t RegHopChannel;                  // 0x1C
    uint8_t RegModemConfig1;                // 0x1D
    uint8_t RegModemConfig2;                // 0x1E
    uint8_t RegSymbTimeoutLsb;              // 0x1F
    uint8_t RegPreambleMsb;                 // 0x20
    uint8_t RegPreambleLsb;                 // 0x21
    uint8_t RegPayloadLength;               // 0x22
    uint8_t RegMaxPayloadLength;            // 0x23
    uint8_t RegHopPeriod;                   // 0x24
    uint8_t RegFifoRxByteAddr;              // 0x25
    uint8_t RegModemConfig3;                // 0x26
    uint8_t RegTestReserved27[0x30 - 0x27]; // 0x27-0x30
    uint8_t RegTestReserved31;              // 0x31
    uint8_t RegTestReserved32[0x40 - 0x32]; // 0x32-0x40
    // I/O settings
    uint8_t RegDioMapping1; // 0x40
    uint8_t RegDioMapping2; // 0x41
    // Version
    uint8_t RegVersion; // 0x42
    // Additional settings
    uint8_t RegTestReserved43[0x4D - 0x43]; // 0x43-0x4D
    uint8_t RegPaDac;                       // 0x4D
    uint8_t RegTestReserved4E[0x5B - 0x4E]; // 0x4E-0x5B
    uint8_t RegFormerTemp;                  // 0x5B
    uint8_t RegTestReserved5C[0x61 - 0x5C]; // 0x5C-0x61

    uint8_t RegAgcRef;                      // 0x61
    uint8_t RegAgcThresh1;                  // 0x62
    uint8_t RegAgcThresh2;                  // 0x63
    uint8_t RegAgcThresh3;                  // 0x64
    uint8_t RegTestReserved65[0x70 - 0x65]; // 0x65-0x6F
    uint8_t RegPll;                         // 0x70
} tSx1278LR;

#define REG_LR_FIFO 0x00
// Common settings
#define REG_LR_OPMODE 0x01
#define REG_LR_BANDSETTING 0x04
#define REG_LR_FRFMSB 0x06
#define REG_LR_FRFMID 0x07
#define REG_LR_FRFLSB 0x08
// Tx settings
#define REG_LR_PACONFIG 0x09
#define REG_LR_PARAMP 0x0A
#define REG_LR_OCP 0x0B
// Rx settings
#define REG_LR_LNA 0x0C
// LoRa registers
#define REG_LR_FIFOADDRPTR 0x0D
#define REG_LR_FIFOTXBASEADDR 0x0E
#define REG_LR_FIFORXBASEADDR 0x0F
#define REG_LR_FIFORXCURRENTADDR 0x10
#define REG_LR_IRQFLAGSMASK 0x11
#define REG_LR_IRQFLAGS 0x12
#define REG_LR_NBRXBYTES 0x13
#define REG_LR_RXHEADERCNTVALUEMSB 0x14
#define REG_LR_RXHEADERCNTVALUELSB 0x15
#define REG_LR_RXPACKETCNTVALUEMSB 0x16
#define REG_LR_RXPACKETCNTVALUELSB 0x17
#define REG_LR_MODEMSTAT 0x18
#define REG_LR_PKTSNRVALUE 0x19
#define REG_LR_PKTRSSIVALUE 0x1A
#define REG_LR_RSSIVALUE 0x1B
#define REG_LR_HOPCHANNEL 0x1C
#define REG_LR_MODEMCONFIG1 0x1D
#define REG_LR_MODEMCONFIG2 0x1E
#define REG_LR_SYMBTIMEOUTLSB 0x1F
#define REG_LR_PREAMBLEMSB 0x20
#define REG_LR_PREAMBLELSB 0x21
#define REG_LR_PAYLOADLENGTH 0x22
#define REG_LR_PAYLOADMAXLENGTH 0x23
#define REG_LR_HOPPERIOD 0x24
#define REG_LR_FIFORXBYTEADDR 0x25
#define REG_LR_MODEMCONFIG3 0x26
// end of documented register in datasheet
// I/O settings
#define REG_LR_DIOMAPPING1 0x40
#define REG_LR_DIOMAPPING2 0x41
// Version
#define REG_LR_VERSION 0x42
// Additional settings
#define REG_LR_PLLHOP 0x44
#define REG_LR_TCXO 0x4B
#define REG_LR_PADAC 0x4D
#define REG_LR_FORMERTEMP 0x5B
#define REG_LR_BITRATEFRAC 0x5D
#define REG_LR_AGCREF 0x61
#define REG_LR_AGCTHRESH1 0x62
#define REG_LR_AGCTHRESH2 0x63
#define REG_LR_AGCTHRESH3 0x64

/*!
 * RegLna
 */
#define RFLR_LNA_GAIN_MASK 0x1F
#define RFLR_LNA_GAIN_G1 0x20 // Default
#define RFLR_LNA_GAIN_G2 0x40
#define RFLR_LNA_GAIN_G3 0x60
#define RFLR_LNA_GAIN_G4 0x80
#define RFLR_LNA_GAIN_G5 0xA0
#define RFLR_LNA_GAIN_G6 0xC0
/*!
 * RegPaConfig
 */
#define RFLR_PACONFIG_PASELECT_MASK 0x7F
#define RFLR_PACONFIG_PASELECT_PABOOST 0x80
#define RFLR_PACONFIG_PASELECT_RFO 0x00 // Default
#define RFLR_PACONFIG_MAX_POWER_MASK 0x8F
#define RFLR_PACONFIG_OUTPUTPOWER_MASK 0xF0
/*!
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK 0x0F

#define RFLR_MODEMCONFIG1_BW_7_81_KHZ 0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ 0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ 0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ 0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ 0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ 0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ 0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ 0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ 0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ 0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK 0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5 0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6 0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7 0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8 0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK 0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON 0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF 0x00 // Default

/*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK 0x0F
#define RFLR_MODEMCONFIG2_SF_6 0x60
#define RFLR_MODEMCONFIG2_SF_7 0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8 0x80
#define RFLR_MODEMCONFIG2_SF_9 0x90
#define RFLR_MODEMCONFIG2_SF_10 0xA0
#define RFLR_MODEMCONFIG2_SF_11 0xB0
#define RFLR_MODEMCONFIG2_SF_12 0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK 0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON 0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF 0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK 0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON 0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF 0x00 // Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK 0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB 0x00 // Default
/*!
 * RegPaRamp
 */
#define RFLR_PARAMP_TXBANDFORCE_MASK 0xEF
#define RFLR_PARAMP_TXBANDFORCE_BAND_SEL 0x10
#define RFLR_PARAMP_TXBANDFORCE_AUTO 0x00 // Default

#define RFLR_PARAMP_MASK 0xF0
#define RFLR_PARAMP_3400_US 0x00
#define RFLR_PARAMP_2000_US 0x01
#define RFLR_PARAMP_1000_US 0x02
#define RFLR_PARAMP_0500_US 0x03
#define RFLR_PARAMP_0250_US 0x04
#define RFLR_PARAMP_0125_US 0x05
#define RFLR_PARAMP_0100_US 0x06
#define RFLR_PARAMP_0062_US 0x07
#define RFLR_PARAMP_0050_US 0x08
#define RFLR_PARAMP_0040_US 0x09 // Default
#define RFLR_PARAMP_0031_US 0x0A
#define RFLR_PARAMP_0025_US 0x0B
#define RFLR_PARAMP_0020_US 0x0C
#define RFLR_PARAMP_0015_US 0x0D
#define RFLR_PARAMP_0012_US 0x0E
#define RFLR_PARAMP_0010_US 0x0F
/*!
 * RegModemConfig3
 */
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK 0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON 0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF 0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK 0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON 0x04 // Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF 0x00
/*!
 * RegOpMode
 */
#define RFLR_OPMODE_LONGRANGEMODE_MASK 0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF 0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON 0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK 0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE 0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE 0x00 // Default

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK 0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF 0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF 0x00

#define RFLR_OPMODE_MASK 0xF8
#define RFLR_OPMODE_SLEEP 0x00
#define RFLR_OPMODE_STANDBY 0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX 0x02
#define RFLR_OPMODE_TRANSMITTER 0x03
#define RFLR_OPMODE_SYNTHESIZER_RX 0x04
#define RFLR_OPMODE_RECEIVER 0x05
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE 0x06
#define RFLR_OPMODE_CAD 0x07

/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK 0x80
#define RFLR_IRQFLAGS_RXDONE_MASK 0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK 0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK 0x10
#define RFLR_IRQFLAGS_TXDONE_MASK 0x08
#define RFLR_IRQFLAGS_CADDONE_MASK 0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK 0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK 0x01

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT 0x80
#define RFLR_IRQFLAGS_RXDONE 0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR 0x20
#define RFLR_IRQFLAGS_VALIDHEADER 0x10
#define RFLR_IRQFLAGS_TXDONE 0x08
#define RFLR_IRQFLAGS_CADDONE 0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL 0x02
#define RFLR_IRQFLAGS_CADDETECTED 0x01

#define RFLR_IRQ_TXDONE 0XF7
#define RFLR_IRQ_RXDONE 0X9F
#define RFLR_IRQ_CADDONE 0XFA

/*!
 * RegHopChannel (Read Only)
 */
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK 0x7F
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL 0x80
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED 0x00 // Default

#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_MASK 0xBF
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_ON 0x40
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_OFF 0x00 // Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK 0x3F
/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK 0x3F
#define RFLR_DIOMAPPING1_DIO0_00 0x00 // Default
#define RFLR_DIOMAPPING1_DIO0_01 0x40
#define RFLR_DIOMAPPING1_DIO0_10 0x80
#define RFLR_DIOMAPPING1_DIO0_11 0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK 0xCF
#define RFLR_DIOMAPPING1_DIO1_00 0x00 // Default
#define RFLR_DIOMAPPING1_DIO1_01 0x10
#define RFLR_DIOMAPPING1_DIO1_10 0x20
#define RFLR_DIOMAPPING1_DIO1_11 0x30

#define RFLR_DIOMAPPING1_DIO2_MASK 0xF3
#define RFLR_DIOMAPPING1_DIO2_00 0x00 // Default
#define RFLR_DIOMAPPING1_DIO2_01 0x04
#define RFLR_DIOMAPPING1_DIO2_10 0x08
#define RFLR_DIOMAPPING1_DIO2_11 0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK 0xFC
#define RFLR_DIOMAPPING1_DIO3_00 0x00 // Default
#define RFLR_DIOMAPPING1_DIO3_01 0x01
#define RFLR_DIOMAPPING1_DIO3_10 0x02
#define RFLR_DIOMAPPING1_DIO3_11 0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK 0x3F
#define RFLR_DIOMAPPING2_DIO4_00 0x00 // Default
#define RFLR_DIOMAPPING2_DIO4_01 0x40
#define RFLR_DIOMAPPING2_DIO4_10 0x80
#define RFLR_DIOMAPPING2_DIO4_11 0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK 0xCF
#define RFLR_DIOMAPPING2_DIO5_00 0x00 // Default
#define RFLR_DIOMAPPING2_DIO5_01 0x10
#define RFLR_DIOMAPPING2_DIO5_10 0x20
#define RFLR_DIOMAPPING2_DIO5_11 0x30

#define RFLR_DIOMAPPING2_MAP_MASK 0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT 0x01
#define RFLR_DIOMAPPING2_MAP_RSSI 0x00 // Default
/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT 0x80
#define RFLR_IRQFLAGS_RXDONE 0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR 0x20
#define RFLR_IRQFLAGS_VALIDHEADER 0x10
#define RFLR_IRQFLAGS_TXDONE 0x08
#define RFLR_IRQFLAGS_CADDONE 0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL 0x02
#define RFLR_IRQFLAGS_CADDETECTED 0x01

extern tLoRaSettings LoRaSettings;
extern uint8_t RFRxBuffer[RF_BUFFER_SIZE];
extern uint8_t RFTxBuffer[RF_BUFFER_SIZE];
extern tLoRaSettings LoRaSettings;

void SX1278LoraInit(uint32_t freq, uint8_t sf, uint8_t bw);
void SX1278LoRaGetRxPacket(void);
void SX1278LoRaStartTx(void);
void SX1278LoRaStartRx(void);
void SX1278LoRaStartCAD(void);

void SX1278LoRaReadRxBuffer(uint8_t *outputBuffer);

void SX1278LoRaSetTxMessage(const uint8_t *inputBuffer);

void SX1278LoRa_PRIVATE_SetOPMode(uint8_t opMode);
void SX1278LoRa_PRIVATE_SetRFFrequency(uint32_t freq);
void SX1278LoRa_PRIVATE_SetRFPower(int8_t power);
void SX1278LoRa_PRIVATE_SetSignalBandwidth(uint8_t bw);
void SX1278LoRa_PRIVATE_SetSpreadingFactor(uint8_t factor);
void SX1278LoRa_PRIVATE_SetErrorCoding(uint8_t value);
void SX1278LoRa_PRIVATE_SetPacketCrcOn(bool enable);
void SX1278LoRa_PRIVATE_SetImplicitHeaderOn(bool enable);
void SX1278LoRa_PRIVATE_SetPa20dBm(bool enale);
void SX1278LoRa_PRIVATE_SetPAOutput(uint8_t outputPin);
void SX1278LoRa_PRIVATE_SetLowDatarateOptimize(bool enable);
void SX1278LoRa_PRIVATE_SetPreambleLength(uint16_t value);
void SX1278LoRa_PRIVATE_SetPreambleLength(uint16_t value);
void SX1278LoRa_PRIVATE_SetSyncWord(uint8_t value);

uint8_t SX1278LoRaGetFlags();
void SX1278LoRaClearFlags();

#endif