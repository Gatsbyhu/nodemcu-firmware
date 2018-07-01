#include "SX1278.h"

uint8_t SX1278Regs[0x70];
tSx1278LR *SX1278LR;

uint8_t RFRxBuffer[RF_BUFFER_SIZE];
uint8_t RFTxBuffer[RF_BUFFER_SIZE];

tLoRaSettings LoRaSettings = {
    LoRa_FREQENCY,      // RFFrequency
    LoRa_POWER,         // Power
    LoRa_BW,            // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz,
    // 4: 31.2 kHz, 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz,
    // 9: 500 kHz, other: Reserved]
    LoRa_SF,            // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11:
    // 2048, 12: 4096  chips]
    LoRa_EC,            // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    LoRa_CRCON,         // CrcOn [0: OFF, 1: ON]
    LoRa_IMPLICTHEADON, // ImplicitHeaderOn [0: OFF, 1: ON]
    LoRa_PREAMBLELEN,   // Preamble Length
    RF_BUFFER_SIZE,     // RF Buffer Size
    LoRa_SYNCWORD       // SyncWord
};

void sx1278_WriteByte(uint8_t addr, uint8_t data) {
  platform_gpio_write(8, 0);
  platform_spi_send( 1, 8, (uint8_t)(addr | 0x80) );
  platform_spi_send( 1, 8, data);
  platform_gpio_write(8, 1);
}

void sx1278_WriteNbytes(uint8_t addr, uint8_t *buffer, uint8_t size) {
  platform_gpio_write(8, 0);
  platform_spi_send( 1, 8, (uint8_t)(addr | 0x80) );
  for (int i = 0; i<size; i++) {
    platform_spi_send( 1, 8, buffer[i]);
  }
  platform_gpio_write(8, 1);
}

void sx1278_ReadByte(uint8_t addr, uint8_t *buffer) {
  platform_gpio_write(8, 0);
  platform_spi_send( 1, 8, (uint8_t)(addr & 0x7f) );
  buffer[0] = (uint8_t) platform_spi_send_recv( 1, 8, 0xFFFFFFFF );
  platform_gpio_write(8, 1);
}

void sx1278_ReadNbytes(uint8_t addr, uint8_t *buffer, uint8_t size) {
  platform_gpio_write(8, 0);
  platform_spi_send( 1, 8, (uint8_t)(addr & 0x7f) );
  for (int i = 0; i<size; i++) {
    buffer[i] = (uint8_t) platform_spi_send_recv( 1, 8, 0xFFFFFFFF );
  }
  platform_gpio_write(8, 1);
}


void SX1278LoraInit(uint32_t freq, uint8_t sf, uint8_t bw) {
  platform_gpio_mode( 8, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT );

  SX1278LR = (tSx1278LR *) SX1278Regs;
  sx1278_ReadNbytes(REG_LR_OPMODE, &SX1278Regs[1], (0x70 - 1)); // read all register value out

  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_SLEEP);
  SX1278LR->RegOpMode = (uint8_t) ((SX1278LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);
  sx1278_WriteByte(REG_LR_OPMODE, SX1278LR->RegOpMode);

  SX1278LoRa_PRIVATE_SetRFFrequency(freq);         // set frequency
  SX1278LoRa_PRIVATE_SetSpreadingFactor(sf); // set spreading factor
  SX1278LoRa_PRIVATE_SetErrorCoding(LoRaSettings.ErrorCoding);         // set errorcoding
  SX1278LoRa_PRIVATE_SetPacketCrcOn(LoRaSettings.CrcOn);               // set crc on
  SX1278LoRa_PRIVATE_SetSignalBandwidth(bw);        // set bw

  SX1278LoRa_PRIVATE_SetImplicitHeaderOn(LoRaSettings.ImplicitHeaderOn); // set implicit header
  SX1278LoRa_PRIVATE_SetPreambleLength(LoRaSettings.PreambleLen);
  SX1278LoRa_PRIVATE_SetLowDatarateOptimize(false); // set datarate optimize

  SX1278LoRa_PRIVATE_SetPAOutput(RFLR_PACONFIG_PASELECT_PABOOST); // set PA out
  SX1278LoRa_PRIVATE_SetPa20dBm(true);                            // set PA 20db
  SX1278LoRa_PRIVATE_SetRFPower(LoRaSettings.Power);              // set rf power

  SX1278LoRa_PRIVATE_SetSyncWord(LoRaSettings.SyncWord);

  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_STANDBY);
}

void SX1278LoRaStartCAD(void) {

  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_STANDBY);
  os_delay_us(100);
  sx1278_WriteByte(REG_LR_IRQFLAGSMASK, RFLR_IRQ_CADDONE);
  sx1278_WriteByte(REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_10 | RFLR_DIOMAPPING1_DIO1_10);
  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_CAD);
}

void SX1278LoRaStartRx(void) {
  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_STANDBY);

  os_delay_us(100);
  sx1278_WriteByte(REG_LR_IRQFLAGSMASK, RFLR_IRQ_RXDONE);
  sx1278_WriteByte(REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00);
  sx1278_WriteByte(REG_LR_FIFOADDRPTR, 0x00);
  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_RECEIVER);
}

void SX1278LoRaGetRxPacket() {
  sx1278_ReadByte(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);
  if ((SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR) == RFLR_IRQFLAGS_PAYLOADCRCERROR) {
    sx1278_WriteByte(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

    return;
  }
  sx1278_ReadByte(REG_LR_FIFORXCURRENTADDR, &SX1278LR->RegFifoRxCurrentAddr);
  SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
  sx1278_WriteByte(REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr);
  sx1278_ReadByte(REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes);

  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_STANDBY);

  if (SX1278LR->RegNbRxBytes <= 8) {
    sx1278_ReadNbytes(0, RFRxBuffer, SX1278LR->RegNbRxBytes);
  }
}

void SX1278LoRaStartTx(void) {
  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_STANDBY);
  os_delay_us(100);
  sx1278_WriteByte(REG_LR_IRQFLAGSMASK, RFLR_IRQ_TXDONE);
  sx1278_WriteByte(REG_LR_PAYLOADLENGTH, LoRa_PAYLOADLEN);
  sx1278_WriteByte(REG_LR_FIFOTXBASEADDR, 0x00);
  sx1278_WriteByte(REG_LR_FIFOADDRPTR, 0x00);
  sx1278_WriteNbytes(0, RFTxBuffer, LoRa_PAYLOADLEN);
  sx1278_WriteByte(REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01);
  SX1278LoRa_PRIVATE_SetOPMode(RFLR_OPMODE_TRANSMITTER);
}

// PRIVATE DEFINITIONS

void SX1278LoRa_PRIVATE_SetOPMode(uint8_t opMode) {
  static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
  opModePrev = (uint8_t) (SX1278LR->RegOpMode & ~RFLR_OPMODE_MASK);
  if (opMode != opModePrev) {
    SX1278LR->RegOpMode = (uint8_t) ((SX1278LR->RegOpMode & RFLR_OPMODE_MASK) | opMode);
    sx1278_WriteByte(REG_LR_OPMODE, SX1278LR->RegOpMode);
  }
}

void SX1278LoRa_PRIVATE_SetRFFrequency(uint32_t freq) {
  LoRaSettings.RFFrequency = freq;
  freq = (uint32_t)((double) freq / (double) FREQ_STEP);
  SX1278LR->RegFrfMsb = (uint8_t)((freq >> 16) & 0xFF);
  SX1278LR->RegFrfMid = (uint8_t)((freq >> 8) & 0xFF);
  SX1278LR->RegFrfLsb = (uint8_t)(freq & 0xFF);
  sx1278_WriteNbytes(REG_LR_FRFMSB, &SX1278LR->RegFrfMsb, 3);
}

void SX1278LoRa_PRIVATE_SetRFPower(int8_t power) {
  sx1278_ReadByte(REG_LR_PACONFIG, &SX1278LR->RegPaConfig);
  sx1278_ReadByte(REG_LR_PADAC, &SX1278LR->RegPaDac);
  if ((SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST) {
    if ((SX1278LR->RegPaDac & 0x87) == 0x87) {
      if (power < 5)
        power = 5;
      if (power > 20)
        power = 20;

      SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70);
      SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) |
                              (uint8_t)((uint16_t)(power - 5) & 0x0F));
    } else {
      if (power < 2)
        power = 2;
      if (power > 17)
        power = 17;
      SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70);
      SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) |
                              (uint8_t)((uint16_t)(power - 2) & 0x0F));
    }
  } else {
    if (power < -1)
      power = -1;
    if (power > 14)
      power = 14;
    SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70);
    SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) |
                            (uint8_t)((uint16_t)(power + 1) & 0x0F));
  }
  sx1278_WriteByte(REG_LR_PACONFIG, SX1278LR->RegPaConfig);
  LoRaSettings.Power = power;
}

void SX1278LoRa_PRIVATE_SetSignalBandwidth(uint8_t bw) {
  sx1278_ReadByte(REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1);
  SX1278LR->RegModemConfig1 = (uint8_t) ((SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4));
  sx1278_WriteByte(REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1);
  LoRaSettings.SignalBw = bw;
}

void SX1278LoRa_PRIVATE_SetNbTrigPeaks(uint8_t value) {
  sx1278_ReadByte(0x31, &SX1278LR->RegTestReserved31);
  SX1278LR->RegTestReserved31 = (uint8_t) ((SX1278LR->RegTestReserved31 & 0xF8) | value);
  sx1278_WriteByte(0x31, SX1278LR->RegTestReserved31);
}

void SX1278LoRa_PRIVATE_SetSpreadingFactor(uint8_t factor) {
  if (factor > 12) {
    factor = 12;
  } else if (factor < 6) {
    factor = 6;
  }

  if (factor == 6) {
    SX1278LoRa_PRIVATE_SetNbTrigPeaks(5);
  } else {
    SX1278LoRa_PRIVATE_SetNbTrigPeaks(3);
  }
  sx1278_ReadByte(REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2);
  SX1278LR->RegModemConfig2 = (uint8_t) ((SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4));
  sx1278_WriteByte(REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2);
  LoRaSettings.SpreadingFactor = factor;
}

void SX1278LoRa_PRIVATE_SetErrorCoding(uint8_t value) {
  sx1278_ReadByte(REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1);
  SX1278LR->RegModemConfig1 =
      (uint8_t) ((SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (value << 1));
  sx1278_WriteByte(REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1);
  LoRaSettings.ErrorCoding = value;
}

void SX1278LoRa_PRIVATE_SetPacketCrcOn(bool enable) {
  sx1278_ReadByte(REG_LR_MODEMCONFIG2, &SX1278LR->RegModemConfig2);
  SX1278LR->RegModemConfig2 =
      (uint8_t) ((SX1278LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (enable << 2));
  sx1278_WriteByte(REG_LR_MODEMCONFIG2, SX1278LR->RegModemConfig2);
  LoRaSettings.CrcOn = enable;
}

void SX1278LoRa_PRIVATE_SetPreambleLength(uint16_t value) {
  sx1278_ReadNbytes(REG_LR_PREAMBLEMSB, &SX1278LR->RegPreambleMsb, 2);
  SX1278LR->RegPreambleMsb = (uint8_t) ((value >> 8) & 0x00FF);
  SX1278LR->RegPreambleLsb = (uint8_t) (value & 0xFF);
  sx1278_WriteNbytes(REG_LR_PREAMBLEMSB, &SX1278LR->RegPreambleMsb, 2);
}

void SX1278LoRa_PRIVATE_SetImplicitHeaderOn(bool enable) {
  sx1278_ReadByte(REG_LR_MODEMCONFIG1, &SX1278LR->RegModemConfig1);
  SX1278LR->RegModemConfig1 =
      (uint8_t) ((SX1278LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (enable));
  sx1278_WriteByte(REG_LR_MODEMCONFIG1, SX1278LR->RegModemConfig1);
  LoRaSettings.ImplicitHeaderOn = enable;
}

void SX1278LoRa_PRIVATE_SetPayloadLength(uint8_t value) {
  SX1278LR->RegPayloadLength = value;
  sx1278_WriteByte(REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength);
  LoRaSettings.PayloadLength = value;
}

void SX1278LoRa_PRIVATE_SetPa20dBm(bool enale) {
  sx1278_ReadByte(REG_LR_PADAC, &SX1278LR->RegPaDac);
  sx1278_ReadByte(REG_LR_PACONFIG, &SX1278LR->RegPaConfig);

  if ((SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST) {
    if (enale == true)
      SX1278LR->RegPaDac = 0x87;
  } else {
    SX1278LR->RegPaDac = 0x84;
  }
  sx1278_WriteByte(REG_LR_PADAC, SX1278LR->RegPaDac);
}

void SX1278LoRa_PRIVATE_SetPAOutput(uint8_t outputPin) {
  sx1278_ReadByte(REG_LR_PACONFIG, &SX1278LR->RegPaConfig);
  SX1278LR->RegPaConfig = (uint8_t) ((SX1278LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK) | outputPin);
  sx1278_WriteByte(REG_LR_PACONFIG, SX1278LR->RegPaConfig);
}

void SX1278LoRa_PRIVATE_SetLowDatarateOptimize(bool enable) {
  sx1278_ReadByte(REG_LR_MODEMCONFIG3, &SX1278LR->RegModemConfig3);
  SX1278LR->RegModemConfig3 =
      (uint8_t) ((SX1278LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (enable << 3));
  sx1278_WriteByte(REG_LR_MODEMCONFIG3, SX1278LR->RegModemConfig3);
}

void SX1278LoRa_PRIVATE_SetSyncWord(uint8_t value) { sx1278_WriteByte(0x39, value); }

// CUSTOM BABY

void SX1278LoRaSetTxMessage(const uint8_t *inputBuffer) {
  uint8_t i;
  for (i = 0; i < RF_BUFFER_SIZE; i++) {
    RFTxBuffer[i] = inputBuffer[i];
  }
}

void SX1278LoRaReadRxBuffer(uint8_t *outputBuffer) {
  uint8_t i;
  for (i = 0; i < RF_BUFFER_SIZE; i++) {
    outputBuffer[i] = RFRxBuffer[i];
  }
}

uint8_t SX1278LoRaGetFlags() {
  sx1278_ReadByte(REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags);
  return SX1278LR->RegIrqFlags;
}

void SX1278LoRaClearFlags() {
  sx1278_ReadByte(REG_LR_OPMODE, &SX1278LR->RegOpMode);
  sx1278_WriteByte(REG_LR_OPMODE, RFLR_OPMODE_STANDBY);
  sx1278_WriteByte(REG_LR_IRQFLAGS, 0xFF);
  sx1278_WriteByte(REG_LR_OPMODE, SX1278LR->RegOpMode);
}
