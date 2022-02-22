#include "si4432.h"
#include "esphome/core/log.h"

namespace esphome {
namespace si4432 {

static const char *const TAG = "si4432";

// Values here are kept in khz x 10 format (for not to deal with decimals) -
// look at AN440 page 26 for whole table
const uint16_t IFFilterTable[8][2] = {
  { 322, 0x26 },
  { 3355, 0x88 },
  { 3618, 0x89 },
  { 4202, 0x8A },
  { 4684, 0x8B },
  { 5188, 0x8C },
  { 5770, 0x8D },
  { 6207, 0x8E },
};

void Si4432Component::setup() {
  this->irq_pin_->setup();
  this->sdn_pin_->setup();
  this->sdn_pin_->digital_write(true);
  this->spi_setup();
  // TODO(maruel): Trying to make it not crash.
  //this->hardReset();

  this->dumpAllRegisters();
  if (this->_rx) {
    this->startListening();
  }
}

void Si4432Component::loop() {
  /* TODO(maruel): Add.
  if (this->_rx) {
    if (this->isPacketReceived()) {
      byte payLoad[64] = {0};
      byte len = 0;
      this->getPacketReceived(&len, payLoad);
      auto fmt = format_hex_pretty(payLoad, len);
      ESP_LOGD(TAG, "PACKET CAME %d [%d] %s", (int)len, fmt.c_str());
      this->publish_state(fmt.c_str());
    }
  }
  */
}

void Si4432Component::hardReset() {
  this->turnOff();
  this->turnOn();
  for (;;) {
    byte reg = this->readRegister(REG_INT_STATUS2);
    ESP_LOGD(TAG, "POR: %x", reg);
    if ((reg & 0x02) == 0x02) {
      break;
    }
    delay(1);
  }
  this->boot();
}

void Si4432Component::boot() {
  //byte currentFix[] = { 0x80, 0x40, 0x7F };
  //// refer to AN440 for reasons
  //this->writeRegisters(REG_CHARGEPUMP_OVERRIDE, currentFix, 3);
  //this->setRegister(REG_GPIO0_CONF, 0x0F); // tx/rx data clk pin
  //this->setRegister(REG_GPIO1_CONF, 0x00); // POR inverted pin
  //this->setRegister(REG_GPIO2_CONF, 0x1C); // clear channel pin

  // refer to AN440 for reasons
  this->setRegister(REG_AFC_TIMING_CONTROL, 0x02);
  // write max value - excel file did that.
  this->setRegister(REG_AFC_LIMITER, 0xFF);
  // max gain control
  this->setRegister(REG_AGC_OVERRIDE, 0x60);
  // turn off AFC
  this->setRegister(REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C);
  // enable rx packet handling, enable tx packet handling, enable CRC, use
  // CRC-IBM
  this->setRegister(REG_DATAACCESS_CONTROL, 0xAD);
  // no broadcast address control, enable check headers for bytes 3 & 2
  this->setRegister(REG_HEADER_CONTROL1, 0x0C);
  // enable headers byte 3 & 2, no fixed package length, sync word 3 & 2
  this->setRegister(REG_HEADER_CONTROL2, 0x22);
  // 8 * 4 bits = 32 bits (4 bytes) preamble length
  this->setRegister(REG_PREAMBLE_LENGTH, 0x08);
  // validate 7 * 4 bits of preamble  in a package
  this->setRegister(REG_PREAMBLE_DETECTION, 0x3A);
  // sync byte 3 val
  this->setRegister(REG_SYNC_WORD3, 0x2D);
  // sync byte 2 val
  this->setRegister(REG_SYNC_WORD2, 0xD4);
  // max power
  this->setRegister(REG_TX_POWER, 0x1F);
  // each channel is of 1 Mhz interval
  this->setRegister(REG_FREQ_CHANNEL_STEPSIZE, 0x64);

  // Set frequency.
  {
    byte highBand = (byte)(this->freqMHz_ >= 480);
    double fPart = (this->freqMHz_ / (10 * (highBand + 1))) - 24;
    uint8_t freqband = (uint8_t)fPart;
    uint16_t freqcarrier = (fPart - freqband) * 64000;
    // sideband is always on (0x40) :
    byte vals[3] = {
      (byte)(0x40 | (highBand << 5) | (freqband & 0x3F)),
      (byte)(freqcarrier >> 8),
      (byte)(freqcarrier & 0xFF),
    };
    this->writeRegisters(REG_FREQ_BAND, vals, 3);
  }

  // Set baud rate.
  {
    // 15khz / 150 khz
    byte freqDev = this->kbps_ <= 10 ? 15 : 150;
    // Use FIFO Mode, GFSK, low baud mode on / off.
    byte modulationValue = this->kbps_ < 30 ? 0x4c : 0x0c;
    // MSB of the kpbs to 3rd bit of register.
    byte modulationVals[] = {
      modulationValue,
      0x23,
      static_cast<byte>(round((freqDev * 1000.) / 625.)),
    };
    this->writeRegisters(REG_MODULATION_MODE1, modulationVals, 3);
    // Set data rate.
    uint16_t bpsRegVal = round((this->kbps_ * (this->kbps_ < 30 ? 2097152 : 65536.)) / 1000.);
    byte datarateVals[] = { (byte)(bpsRegVal >> 8), (byte)(bpsRegVal) };
    this->writeRegisters(REG_TX_DATARATE1, datarateVals, 2);

    //now set the timings
    uint16_t minBandwidth = (2 * (uint32_t) freqDev) + this->kbps_;
    ESP_LOGD(TAG, "min Bandwidth value: %x", minBandwidth);
    byte IFValue = 0xff;
    // Since the table is ordered (from low to high), just find the 'minimum
    // bandwidth which is greater than required'
    for (byte i = 0; i < 8; ++i) {
      if (IFFilterTable[i][0] >= (minBandwidth * 10)) {
        IFValue = IFFilterTable[i][1];
        break;
      }
    }
    ESP_LOGD(TAG, "Selected IF value: %x", IFValue);
    this->setRegister(REG_IF_FILTER_BW, IFValue);
    // if msb is set
    byte dwn3_bypass = (IFValue & 0x80) ? 1 : 0;
    // only 3 bits
    byte ndec_exp = (IFValue >> 4) & 0x07;
    uint16_t rxOversampling = round(
        (500. * (1 + 2*dwn3_bypass)) / ((pow(2, ndec_exp-3)) * (double)this->kbps_));
    uint32_t ncOffset = ceil(
        ((double)this->kbps_ * (pow(2, ndec_exp+20))) / (500. * (1 + 2*dwn3_bypass)));
    uint16_t crGain = 2 + (
        (65535 * (int64_t)this->kbps_) / ((int64_t)rxOversampling * freqDev));
    byte crMultiplier = 0x00;
    if (crGain > 0x7FF) {
      crGain = 0x7FF;
    }
    ESP_LOGD(TAG, "dwn3_bypass value: %x", dwn3_bypass);
    ESP_LOGD(TAG, "ndec_exp value: %x", ndec_exp);
    ESP_LOGD(TAG, "rxOversampling value: %x", rxOversampling);
    ESP_LOGD(TAG, "ncOffset value: %x", ncOffset);
    ESP_LOGD(TAG, "crGain value: %x", crGain);
    ESP_LOGD(TAG, "crMultiplier value: %x", crMultiplier);
    byte timingVals[] = {
      (byte)(rxOversampling),
      (byte)(((rxOversampling & 0x0700) >> 3) | ((ncOffset >> 16) & 0x0F)),
      (byte)(ncOffset >> 8),
      (byte)(ncOffset),
      (byte)(((crGain & 0x0700) >> 8) | crMultiplier),
      (byte)(crGain),
    };
    this->writeRegisters(REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);
  }

  // default channel is 0
  this->setRegister(REG_FREQ_CHANNEL, this->channel_);

  // Used to 'sign' packets with a predetermined signature.
  {
    // Header (signature) byte 3 val.
    this->setRegister(REG_TRANSMIT_HEADER3, this->packageSign_ >> 8);
    // Header (signature) byte 2 val.
    this->setRegister(REG_TRANSMIT_HEADER2, this->packageSign_ & 0xFF);
    // Header (signature) byte 3 val for receive checks.
    this->setRegister(REG_CHECK_HEADER3, this->packageSign_ >> 8);
    // Header (signature) byte 2 val for receive checks.
    this->setRegister(REG_CHECK_HEADER2, this->packageSign_ & 0xFF);
    ESP_LOGD(TAG, "Package signature is set!");
  }

  this->setMode(Ready);
}

void Si4432Component::startListening() {
  /* TODO(maruel): Add back.
  // Clear first, so it doesn't overflow if packet is big.
  this->clearRxFIFO();
  // Set interrupts on for package received and CRC error.
  this->setRegister(REG_INT_ENABLE1, 0x03);
  // TODO(maruel): Enable debug mode as a variable.
#ifdef DEBUG
  this->setRegister(REG_INT_ENABLE2, 0xC0);
#else
  // Set other interrupts off.
  this->setRegister(REG_INT_ENABLE2, 0x00);
#endif
  // Read interrupt registers to clean them.
  this->readRegister(REG_INT_STATUS1);
  this->readRegister(REG_INT_STATUS2);
  this->setMode(RXMode | Ready);
  */
}

void Si4432Component::setMode(Si4432Component::AntennaMode mode) {
  this->setRegister(REG_STATE, mode);
  //delay(20);
  byte val = this->readRegister(REG_DEV_STATUS);
  if (val == 0 || val == 0xFF) {
    ESP_LOGW(TAG, "Failed to switch mode: %x", val);
  }
}

void Si4432Component::writeRegisters(Si4432Component::Register reg, const byte value[], uint8_t length) {
  // SPIDevice method names are quite generic sounding.
  this->enable();
  // Set MSB for write.
  this->write_byte((byte)(reg | 0x80));
  // TODO(maruel): write_array() requires alignment.
  //this->write_array(value, length);
  for (int i = 0; i < length; i++) {
    this->write_byte(value[i]);
  }
  this->disable();
  //ESP_LOGV(TAG, "Writing: %x | %x", regVal!=0xFF ? (regVal+i) & 0x7F : 0x7F, value[i]);
  //ESP_LOGV(TAG, "Write %x: %s", startReg, format_hex_pretty(value, length).c_str());
}

void Si4432Component::readRegisters(Si4432Component::Register reg, byte value[], uint8_t length) {
  // SPIDevice method names are quite generic sounding.
  this->enable();
  this->write_byte((byte)(reg & 0x7F));
  // TODO(maruel): read_array() requires alignment.
  //this->read_array(value, length);
  for (int i = 0; i < length; i++) {
    value[i] = this->read_byte();
  }
  //ESP_LOGV(TAG, "Reading: %x | %x", regVal!=0x7F ? (regVal+i) & 0x7F : 0x7F, value[i]);
  //ESP_LOGV(TAG, "Read %x: %s", regVal, format_hex_pretty(value, length).c_str());
}

void Si4432Component::dumpAllRegisters() {
  byte allValues[0x7F];
  this->readRegisters(REG_DEV_TYPE, allValues, 0x7F);
  for (byte i = 0; i < 0x7f; ++i) {
    ESP_LOGV(TAG, "REG(%x) : %x", (int)REG_DEV_TYPE+i, (int)allValues[i]);
  }
}

}  // namespace si4432
}  // namespace esphome
