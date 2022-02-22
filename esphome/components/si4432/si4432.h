// https://www.silabs.com/documents/public/data-sheets/Si4430-31-32.pdf

#ifndef SI4432_H_
#define SI4432_H_
#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
//#include "esphome/core/automation.h"

namespace esphome {
namespace si4432 {

/*
class Si4432 {
public:
  Si4432(uint8_t sdnPin, uint8_t intPin, uint8_t csPin, uint16_t kbps, uint16_t freqMHz)
    : _freqCarrierMHz(freqMHz), _sdnPin(sdnPin), _intPin(intPin),
    _csPin(csPin), _kbps(kbps), packageSign_(0xDEAD), _freqChannel(0) {
  }

  // switches to Tx mode and sends the package, then optionally receives
  // response package.
  // TODO(maruel): const
  bool sendPacket(
      uint8_t length, byte* data, bool waitResponse = false,
      uint32_t ackTimeout = 100, uint8_t *responseLength = 0,
      byte* responseBuffer = 0) {
    this->clearTxFIFO();
    this->setRegister(REG_PKG_LEN, length);
    this->writeRegisters(REG_FIFO, data, length);
    // set interrupts on for package sent
    this->setRegister(REG_INT_ENABLE1, 0x04);
    // set interrupts off for anything else
    this->setRegister(REG_INT_ENABLE2, 0x00);
    // Read interrupt registers to clean them
    this->readRegister(REG_INT_STATUS1);
    this->readRegister(REG_INT_STATUS2);
    this->setMode(TXMode | Ready);
    uint64_t enterMillis = millis();
    while (millis() - enterMillis < MAX_TRANSMIT_TIMEOUT) {
      if (digitalRead(this->_intPin) != 0) {
        continue;
      }
      byte intStatus = this->readRegister(REG_INT_STATUS1);
      this->readRegister(REG_INT_STATUS2);
      if (intStatus & 0x04) {
        this->setMode(Ready | TuneMode);
        ESP_LOGD(TAG, "Package sent: %x", intStatus);
        // package sent. now, return true if not to wait ack, or wait ack (wait
        // for packet only for 'remaining' amount of time)
        if (waitResponse) {
          if (waitForPacket(ackTimeout)) {
            this->getPacketReceived(responseLength, responseBuffer);
            return true;
          }
          return false;
        }
        return true;
      }
    }
    ESP_LOGW(TAG, "Timeout in Transmit");
    this->setMode(Ready);
    if (this->readRegister(REG_DEV_STATUS) & 0x80) {
      this->clearFIFO();
    }
    return false;
  }

  // check for the packet received flags
  bool isPacketReceived() {
    if (digitalRead(this->_intPin) != 0) {
      // if no interrupt occurred, no packet received is assumed (since
      // startListening will be called prior, this assumption is enough)
      return false;
    }
    // check for package received status interrupt register
    byte intStat = this->readRegister(REG_INT_STATUS1);
    byte intStat2 = this->readRegister(REG_INT_STATUS2);
    if (intStat2 & 0x40) {
      // interrupt occurred, check it && read the Interrupt Status1 register for
      // 'preamble'
      ESP_LOGV(TAG, "Valid Preamble detected %x", intStat2);
    }
    if (intStat2 & 0x80) {
      // interrupt occurred, check it && read the Interrupt Status1 register
      // for 'preamble'
      ESP_LOGV(TAG, "HEY!! SYNC WORD detected %x", intStat2);
    }
    if (intStat & 0x02) {
      // Interrupt occurred, check it && read the Interrupt Status1 register
      // for 'valid packet'
      // if packet came, get out of Rx mode till the packet is read out. Keep
      // PLL on for fast reaction
      this->setMode(Ready | TuneMode);
      ESP_LOGD(TAG, "Packet detected %x", intStat);
      return true;
    }
    if (intStat & 0x01) {
      // packet crc error
      // get out of Rx mode till buffers are cleared
      this->setMode(Ready);
      ESP_LOGW(TAG, "CRC Error in Packet detected! %x", intStat);
      this->clearRxFIFO();
      // get back to work
      this->setMode(RXMode | Ready);
      return false;
    }
    // No relevant interrupt? no packet!
    return false;
  }

  // switch to Rx mode and wait until timeout or 'valid' package to arrive.
  bool waitForPacket(uint64_t waitMs) {
    this->startListening();
    uint64_t enterMillis = millis();
    while (millis() - enterMillis < waitMs) {
      if (this->isPacketReceived()) {
        return true;
      }
    }
    ESP_LOGW(TAG, "Timeout in receive");
    this->setMode(Ready);
    this->clearRxFIFO();
    return false;
  }

  // read from FIFO, up to 64 bytes.
  void getPacketReceived(uint8_t* length, byte* readData) {
    *length = this->readRegister(REG_RECEIVED_LENGTH);
    if (*length > 64) {
      *length = 64;
    }
    this->readRegisters(REG_FIFO, readData, *length);
    this->clearRxFIFO();
  }


  / *
  void softReset() {
    this->setRegister(REG_STATE, 0x80);
    byte reg = this->readRegister(REG_INT_STATUS2);
    while ((reg & 0x02) != 0x02) {
      delay(1);
      reg = this->readRegister(REG_INT_STATUS2);
    }
    this->boot();
  }
  // * /

private:
  static const int MAX_TRANSMIT_TIMEOUT = 200;
};

*/

// Pinout:
//  GND
//  SDN (hard reset)
//  MIRQ (interrupt)
//  NSEL (CS)
//  SCLK
//  SDI (MOSI)
//  SDO (MISO)
//  VCC
//  GPIO2 (unused)
//  GPIO1 (unused)
//  GPIO0 (unused)
//  GND (unused)
// https://esphome.io/custom/spi.html
// TODO(maruel): 1MHz once confirmed to work.
class Si4432Component :
  public text_sensor::TextSensor,
  public Component,
  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void set_irq_pin(GPIOPin *pin) { this->irq_pin_ = pin; }
  void set_sdn_pin(GPIOPin *pin) { this->sdn_pin_ = pin; }

  void setFrequency(uint16_t freqMHz) { this->freqMHz_ = freqMHz; }
  // sets the  bps. call before switching to tx or rx mode - min:1, max: 256
  void setBaudRate(uint16_t kbps) { this->kbps_ = kbps; }
  void setChannel(uint8_t channel) { this->channel_ = channel; }

  void setup() override;
  void loop() override;

/*
  void send() {
    ESP_LOGD(TAG, "Sending stuff");
    byte dummy[70] = { 0x01, 0x3, 0x11, 0x13 };
    byte resLen = 0;
    byte answer[64] = { 0 };
    while (!this->radio.sendPacket(32, dummy, true, 70, &resLen, answer)) {
    }
    ESP_LOGD(TAG, " SENT!");
    ESP_LOGV(TAG, "PACKET CAME [%d] %s",
        (int)resLen, format_hex_pretty(answer, resLen).c_str());
    if (this->_rx) {
      // restart the listening.
      this->radio.startListening();
    }
  }
*/
protected:
  enum AntennaMode {
    Ready = 0x01,
    TuneMode = 0x02,
    RXMode = 0x04,
    TXMode = 0x08,
  };

  enum Register {
    REG_DEV_TYPE = 0x00,
    REG_DEV_VERSION = 0x01,
    REG_DEV_STATUS = 0x02,

    REG_INT_STATUS1 = 0x03,
    REG_INT_STATUS2 = 0x04,
    REG_INT_ENABLE1 = 0x05,
    REG_INT_ENABLE2 = 0x06,
    REG_STATE = 0x07,
    REG_OPERATION_CONTROL = 0x08,
    REG_CRYSTAL = 0x09,
    REG_GPIO0_CONF = 0x0B,
    REG_GPIO1_CONF = 0x0C,
    REG_GPIO2_CONF = 0x0D,
    REG_IOPORT_CONF = 0x0E,

    REG_IF_FILTER_BW = 0x1C,
    REG_AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,
    REG_AFC_TIMING_CONTROL = 0x1E,
    REG_CLOCK_RECOVERY_GEARSHIFT = 0x1F,
    REG_CLOCK_RECOVERY_OVERSAMPLING = 0x20,
    REG_CLOCK_RECOVERY_OFFSET2 = 0x21,
    REG_CLOCK_RECOVERY_OFFSET1 = 0x22,
    REG_CLOCK_RECOVERY_OFFSET0 = 0x23,
    REG_CLOCK_RECOVERY_TIMING_GAIN1 = 0x24,
    REG_CLOCK_RECOVERY_TIMING_GAIN0 = 0x25,
    REG_RSSI = 0x26,
    REG_RSSI_THRESHOLD = 0x27,

    REG_AFC_LIMITER = 0x2A,
    REG_AFC_CORRECTION_READ = 0x2B,

    REG_DATAACCESS_CONTROL = 0x30,
    // Read only
    REG_EZMAC_STATUS = 0x31,
    REG_HEADER_CONTROL1 = 0x32,
    REG_HEADER_CONTROL2 = 0x33,
    REG_PREAMBLE_LENGTH = 0x34,
    REG_PREAMBLE_DETECTION = 0x35,
    REG_SYNC_WORD3 = 0x36,
    REG_SYNC_WORD2 = 0x37,
    REG_SYNC_WORD1 = 0x38,
    REG_SYNC_WORD0 = 0x39,
    REG_TRANSMIT_HEADER3 = 0x3A,
    REG_TRANSMIT_HEADER2 = 0x3B,
    REG_TRANSMIT_HEADER1 = 0x3C,
    REG_TRANSMIT_HEADER0 = 0x3D,
    REG_PKG_LEN = 0x3E,
    REG_CHECK_HEADER3 = 0x3F,
    REG_CHECK_HEADER2 = 0x40,
    REG_CHECK_HEADER1 = 0x41,
    REG_CHECK_HEADER0 = 0x42,
    REG_ENABLE_HEADER3 = 0x43,
    REG_ENABLE_HEADER2 = 0x44,
    REG_ENABLE_HEADER1 = 0x45,
    REG_ENABLE_HEADER0 = 0x46,
    // Read only
    REG_RECEIVED_HEADER3 = 0x47,
    // Read only
    REG_RECEIVED_HEADER2 = 0x48,
    // Read only
    REG_RECEIVED_HEADER1 = 0x49,
    // Read only
    REG_RECEIVED_HEADER0 = 0x4A,
    // Read only
    REG_RECEIVED_LENGTH = 0x4B,

    REG_CHARGEPUMP_OVERRIDE = 0x58,
    REG_DIVIDER_CURRENT_TRIM = 0x59,
    REG_VCO_CURRENT_TRIM = 0x5A,

    REG_AGC_OVERRIDE = 0x69,

    REG_TX_POWER = 0x6D,
    REG_TX_DATARATE1 = 0x6E,
    REG_TX_DATARATE0 = 0x6F,

    REG_MODULATION_MODE1 = 0x70,
    REG_MODULATION_MODE2 = 0x71,

    REG_FREQ_DEVIATION = 0x72,
    REG_FREQ_OFFSET1 = 0x73,
    REG_FREQ_OFFSET2 = 0x74,
    REG_FREQ_BAND = 0x75,
    REG_FREQ_CARRIER_H = 0x76,
    REG_FREQ_CARRIER_L = 0x77,

    REG_FREQ_CHANNEL = 0x79,
    REG_FREQ_CHANNEL_STEPSIZE = 0x7A,
    REG_TX_FIFO_CONTROL_1 = 0x7C,
    REG_TX_FIFO_CONTROL_2 = 0x7D,
    REG_RX_FIFO_CONTROL = 0x7E,
    REG_FIFO = 0x7F,
  };

  // TODO(maruel): Can't be called from within loop() due to delay().
  void hardReset();

  // Boots the radio.
  void boot();

  // Switch to Rx mode.
  void startListening();

  void setMode(AntennaMode mode);

  void setRegister(Register reg, byte value) {
    this->writeRegisters(reg, &value, 1);
  }
  byte readRegister(Register reg) {
    byte val = 0xFF;
    this->readRegisters(reg, &val, 1);
    return val;
  }

  void writeRegisters(Register reg, const byte value[], uint8_t length);
  void readRegisters(Register reg, byte value[], uint8_t length);

  // Logs all the register values.
  void dumpAllRegisters();

  void clearTxFIFO() {
    this->setRegister(REG_OPERATION_CONTROL, 0x01);
    this->setRegister(REG_OPERATION_CONTROL, 0x00);
  }

  void clearRxFIFO() {
    this->setRegister(REG_OPERATION_CONTROL, 0x02);
    this->setRegister(REG_OPERATION_CONTROL, 0x00);
  }

  // Clear FIFOs and interrupts.
  void clearFIFO() {
    this->setRegister(REG_OPERATION_CONTROL, 0x03);
    this->setRegister(REG_OPERATION_CONTROL, 0x00);
  }

  // Turn on the chip now.
  // TODO(maruel): Can't be called from within loop() due to delay().
  void turnOn() {
    this->sdn_pin_->digital_write(false);
    delay(20);
  }

  // Turn off the chip now.
  void turnOff() {
    this->sdn_pin_->digital_write(true);
    delay(1);
  }

private:
  const bool _rx{false};
  uint16_t freqMHz_{433};
  uint16_t kbps_{70};
  uint16_t packageSign_{0}; // 0xDEAD
  uint8_t channel_{0};
  GPIOPin *irq_pin_;
  GPIOPin *sdn_pin_;
};


}  // namespace si4432
}  // namespace esphome

#endif
