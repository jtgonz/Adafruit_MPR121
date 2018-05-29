/***************************************************
  This is a library for the MPR121 I2C 12-chan Capacitive Sensor

  Designed specifically to work with the MPR121 sensor from Adafruit
  ----> https://www.adafruit.com/products/1982

  These sensors use I2C to communicate, 2+ pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MPR121.h"

Adafruit_MPR121::Adafruit_MPR121() {
}

boolean Adafruit_MPR121::begin(uint8_t i2caddr) {
  Wire.begin();

  _i2caddr = i2caddr;

  // soft reset
  writeRegister(MPR121_SOFTRESET, 0x63);
  delay(1);
  for (uint8_t i=0; i<0x7F; i++) {
  //  Serial.print("$"); Serial.print(i, HEX);
  //  Serial.print(": 0x"); Serial.println(readRegister8(i));
  }


  writeRegister(MPR121_ECR, 0x0);

  uint8_t c = readRegister8(MPR121_CONFIG2);

  if (c != 0x24) return false;


  setThreshholds(12, 6);

  // These parameters (max half delta, noise half delta, noise count limit, and
  // filter delay limit) control how the baseline changes in response to noise
  // and environmental variations.
  writeRegister(MPR121_MHDR, 0x01);
  writeRegister(MPR121_NHDR, 0x01);
  writeRegister(MPR121_NCLR, 0x0E);
  writeRegister(MPR121_FDLR, 0x00);

  writeRegister(MPR121_MHDF, 0x01);
  writeRegister(MPR121_NHDF, 0x05);
  writeRegister(MPR121_NCLF, 0x01);
  writeRegister(MPR121_FDLF, 0x00);

  writeRegister(MPR121_NHDT, 0x00);
  writeRegister(MPR121_NCLT, 0x00);
  writeRegister(MPR121_FDLT, 0x00);

  writeRegister(MPR121_DEBOUNCE, 0);
  writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
  writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

//  writeRegister(MPR121_AUTOCONFIG0, 0x8F);

//  writeRegister(MPR121_UPLIMIT, 150);
//  writeRegister(MPR121_TARGETLIMIT, 100); // should be ~400 (100 shifted)
//  writeRegister(MPR121_LOWLIMIT, 50);

  // enable all electrodes
  writeRegister(MPR121_ECR, 0x8F);  // start with first 5 bits of baseline tracking

  return true;
}

void Adafruit_MPR121::setThreshholds(uint8_t touch, uint8_t release) {

  setThresholds(touch, release);
  }

void Adafruit_MPR121::setThresholds(uint8_t touch, uint8_t release) {
  for (uint8_t i=0; i<12; i++) {
    writeRegister(MPR121_TOUCHTH_0 + 2*i, touch);
    writeRegister(MPR121_RELEASETH_0 + 2*i, release);
  }
}

uint16_t  Adafruit_MPR121::filteredData(uint8_t t) {
  if (t > 12) return 0;
  return readRegister16(MPR121_FILTDATA_0L + t*2);
}

uint16_t  Adafruit_MPR121::baselineData(uint8_t t) {
  if (t > 12) return 0;
  uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
  return (bl << 2);
}

uint16_t  Adafruit_MPR121::touched(void) {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

boolean Adafruit_MPR121::autoconfig(void) {
  // store current value of electrode configuration register (ECR), so we
  // can re-enable the same electrodes afterwards
  uint8_t settings_ECR = readRegister8(MPR121_ECR);

  // put MPR121 in stop mode by disabling all electrodes (this allows us to
  // write to other registers)
  writeRegister(MPR121_ECR, settings_ECR & 0xC0);

  // limit electrode baseline to 0.7V below supply voltage -- outside this
  // range, system is nonlinear and ADC counts are invalid
  writeRegister(MPR121_UPLIMIT, 0xC9);     // 2.6V upper limit (2.6/3.3 * 256)
  writeRegister(MPR121_LOWLIMIT, 0x84);    // 1.7V lower limit (65% of upper)
  writeRegister(MPR121_TARGETLIMIT, 0xB5); // 2.3V target (90% of upper lim)

  // configure auto-config control register with these settings:
  // * six samples taken as input to the first filter
  // * auto-config will not retry upon failure
  // * baseline set to auto-config basline w/ lower 3 bits cleared
  // * trigger reconfiguration if baseline drifts out of upper/lower bounds
  // * enable auto-config (if set to zero, this is all ignored)
  writeRegister(MPR121_AUTOCONFIG0, 0x0B);

  // re-enable electrodes from earlier
  writeRegister(MPR121_ECR, settings_ECR);

  return true;
}

/*********************************************************************/


uint8_t Adafruit_MPR121::readRegister8(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    while (Wire.requestFrom(_i2caddr, 1) != 1);
    return ( Wire.read());
}

uint16_t Adafruit_MPR121::readRegister16(uint8_t reg) {
    Wire.beginTransmission(_i2caddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    while (Wire.requestFrom(_i2caddr, 2) != 2);
    uint16_t v = Wire.read();
    v |=  ((uint16_t) Wire.read()) << 8;
    return v;
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void Adafruit_MPR121::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)(value));
    Wire.endTransmission();
}
