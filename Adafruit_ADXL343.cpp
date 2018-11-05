/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL343 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL343 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Adafruit_ADXL343.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline uint8_t Adafruit_ADXL343::i2cread(void) {
  #if ARDUINO >= 100
  return _wire->read();
  #else
  return _wire->receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline void Adafruit_ADXL343::i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  _wire->write((uint8_t)x);
  #else
  _wire->send(x);
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away SPI receiver & transmitter
*/
/**************************************************************************/
static uint8_t spixfer(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t data) {
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(clock, LOW);
    digitalWrite(mosi, data & (1<<i));
    digitalWrite(clock, HIGH);
    if (digitalRead(miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void Adafruit_ADXL343::writeRegister(uint8_t reg, uint8_t value) {
  if (_i2c) {
    _wire->beginTransmission(ADXL343_ADDRESS);
    i2cwrite((uint8_t)reg);
    i2cwrite((uint8_t)(value));
    _wire->endTransmission();
  } else {
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    spixfer(_clk, _di, _do, value);
    digitalWrite(_cs, HIGH);
  }
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t Adafruit_ADXL343::readRegister(uint8_t reg) {
  if (_i2c) {
    _wire->beginTransmission(ADXL343_ADDRESS);
    i2cwrite(reg);
    _wire->endTransmission();
    _wire->requestFrom(ADXL343_ADDRESS, 1);
    return (i2cread());
  } else {
    reg |= 0x80; // read byte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint8_t reply = spixfer(_clk, _di, _do, 0xFF);
    digitalWrite(_cs, HIGH);
    return reply;
  }
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t Adafruit_ADXL343::read16(uint8_t reg) {
  if (_i2c) {
    _wire->beginTransmission(ADXL343_ADDRESS);
    i2cwrite(reg);
    _wire->endTransmission();
    _wire->requestFrom(ADXL343_ADDRESS, 2);
    return (uint16_t)(i2cread() | (i2cread() << 8));
  } else {
    reg |= 0x80 | 0x40; // read byte | multibyte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint16_t reply = spixfer(_clk, _di, _do, 0xFF)  | (spixfer(_clk, _di, _do, 0xFF) << 8);
    digitalWrite(_cs, HIGH);
    return reply;
  }
}

/**************************************************************************/
/*!
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t Adafruit_ADXL343::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL343_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL343::getX(void) {
  return read16(ADXL343_REG_DATAX0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL343::getY(void) {
  return read16(ADXL343_REG_DATAY0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL343::getZ(void) {
  return read16(ADXL343_REG_DATAZ0);
}

/**************************************************************************/
/*!
*   @brief  Instantiates a new ADXL343 class
*
*   @param sensorID  An optional ID # so you can track this sensor, it will
*                    tag sensorEvents you create.
*/
/**************************************************************************/
Adafruit_ADXL343::Adafruit_ADXL343(int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL343_RANGE_2_G;
  _i2c = true;
  _wire = &Wire;
}

/**************************************************************************/
/*!
*   @brief  Instantiates a new ADXL343 class
*
*   @param sensorID  An optional ID # so you can track this sensor, it will
*                    tag sensorEvents you create.
*   @param wireBus   TwoWire instance to use for I2C communication.
*/
/**************************************************************************/
Adafruit_ADXL343::Adafruit_ADXL343(int32_t sensorID, TwoWire* wireBus) {
  _sensorID = sensorID;
  _range = ADXL343_RANGE_2_G;
  _i2c = true;
  _wire = wireBus;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL343 class in SPI mode
*/
/**************************************************************************/
Adafruit_ADXL343::Adafruit_ADXL343(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs, int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL343_RANGE_2_G;
  _cs = cs;
  _clk = clock;
  _do = mosi;
  _di = miso;
  _i2c = false;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ADXL343::begin() {

  if (_i2c)
    _wire->begin();
  else {
    pinMode(_cs, OUTPUT);
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, HIGH);
    pinMode(_do, OUTPUT);
    pinMode(_di, INPUT);
  }

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  Serial.print("Device ID: "); Serial.println(deviceid, HEX);
  if (deviceid != 0xE5)
  {
    /* No ADXL343 detected ... return false */
    return false;
  }

  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void Adafruit_ADXL343::setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
range_t Adafruit_ADXL343::getRange(void)
{
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL343 (controls power consumption)
*/
/**************************************************************************/
void Adafruit_ADXL343::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL343_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL343 (controls power consumption)
*/
/**************************************************************************/
dataRate_t Adafruit_ADXL343::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_ADXL343::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x = getX() * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = getY() * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = getZ() * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_ADXL343::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ADXL343", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value   = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution  = 0.03923F;   /*  4mg = 0.0392266 m/s^2 */
}
