/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <math.h>
#include <FastSerial.h>
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WConstants.h"
#endif

#include <I2C.h>
#include "AP_Compass_LSM303.h"

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define MAG_ADDRESS            (0x3C >> 1)

#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

// Constructors ////////////////////////////////////////////////////////////////

AP_Compass_LSM303::AP_Compass_LSM303(void) : Compass()
{  
  _device = LSM303_DEVICE_AUTO;
  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Writes a magnetometer register
bool AP_Compass_LSM303::write_register(uint8_t reg, uint8_t value)
{
  if (I2c.write((uint8_t)MAG_ADDRESS, reg, value) != 0) {
      healthy = false;
      return false;
  }
  return true;
}


// Reads a magnetometer register
bool AP_Compass_LSM303::read_register(int reg, byte *value)
{  
  // if dummy register address (magnetometer Y/Z), use device type to determine actual address
  if (reg < 0)
  {
    switch (reg)
    {
      case LSM303_OUT_Y_H_M:
        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_H_M : LSM303DLM_OUT_Y_H_M;
        break;
      case LSM303_OUT_Y_L_M:
        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_L_M : LSM303DLM_OUT_Y_L_M;
        break;
      case LSM303_OUT_Z_H_M:
        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_H_M : LSM303DLM_OUT_Z_H_M;
        break;
      case LSM303_OUT_Z_L_M:
        reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_L_M : LSM303DLM_OUT_Z_L_M;
        break;
    }
  }
    
  if (I2c.read((uint8_t)MAG_ADDRESS, reg, 1, value) != 0) {
        healthy = false;
        return false;
  }
  return true;
}

  // Read Sensor data
bool AP_Compass_LSM303::read_raw()
{
  uint8_t buff[6];

  if (I2c.read(MAG_ADDRESS, 0x03, 6, buff) != 0) {
      healthy = false;
      return false;
  }


  int16_t rx, ry, rz;
  rx = (int16_t)(buff[0] << 8 | buff[1]);
  rz = (int16_t)(buff[2] << 8 | buff[3]);
  ry = (int16_t)(buff[4] << 8 | buff[5]);

  // Serial.printf("\t %u \t %u \t %u\n", -rx, ry, -rz);

  if(rx == -4096 || ry == -4096 || rz == -4096){ // what should these values be??
    return false;
  }

  _mag_x = rx; // why is this negative
  _mag_y = ry;
  _mag_z = -rz; // why is this negative
  
  return true;
}

// accumulate a reading from the magnetometer
void AP_Compass_LSM303::accumulate(void)
{
  uint32_t tnow = micros();
  if (healthy && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {
  // the compass gets new data at 75Hz
  return;

  }
  if (read_raw()) {
  // the _mag_N values are in the range -2048 to 2047, so we can
  // accumulate up to 15 of them in an int16_t. Let's make it 14
  // for ease of calculation. We expect to do reads at 10Hz, and
  // we get new data at most 75Hz, so we don't expect to
  // accumulate more than 8 before a read
  _mag_x_accum += _mag_x;
  _mag_y_accum += _mag_y;
  _mag_z_accum += _mag_z;
  _accum_count++;
  if (_accum_count == 14) {
   _mag_x_accum /= 2;
   _mag_y_accum /= 2;
   _mag_z_accum /= 2;
   _accum_count = 7;
  }
  _last_accum_time = tnow;
  }
}

/*
 *  re-initialise after a IO error
 */
bool AP_Compass_LSM303::re_initialise()
{
  if (!write_register((uint8_t)LSM303_CRA_REG_M, (uint8_t)_base_config) ||
      !write_register((uint8_t)LSM303_CRB_REG_M, (uint8_t)magGain_19) ||
      !write_register((uint8_t)LSM303_MR_REG_M, 0x00) )
      return false;
  return true;
}

bool AP_Compass_LSM303::init(byte device)
{
  _device = device;
  int numAttempts = 0, good_count = 0;
  bool success = false;
  byte calibration_gain = magGain_19;
  uint16_t expected_x = 715;
  uint16_t expected_yz = 715;
  float gain_multiple = 1.0;
  
  // change output data rate to 75Hz to match HMC5843
  _base_config = 0x18;
  write_register((uint8_t)LSM303_MR_REG_M, 0x00);

  calibration[0] = 0;
  calibration[1] = 0;
  calibration[2] = 0;

  while ( success == 0 && numAttempts < 20 && good_count < 5)
  {
      // record number of attempts at initialisation
      numAttempts++;

      // force positiveBias (compass should return 715 for all channels)
      if (!write_register(LSM303_CRA_REG_M, 0x11))
          continue;      // compass not responding on the bus
      delay(50);

      // set gains
      if (!write_register(LSM303_CRB_REG_M, calibration_gain) ||
          !write_register(LSM303_MR_REG_M, 0x01))
          continue;

      // read values from the compass
      delay(50);
      if (!read_raw())
          continue;      // we didn't read valid values

      delay(10);

      float cal[3];

      cal[0] = fabs(expected_x / (float)_mag_x);
      cal[1] = fabs(expected_yz / (float)_mag_y);
      cal[2] = fabs(expected_yz / (float)_mag_z);

      if (cal[0] > 0.7 && cal[0] < 1.3 &&
          cal[1] > 0.7 && cal[1] < 1.3 &&
          cal[2] > 0.7 && cal[2] < 1.3) {
          good_count++;
          calibration[0] += cal[0];
          calibration[1] += cal[1];
          calibration[2] += cal[2];
      }

#if 0
        /* useful for debugging */
        Serial.print("mag_x: ");
        Serial.print(_mag_x);
        Serial.print(" mag_y: ");
        Serial.print(_mag_y);
        Serial.print(" mag_z: ");
        Serial.println(_mag_z);
        Serial.print("CalX: ");
        Serial.print(calibration[0]/good_count);
        Serial.print(" CalY: ");
        Serial.print(calibration[1]/good_count);
        Serial.print(" CalZ: ");
        Serial.println(calibration[2]/good_count);
#endif
  }

  if (good_count >= 5) {
      calibration[0] = calibration[0] * gain_multiple / good_count;
      calibration[1] = calibration[1] * gain_multiple / good_count;
      calibration[2] = calibration[2] * gain_multiple / good_count;
      success = true;
  } else {
      /* best guess */
      calibration[0] = 1.0;
      calibration[1] = 1.0;
      calibration[2] = 1.0;
  }

  // leave test mode
  if (!re_initialise()) {
      return false;
  }

  _initialised = true;

  // perform an initial read
  healthy = true;
  read();

  return true;
}

bool AP_Compass_LSM303::readMag()
{
  if (!_initialised) {
      // someone has tried to enable a compass for the first time
      // mid-flight .... we can't do that yet (especially as we won't
      // have the right orientation!)
      return false;
  }
  if (!healthy) {
      if (millis() < _retry_time) {
          return false;
      }
      if (!re_initialise()) {
          _retry_time = millis() + 1000;
          return false;
      }
  }

if (_accum_count == 0) {
   accumulate();
   if (!healthy || _accum_count == 0) {
    // try again in 1 second, and set I2c clock speed slower
    _retry_time = millis() + 1000;
    return false;
   }
}

mag_x = _mag_x_accum * calibration[0] / _accum_count;
mag_y = _mag_y_accum * calibration[1] / _accum_count;
mag_z = _mag_z_accum * calibration[2] / _accum_count;
_accum_count = 0;
_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

  last_update = micros(); // record time of update

  // rotate to the desired orientation
  Vector3f rot_mag = Vector3f(mag_x, mag_y, mag_z);
  if (product_id == AP_COMPASS_TYPE_HMC5883L) {
      rot_mag.rotate(ROTATION_ROLL_180);
  }
  rot_mag.rotate(_orientation);

  rot_mag += _offset.get();
  mag_x = rot_mag.x;
  mag_y = rot_mag.y;
  mag_z = rot_mag.z;
  healthy = true;

  return true;
}

// Reads all mag channels of the LSM303 and stores them in the object variables
bool AP_Compass_LSM303::read()
{
  if(readMag()){
    return true;
  }
  return false;
}

// set orientation
void
AP_Compass_LSM303::set_orientation(enum Rotation rotation)
{
    _orientation = rotation;
}