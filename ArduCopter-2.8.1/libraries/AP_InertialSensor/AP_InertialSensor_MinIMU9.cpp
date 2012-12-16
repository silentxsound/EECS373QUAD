/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <FastSerial.h>
#include "AP_InertialSensor_MinIMU9.h"
#include "../AP_Math/AP_Math.h"
#include <SPI.h>
#include <I2C.h>

/*================ Defines ====================*/

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain

// accel as 2500 LSB/mg at scale factor of +/- 8g (FS==2)
// CTRL_REG4_A 
#define Accel_Scale_X 9.81 / 2500.0 //X axis Accel gain
#define Accel_Scale_Y 9.81 / 2500.0 //Y axis Accel gain
#define Accel_Scale_Z 9.81 / 2500.0 //Z axis Accel gain

#define Gyro_X_sign 1
#define Gyro_Y_sign 1
#define Gyro_Z_sign 1

#define Accel_X_sign -1
#define Accel_Y_sign 1
#define Accel_Z_sign 1

/*================ Public ====================*/

AP_InertialSensor_MinIMU9::AP_InertialSensor_MinIMU9() // : AP_InertialSensor
{
    _gyro.x = 0;
    _gyro.y = 0;
    _gyro.z = 0;
    _accel.x = 0;
    _accel.y = 0;
    _accel.z = 0;
}

uint16_t AP_InertialSensor_MinIMU9::init(AP_PeriodicProcess * scheduler)
{
    _deviceGyro = L3GD20_DEVICE;
    _gyro_address = L3GD20_ADDRESS_SA0_HIGH;
    _deviceAccel = LSM303DLHC_DEVICE;
    _accel_address = ACC_ADDRESS_SA0_A_HIGH;
    enableDefault();
    return AP_PRODUCT_ID_NONE; // ?? should this change
}

bool AP_InertialSensor_MinIMU9::update( void )
{
  read();

  // with gain & scale
  // _gyro.x = Gyro_Gain_X * Gyro_X_sign * _gyro.x;
  // _gyro.y = Gyro_Gain_Y * Gyro_X_sign * _gyro.y;
  // _gyro.z = Gyro_Gain_Z * Gyro_X_sign * _gyro.z;

  // _accel.x = Accel_Scale_X * Accel_X_sign * _accel.x;
  // _accel.y = Accel_Scale_Y * Accel_X_sign * _accel.y;
  // _accel.z = Accel_Scale_Z * Accel_X_sign * _accel.z;

  // without gain & scale
  _gyro.x = Gyro_X_sign * _gyro.x;
  _gyro.y = Gyro_X_sign * _gyro.y;
  _gyro.z = Gyro_X_sign * _gyro.z;

  _accel.x = Accel_X_sign * _accel.x;
  _accel.y = Accel_X_sign * _accel.y;
  _accel.z = Accel_X_sign * _accel.z;

}

bool AP_InertialSensor_MinIMU9::new_data_available( void )
{
    return 1;
}

float AP_InertialSensor_MinIMU9::gx()
{
    return _gyro.x;
}
float AP_InertialSensor_MinIMU9::gy()
{
    return _gyro.y;
}
float AP_InertialSensor_MinIMU9::gz()
{
    return _gyro.z;
}

void AP_InertialSensor_MinIMU9::get_gyros( float * g )
{
    g[0] = _gyro.x;
    g[1] = _gyro.y;
    g[2] = _gyro.z;
}

float AP_InertialSensor_MinIMU9::ax()
{
    return _accel.x;
}
float AP_InertialSensor_MinIMU9::ay()
{
    return _accel.y;
}
float AP_InertialSensor_MinIMU9::az()
{
    return _accel.z;
}

void AP_InertialSensor_MinIMU9::get_accels( float * a )
{
    a[0] = _accel.x;
    a[1] = _accel.y;
    a[2] = _accel.z;
}

void AP_InertialSensor_MinIMU9::get_sensors( float * sensors )
{
    sensors[0] = _gyro.x;
    sensors[1] = _gyro.y;
    sensors[2] = _gyro.z;
    sensors[3] = _accel.x;
    sensors[4] = _accel.y;
    sensors[5] = _accel.z;
}

float AP_InertialSensor_MinIMU9::temperature()
{
    uint8_t buff[2];
    I2c.read(MAG_ADDRESS, LSM303_TEMP_OUT_H_M | (1 << 7), 2, buff);
    _temp = ((int16_t)(buff[0] << 8 | buff[1])) >> 4;
    return _temp;
}

uint32_t AP_InertialSensor_MinIMU9::sample_time()
{
    return 0;
}

float AP_InertialSensor_MinIMU9::get_gyro_drift_rate(void)
{
    // 1.0 degrees/second/minute
    return ToRad(1.0/60);
}

uint16_t AP_InertialSensor_MinIMU9::num_samples_available()
{
    return 1;
}

void AP_InertialSensor_MinIMU9::read(void)
{
    readAccel();
    readGyro();
}

/*================ Private ====================*/

/*================ HARDWARE FUNCTIONS ====================*/

// Reads a gyro register
bool AP_InertialSensor_MinIMU9::readGyroReg(int reg, byte *value)
{
  if (I2c.read((uint8_t)_gyro_address, reg | (1 << 7), 1, value) != 0) {
        // healthy = false;
        return false;
  }
  return true;
}

// Writes a gyro register
bool AP_InertialSensor_MinIMU9::writeGyroReg(byte reg, byte value)
{
  if (I2c.write((uint8_t)_gyro_address, reg, value) != 0) {
      // healthy = false;
      return false;
  }
  return true;
}

bool AP_InertialSensor_MinIMU9::readGyro()
{
  // Serial.print("Doing readGyro(): start\n");
  uint8_t buff[6];

  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  if (I2c.read(_gyro_address, L3G_OUT_X_L | (1 << 7), 6, buff) != 0) {
      // healthy = false;
      return false;
  }

  // combine high and low bytes
  _gyro.x = (int16_t)(buff[1] << 8 | buff[0]);
  _gyro.y = (int16_t)(buff[3] << 8 | buff[2]);
  _gyro.z = (int16_t)(buff[5] << 8 | buff[4]);
  // Serial.print("G ");
  // Serial.print("X: ");
  // Serial.print((int)_gyro.x);
  // Serial.print(" Y: ");
  // Serial.print((int)_gyro.y);
  // Serial.print(" Z: ");
  // Serial.println((int)_gyro.z);
  // delay(50);
  // Serial.print("Doing readGyro(): complete\n");
}

// Reads an accelerometer register
bool AP_InertialSensor_MinIMU9::readAccelReg(int reg, byte *value)
{
  if (I2c.read((uint8_t)_accel_address, reg | (1 << 7), 1, value) != 0) {
        // healthy = false;
        return false;
  }
  return true;
}

// Writes an accelerometer register
bool AP_InertialSensor_MinIMU9::writeAccelReg(byte reg, byte value)
{
  if (I2c.write((uint8_t)_accel_address, reg, value) != 0) {
      // healthy = false;
      return false;
  }
  return true;
}

// Reads the 3 accelerometer channels and stores them in vector a
bool AP_InertialSensor_MinIMU9::readAccel(void)
{
  // Serial.print("Doing readAccel(): start\n");
  uint8_t buff[6];
  // assert the MSB of the address to get the accelerometer 
  // to do slave-transmit subaddress updating.
  if (I2c.read(_accel_address, LSM303_OUT_X_L_A | (1 << 7), 6, buff) != 0) {
      // healthy = false;
      return false;
  }
  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  _accel.x = ((int16_t)(buff[1] << 8 | buff[0])) >> 4;
  _accel.y = ((int16_t)(buff[3] << 8 | buff[2])) >> 4;
  _accel.z = ((int16_t)(buff[5] << 8 | buff[4])) >> 4;

  // without shift
  // _accel.x = (int16_t)(buff[1] << 8 | buff[0]);
  // _accel.y = (int16_t)(buff[3] << 8 | buff[2]);
  // _accel.z = (int16_t)(buff[5] << 8 | buff[4]);
  
  // Serial.print("A ");
  // Serial.print("X: ");
  // Serial.print((int)_accel.x);
  // Serial.print(" Y: ");
  // Serial.print((int)_accel.y);
  // Serial.print(" Z: ");
  // Serial.println((int)_accel.z);
  // delay(50);
  // Serial.print("Doing readAccel(): complete\n");
}

// Turns on the L3G's gyro and places it in normal mode.
void AP_InertialSensor_MinIMU9::enableDefault(void)
{
    // Enable Accelerometer
    // 0x27 = 0b00100111  10Hz
    // 0x37 = 25Hz
    // 0x47 = 50Hz
    // 0x57 = 100Hz
    // 0x67 = 200Hz
    // 0x77 = 400Hz
    // Normal power mode, all axes enabled
    // Serial.print("Doing writeAccelReg(): start\n");
    writeAccelReg(LSM303_CTRL_REG1_A, 0x67);
    // Serial.print("Doing writeAccelReg(): complete\n");
    // Enable Gyroscope
    // 0x0F = 0b00001111
    // Normal power mode, all axes enabled
    // Serial.print("Doing writeGyroReg(): start\n");
    writeGyroReg(L3G_CTRL_REG1, 0x0F); // 0x0F may need to change
    // Serial.print("Doing writeGyroReg(): complete\n");
}
