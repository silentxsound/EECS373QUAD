/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MINIMU9_H__
#define __AP_INERTIAL_SENSOR_MINIMU9_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"
#include <Arduino.h> // for byte data type

// device types gyro

#define L3G_DEVICE_AUTO 0
#define L3G4200D_DEVICE 1
#define L3GD20_DEVICE   2

// device types accel

#define LSM303DLH_DEVICE   0
#define LSM303DLM_DEVICE   1
#define LSM303DLHC_DEVICE  2 // we are using this one
#define LSM303_DEVICE_AUTO 3

// SA0_G states

#define L3G_SA0_LOW  0
#define L3G_SA0_HIGH 1
#define L3G_SA0_AUTO 2

// SA0_A states

#define LSM303_SA0_A_LOW  0
#define LSM303_SA0_A_HIGH 1
#define LSM303_SA0_A_AUTO 2

// register addresses gyro

#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F

#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38

// register addresses accel

#define LSM303_CTRL_REG1_A       0x20
#define LSM303_CTRL_REG2_A       0x21
#define LSM303_CTRL_REG3_A       0x22
#define LSM303_CTRL_REG4_A       0x23
#define LSM303_CTRL_REG5_A       0x24
#define LSM303_CTRL_REG6_A       0x25 // DLHC only
#define LSM303_HP_FILTER_RESET_A 0x25 // DLH, DLM only
#define LSM303_REFERENCE_A       0x26
#define LSM303_STATUS_REG_A      0x27

#define LSM303_OUT_X_L_A         0x28
#define LSM303_OUT_X_H_A         0x29
#define LSM303_OUT_Y_L_A         0x2A
#define LSM303_OUT_Y_H_A         0x2B
#define LSM303_OUT_Z_L_A         0x2C
#define LSM303_OUT_Z_H_A         0x2D

#define LSM303_FIFO_CTRL_REG_A   0x2E // DLHC only
#define LSM303_FIFO_SRC_REG_A    0x2F // DLHC only

#define LSM303_INT1_CFG_A        0x30
#define LSM303_INT1_SRC_A        0x31
#define LSM303_INT1_THS_A        0x32
#define LSM303_INT1_DURATION_A   0x33
#define LSM303_INT2_CFG_A        0x34
#define LSM303_INT2_SRC_A        0x35
#define LSM303_INT2_THS_A        0x36
#define LSM303_INT2_DURATION_A   0x37

#define LSM303_CLICK_CFG_A       0x38 // DLHC only
#define LSM303_CLICK_SRC_A       0x39 // DLHC only
#define LSM303_CLICK_THS_A       0x3A // DLHC only
#define LSM303_TIME_LIMIT_A      0x3B // DLHC only
#define LSM303_TIME_LATENCY_A    0x3C // DLHC only
#define LSM303_TIME_WINDOW_A     0x3D // DLHC only

#define MAG_ADDRESS             (0x3C >> 1)
#define LSM303_TEMP_OUT_H_M      0x31
#define LSM303_TEMP_OUT_L_M      0x32

class AP_InertialSensor_MinIMU9 : public AP_InertialSensor
{
public:

    AP_InertialSensor_MinIMU9();

    uint16_t        init(AP_PeriodicProcess * scheduler);
    bool            update();
    bool            new_data_available();
    float           gx();
    float           gy();
    float           gz();
    void            get_gyros( float * );
    float           ax();
    float           ay();
    float           az();
    void            get_accels( float * );
    void            get_sensors( float * );
    float           temperature();
    uint32_t        sample_time();
    float           get_gyro_drift_rate(); // maybe ToRad(1.0/60) ??
    uint16_t        num_samples_available();
    void            read();


private:
    byte _deviceGyro; // chip type (4200D or D20)
    byte _gyro_address;
    bool readGyroReg(int reg, byte *value);
    bool writeGyroReg(byte reg, byte value);
    bool readGyro(void);
    Vector3f                    _gyro;

    byte _deviceAccel; // chip type (DLH, DLM or DLHC)
    byte _accel_address;
    bool readAccelReg(int reg, byte *value);
    bool writeAccelReg(byte reg, byte value);
    bool readAccel(void);
    Vector3f                    _accel;
    
    void enableDefault(void);

    float                       _temp;
    static const uint8_t        _sensors[6];

    // float                       _gyro_apply_std_offset( float adc_value );
    // float                       _accel_apply_std_offset( float adc_value );
};

#endif // __AP_INERTIAL_SENSOR_MINIMU9_H__
