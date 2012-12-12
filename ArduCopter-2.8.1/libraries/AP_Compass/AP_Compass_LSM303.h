/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_LSM303_h
#define AP_Compass_LSM303_h

#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"

// device types

#define LSM303DLH_DEVICE   0
#define LSM303DLM_DEVICE   1
#define LSM303DLHC_DEVICE  2
#define LSM303_DEVICE_AUTO 3

// register addresses

#define LSM303_CRA_REG_M         0x00
#define LSM303_CRB_REG_M         0x01
#define LSM303_MR_REG_M          0x02

#define LSM303_OUT_X_H_M         0x03
#define LSM303_OUT_X_L_M         0x04
#define LSM303_OUT_Y_H_M         -1   // The addresses of the Y and Z magnetometer output registers 
#define LSM303_OUT_Y_L_M         -2   // are reversed on the DLM and DLHC relative to the DLH.
#define LSM303_OUT_Z_H_M         -3   // These four defines have dummy values so the library can 
#define LSM303_OUT_Z_L_M         -4   // determine the correct address based on the device type.

#define LSM303_SR_REG_M          0x09
#define LSM303_IRA_REG_M         0x0A
#define LSM303_IRB_REG_M         0x0B
#define LSM303_IRC_REG_M         0x0C

#define LSM303_WHO_AM_I_M        0x0F // DLM only

#define LSM303_TEMP_OUT_H_M      0x31 // DLHC only
#define LSM303_TEMP_OUT_L_M      0x32 // DLHC only

#define LSM303DLH_OUT_Y_H_M      0x05
#define LSM303DLH_OUT_Y_L_M      0x06
#define LSM303DLH_OUT_Z_H_M      0x07
#define LSM303DLH_OUT_Z_L_M      0x08

#define LSM303DLM_OUT_Z_H_M      0x05
#define LSM303DLM_OUT_Z_L_M      0x06
#define LSM303DLM_OUT_Y_H_M      0x07
#define LSM303DLM_OUT_Y_L_M      0x08

#define LSM303DLHC_OUT_Z_H_M     0x05
#define LSM303DLHC_OUT_Z_L_M     0x06
#define LSM303DLHC_OUT_Y_H_M     0x07
#define LSM303DLHC_OUT_Y_L_M     0x08

#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD ROTATION_NONE
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_45
#define AP_COMPASS_COMPONENTS_UP_PINS_RIGHT ROTATION_YAW_90
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_135
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK ROTATION_YAW_180
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_225
#define AP_COMPASS_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_270
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_315
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_45
#define AP_COMPASS_COMPONENTS_DOWN_PINS_RIGHT ROTATION_ROLL_180_YAW_90
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_135
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK ROTATION_PITCH_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_225
#define AP_COMPASS_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180_YAW_270
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_315
#define AP_COMPASS_APM2_SHIELD ROTATION_NONE

class AP_Compass_LSM303 : public Compass
{
  private:
    byte                _device; // chip type (DLH, DLM, or DLHC)
    byte                acc_address;
    unsigned int        io_timeout;
    bool                did_timeout;
    
    // byte detectSA0_A(void);
    float               calibration[3];
    bool                _initialised;
    virtual bool        read_raw(void);
    uint8_t             _base_config;
    virtual bool        re_initialise(void);
    bool                write_register(uint8_t reg, uint8_t value);
    bool                read_register(int reg, byte *value);
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at

    int16_t             _mag_x;
    int16_t             _mag_y;
    int16_t             _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t             _accum_count;
    uint32_t            _last_accum_time;

  public:
    
    // HEX  = BIN          RANGE    GAIN X/Y/Z        GAIN Z
    //                               DLH (DLM/DLHC)    DLH (DLM/DLHC)
    // 0x20 = 0b00100000   ±1.3     1055 (1100)        950 (980) (default)
    // 0x40 = 0b01000000   ±1.9      795  (855)        710 (760)
    // 0x60 = 0b01100000   ±2.5      635  (670)        570 (600)
    // 0x80 = 0b10000000   ±4.0      430  (450)        385 (400)
    // 0xA0 = 0b10100000   ±4.7      375  (400)        335 (355)
    // 0xC0 = 0b11000000   ±5.6      320  (330)        285 (295)
    // 0xE0 = 0b11100000   ±8.1      230  (230)        205 (205)
    enum magGain { magGain_13 = 0x20, magGain_19 = 0x40, magGain_25 = 0x60, magGain_40 = 0x80,
                   magGain_47 = 0xA0, magGain_56 = 0xC0, magGain_81 = 0xE0 };

    AP_Compass_LSM303(void);

    bool        read(void);
    void        accumulate(void);
    bool        init(byte device = LSM303DLHC_DEVICE);
    // byte        getDeviceType(void) { return _device; }
    bool        enableDefault(void);
    bool        readMag(void);
    void        set_orientation(enum Rotation rotation);
};

#endif
