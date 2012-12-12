// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor MPU6000 driver.
//

#include <FastSerial.h>
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;
AP_InertialSensor_MinIMU9 ins;

void setup(void)
{
    Serial.begin(115200);
    Serial.println("INS library test: MinIMU9");
    I2c.begin();
    I2c.timeOut(20);

    I2c.setSpeed(true);
    isr_registry.init();
    scheduler.init(&isr_registry);
    ins.init(&scheduler);
}

void loop(void)
{
    float accel[3];
    float gyro[3];
    float temperature;

    delay(20);
    ins.update();
    ins.get_gyros(gyro);
    ins.get_accels(accel);
    temperature = ins.temperature();

    Serial.printf("AX: %f  AY: %f  AZ: %f  GX: %f  GY: %f  GZ: %f T=%f\n",
                  accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temperature);
}
