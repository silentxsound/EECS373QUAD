// demo test for EECS 373

#include <stdint.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <I2C.h>
#include <SPI.h>
#include <Filter.h>
#include <AverageFilter.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_Baro.h> // ArduPilot Mega ADC Library
#include <AP_Compass.h> // Compass Library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <APM_RC.h> // ArduPilot Mega RC Library

#ifndef APM2_HARDWARE
 # define APM2_HARDWARE 0
#endif

FastSerialPort0(Serial);

// Barometric Sensor 
AP_Baro_BMP085 APM_BMP085(APM2_HARDWARE);
Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

// Magnetometer
#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

AP_Compass_LSM303 compass;
unsigned long timer;

// Accelerometer & Gyroscope
AP_InertialSensor_MinIMU9 ins;

// Radio Control
APM_RC_APM1 APM_RC;

void setup(){
    Serial.println("EECS 373 Project Demo Test");

    I2c.begin();
    I2c.timeOut(20);
    I2c.setSpeed(true);

    isr_registry.init();
    scheduler.init(&isr_registry);

	// Barometric Pressure Sensor
    Serial.println("Initialising BMP085 barometer...");
    delay(100);
    if (!APM_BMP085.init(&scheduler)) {
        Serial.println("Barometer initialisation FAILED\n");
    }
    Serial.println("initialisation complete.");
    delay(1000);
    timer = micros();

	// Magnetometer
    Serial.println("Initialising LSM303 Magnetometer");
    delay(100);
    if (!compass.init()) {
        Serial.println("Compass Initialization FAILED!");
        while (1) ;
    }
    compass.set_orientation(ROTATION_ROLL_180); // set compass's orientation on aircraft.
    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(11.0)); // set local difference between magnetic north and true north
    Serial.println("Initialization complete.");
    delay(1000);
    timer = micros();
    
    // Accelerometer && Gyroscope
    Serial.println("Initializing LSM303 Accelerometer and L3G20 Gyroscope");
    delay(100);
    ins.init(&scheduler);
    Serial.println("Initialization complete.");
    delay(1000);
    timer = micros();

    // Radio Control
    Serial.println("Initializing Radio Controls");
    delay(100);
    APM_RC.Init(&isr_registry);          // APM Radio Initialization
    APM_RC.enable_out(CH_1);
    APM_RC.enable_out(CH_2);
    APM_RC.enable_out(CH_3);
    APM_RC.enable_out(CH_4);
    Serial.println("Initialization complete.");
    delay(1000);
    timer = micros();
}

void loop(){
	// main
	int value;
	int test_counter = 0;
    // display help
    Serial.print("Press 'a' to test accelerometer.");
    Serial.print("Press 'b' to test barometric pressure.");
    Serial.print("Press 'c' to test compass.");
    Serial.print("Press 'g' to test gyroscope.");
    Serial.println("Press 'r' to test radio control.  Becareful they will spin!");
    // wait for user to enter something
    while(!Serial.available()){
        delay(20);
    }
    // get character from user
    value = Serial.read();
	// Barometric Pressure Sensor
	float tmp_float;
    float Altitude;
	if(value == 'b' ||value == 'B'){
		while(test_counter < 100){
		    if((micros()- timer) > 50000L) {
		        timer = micros();
		        APM_BMP085.read();
		        unsigned long read_time = micros() - timer;
		        if(!APM_BMP085.healthy){
		            Serial.println("not healthy");
		            return;
		        }
		        Serial.print("Pressure:");
		        Serial.print(APM_BMP085.get_pressure());
		        Serial.print(" Temperature:");
		        Serial.print(APM_BMP085.get_temperature());
		        Serial.print(" Altitude:");
		        tmp_float = (APM_BMP085.get_pressure() / 101325.0);
		        tmp_float = pow(tmp_float, 0.190295);
		        Altitude = 44330.0 * (1.0 - tmp_float);
		        Serial.print(Altitude);
		        Serial.printf(" t=%u", (unsigned)read_time);
		        Serial.println();
		    }
		    test_counter++;
		}
	}

	// Magnetometer
	static float min[3], max[3], offset[3];
	if(value == 'm' ||value == 'M'){
		while(test_counter < 100){
			compass.accumulate();
	    	if((micros()- timer) > 100000L){
     	   		timer = micros();
	        	compass.read();
	        	unsigned long read_time = micros() - timer;
	        	float heading;
	        	if(!compass.healthy){
	            	Serial.println("not healthy");
	            	return;
		        }
		        heading = compass.calculate_heading(0,0); // roll = 0, pitch = 0 for this example
			    compass.null_offsets();
			   	// capture min
		        if( compass.mag_x < min[0] )
		            min[0] = compass.mag_x;
		        if( compass.mag_y < min[1] )
		            min[1] = compass.mag_y;
		        if( compass.mag_z < min[2] )
		            min[2] = compass.mag_z;
		        // capture max
		        if( compass.mag_x > max[0] )
		            max[0] = compass.mag_x;
		        if( compass.mag_y > max[1] )
		            max[1] = compass.mag_y;
		        if( compass.mag_z > max[2] )
			        max[2] = compass.mag_z;
		        // calculate offsets
		        offset[0] = -(max[0]+min[0])/2;
		        offset[1] = -(max[1]+min[1])/2;
		        offset[2] = -(max[2]+min[2])/2;
		        // display all to user
		        Serial.printf("Heading: %.2f (%3u,%3u,%3u) ",
		                      ToDeg(heading),
		                      compass.mag_x,
		                      compass.mag_y,
		                      compass.mag_z);
		        // display offsets
		        Serial.printf("\t offsets(%.2f, %.2f, %.2f)",
		                      offset[0], offset[1], offset[2]);
		        Serial.printf(" t=%u", (unsigned)read_time);
	        	Serial.println();
			}
	    test_counter++;
		}
	}

	// Accelerometer & Gyroscope
	if(value == 'b' ||value == 'B'){
	    float gyro[3];
		while(test_counter < 100){
		    delay(100);
		    ins.update();
		    ins.get_gyros(gyro);
		    Serial.printf("GX: %f  GY: %f  GZ: %f\n", gyro[0], gyro[1], gyro[2]);
		    test_counter++;
		}
	}

	if(value == 'g' || value =='G'){
		float accel[3];
		while(test_counter < 100){
		    delay(100);
		    ins.update();
		    ins.get_accels(accel);
		    Serial.printf("AX: %f  AY: %f  AZ: %f\n", accel[0], accel[1], accel[2]);
		    test_counter++;
		}
	}

    // Radio Control
    if(value == 'r' ||value == 'R'){
    	while(test_counter < 100){
		    if(APM_RC.GetState() == 1){
		        Serial.print("CH:");
		        for(int i = 0; i < 4; i++){
		            Serial.print(APM_RC.InputCh(i));                    // Print channel values
		            Serial.print(",");
		        }
		        APM_RC.OutputCh(CH_1, APM_RC.InputCh(CH_3));
		        APM_RC.OutputCh(CH_2, APM_RC.InputCh(CH_3));
		        APM_RC.OutputCh(CH_3, APM_RC.InputCh(CH_3));
		        APM_RC.OutputCh(CH_4, APM_RC.InputCh(CH_3));
		        Serial.println();
	    	}
	    	test_counter++;
	 	}   	
	}
}
