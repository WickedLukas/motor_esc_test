#include <Arduino.h>
#include <Servo.h>
#include "DShot.h"
#include "iBus.h"

// print debug outputs through serial
#define DEBUG

#ifdef DEBUG
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
	#define DEBUG_PRINT2(x,y) Serial.print(x,y)
	#define DEBUG_PRINTLN2(x,y) Serial.println(x,y)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
	#define DEBUG_PRINT2(x,y)
	#define DEBUG_PRINTLN2(x,y)
#endif

// rc channel assignment
#define ROLL		0
#define PITCH		1
#define YAW			3
#define THROTTLE	2
#define STOP		10

// object for radio control (rc)
IBUS rc;

// esc inputs
// TODO: ESC calibration with analog protocol after flashing but before using dshot is recommended
// 1: top-left (CW); 2: top-right (CCW); 3: bottom-left (CW); 4: bottom-right (CCW);
Servo esc_motor_1;
Servo esc_motor_2;
Servo esc_motor_3;
Servo esc_motor_4;

/*DShot esc_motor_1(1);
DShot esc_motor_2(2);
DShot esc_motor_3(3);
DShot esc_motor_4(4);*/

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

void setup() {
	#if defined(DEBUG)
		// initialize serial for monitoring
		Serial.begin(115200);
		while (!Serial);
	#endif
	
	// initialize serial for iBus communication
	Serial3.begin(115200, SERIAL_8N1);
	while (!Serial3);
	
	// initialize rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(Serial3);
}

void loop() {
	delayMicroseconds(100);
	
	// update rc
	rc.update();
	
	if (rc_channelValue[STOP] == 1000) {
		esc_motor_1.writeMicroseconds(rc_channelValue[THROTTLE]);
		esc_motor_2.writeMicroseconds(rc_channelValue[THROTTLE]);
		esc_motor_3.writeMicroseconds(rc_channelValue[THROTTLE]);
		esc_motor_4.writeMicroseconds(rc_channelValue[THROTTLE]);
		/*
		esc_motor_1.setThrottle(rc_channelValue[THROTTLE]);
		esc_motor_2.setThrottle(rc_channelValue[THROTTLE]);
		esc_motor_3.setThrottle(rc_channelValue[THROTTLE]);
		esc_motor_4.setThrottle(rc_channelValue[THROTTLE]);
		*/
		
	}
	else {
		esc_motor_1.writeMicroseconds(1000);
		esc_motor_2.writeMicroseconds(1000);
		esc_motor_3.writeMicroseconds(1000);
		esc_motor_4.writeMicroseconds(1000);
		/*
		esc_motor_1.setThrottle(1000);
		esc_motor_2.setThrottle(1000);
		esc_motor_3.setThrottle(1000);
		esc_motor_4.setThrottle(1000);
		*/
	}
	
	// run serial print at a rate independent of the main loop (t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		
		// print channel values
		for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();
	}
}