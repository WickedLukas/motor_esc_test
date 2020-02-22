#include <Arduino.h>
//#include <pwmServo.h>
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
#define STOP		9

// motor pwm resolution
#define MOTOR_PWM_RES 11

// pwm frequency
#define MOTOR_PWM_FREQENCY 12000

// pwm pin to control motor
#define MOTOR_PIN 20

// object for radio control (rc)
IBUS rc;

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
	
	analogWriteFrequency(MOTOR_PIN, MOTOR_PWM_FREQENCY);
	digitalWrite(MOTOR_PIN, LOW);
	pinMode(MOTOR_PIN, OUTPUT);
	analogWriteResolution(8);
}


void loop() {
	delayMicroseconds(100);
	
	// update rc
	rc.update();
	
	uint32_t oldRes;
	noInterrupts();
	oldRes = analogWriteResolution(MOTOR_PWM_RES);
	analogWrite(MOTOR_PIN, rc_channelValue[THROTTLE]);
	analogWriteResolution(oldRes);
	interrupts();
	
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