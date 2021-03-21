/*
 * main.c
 *
 * Created: 1/27/2021 9:46:44 PM
 *  Author: Ben
 */ 

#include <xc.h>
#include <stdbool.h>
#include "bin/pinDefinitions.h"
#include "bin/pinFunctions.h"
#include <avr/interrupt.h>

#define F_CPU						16000000UL
#include <util/delay.h>

void setPinModes();
void setPwmTimers();
void setAnalogInputs();
void setSampleTimer();
void updateMotorPID();
void updateResistorPID();
void updateAnalogInputs();

void sampleInputs();

uint8_t testSampling;
uint8_t iEncoderPos;

// Analog inputs
uint8_t currentADC;
uint8_t potSlope_steps;
uint8_t potFriction_steps;
uint8_t	potMomentum_steps;
uint8_t battTemp_steps;
uint8_t battVoltageRef_steps;
uint8_t motorCurrent_steps;	// 128 should be 0 current
uint8_t controllerTemp_steps;

#warning "Get temperature of controller from ADC8"

ISR (TIMER2_OVF_vect) {
	TCNT2 = 99;	// 100 Hz sampling	
	sampleInputs();
	updateMotorPID();
	updateResistorPID();
}

ISR (INT0_vect) {
	if (digitalRead(MOTOR_ENC_A)) {
		iEncoderPos += 1;
	} else {
		iEncoderPos -= 1;
	}
}

int main(void)
{
	/* Setup:
		- Configure PWM outputs for motor and resistor
		- Configure watchdog for cycle reset
		- Set I/O pins
	*/
	setPinModes();
	setPwmTimers();
	setAnalogInputs();
	setSampleTimer();
	sei();	
	
    while(1)
    {
		updateAnalogInputs();
		
		digitalWrite(MOTOR_REV, HIGH);
		
		//Turn on PD5 if PD4 is true
		/*if (digitalRead(SW_AUTO_MAN)) {
			digitalWrite(EXT_STATUS_LT, TOGGLE);
		} else {	
			digitalWrite(EXT_STATUS_LT, LOW);
		};
		_delay_ms(1000);*/
		
		RESISTOR_DUTY_CYCLE = motorCurrent_steps;
		
		/* Functions:
			- Scan analog inputs 
			- From encoder position array
				> Calculate current velocity and acceleration
			- Calculate target torque
				> Friction - square of velocity opposing velocity + static offset for roller friction (driving vs. regen)
				> Slope - steady in one direction (if moving?)
				> Momentum - oppose acceleration
			- PID loop to control motor torque output
			- Auto mode to follow preset pattern
		*/
    }
	
	/* Interrupts:
		- Encoder line A
		- Encoder line B
		- Watchdog timer restarts cycle
	*/
}

void setPinModes() {
	pinMode(RESISTOR_FWD, OUTPUT);
	pinMode(RESISTOR_REV, OUTPUT);
	pinMode(RESISTOR_PWM, OUTPUT);

	pinMode(MOTOR_FWD, OUTPUT);
	pinMode(MOTOR_REV, OUTPUT);
	pinMode(MOTOR_PWM, OUTPUT);

	pinMode(BATT_V_REF, INPUT);
	pinMode(BATT_TEMP, INPUT);
	pinMode(MOT_I_REF, INPUT);

	pinMode(POT_FRICTION, INPUT);
	pinMode(POT_SLOPE, INPUT);
	pinMode(POT_MOMENTUM, INPUT);

	pinMode(SW_AUTO_MAN, INPUT);

	pinMode(EXT_STATUS_LT, OUTPUT);
	pinMode(EXT_ERROR_LT, OUTPUT);
	
	pinMode(MOTOR_ENC_A, INPUT);
	pinMode(MOTOR_ENC_B, INPUT);	
	EICRA = EICRA & 0b11111100 | 0b00000011; // Rising edge triggers an interrupt for INT0
	EIMSK = EIMSK & 0b11111110 | 0b00000001; // Enable INT0 interrupt
}

void setPwmTimers(){
	// TCCR1A - Timer/Counter1 Control Registers
	TCCR1A = TCCR1A & 0b00001100 | 0b10100001; // Clear OC1A/OC1B on compare match, Fast PWM, 8-bit
	TCCR1B = TCCR1B & 0b11100000 | 0b00001010; // Fast PWM, 8-bit, no prescaling (31.3 kHz base)
}

void setAnalogInputs(){
	// Analog read setup
	DIDR0 |= 0b00111111;
	ADMUX = ADMUX & 0b00011111 | 0b01100000;	// Voltage reference from AVcc with capacitor at AREF. Left adjust ADCL and ADCH. Read only ADCH for 8-bit value.
	ADCSRA = ADCSRA & 0b00111000 | 0b10000100;	// Enable ADC converter and set prescaler to 16
	ADCSRB = ADCSRB & 0b11111000 | 0b00000100;
}

void setSampleTimer(){
	TCCR2A = TCCR2A & 0b00001100 | 0b00000011;	// Fast PWM, no output
	TCCR2B = TCCR2A & 0b11110000 | 0b00000111;	// Fast PWM, clk/1024
	TCNT2 = 99;	// 100 Hz sampling
	TIMSK2 = (1 << TOIE2);
}

void sampleInputs(){
	// Update array with encoder position
}

void updateMotorPID(){
	/*testSampling += 1;
	if (testSampling >= 100){
		testSampling = 0;
		
		if (OCR1A < 128) {
			OCR1A = 192;
			} else {
			OCR1A = 64;
		}
	}*/
	
	MOTOR_DUTY_CYCLE = potFriction_steps;
}

void updateResistorPID(){
	
}

void updateAnalogInputs(){
	if (!(ADCSRA & (1 << ADSC))) {
		// ADC conversion not running, start new conversion
		switch (currentADC){
			case 0:
				potSlope_steps = ADCH;
				currentADC = 1;
				break;
			case 1:
				potFriction_steps = ADCH;
				currentADC = 2;
				break;
			case 2:
				battTemp_steps = ADCH;
				currentADC = 3;
				break;
			case 3:
				motorCurrent_steps = ADCH;
				currentADC = 4;
				break;
			case 4:
				battVoltageRef_steps = ADCH;
				currentADC = 7;
				break;
			case 7:
				potMomentum_steps = ADCH;
				currentADC = 8;
				break;
			case 8:
				controllerTemp_steps = ADCH;
				currentADC = 0;
				break;
			default:
				currentADC = 0;
				break;
		}
			
		ADMUX = ADMUX & 0b11110000 | currentADC; // Set ADC channel to scan
		ADCSRA |= (1 << ADSC); // Start conversion again
	}	
}