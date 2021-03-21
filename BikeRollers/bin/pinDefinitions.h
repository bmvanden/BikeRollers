#define LOW							0
#define	HIGH						1
#define TOGGLE						2
#define INPUT						3
#define OUTPUT						4

#define RESISTOR_FWD				'D', 1
#define RESISTOR_REV				'D', 0
#define RESISTOR_PWM				'B', 1
#define RESISTOR_DUTY_CYCLE			OCR1A	// 0-255

#define MOTOR_FWD					'B', 0
#define MOTOR_REV					'D', 4
#define MOTOR_PWM					'B', 2
#define MOTOR_DUTY_CYCLE			OCR1B	// 0-255

/*#define RESISTOR_FWD				'B', 0
#define RESISTOR_REV				'D', 4
#define RESISTOR_PWM				'B', 2
#define RESISTOR_DUTY_CYCLE			OCR1B	// 0-255

#define MOTOR_FWD					'D', 1
#define MOTOR_REV					'D', 0
#define MOTOR_PWM					'B', 1
#define MOTOR_DUTY_CYCLE			OCR1A	// 0-255*/

#define BATT_V_REF					'C', 4
#define BATT_TEMP					'C', 2
#define MOT_I_REF					'C', 3

#define POT_FRICTION				'C', 1
#define POT_SLOPE					'C', 0
#define POT_MOMENTUM				'C', 7

#define SW_AUTO_MAN					'D', 7	// 1 = Auto; 0 = Manual

#define EXT_STATUS_LT				'D', 5
#define EXT_ERROR_LT				'D', 6

#define MOTOR_ENC_A					'D', 3
#define MOTOR_ENC_B					'D', 2