
//------------------------------------- Pins -------------------------------------------------------------

#define LED_PIN				13

#define BUTTON_0			A0
#define BUTTON_1			A1
#define BUTTON_2			A2

#define	PWM_PINA			9  // Timer1 PWM pin A
#define	DIR_PINA			12
#define	REVERSE_DIR_PINA	11
#define	PWM_PINB			10  //Timer1 PWM pin B
#define	DIR_PINB			8 //7
#define	REVERSE_DIR_PINB	13 //6

#define EncodAilPin1  6
#define EncodAilPin2  2
#define EncodProfPin1 2
#define EncodProfPin2 3

#define TX1 7 //rose -> marron RX1 PA4
#define RX1 6 //violet -> violet TX1 PA5

// --- FFB / PWM

int MM_MIN_MOTOR_TORQUE=200;
int MM_MAX_MOTOR_TORQUE=409;

#define MM_MAX_MOTOR  400			// Set 20khz PWM