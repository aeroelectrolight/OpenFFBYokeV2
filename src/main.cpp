#include "Arduino.h"
#include "Config.h"
//#include "Wire.h"
//#include "pwm.cpp"
#include "Joystick.h"
#include <Encoder.h>
//#include <RotaryEncoderAdvanced.h>
#include <digitalWriteFast.h>
#include <EEPROM.h>
//#include <SoftwareSerial.h>
//#include "SerialTransfer.h"
//#include <SoftEasyTransfer.h>

void calibration();

//X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 12, 0,
  true, true, false, //X,Y,Z
  true, true, false,//Rx,Ry,Rz
  false, false, false, false, false);

// FFB
Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};
int maxMinX=400,maxMinY=300,zeroX=180,zeroY=180;

// Last state of the button
//int lastButtonState[4] = {0,0,0,0};
//int pinToButtonMap = 0;

// encodeur 
int zero;
#define BUTTON  5 //ky-040 sw  pin, interrupt & add 100nF/0.1uF capacitors between pin & ground!!!
int16_t ailerons, aileronsAnalog ,aileronsMax,aileronsMin,elevator, elevatorEncod ,elevatorMax = 1481,elevatorMin = -1479, mapForce;
boolean serialTest = false;
//RotaryEncoderAdvanced<int> YokeEncAil(EncodAilPin1, EncodAilPin2, BUTTON,1,0,4000);
//RotaryEncoderAdvanced<int> YokeEncProf(EncodProfPin1, EncodProfPin2, BUTTON,1,-2000,2000);
//Encoder YokeEncAil(EncodAilPin1,EncodAilPin2);
Encoder YokeEncProf(EncodProfPin1,EncodProfPin2);

#define HALLPIN A2
/*
void encoderISRA()                                            //interrupt service routines need to be in ram
{
  YokeEncAil.readAB();
}
void encoderISRP()                                            //interrupt service routines need to be in ram
{
  YokeEncProf.readAB();
}
*/
//SoftwareSerial mySerial(RX1, TX1);
//SoftEasyTransfer ET; 
//SerialTransfer myTransfer;
byte transmit[6], cpt = 0, start = 0;
struct STRUCT {
  int16_t axe_x;
  int16_t axe_y;
  uint16_t buttons;
} data;

//uint16_t axe_x, axe_y, buttons;
//byte transmit[6];
// Sync Byte flag
//boolean syncByteFound = 0,transmitOK = 0;
//unsigned char START_BYTE = 0x53; // ASCII "S"
//unsigned char STOP_BYTE = 0x52; // ASCII "S"



/**************************
 * Protocole
 */
unsigned char START_BYTE = 0x53; // ASCII "S"
unsigned char STOP_BYTE = 0x52; // ASCII "S"
 
boolean isStart(byte val){
   if (val == START_BYTE){
    return true;
   }else{
    return false;
   }
}

boolean isEnd(byte val){
   if (val == STOP_BYTE){
    return true;
   }else{
    return false;
   }
}
/****************
 * 
 */
void decode(){
  uint16_t tempo = 0;
  tempo = transmit[1] << 8 | transmit[0];
  data.axe_x = map(tempo, 0, 4096,0,1023 );
  tempo = transmit[3] << 8 | transmit[2];
  data.axe_y = map(tempo, 0, 4096,0,1023 );
  data.buttons = transmit[5] << 8 | transmit[4];
}

void receive(){
  //cli();
  //Wire.requestFrom(8, 6);    // request 6 bytes from peripheral device #8
  //int i = 0;
  //while (Wire.available()) { // peripheral may send less than requested
  //for(int i=0; i<6; i++){
    //transmit[i] = Wire.read();
    //Serial.print(transmit[i]);         // print the character
    //i++;
    //char c = Wire.read(); // receive a byte as character
  //}

  if (Serial1.available()) {
      byte inByte = Serial1.read();
      if(isStart(inByte) && start == 0){
        cpt = 0;
        start = 1;
        //Serial.println("start");
      }else if(isEnd(inByte) && cpt > 5){
        //Serial.println("end");
        decode();
        start = 0;
        cpt = 0;
      }else if(cpt > 5){
        //Serial.println("end");
        start = 0;
        cpt = 0;
      }else{
        transmit[cpt] = inByte;
        cpt++;
      }
  }
  //sei();
}




void encodCenter(){
  //YokeEncAil.write(0);
  //YokeEncProf.write(0);
}

void testButton(){
  // Read pin values
  
  //bool Bvalue = 0;
  
  for (uint8_t i = 0; i < 10; i++)
  {
    //Bvalue = bitRead(buttons,i);
    Joystick.setButton(i, bitRead(data.buttons,i));
  }
  
  // boutton de remise à zero du volant
  int currentButtonState;
  currentButtonState = !digitalRead(BUTTON_0);
  if(currentButtonState){
    if(bitRead(data.buttons,4)){
      calibration();
    }
    encodCenter();
    delay(50);
  }
}


void readEncoder(){
  //long position = YokeEncAil.read();
  long position = analogRead(HALLPIN);
  if(serialTest)Serial.println(position);
  //Serial.println(position);
  if(position != aileronsAnalog){
    ailerons = map(position, 155,880,-1023,1023 );
    //Serial.print("X : ");
    //Serial.println(position);
    aileronsAnalog = position;
  }
  position = YokeEncProf.read();
  if(position != elevatorEncod){
    //Serial.print("Y : ");
    //Serial.println(position);
    if (position < 0)
    {
      elevator = map(position, elevatorMin,-1,-1023,0 );
    }
    else
    {
      elevator = map(position, 0, elevatorMax, 0, 1023);
    }
    elevatorEncod = position;
    //Serial.println(elevator);
  }
}

void InitPWM ()
{
	// Define pinMode for the pins and set the frequency for timer1.
	pinMode(DIR_PINA,OUTPUT);
	pinMode(REVERSE_DIR_PINA, OUTPUT);
	pinMode(PWM_PINA,OUTPUT);
	pinMode(DIR_PINB,OUTPUT);
	pinMode(REVERSE_DIR_PINB, OUTPUT);
	pinMode(PWM_PINB,OUTPUT);

	// Timer 1 configuration : prescaler: clockI/O / 1
	// outputs enabled, phase-correct PWM, top of 400
	// PWM frequency calculation : 16MHz / 1 (prescaler) / 2 (phase-correct) / 1000 (top) = 8 kHz
	// PWM frequency calculation : 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20 kHz
	TCCR1A = 0b10100000;
	TCCR1B = 0b00010001;//b00010001;
	ICR1 = MM_MAX_MOTOR;

}

void setPWMDir (int16_t torque, bool pin)		// torque between -PWM_TOP and +PWM_TOP
{
	uint8_t dir = 1,rev = 0;

 	torque = -torque;
	if (torque < 0)
	{
		torque = -torque;
		dir = 0; 
		rev = 1;
	}
	if (pin)
	{
		OCR1A = constrain(torque,0,MM_MAX_MOTOR_TORQUE - 10);
		digitalWriteFast(DIR_PINA,dir);
		digitalWriteFast(REVERSE_DIR_PINA, rev);
	}else{
		OCR1B = constrain(torque,0,MM_MAX_MOTOR_TORQUE - 10);
		digitalWriteFast(DIR_PINB,dir);
		digitalWriteFast(REVERSE_DIR_PINB, rev);
	}
	//DEBUG_SERIAL.println(temp);
}

void setPWM (int16_t torque, bool pin)		// torque between -PWM_TOP and +PWM_TOP
{
	//torque = (torque + MM_MAX_MOTOR_TORQUE) >> 1;
	torque = constrain(torque,0,MM_MAX_MOTOR_TORQUE - 10);
	if (pin){
		OCR1A = torque;
	}else{
		OCR1B = torque;
	}
	
}

// fonction de calibration

void calibration(){
	bool test = true;
	//long position;
  Serial.println("center");
  while(test){
		//position = YokeEncAil.read();
		if(!digitalRead(BUTTON_0)){
			test = false;
			Serial.println("center valide");
      encodCenter();
      YokeEncProf.write(1432);
		}
	}
  /*
  Serial.println("min ailerons");
	delay(1000);
  test = true;
	while(test){
		position = YokeEncAil.read();
		if(!digitalRead(BUTTON_0)){
			test = false;
			Serial.println("min ailerons valide");
		}
		aileronsMin = position;
	}
  Serial.println("max ailerons");
	delay(1000);
	test = true;
	while(test){
		position = YokeEncAil.read();
		if(!digitalRead(BUTTON_0)){
			test = false;
      Serial.println("max ailerons valide");
		}
		aileronsMax = position;
	}
	zero = (aileronsMin + aileronsMax)/2;
	Serial.println("min profondeur");
	delay(1000);
  test = true;
	while(test){
		position = YokeEncProf.read();
		if(!digitalRead(BUTTON_0)){
			test = false;
			Serial.println("min profondeur valide");
		}
		elevatorMin = position;
	}
  Serial.println("max profondeur");
	delay(1000);
	test = true;
	while(test){
		position = YokeEncProf.read();
		if(!digitalRead(BUTTON_0)){
			test = false;
      Serial.println("max profondeur valide");
		}
		elevatorMax = position;
	}
  */
	delay(1000);
}

void readMyGains(){
  Serial.println("Valeurs des gains :");
  Serial.print("T totalGain X : ");
  Serial.println(mygains[0].totalGain);
  Serial.print("S springGain X : ");
  Serial.println(mygains[0].springGain);
  Serial.print("D damperGain X : ");
  Serial.println(mygains[0].damperGain);
  Serial.print("F frictionGain X : ");
  Serial.println(mygains[0].frictionGain);
  Serial.print("I inertiaGain X : ");
  Serial.println(mygains[0].inertiaGain);
  Serial.print("t totalGain Y : ");
  Serial.println(mygains[1].totalGain);
  Serial.print("s springGain Y : ");
  Serial.println(mygains[1].springGain);
  Serial.print("d damperGain Y : ");
  Serial.println(mygains[1].damperGain);
  Serial.print("f frictionGain Y : ");
  Serial.println(mygains[1].frictionGain);
  Serial.print("i inertiaGain Y : ");
  Serial.println(mygains[1].inertiaGain);
}

void readValMaxMin(){
  Serial.println("Valeurs des Min et Max :");
  Serial.print("A valeur max/min  X : ");
  Serial.println(maxMinX);
  Serial.print("B valeur zero X : ");
  Serial.println(zeroX);
  Serial.print("a valeur max/min Y : ");
  Serial.println(maxMinY);
  Serial.print("b valeur zero Y : ");
  Serial.println(zeroY);
  Serial.print("Tap Z for record ! ");
}

void recordXYVal(){
  int adresse = 0;
  EEPROM.put(adresse, maxMinX);
  adresse += sizeof(maxMinX);
  EEPROM.put(adresse, zeroX);
  adresse += sizeof(zeroX);
  EEPROM.put(adresse, maxMinY);
  adresse += sizeof(maxMinY);
  EEPROM.put(adresse, zeroY);
  adresse += sizeof(zeroY);
}

void getXYVal(){
  int adresse = 0;
  EEPROM.get(adresse, maxMinX);
  adresse += sizeof(maxMinX);
  EEPROM.get(adresse, zeroX);
  adresse += sizeof(zeroX);
  EEPROM.get(adresse, maxMinY);
  adresse += sizeof(maxMinY);
  EEPROM.get(adresse, zeroY);
  adresse += sizeof(zeroY);
}

int readVal(){
  Serial.println("indiquer la valeur");
  while(Serial.available() == 0);
  Serial.println("OK");
  return((int)Serial.parseInt());
}

void RWSerial(){
  if (Serial.available())
	{
		u8 c = Serial.read();
		switch(c){
      case 'H':
        readMyGains();
        break;
      case 'h':
        readValMaxMin();
        break;
      case 'A':
				maxMinX = readVal();
				break;
      case 'B':
				zeroX = readVal();
				break;
      case 'a':
				maxMinY = readVal();
				break;
      case 'b':
				zeroY = readVal();
				break;
			case 'T':
				mygains[0].totalGain = readVal();
				break;
			case 'S':
				mygains[0].springGain = readVal();
				break;
			case 'D':;
				mygains[0].damperGain = readVal();
				break;
			case 'F':
				mygains[0].frictionGain = readVal();
				break;
      case 'I':
				mygains[0].inertiaGain = readVal();
				break;
      case 't':
				mygains[1].totalGain = readVal();
				break;
			case 's':
				mygains[1].springGain = readVal();
				break;
			case 'd':;
				mygains[1].damperGain = readVal();
				break;
			case 'f':
				mygains[1].frictionGain = readVal();
				break;
      case 'i':
				mygains[1].inertiaGain = readVal();
				break;
      case 'p':
        if(serialTest){
          serialTest = false;
        }else{
          serialTest = true;
        }
      case 'Z':
				recordXYVal();
				break;
      default :
        Serial.println("Tap H or h for read variable !");
        Serial.println("Tap p for Serial Test");
        break;
		}
    delay(250); // for antirebond
	}
}

void setup(){
    pinMode(BUTTON_0,INPUT_PULLUP);
    //pinMode(BUTTON_1,INPUT_PULLUP);
    //pinMode(BUTTON_2,INPUT_PULLUP);
    //YokeEncAil.begin();
    //YokeEncProf.begin();

    //attachInterrupt(digitalPinToInterrupt(0), encoderISRA, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(1), encoderISRP, CHANGE);
    // set X and Y axis range 
    Joystick.setXAxisRange(-1023, 1023);
    Joystick.setYAxisRange(-1023, 1023);

    //set X Axis gains
    mygains[0].totalGain = 100;
    mygains[0].constantGain = 100;
    mygains[0].rampGain = 100;
    mygains[0].squareGain = 100;
    mygains[0].sineGain = 100;
    mygains[0].triangleGain = 100;
    mygains[0].sawtoothdownGain = 100;
    mygains[0].sawtoothupGain = 100;
    mygains[0].springGain = 100;
    mygains[0].damperGain = 100;
    mygains[0].inertiaGain = 100;
    mygains[0].frictionGain = 100;
    mygains[0].customGain = 100;


    //set Y Axis gains
    mygains[1].totalGain = 100;
    mygains[1].constantGain = 100;
    mygains[1].rampGain = 100;
    mygains[1].squareGain = 100;
    mygains[1].sineGain = 100;
    mygains[1].triangleGain = 100;
    mygains[1].sawtoothdownGain = 100;
    mygains[1].sawtoothupGain = 100;
    mygains[1].springGain = 100;
    mygains[1].damperGain = 100;
    mygains[1].inertiaGain = 100;
    mygains[1].frictionGain = 100;
    mygains[1].customGain = 100;
    //enable gains REQUIRED
    Joystick.setGains(mygains);

    // init pwm
    getXYVal();
    InitPWM();
    setPWM(0,false);

    Serial.begin(115200);
    Joystick.begin();

    calibration();


    // begin i2c on adress 9
    //Wire.begin();
    Serial1.begin(115200);
    //ET.begin(details(data), &mySerial);
    //myTransfer.begin(mySerial);
}

void loop(){

  RWSerial();
  //ET.receiveData();

  receive();
  ////////decode();
  //delay(200);

  readEncoder();

  testButton();


  //set X Axis Spring Effect Param
  myeffectparams[0].springMaxPosition = 1023;
  myeffectparams[0].springPosition = ailerons;
  myeffectparams[0].damperMaxVelocity = 1023;
  myeffectparams[0].damperVelocity = ailerons;
  myeffectparams[0].inertiaMaxAcceleration = 1023;
  myeffectparams[0].inertiaAcceleration = ailerons;
  myeffectparams[0].frictionMaxPositionChange = 1023;
  myeffectparams[0].frictionPositionChange = ailerons;

  
  
  //set Y Axis Spring Effect Param
  myeffectparams[1].springMaxPosition = 1023;
  myeffectparams[1].springPosition = elevator;
  myeffectparams[1].damperMaxVelocity = 1023;
  myeffectparams[1].damperVelocity = elevator;
  myeffectparams[1].inertiaMaxAcceleration = 1023;
  myeffectparams[1].inertiaAcceleration = elevator;
  myeffectparams[1].frictionMaxPositionChange = 1023;
  myeffectparams[1].frictionPositionChange = elevator;

  //Send HID data to PC
  Joystick.setXAxis(ailerons);
  Joystick.setYAxis(elevator);
  Joystick.setRxAxis(data.axe_x);
  Joystick.setRyAxis(data.axe_y);

  
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);

  //Serial.println("forces[1]");
  //Serial.println(forces[1]);
  
  byte zero1 = 0;
  if (forces[0] < zero1 )
  {
    mapForce = map(forces[0],-255,zero1,-maxMinX,-zeroX);
  } 
  else
  {
    mapForce = map(forces[0],zero1,255,zeroX,maxMinX);
  }
  //Serial.println("mapForce");
  //Serial.println(mapForce);
  setPWMDir(mapForce,true);
  if (forces[1] < zero1 )
  {
    mapForce = map(forces[1],-255,zero1,-maxMinY,-zeroY);
  } 
  else
  {
    mapForce = map(forces[1],zero1,255,zeroY,maxMinY);
  }
  //Serial.println(mapForce);
  setPWMDir(mapForce,false);

  delay(1);
}
