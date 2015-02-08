/*

MSE 2202 MSEBot base code for Labs 3 and 4
Language: Arduino
Authors: Michael Naish and Eugen Porter
Date: 15/01/18

Rev 1 - Initial version

*/

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_GripMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 10;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 1900;
const int ci_Grip_Motor_Closed = 1250;
const int ci_Grip_Motor_Neutral = 1500;
const int ci_Arm_Servo_Retracted = 70;      //  "
const int ci_Arm_Servo_Extended = 110;      //  "
const int ci_Arm_Servo_Mid = 95;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 50;   // May need to adjust this
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned long ui_Left_Motor_Speed;
unsigned long ui_Right_Motor_Speed;
unsigned long ul_Left_Motor_Position;
unsigned long ul_Right_Motor_Position;
unsigned long ul_Grip_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

// debugging variables
byte serialBuffer;
int debugValue = 1500;

// boolean for quicker access of binary line tracker readings
boolean leftOnLine;
boolean middleOnLine;
boolean rightOnLine;

// line following
long confidence = 0;
long confidenceIncrement = 20;
long confidenceMin = 200;
long confidenceMax = 600;

byte direction = 0;

// variable for open loop functions
boolean loopStarted = true;
long leftEncoderStopTime = 0;
long rightEncoderStopTime = 0;
const int leftEncoderCalibration = 2800;
const int rightEncoderCalibration = 2800;

// stage variables
int stage = 0;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
	0x00,    //B0000000000000000,  //Stop
	0x00FF,  //B0000000011111111,  //Run
	0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
	0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
	0xAAAA,  //B1010101010101010,  //Calibrate motors
	0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536 };
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

void setup() {
	Wire.begin();	      // Wire library required for I2CEncoder library
	Serial.begin(9600);

	CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
		ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

	// set up ultrasonic
	pinMode(ci_Ultrasonic_Ping, OUTPUT);
	pinMode(ci_Ultrasonic_Data, INPUT);

	// set up drive motors
	pinMode(ci_Right_Motor, OUTPUT);
	servo_RightMotor.attach(ci_Right_Motor);
	pinMode(ci_Left_Motor, OUTPUT);
	servo_LeftMotor.attach(ci_Left_Motor);

	// set up arm motors
	pinMode(ci_Arm_Motor, OUTPUT);
	servo_ArmMotor.attach(ci_Arm_Motor);
	pinMode(ci_Grip_Motor, OUTPUT);
	servo_GripMotor.attach(ci_Grip_Motor);
	servo_GripMotor.write(ci_Grip_Motor_Neutral);

	// set up motor enable switch
	pinMode(ci_Motor_Enable_Switch, INPUT);

	// set up encoders. Must be initialized in order that they are chained together, 
	// starting with the encoder directly connected to the Arduino. See I2CEncoder docs
	// for more information
	encoder_LeftMotor.init(1.0 / 3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
	encoder_RightMotor.init(1.0 / 3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
	encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
	encoder_GripMotor.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

	// set up line tracking sensors
	pinMode(ci_Right_Line_Tracker, INPUT);
	pinMode(ci_Middle_Line_Tracker, INPUT);
	pinMode(ci_Left_Line_Tracker, INPUT);
	ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

	// read saved values from EEPROM
	b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
	ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
	b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
	ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
	b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
	b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
	ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
	if ((millis() - ul_3_Second_timer) > 3000)
	{
		bt_3_S_Time_Up = true;
	}
	// button-based mode selection
	if (CharliePlexM::ui_Btn)
	{
		if (bt_Do_Once == false)
		{
			bt_Do_Once = true;
			ui_Robot_State_Index++;
			ui_Robot_State_Index = ui_Robot_State_Index & 7;
			ul_3_Second_timer = millis();
			bt_3_S_Time_Up = false;
			bt_Cal_Initialized = false;
		}
	}
	else
	{
		bt_Do_Once = LOW;
	}

	// check if drive motors should be powered
	bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

	// modes 
	// 0 = default after power up/reset
	// 1 = Press mode button once to enter. Run robot.
	// 2 = Press mode button twice to enter. Calibrate line tracker light level.
	// 3 = Press mode button three times to enter. Calibrate line tracker dark level.
	// 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
	switch (ui_Robot_State_Index)
	{

	case 0:    //Robot stopped
	{
		readLineTrackers();
		Ping();
		servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
		servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
		retractArm();

		// default value is true, this causes claw to opened open and then motor off to conserve power.
		if (loopStarted)
		{
			openClaw();
			delay(500);
		}
		else
		{
			apatheticClaw();
		}

		// code showing testing of claw opening
		if (Serial.available())
		{
			serialBuffer = Serial.read();

			if (serialBuffer == '0')
			{
				debugValue = debugValue - 30;
			}
			else if (serialBuffer == '1')
			{
				debugValue = debugValue + 30;
			}

			Serial.println(debugValue);
			servo_GripMotor.writeMicroseconds(debugValue);
		}

		encoder_LeftMotor.zero();
		encoder_RightMotor.zero();
		encoder_GripMotor.zero();

		ui_Mode_Indicator_Index = 0;

		// Serial.println(analogRead(ci_Light_Sensor));

		break;
	}

	case 1:    //Robot Run after 3 seconds
	{
		if (bt_3_S_Time_Up)
		{
#ifdef DEBUG_ENCODERS           
			ul_Left_Motor_Position = encoder_LeftMotor.getPosition();
			ul_Right_Motor_Position = encoder_RightMotor.getPosition();
			ul_Grip_Motor_Position = encoder_GripMotor.getPosition();

			Serial.print("Encoders L: ");
			Serial.print(encoder_LeftMotor.getPosition());
			Serial.print(", R: ");
			Serial.print(encoder_RightMotor.getPosition());
			Serial.print(", G: ");
			Serial.println(ul_Grip_Motor_Position, DEC);
#endif
			/***************************************************************************************
			* 			Add line tracking code here.
			* 			Adjust motor speed according to information from line tracking sensors and
			* 			possibly encoder counts.
			/*************************************************************************************/

			readLineTrackers();

			switch (stage)
			{
			case 0:
				/*
				Position: before 1st stop
				What Doing: following line
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
						stage++;
				}
				else
					followLine();
				break;
			case 1:
				/*
				Position: 1st stop
				What Doing: go straight
				When to Increment Stage: not 111
				*/
				if (!atStop())
				{
					if (!checkedAtStop(1))
					stage++;
				}
				else
					goForward(confidence);
				break;
			case 2:
				/*
				Position: before 2nd stop
				What Doing: fl
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
						stage++;
				}
				else
					followLine();
				break;
			case 3:
				/*
				Position: 2nd stop
				What Doing: go straight
				When to Increment Stage: not 111
				*/
				if (!atStop())
				{
					if (!checkedAtStop(1))
						stage++;
				}
				else
					goForward(confidence);
				break;
			case 4:
				/*
				Position: before 3rd stop
				What Doing: fl
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
						stage++;
				}
				else
					followLine();
				break;
			case 5:
				/*
				Position:3rd stop
				What Doing: s
				When to Increment Stage: not 111
				*/
				if (!atStop())
				{
					if (!checkedAtStop(1))
						stage++;
				}
				else
					goForward(confidence);
				break;
			case 6:
				/*
				Position: before 4th stop
				What Doing: fl
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
					{
						halt();
						stage++;
					}
				}
				else
					followLine();
				break;
			case 7:
				/*
				Position: 4th stop
				What Doing: turning right approximately on spot to be on left side of light
				When to Increment Stage: done turning right
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					calcRightTurn(2800, 120);
					turnRight(480); // same turn speed as before
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 8:
				/*
				Position: 4th stop,  done turning right
				What Doing: extending arm and opening claw early to save time
				When to Increment Stage: started extending arm and opening claw
				*/
				extendArm();
				openClaw();
				stage++;
				break;
			case 9:
				/*
				Position: done turning right, arm extend, and claw opened
				What Doing: moving forward toward the beacon
				When to Increment Stage: done moving toward the beacon
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcRightTurn(2800, 120);

					goForward(400); // double speed as before
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 10:
				/*
				Position: in front of beacon, not aligned
				What Doing: turning right until aligned with the beacon
				When to Increment Stage: aligned with the beacon
				*/
				if (analogRead(ci_Light_Sensor) < 100)
				{
					halt();
					stage++;
				}
				else
					turnRightOnSpot(240); // double speed as before
				break;
			case 11:
				/*
				Position: in front of beacon, aligned
				What Doing: moving forward until 5cm or less away from the beacon
				When to Increment Stage: 5cm or less away from the beacon
				*/
				// sensor still not understood
				int x;
				x = sensorDistance();
				Ping();


				if (x < 5)
				{
					halt();
					stage++;
				}
				else
				{
					goForward(200); // same speed as before
				}
				break;
			case 12:
				/*
				Position: 5cm from beacon
				What Doing: closing the claw around the beacon, waiting, and then lifting the arm
				When to Increment Stage: arm lift started
				*/
				closeClaw();
				delay(1200); // minimum value needs to be found
				liftArm();
				stage++;
				break;
			case 13:
				/*
				Position: 5cm from beacon, got beacon
				What Doing: move backward ontop section after branch 3
				When to Increment Stage: when 110 or 011 seen twice in a row
				*/
				if ((leftOnLine && middleOnLine) || (middleOnLine && rightOnLine))
				{
					readLineTrackers();

					// stops going backward if line detected twice
					if ((leftOnLine && middleOnLine) || (middleOnLine && rightOnLine))
						stage++;
				}
				else
					veerRightBackward(240,0); // a little bit faster than before
				break;
			case 14:
				/*
				Position: section after branch 3
				What Doing: go forward a bit to get off line
				When to Increment Stage: done going forward
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcRightTurn(2800, 15);
					goForward(200); // same speed
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 15:
				/*
				Position: re-capturing line
				What Doing: re-capturing line
				When to Increment Stage: line recaptured (left line tracking is on the line)
				*/
				if (leftOnLine)
					stage++;
				else
					veerRight(100, 200); // slower than before to increasing angle and ensure line recapturing
				break;
			case 16:
				/*
				Position: after branch 3, line re-captured
				What Doing: following line
				When to Increment Stage: when branch detected
				*/
				if (atBranch())
				{
					halt();
					stage++;
				}
				else
					followLine();
				break;
			case 17:
				/*
				Position: at branch 3, line re-captured
				What Doing: going forward set distance
				When to Increment Stage: done going forward
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcRightTurn(2800, 15); // further
					goForward(300); // 1.5x speed
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 18:
				/*
				Position: before branch 3
				What Doing: veering right
				When to Increment Stage: line recaptured (left line tracking is on the line)
				*/
				if (leftOnLine)
				{
					stage++;
				}
				else
				{
					veerRight(200,80);
				}
				break;
			case 19:
				/*
				Position: after 3rd stop
				What Doing: fl
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
						stage++;
				}
				else
					followLine();
				break;
			case 20:
				/*
				Position:3rd stop
				What Doing: s
				When to Increment Stage: not 111
				*/
				if (!atStop())
				{
					if (!checkedAtStop(1))
						stage++;
				}
				else
					goForward(confidence);
				break;
			case 21:
				/*
				Position: after branch 2
				What Doing: following line
				When to Increment Stage: when branch detected
				*/
				if (atBranch())
				{
					halt();
					stage++;
				}
				else
					followLine();
				break;
			case 22:
				/*
				Position: at branch 2
				What Doing: going forward set distance
				When to Increment Stage: done going forward
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcRightTurn(2800, 15); // further
					goForward(300); // 1.5x speed
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 23:
				/*
				Position: before branch 3
				What Doing: veering left
				When to Increment Stage: line recaptured (left line tracking is on the line)
				*/
				if (rightOnLine)
				{
					stage++;
				}
				else
				{
					veerLeft(200, 80);
				}
				break;
			case 24:
				/*
				Position: after 2nd stop
				What Doing: fl
				When to Increment Stage: see 111
				*/
				if (atStop())
				{
					if (checkedAtStop(1))
						stage++;
				}
				else
					followLine();
				break;
			case 25:
				/*
				Position:2nd stop
				What Doing: s
				When to Increment Stage: not 111
				*/
				if (!atStop())
				{
					if (!checkedAtStop(1))
						stage++;
				}
				else
					goForward(confidence);
				break;
			case 26:
				/*
				Position: after branch 1
				What Doing: following line
				When to Increment Stage: when branch detected
				*/
				if (atBranch())
				{
					halt();
					stage++;
				}
				else
					followLine();
				break;
			case 27:
				/*
				Position: at branch 1
				What Doing: going forward set distance
				When to Increment Stage: done going forward
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcRightTurn(2800, 15); // further
					goForward(300); // 1.5x speed
				}
				else if (doneRightTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 28:
				/*
				Position: before branch 1
				What Doing: veering left
				When to Increment Stage: line recaptured (right line tracking is on the line)
				*/
				if (rightOnLine)
				{
					stage++;
				}
				else
				{
					veerLeft(200, 80);
				}
				break;
			case 29:
				/*
				Position: after stop 1
				What Doing: following line
				When to Increment Stage: see 111 (once?)
				*/
				if (atStop())
				{
					halt();
					stage++;
				}
				else
					followLine();
				break;
			case 30:
				/*
				Position: stop 1
				What Doing: lefting left 90?
				When to Increment Stage: done turning left
				*/
				if (!loopStarted)
				{
					loopStarted = true;
					halt();
					calcLeftTurn(2800, 45); // halved because on stop, tweak
					turnLeftOnSpot(240);
				}
				else if (doneLeftTurn())
				{
					halt();
					loopStarted = false;
					stage++;
				}
				break;
			case 31:
				/*
				Position: in front of drop-box box, aligned hopefully
				What Doing: drive forward slowly, extending arm, amd opening claw
				When to Increment Stage: done opening claw
				*/
				goForward(100);
				delay(200);
				extendArm();
				delay(200);
				openClaw();
				delay(1200);
				stage++;
				break;
			case 32:
				/*
				Position:  done opening claw, potentially with beacon still in claw
				What Doing: shaking arm
				When to Increment Stage: done shaking arm
				*/
				shakeArm();
				stage++;
			default:
				loopStarted = true; // causes mode 0 to reset arm and claw
				ui_Robot_State_Index = 0; // enter mode 0
			}

#ifdef DEBUG_MOTORS  
			Serial.print("Motors: Default: ");
			Serial.print(ui_Motors_Speed);
			Serial.print(" , Left = ");
			Serial.print(ui_Left_Motor_Speed);
			Serial.print(" . Right = ");
			Serial.println(ui_Right_Motor_Speed);
#endif    
			ui_Mode_Indicator_Index = 1;
		}
		break;
	}

	case 2:    //Calibrate line tracker light levels after 3 seconds
	{
		if (bt_3_S_Time_Up)
		{
			if (!bt_Cal_Initialized)
			{
				bt_Cal_Initialized = true;
				ui_Left_Line_Tracker_Light = 0;
				ui_Middle_Line_Tracker_Light = 0;
				ui_Right_Line_Tracker_Light = 0;
				ul_Calibration_Time = millis();
				ui_Cal_Count = 0;
			}
			else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
			{
				ul_Calibration_Time = millis();
				readLineTrackers();
				ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
				ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
				ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
				ui_Cal_Count++;
			}
			if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
			{
				ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
				ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
				ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
				Serial.print("Light Levels: Left = ");
				Serial.print(ui_Left_Line_Tracker_Light, DEC);
				Serial.print(", Middle = ");
				Serial.print(ui_Middle_Line_Tracker_Light, DEC);
				Serial.print(", Right = ");
				Serial.println(ui_Right_Line_Tracker_Light, DEC);
#endif           
				EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
				EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
				EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
				EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
				EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
				EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
				ui_Robot_State_Index = 0;    // go back to Mode 0
			}
			ui_Mode_Indicator_Index = 2;
		}
		break;
	}

	case 3:    // Calibrate line tracker dark levels after 3 seconds
	{
		if (bt_3_S_Time_Up)
		{
			if (!bt_Cal_Initialized)
			{
				bt_Cal_Initialized = true;
				ui_Left_Line_Tracker_Dark = 0;
				ui_Middle_Line_Tracker_Dark = 0;
				ui_Right_Line_Tracker_Dark = 0;
				ul_Calibration_Time = millis();
				ui_Cal_Count = 0;
			}
			else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
			{
				ul_Calibration_Time = millis();
				readLineTrackers();
				ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
				ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
				ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
				ui_Cal_Count++;
			}
			if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
			{
				ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
				ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
				ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
				Serial.print("Dark Levels: Left = ");
				Serial.print(ui_Left_Line_Tracker_Dark, DEC);
				Serial.print(", Middle = ");
				Serial.print(ui_Middle_Line_Tracker_Dark, DEC);
				Serial.print(", Right = ");
				Serial.println(ui_Right_Line_Tracker_Dark, DEC);
#endif           
				EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
				EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
				EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
				EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
				EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
				EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
				ui_Robot_State_Index = 0;    // go back to Mode 0
			}
			ui_Mode_Indicator_Index = 3;
		}
		break;
	}

	case 4:    //Calibrate motor straightness after 3 seconds.
	{
		if (bt_3_S_Time_Up)
		{
			if (!bt_Cal_Initialized)
			{
				bt_Cal_Initialized = true;
				encoder_LeftMotor.zero();
				encoder_RightMotor.zero();
				ul_Calibration_Time = millis();
				servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
				servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
			}
			else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
			{
				servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
				servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
				ul_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
				ul_Right_Motor_Position = encoder_RightMotor.getRawPosition();
				if (ul_Left_Motor_Position > ul_Right_Motor_Position)
				{
					// May have to update this if different calibration time is used
					ui_Right_Motor_Offset = (ul_Left_Motor_Position - ul_Right_Motor_Position) / 3.43;
					ui_Left_Motor_Offset = 0;
				}
				else
				{
					// May have to update this if different calibration time is used
					ui_Right_Motor_Offset = 0;
					ui_Left_Motor_Offset = (ul_Right_Motor_Position - ul_Left_Motor_Position) / 3.43;
				}

#ifdef DEBUG_MOTOR_CALIBRATION
				Serial.print("Motor Offsets: Right = ");
				Serial.print(ui_Right_Motor_Offset);
				Serial.print(", Left = ");
				Serial.println(ui_Left_Motor_Offset);
#endif              
				EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
				EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
				EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
				EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

				ui_Robot_State_Index = 0;    // go back to Mode 0 
			}
#ifdef DEBUG_ENCODERS           
			Serial.print("Encoders L: ");
			Serial.print(encoder_LeftMotor.getRawPosition());
			Serial.print(", R: ");
			Serial.println(encoder_RightMotor.getRawPosition());
#endif        
			ui_Mode_Indicator_Index = 4;
			}
		break;
		}
	}

	if ((millis() - ul_Display_Time) > ci_Display_Time)
	{
		ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY  
		Serial.print("Mode: ");
		Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
		bt_Heartbeat = !bt_Heartbeat;
		CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
		digitalWrite(13, bt_Heartbeat);
		Indicator();
	}
	}

// set mode indicator LED state
void Indicator()
{
	//display routine, if true turn on led
	CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
		(iArray[iArrayIndex])));
	iArrayIndex++;
	iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
	ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
	ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
	ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

	if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
	{
		CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
		leftOnLine = true;
	}
	else
	{
		CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
		leftOnLine = false;
	}
	if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
	{
		CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
		middleOnLine = true;
	}
	else
	{
		CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
		middleOnLine = false;
	}
	if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
	{
		CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
		rightOnLine = true;
	}
	else
	{
		CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
		rightOnLine = false;
	}

#ifdef DEBUG_LINE_TRACKERS
	Serial.print("Trackers: Left = ");
	Serial.print(ui_Left_Line_Tracker_Data, DEC);
	Serial.print(", Middle = ");
	Serial.print(ui_Middle_Line_Tracker_Data, DEC);
	Serial.print(", Right = ");
	Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif

}

// measure distance to target using ultrasonic sensor  
void Ping()
{
	//Ping Ultrasonic
	//Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
	digitalWrite(ci_Ultrasonic_Ping, HIGH);
	delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
	digitalWrite(ci_Ultrasonic_Ping, LOW);
	//use command pulseIn to listen to Ultrasonic_Data pin to record the
	//time that it takes from when the Pin goes HIGH until it goes LOW 
	ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);
	// Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
	Serial.print("Time (microseconds): ");
	Serial.print(ul_Echo_Time, DEC);
	Serial.print(", Inches: ");
	Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
	Serial.print(", cm: ");
	Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm 
#endif
}

// NAVIGATIONAL MOTOR FUNCTIONS

// checking for stops and branches
boolean atStop()
{
	if (leftOnLine && middleOnLine && rightOnLine)
		return true;
	else
		return false;
}
boolean atBranch()
{
	if (leftOnLine && !middleOnLine && rightOnLine)
		return true;
	else
		return false;
}
boolean onTrack()
{
	if (!leftOnLine && middleOnLine && !rightOnLine)
		return true;
	else
		return false;
}
boolean offTrackCompletely()
{
	if (!leftOnLine && !middleOnLine && !rightOnLine)
		return true;
	else
		return false;
}
boolean leftOnly()
{
	if (leftOnLine && !rightOnLine)
		return true;
	else
		return false;
}
boolean rightOnly()
{
	if (!leftOnLine && rightOnLine)
		return true;
	else
		return false;
}

boolean checkedAtStop(byte times)
{
	for (int i = 0; i < times; i++)
	{
		readLineTrackers();
		if (!atStop())
			return false;
	}

	return true;
}
boolean checkedAtBranched(byte times)
{
	for (int i = 0; i < times; i++)
	{
		readLineTrackers();
		if (!atBranch())
			return false;
	}

	return true;
}

// Basic Motions
void turnLeft(long speedFactor)
{
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop + speedFactor), 1500, 2100);
	ui_Left_Motor_Speed = ci_Right_Motor_Stop;
	implementMotorSpeed();
}
void turnRight(long speedFactor)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop + speedFactor), 1500, 2100);
	ui_Right_Motor_Speed = ci_Left_Motor_Stop;
	implementMotorSpeed();
}
void goForward(long speedFactor)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop + speedFactor), 1500, 2100);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop + speedFactor), 1500, 2100);
	implementMotorSpeed();
}
void halt()
{
	ui_Left_Motor_Speed = ci_Left_Motor_Stop;
	ui_Right_Motor_Speed = ci_Right_Motor_Stop;
	implementMotorSpeed();
}
void turnLeftOnSpot(long speedFactor)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop - speedFactor), 900, 1500);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop + speedFactor), 1500, 2100);
	implementMotorSpeed();
}
void turnRightOnSpot(long speedFactor)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop + speedFactor), 1500, 2100);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop - speedFactor), 900, 1500);
	implementMotorSpeed();
}

// Veering
// goes mostly forward slowly, but slightly to the left
void veerLeft(long speedFactor, long intensity)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop + speedFactor), 1500, 2100);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop + speedFactor + intensity), 1500, 2100);
	implementMotorSpeed();
}
// goes mostly forward slowly, but slightly to the right
void veerRight(long speedFactor, long intensity)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop + speedFactor + intensity), 1500, 2100);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop + speedFactor), 1500, 2100);
	implementMotorSpeed();
}
void veerRightBackward(long speedFactor, long intensity)
{
	ui_Left_Motor_Speed = constrain((ci_Left_Motor_Stop - speedFactor + intensity), 900, 1500);
	ui_Right_Motor_Speed = constrain((ci_Right_Motor_Stop - speedFactor), 900, 1500);
	implementMotorSpeed();
}

// Open Loop Control Calulation
void calcLeftTurn(long fullCircle, int angle) // full circle should be around 2800
{
	rightEncoderStopTime = encoder_RightMotor.getRawPosition();
	rightEncoderStopTime += (fullCircle * angle) / 360;
}
void calcRightTurn(long fullCircle, int angle)
{
	leftEncoderStopTime = encoder_LeftMotor.getRawPosition();
	leftEncoderStopTime += (fullCircle * angle) / 360;
}

// Open Loop Control Status
boolean doneLeftTurn()
{
	if (encoder_RightMotor.getRawPosition() > rightEncoderStopTime)
		return true;
	else
		return false;
}
boolean doneRightTurn()
{
	if (encoder_LeftMotor.getRawPosition() > leftEncoderStopTime)
		return true;
	else
		return false;
}

// Line Following
void followLine()
{
	// if line on left, turn left
	if (leftOnly())
	{
		if (direction == 0 && ((confidence + confidenceIncrement) <= confidenceMax))
			confidence += confidenceIncrement;
		else if (((confidence - confidenceIncrement) >= confidenceMin))
			confidence -= confidenceIncrement;

		turnLeft(confidence);
		direction = 0;
	}
	// if on track, go straight
	else if (onTrack())
	{
		if (direction == 1 && ((confidence + confidenceIncrement) <= confidenceMax))
			confidence += confidenceIncrement;
		else if (((confidence - confidenceIncrement) >= confidenceMin))
			confidence -= confidenceIncrement;

		goForward(confidence);
		direction = 1;
	}
	// if line on right, turn right
	else if (rightOnly())
	{
		if (direction == 2 && ((confidence + confidenceIncrement) <= confidenceMax))
			confidence += confidenceIncrement;
		else if (((confidence - confidenceIncrement) >= confidenceMin))
			confidence -= confidenceIncrement;

		turnRight(confidence);
		direction = 2;
	}
	// if don't know because both on line, go straight
	else if(leftOnLine && rightOnLine)
	{
		goForward(confidence);
		direction = 1;
	}
	// if off line completely, change directions
	else if (offTrackCompletely())
	{
		// sets confidence of 1 (minimum) and direction = 3
		confidence = confidenceMin;
		direction = 3;

		// change direction (once)
		if (direction = 0)
			turnRight(confidence);
		else if(direction = 1)
			turnLeft(confidence);
	}
}

// Implements Motor Speed
void implementMotorSpeed()
{
	if (bt_Motors_Enabled)
	{
		servo_LeftMotor.writeMicroseconds(constrain((ui_Left_Motor_Speed + ui_Left_Motor_Offset), 900, 2100));
		servo_RightMotor.writeMicroseconds(constrain((ui_Right_Motor_Speed + ui_Right_Motor_Offset), 900, 2100));
	}
	else
	{
		servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
		servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
	}
}

// ARM FUNCTIONS
void extendArm()
{
	servo_ArmMotor.write(ci_Arm_Servo_Extended);
}
void retractArm()
{
	servo_ArmMotor.write(ci_Arm_Servo_Retracted);
}
void liftArm()
{
	servo_ArmMotor.write(ci_Arm_Servo_Mid);
}
void shakeArm()
{
	// shake arm 20 times
	for (int i = 0; i < 10; i++)
	{
		// shaking is done from extended to middle arm positions
		for (int x = 0; x < 15; x++)
		{
			servo_ArmMotor.write(ci_Arm_Servo_Extended - x);
		}
	}
}


// CLAW FUNCTIONS
void apatheticClaw()
{
	servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Neutral);
}
void openClaw()
{
	servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Open);
}
void closeClaw()
{
	servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Closed);
}

// ULTRASONIC SENSOR FUNCTIONS
// returns ultrasonic reading in cm
int sensorDistance()
{
	Ping();
	return (ul_Echo_Time / 58);
}