#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
#include <LiquidCrystal.h>

bool DistanceOn = false;
bool DisplayOn = false;
bool COMMUNICATION_ENABLED = false;

// wheels
// BACK - LEFT BACK WHEEL - LEFT CHIP IN1IN2 (in1 CC)
// RIGHT - RIGHT BACK WHEEL - LEFT CHIP IN3IN4
// LEFT - LEFT FRONT WHEEL - RIGHT CHIP IN1IN2 (in1 CC)
// FRONT - RIGHT FRONT WHEEL - RIGHT CHIP IN3IN4 (in1 C)
#define B_enA 4//2//9
#define B_in1 39//6
#define B_in2 41//7
#define R_enA 5//3//9
#define R_in1 43//6
#define R_in2 45//7
#define L_enA 2//4//9
#define L_in1 47//6
#define L_in2 49//7
#define F_enA 3//5//9
#define F_in1 51//6
#define F_in2 53//7

// wheel encoders
#define FR_encoder 18
#define FL_encoder 19
#define BL_encoder 20
#define BR_encoder 21
volatile int FR_count = 0;
volatile int FL_count = 0;
volatile int BL_count = 0;
volatile int BR_count = 0;

// motor speed
//FR-F, FL-L, BR-R, BL-B
bool disableMotors = false;
// motor 6V, input power 12V, L298N drop 2V, 255 x 60% = 150 max
int F_motorSpeed = 98;//0-255, FR
int L_motorSpeed = 92;//0-255, FL
int R_motorSpeed = 100;//0-255, BR
int B_motorSpeed = 112;//0-255, BL

// eyes
#define L_TRIG 22
#define L_ECHO 24
#define F_TRIG 26
#define F_ECHO 28
#define R_TRIG 30
#define R_ECHO 32
const int TARGET_OFFSET = 3;
const int COLLISION_THRESHOLD = 12; //cm
const int PICKING_DISTANCE = 5; //cm
int leftDistance = 0;
int rightDistance = 0;
int frontDistance = 0;
int stuckCounter = 0;
int lastFrontDistance = 0;
String currentDirection = "stopped";

// gripper
#define STEP 35
#define DIR 37
#define STALL_DETECTION A3
// limit switch
#define L_LOW_POS A5
#define R_LOW_POS A6
#define TOP_POS A7
bool lowerTriggered = false;
bool upperTriggered = false;

// left first, right second
#define R_Gripper 6
#define L_Gripper 7
Servo rightGripper;
Servo leftGripper;
bool rightGripperClosed = false;
bool leftGripperClosed = false;
const int OPEN_DEG = 5;//-30;
const int CLOSE_DEG = 80;//100//110
bool defaultOpen = true;

int counter;
int turn90Count = 60;
int incomingByte = 0; // for incoming serial data

//IMU
MMA8452Q accel;
String headingData;

// joystick
//bool autoMode = true;
//String currentMode = "auto";
bool autoMode = false;
String currentMode = "manual";
#define sw 31
#define jX A0
#define jY A1
int joystickCommand = 5;
int lastJoystickCommand = 5;
int pressed = false;
int jXval = 0;
int jYval = 0;
String DirName;

//face
#define contrast 8
const int rs = 36, en = 38, d4 = 40, d5 = 42, d6 = 44, d7 = 46;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// encoded pulse lengths for specific moves
const int TINY_STEP = 50;
const int SMALL_STEP = 200;
const int LARGE_STEP = 250;
const int TURN_90_STEP = 300;
bool LOCOMOTION_ACTIVE = false;

// ENCODER FILTERS
// 40 FIRINGS PER REVOLUTION
const int MIN_ELAPSED_TIME = 20;//30;
volatile unsigned long elapsedTimeFR;
volatile unsigned long previousTimeFR;
volatile unsigned long elapsedTimeFL;
volatile unsigned long previousTimeFL;
volatile unsigned long elapsedTimeBR;
volatile unsigned long previousTimeBR;
volatile unsigned long elapsedTimeBL;
volatile unsigned long previousTimeBL;
volatile int filterActiveCountFR;
volatile int filterActiveCountFL;
volatile int filterActiveCountBR;
volatile int filterActiveCountBL;
volatile unsigned long filterSignalSpeedFR = 0.0;
volatile unsigned long filterSignalSpeedFL = 0.0;
volatile unsigned long filterSignalSpeedBR = 0.0;
volatile unsigned long filterSignalSpeedBL = 0.0;

// WHEEL PID
//double Kp=30.00, Ki=0.00, Kd=00.00;
double Kp=30.00, Ki=0.50, Kd=0.10;
//double Kp=30.00, Ki=200.00, Kd=00.00;
double SetpointFR, InputFR, OutputFR;
PID pidFR(&InputFR, &OutputFR, &SetpointFR, Kp, Ki, Kd, DIRECT);
double SetpointFL, InputFL, OutputFL;
PID pidFL(&InputFL, &OutputFL, &SetpointFL, Kp, Ki, Kd, DIRECT);
double SetpointBR, InputBR, OutputBR;
PID pidBR(&InputBR, &OutputBR, &SetpointBR, Kp, Ki, Kd, DIRECT);
double SetpointBL, InputBL, OutputBL;
PID pidBL(&InputBL, &OutputBL, &SetpointBL, Kp, Ki, Kd, DIRECT);
bool FR_direction = true, FL_direction = true, BR_direction = true, BL_direction = true;
bool FR_reached = true, FL_reached = true, BR_reached = true, BL_reached = true;
bool FR_reversed = false, FL_reversed  = false, BR_reversed  = false, BL_reversed  = false;

int badCounter = 0;
unsigned long previousMicros;
int motorUpdateTime = 0;
int minPwm = 60;
int maxPwm = 120;

//COMM TEST - variables
String curString; // variable to store incoming serial data as String
int curVal;
int rotVal;

void setup() {
  Serial.begin(9600);
  // Serial.begin(115200);
  //COMM TEST - initialize bobbin count to 0
  Serial.print("bobbins/0");
  
  pinMode(B_enA, OUTPUT);
  pinMode(B_in1, OUTPUT);
  pinMode(B_in2, OUTPUT);
  pinMode(R_enA, OUTPUT);
  pinMode(R_in1, OUTPUT);
  pinMode(R_in2, OUTPUT);
  pinMode(L_enA, OUTPUT);
  pinMode(L_in1, OUTPUT);
  pinMode(L_in2, OUTPUT);
  pinMode(F_enA, OUTPUT);
  pinMode(F_in1, OUTPUT);
  pinMode(F_in2, OUTPUT);
  analogWrite(B_enA, B_motorSpeed);
  analogWrite(R_enA, R_motorSpeed);
  analogWrite(L_enA, L_motorSpeed);
  analogWrite(F_enA, F_motorSpeed);
  // motor encoders
  pinMode(FR_encoder, INPUT);
  pinMode(FL_encoder, INPUT);
  pinMode(BR_encoder, INPUT);
  pinMode(BL_encoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(FR_encoder), FR_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_encoder), FL_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_encoder), BR_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_encoder), BL_callback, CHANGE);
  // stepper and stop switches
  pinMode(L_LOW_POS, INPUT_PULLUP);
  pinMode(R_LOW_POS, INPUT_PULLUP);
  pinMode(TOP_POS, INPUT_PULLUP);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, LOW);

  pinMode(F_ECHO, INPUT);
  pinMode(F_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT);
  Wire.begin();
  //if (accel.begin() == false) {
    //Serial.println("Compass not Connected");
  //}

  //  Wire.begin();
  //  if (accel.begin() == false) {
  //    Serial.println("Compass not Connected");
  //  }
  pinMode(STALL_DETECTION, INPUT);
  rightGripper.attach(R_Gripper);
  leftGripper.attach(L_Gripper);
  if (defaultOpen) {
    rightGripper.write(180 - OPEN_DEG);
    leftGripper.write(OPEN_DEG);
  } else {
    rightGripper.write(180 - CLOSE_DEG);
    leftGripper.write(CLOSE_DEG);
  }

  pinMode(sw, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  lcd.begin(16, 2);
  pinMode(contrast, OUTPUT);
  analogWrite(contrast, 128);
  lcd.print("F: 0 R: 0 L: 0");
  lcd.setCursor(0, 1);
  lcd.print("Heading: 0,0,0");

  randomSeed(analogRead(0));
  //  delay(200);

  pidFR.SetMode(AUTOMATIC);
  pidFR.SetOutputLimits(-100, 100);
  pidFR.SetSampleTime(10); // in ms
  pidFL.SetMode(AUTOMATIC);
  pidFL.SetOutputLimits(-100, 100);
  pidFL.SetSampleTime(10); // in ms
  pidBR.SetMode(AUTOMATIC);
  pidBR.SetOutputLimits(-100, 100);
  pidBR.SetSampleTime(10); // in ms
  pidBL.SetMode(AUTOMATIC);
  pidBL.SetOutputLimits(-100, 100);
  pidBL.SetSampleTime(10); // in ms
  SetInitialMotorTargets();
}
