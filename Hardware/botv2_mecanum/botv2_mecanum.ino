#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
#include <LiquidCrystal.h>

//// ======= BOBETTE ======== //
//// LAST UPDATED: 210929 12:10 ////

//const int OPEN_DEG_L = 25;
//const int CLOSE_DEG_L = 80;
//const int OPEN_DEG_R = 175;//180 - OPEN_DEG
//const int CLOSE_DEG_R = 100;//180 - CLOSE_DEG

//// ======= BOB ======== ////
//// LAST UPDATED: 210929 12:30 ////

const int OPEN_DEG_L = 15;
const int CLOSE_DEG_L = 75;
const int OPEN_DEG_R = 170;//180 - OPEN_DEG
const int CLOSE_DEG_R = 90;//180 - CLOSE_DEG

///////////////////////////////////////////////////////////////////////////////////////////

bool DisplayOn = true;
//bool COMMUNICATION_ENABLED = false;

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
#define L_enA 9//4//9
#define L_in1 47//6
#define L_in2 49//7
#define F_enA 10//5//9
#define F_in1 51//6
#define F_in2 53//7

// wheel encoders
#define FR_encoder 18
#define FL_encoder 19
#define BL_encoder 2
#define BR_encoder 3
volatile int FR_count = 0;
volatile int FL_count = 0;
volatile int BL_count = 0;
volatile int BR_count = 0;

// motor speed (adjusted by PID loop)
//FR-F, FL-L, BR-R, BL-B
bool disableMotors = false;
// motor max 6V, max input 10V (input power 12V, L298N drop 2V)
// 255 x 60% = 150 max
int F_motorSpeed = 100;//98, 0-255, FR
int L_motorSpeed = 100;//92, 0-255, FL
int R_motorSpeed = 100;//100, 0-255, BR
int B_motorSpeed = 100;//112, 0-255, BL

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
const bool GRIPPER_DEFAULT_OPEN = true;
char incomingByte; // for incoming serial data

//IMU
const bool IMU_ACTIVE = false;
MMA8452Q accel;

// joystick
//bool autoMode = true;
//String currentMode = "auto";
bool autoMode = false;
String currentMode = "Idle";
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

// encoded pulse lengths for specific moves (deprecated)
const int TINY_STEP = 50;
const int SMALL_STEP = 200;
const int TURN_90_STEP = 300;

// ENCODER FILTERS (not necessary)
//const int MIN_ELAPSED_TIME = 1;//30;
//volatile unsigned long elapsedTimeFR;
//volatile unsigned long previousTimeFR;
//volatile unsigned long elapsedTimeFL;
//volatile unsigned long previousTimeFL;
//volatile unsigned long elapsedTimeBR;
//volatile unsigned long previousTimeBR;
//volatile unsigned long elapsedTimeBL;
//volatile unsigned long previousTimeBL;
//volatile int filterActiveCountFR;
//volatile int filterActiveCountFL;
//volatile int filterActiveCountBR;
//volatile int filterActiveCountBL;
//volatile unsigned long filterSignalSpeedFR = 0.0;
//volatile unsigned long filterSignalSpeedFL = 0.0;
//volatile unsigned long filterSignalSpeedBR = 0.0;
//volatile unsigned long filterSignalSpeedBL = 0.0;

// WHEEL PID
bool LOCOMOTION_ACTIVE = false;
unsigned long now; // debug variable
// https://www.youtube.com/watch?v=IB1Ir4oCP5k&ab_channel=RealPars
// https://playground.arduino.cc/Code/PIDLibrary/
double Kp_agg = 8.0, Ki_agg = 0.2, Kd_agg = 0.0;
double Kp_con = 1.0, Ki_con = 0.05, Kd_con = 0.0;
// Kp = 1.0, reaches and oscillates a lot
// Kp = 0.1, turn negative
//double Kp=30.00, Ki=0.0, Kd=0.0;
//double Kp=30.00, Ki=0.50, Kd=0.10;
//double Kp=30.00, Ki=200.00, Kd=00.00;
double SetpointFR, InputFR, OutputFR;
PID pidFR(&InputFR, &OutputFR, &SetpointFR, Kp_agg, Ki_agg, Kd_agg, DIRECT);
double SetpointFL, InputFL, OutputFL;
PID pidFL(&InputFL, &OutputFL, &SetpointFL, Kp_agg, Ki_agg, Kd_agg, DIRECT);
double SetpointBR, InputBR, OutputBR;
PID pidBR(&InputBR, &OutputBR, &SetpointBR, Kp_agg, Ki_agg, Kd_agg, DIRECT);
double SetpointBL, InputBL, OutputBL;
PID pidBL(&InputBL, &OutputBL, &SetpointBL, Kp_agg, Ki_agg, Kd_agg, DIRECT);
bool FR_direction = true, FL_direction = true, BR_direction = true, BL_direction = true;
bool FR_reached = true, FL_reached = true, BR_reached = true, BL_reached = true;
bool FR_reversed = false, FL_reversed  = false, BR_reversed  = false, BL_reversed  = false;

int badCounter = 0;
unsigned long previousMicros;
unsigned long previousMillis;
int motorUpdateTime = 0;
int minPwm = 75; // tested minimum before motor stalls: 75
int maxPwm = 120; //measured voltage equivalent: 4.46 V

//COMM TEST - variables
int SET_DISTANCE = 10; // distance to travel per key press (UNIT IS CM!)
int SET_ANGLE= 90; // distance to travel per key press (UNIT IS deg!)
String curString; // variable to store incoming serial data as String
int curVal;
int rotVal;
int distVal;
uint8_t buf[8] = {
  0
};
char myChar;

void setup() {
//  Serial.begin(9600);
    Serial.begin(115200);
  //COMM TEST - initialize bobbin count to 0
//    Serial.print("bobbins/0");

  //  for (int i=0; i<360; i++){
  //    Serial.print(i);
  //    Serial.print("    ");
  //    long number = (i * 40 * 487.0) / (360 * 150.8);
  //    Serial.println(int(number));
  //  }
  //
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
  // use RISING instead of CHANGE, the dc spins a bit too fast for this to detect correctly
  attachInterrupt(digitalPinToInterrupt(FR_encoder), FR_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(FL_encoder), FL_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_encoder), BR_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_encoder), BL_callback, RISING);
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
  
  if (IMU_ACTIVE) {
    Wire.begin(); //initiate the Wire library and join the I2C bus as master, occupies pin 20/21 on Arduino mega
    if (accel.begin() == false) {
      Serial.println("Compass not Connected");
    }
  }

  pinMode(STALL_DETECTION, INPUT);
  rightGripper.attach(R_Gripper);
  leftGripper.attach(L_Gripper);
  if (GRIPPER_DEFAULT_OPEN) {
    rightGripper.write(OPEN_DEG_R);
    leftGripper.write(OPEN_DEG_L);
  } else {
    rightGripper.write(CLOSE_DEG_R);
    leftGripper.write(CLOSE_DEG_L);
  }

  pinMode(sw, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  lcd.begin(16, 2);
  pinMode(contrast, OUTPUT);
  analogWrite(contrast, 128);
  lcd.print("");
  lcd.setCursor(0, 1);
  lcd.print("F: 0 R: 0 L: 0");

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
