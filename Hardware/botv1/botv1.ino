#include <Servo.h>
#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
#include <LiquidCrystal.h>

// wheels
//back
#define B_enA 4//2//9
#define B_in1 39//6
#define B_in2 41//7
//right
#define R_enA 5//3//9
#define R_in1 43//6
#define R_in2 45//7
//left
#define L_enA 2//4//9
#define L_in1 47//6
#define L_in2 49//7
//front
#define F_enA 3//5//9
#define F_in1 51//6
#define F_in2 53//7
int motorSpeed = 150; //0-255
// calibrated signals for 3.0v with 9v battery
//int B_motorSpeed = 234;//155; //0-255
//int R_motorSpeed = 212;//140; //0-255
//int L_motorSpeed = 242;//160; //0-255
//int F_motorSpeed = 250;//165; //0-255
// scaled to 12v input (wall mount)
//int B_motorSpeed = 98;//0-255
//int R_motorSpeed = 88;//0-255
//int L_motorSpeed = 100;//0-255
//int F_motorSpeed = 104;//0-255
int B_motorSpeed = 100;//0-255
int R_motorSpeed = 93;//0-255
int L_motorSpeed = 96;//0-255
int F_motorSpeed = 90;//0-255
// BACK - LEFT BACK WHEEL - LEFT CHIP IN1IN2 (in1 CC)
// RIGHT - RIGHT BACK WHEEL - LEFT CHIP IN3IN4
// LEFT - LEFT FRONT WHEEL - RIGHT CHIP IN1IN2 (in1 CC)
// FRONT - RIGHT FRONT WHEEL - RIGHT CHIP IN3IN4 (in1 C)

// eyes
#define L_TRIG 22
#define L_ECHO 24
#define F_TRIG 26
#define F_ECHO 28
#define R_TRIG 30
#define R_ECHO 32

//#define L_ECHO 24
//#define L_TRIG 22
//#define F_ECHO 28
//#define F_TRIG 26
//#define R_ECHO 32
//#define R_TRIG 30
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
// left first, right second
#define R_Gripper 8
#define R_Lifter 9
#define L_Gripper 10
#define L_Lifter 11
Servo rightGripper;
Servo rightLifter;
Servo leftGripper;
Servo leftLifter;
bool rightGripperClosed = false;
bool leftGripperClosed = false;
//opened
const int OPEN_DEG = 0;
const int LOW_DEG = 90;
//closed
const int CLOSE_DEG = 80;//100//110
const int HIGH_DEG = 10;
//const int OPEN_DEG = 20;
//const int CLOSE_DEG = 80;

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
#define contrast 7
const int rs = 34, en = 36, d4 = 46, d5 = 48, d6 = 50, d7 = 52;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// encoded pulse lengths for specific moves
const int TINY_STEP = 50;
const int SMALL_STEP = 200;
const int LARGE_STEP = 250; //250
const int TURN_90_STEP = 300; //300
bool waiting = false;

//COMM TEST - variables
String curString; // variable to store incoming serial data as String
int curVal;
int rotVal;

void setup() {
  Serial.begin(115200);
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

//  pinMode(F_ECHO, INPUT);
//  pinMode(F_TRIG, OUTPUT);
//  pinMode(L_ECHO, INPUT);
//  pinMode(L_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT);
  Wire.begin();
  if (accel.begin() == false) {
    Serial.println("Compass not Connected");
  }

  rightGripper.attach(R_Gripper);
  rightGripper.write(OPEN_DEG);
  rightLifter.attach(R_Lifter);
  rightLifter.write(LOW_DEG);
//  leftGripper.attach(L_Gripper);
//  leftGripper.write(OPEN_DEG);
//  leftLifter.attach(L_Lifter);
//  leftLifter.write(LOW_DEG);
  pinMode(sw, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  lcd.begin(16, 2);
  pinMode(contrast, INPUT);
  analogWrite(contrast, 128);
  lcd.print("F: 0 R: 0 L: 0");
  lcd.setCursor(0, 1);
  lcd.print("Heading: 0,0,0");

  randomSeed(analogRead(0));
//  delay(200);
}
