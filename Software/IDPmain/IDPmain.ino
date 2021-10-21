#include <RunningAverage.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "SharpIR.h"
using namespace std;



// ___ OBJECTS ___
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
Servo grabberServo; 



// ___ CONSTANTS ___
const int MOTOR_ACCEL = 1000;
#define leftLineSensor 4
#define rightLineSensor 5



// ___ VARIABLE INITS ___
// Cache for the current motor speeds
// The caches are signed to signify forward or backward movement
int leftMotorCurrent = 0; 
int rightMotorCurrent = 0;

// Cache for target velocities for the motor speeds
// These are used to allow smooth acceleration from the current motor speeds to the new ones
int leftMotorTarget = 0;
int rightMotorTarget = 0;

// Running average of sensor values, for more accurate read values
RunningAverage leftLineSensorRA(4);
RunningAverage rightLineSensorRA(4);
RunningAverage distSensorRA(4);

// When first detecting a line, detectedPerpendicularLine will tick true for one loop, then go back to false.
bool detectedPerpendicularLine = false; 
bool onPerpendicularLine = false;

// State
enum State_enum {START, STARTING_BLOCK_SEARCH, SWEEP_LEFT, SWEEP_RIGHT, APPROACH_BLOCK, PICK_UP_BLOCK};
uint8_t state = START;              //uint8_t is an 8-bit integer / byte
int t = 0; // t is incremented after each loop.



// ___ FUNCTIONS ___



// ___ SENSOR INTERFACE ___
void updateSensing() {
  leftLineSensorRA.addValue(digitalRead(leftLineSensor));
  rightLineSensorRA.addValue(digitalRead(rightLineSensor));
  
  updateLineDetection();
}


void updateLineDetection() {
  if (detectedPerpendicularLine) detectedPerpendicularLine = false;
  if (!onPerpendicularLine && leftLineSensorVal() && rightLineSensorVal()) detectedPerpendicularLine = true;

  onPerpendicularLine = leftLineSensorVal() && rightLineSensorVal();
}


bool leftLineSensorVal() {
  return (leftLineSensorRA.getAverage() > 0.5) ? true : false; // Return true if average is bigger than 0.5, else return false
}


bool rightLineSensorVal() {
  return (rightLineSensorRA.getAverage() > 0.5) ? true : false;
}


bool detectBlock() {
  // if ultrasonic sensor below threshold
  //   return true
}


// ___ MOTOR MANAGEMENT ___
void updateMotorSpeeds() { 
  // Move the motorCurrent variables towards their target speeds
  if (leftMotorTarget > leftMotorCurrent) {
    leftMotorCurrent = min(leftMotorCurrent + MOTOR_ACCEL, leftMotorTarget);
  } else {
    leftMotorCurrent = max(leftMotorCurrent - MOTOR_ACCEL, leftMotorTarget);
  }
  
  if (rightMotorTarget > rightMotorCurrent) {
    rightMotorCurrent = min(rightMotorCurrent + MOTOR_ACCEL, rightMotorTarget);
  } else {
    rightMotorCurrent = max(rightMotorCurrent - MOTOR_ACCEL, rightMotorTarget);
  }
}


void setMotorSpeeds() {
  // Set motor speeds based on the motorCurrent values
  leftMotor->run( (leftMotorCurrent < 0) ? BACKWARD : (leftMotorCurrent > 0) ? FORWARD : RELEASE);
  leftMotor->setSpeed(abs(leftMotorCurrent));

  rightMotor->run( (rightMotorCurrent < 0) ? BACKWARD : (rightMotorCurrent > 0) ? FORWARD : RELEASE);
  rightMotor->setSpeed(abs(rightMotorCurrent));
}


void updateMotors() {
  // Call one-per-loop methods for motors
  updateMotorSpeeds();
  setMotorSpeeds();
}



// ___ DRIVING INTERFACE ___
void setDriveDir(String dir) { 
  // Set the direction for the robot to drive in
  // Takes a String parameter
  if (dir == "Stop") {leftMotorTarget = 0; rightMotorTarget = 0;}
  if (dir == "Forward") {leftMotorTarget = 255; rightMotorTarget = 255;}
  if (dir == "SlowForward") {leftMotorTarget = 70; rightMotorTarget = 70;}
  if (dir == "Backward") {leftMotorTarget = -255; rightMotorTarget = -255;}
  if (dir == "Left") {leftMotorTarget = -120; rightMotorTarget = 120;}
  if (dir == "Right") {leftMotorTarget = 120; rightMotorTarget = -120;}
  if (dir == "SweepLeft") {leftMotorTarget = -70; rightMotorTarget = 120;}
  if (dir == "SweepRight") {leftMotorTarget = 120; rightMotorTarget = -70;}
}



// ___ GRABBER ___
void GrabberUp() {
    int pos = 0;
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        grabberServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
    }
}


void GrabberDown() {
    int pos = 180;
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        grabberServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position       
    }
}



// ___ ALGORITHMS ___
void followLine(){
  if (onPerpendicularLine) setDriveDir("Forward");
  else if(!leftLineSensorVal() && !rightLineSensorVal()) setDriveDir("Forward");
  else if(leftLineSensorVal() && !rightLineSensorVal()) setDriveDir("Left");
  else if(!leftLineSensorVal() && rightLineSensorVal()) setDriveDir("Right");
}



// ___ MAIN ___
void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  grabberServo.attach(10);  // attaches the servo on pin 10 to the servo object
  pinMode(leftLineSensor,INPUT);
  pinMode(rightLineSensor,INPUT);
  Serial.begin(9600);
}


void loop() {
  // Logic
  static int counter = 0;
  static int timer = 0; 
  static bool isBlockMetal = false; // To remember what the currently held block's type is

  switch (state){
    
    case START: // Travel from start to pickup box
      followLine();
      if (detectedPerpendicularLine) counter = counter + 1;
      if (counter == 2) state = STARTING_BLOCK_SEARCH;  
    break;

    
    case STARTING_BLOCK_SEARCH:
      state = SWEEP_LEFT;
    break;

    case SWEEP_LEFT:
      setDriveDir("SweepLeft");
      if (leftLineSensorVal()) state = SWEEP_RIGHT;
      
    break;

    case SWEEP_RIGHT:
      setDriveDir("SweepRight");
      if (rightLineSensorVal()) state = SWEEP_RIGHT;
    break;


    case APPROACH_BLOCK:
      setDriveDir("SlowForward");
//      if (ultrasonic sensor below pick up threshold) {
//        setDriveDir("Stop");
//        state = PICK_UP_BLOCK;
//      }
    break;

    case PICK_UP_BLOCK:
      
    break;
  
  }

  // Loop updates 
  updateMotors();
  updateSensing();
  t = t+1;
  delay(2);
}
