#include <RunningAverage.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
using namespace std;
#include <algorithm>
#include <cmath>



// ___ OBJECTS ___
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
Servo grabberServo; 



// ___ CONSTANTS ___
const int MOTOR_ACCEL = 10
#define leftLineSensor A0
#define rightLineSensor A1



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
RunningAverage leftLineSensorRA(10);
RunningAverage rightLineSensorRA(10);



// ___ FUNCTIONS ___



// ___ SENSOR INTERFACE ___
// TODO



// ___ MOTOR MANAGEMENT ___
void updateMotorSpeeds() { 
  // Move the motorCurrent variables towards their target speeds
  if (leftMotorTarget > leftMotorCurrent) {
    leftMotorCurrent = min(leftMotorCurrent + MOTOR_ACCEL, leftMotorTarget)
  } else {
    leftMotorCurrent = max(leftMotorCurrent - MOTOR_ACCEL, leftMotorTarget)
  }
  
  if (rightMotorTarget > rightMotorCurrent) {
    rightMotorCurrent = min(rightMotorCurrent + MOTOR_ACCEL, rightMotorTarget)
  } else {
    rightMotorCurrent = max(rightMotorCurrent - MOTOR_ACCEL, rightMotorTarget)
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
String getDriveDir() { 
  // Get the direction the robot is moving
  // Returns a String
  if (leftMotorTarget == 0) and (rightMotorTarget == 0) {return "None";}
  else if (leftMotorTarget >= 0) and (rightMotorTarget >= 0) {return "Forward";}
  else if (leftMotorTarget >= 0) and (rightMotorTarget <= 0) {return "Right";}
  else if (leftMotorTarget <= 0) and (rightMotorTarget >= 0) {return "Left";}
  else if (leftMotorTarget <= 0) and (rightMotorTarget <= 0) {return "Backward";}
}


void setDriveDir(String dir) { 
  // Set the direction for the robot to drive in
  // Takes a String parameter
  if (dir == "None") {leftMotorTarget = 0; rightMotorTarget = 0;}
  if (dir == "Forward") {leftMotorTarget = 255; rightMotorTarget = 255;}
  if (dir == "Backward") {leftMotorTarget = -255; rightMotorTarget = -255;}
  if (dir == "Left") {leftMotorTarget = 120; rightMotorTarget = -120;}
  if (dir == "Right") {leftMotorTarget = -120; rightMotorTarget = 120;}
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
  
  //line detected by neither
  if(digitalRead(leftLineSensor)==0 && digitalRead(rightLineSensor)==0){
    setDriveDir("Forward");
  }
  //line detected by left sensor
  else if(digitalRead(left)==1 && digitalRead(right)==0){
    setDriveDir("Left");
    }
  }
  //line detected by right sensor
  else if(digitalRead(left)==0 && digitalRead(right)==1){
    setDriveDir("Right");
  }
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
  // put your main code here, to run repeatedly:
  followLine();
  updateMotors();
  delay(10);
}