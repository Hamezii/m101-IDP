#include <RunningAverage.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
using namespace std;



// ___ OBJECTS ___
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
Adafruit_DCMotor *grabberMotor = AFMS.getMotor(2);
Servo liftServo;



// ___ CONSTANTS ___
const int MOTOR_ACCEL = 1000;
const unsigned long RotateTime = 500;
#define leftLineSensor 7
#define rightLineSensor 11
<<<<<<< Updated upstream
#define ledRed 0
#define ledGreen 1
#define ledOrange 2

=======
>>>>>>> Stashed changes


// ___ VARIABLE INITS ___
// LEDs
unsigned long ledActiveT = 0;

// Ultrasonic sensor
int trigPin = 12;    // Trigger
int echoPin = 13;    // Echo
//long H_Ultra = 8;   // Height of ultrasonic sensor (cm)
long T_Gnd_min = 9.5;
long T_Block_max = 9.5;
// need to test value after fix ultrasonic

int up=1; //******


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

bool rightLineSensorPrev = false;
bool leftLineSensorPrev = false;

// When first detecting a line, detectedPerpendicularLine will tick true for one loop, then go back to false.
bool detectedPerpendicularLine = false; 
bool onPerpendicularLine = false;

// State
enum State_enum {START, STARTING_BLOCK_SEARCH, SWEEP, SWEEP_FORWARD, PICK_UP_BLOCK, IDENTIFY_BLOCK, RETURN_TO_LINE, GO_TO_DROPOFF, PICKING_DROPOFF};
uint8_t state =START;              //uint8_t is an 8-bit integer / byte
unsigned long t = 0; // t is incremented after each loop.



// ___ FUNCTIONS ___



// ___ ULTRASONIC SENSOR ___
void Set_Ultrasonic(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


long GetDis_Ultrasonic(){
  long duration=0, cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  for(int m=0;m<5;m++){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration += pulseIn(echoPin, HIGH);
    delay(2);
  }
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.


  // Convert the time into a distance
  //cm = (duration/2) / 29.1+1;     // Divide by 29.1 or multiply by 0.0343
  duration/=5;
  cm = (duration/2) / 29.1;
  if(cm<50) return(cm);
  return 50;
}


int isBlock(){ // Ground->0; Block->1
  long dis=GetDis_Ultrasonic();
  delay(2);
  dis+=GetDis_Ultrasonic();
  delay(2);
  dis=GetDis_Ultrasonic();
  dis/=3;
  if(dis<T_Block_max) return 1;
  else if(dis>T_Gnd_min) return 0;
  else return -1;
}



// ___ SENSOR INTERFACE ___
void updateSensing() {
  leftLineSensorPrev = leftLineSensorVal();
  rightLineSensorPrev = rightLineSensorVal();
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
  if (dir == "Forward") {leftMotorTarget = -255; rightMotorTarget = -255;}
  if (dir == "SlowForward") {leftMotorTarget = -180; rightMotorTarget = -180;}
  if (dir == "Backward") {leftMotorTarget = 255; rightMotorTarget = 255;}
  if (dir == "Left") {leftMotorTarget = 255; rightMotorTarget = -255;}
  if (dir == "Right") {leftMotorTarget = -255; rightMotorTarget = 255;}
  if (dir == "SlowLeft") {leftMotorTarget = 255; rightMotorTarget = -255;}
  if (dir == "SlowRight") {leftMotorTarget = -255; rightMotorTarget = 255;}
  if (dir == "BackLeft") {leftMotorTarget = 0; rightMotorTarget = -180;}
  if (dir == "BackRight") {leftMotorTarget = -180; rightMotorTarget = 0;}
}



// ___ GRABBER ___
void GrabberDown() {
    int pos = 0;
    for (pos = 0; pos <= 90; pos += 1) { // goes from 40 degrees to 150 degrees
        // in steps of 1 degree
        liftServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(10);                       // waits 15 ms for the servo to reach the position
    }
}


void GrabberUp() {
    int pos = 0;
    for (pos = 90; pos >= 0; pos -= 1) { // goes from 150 degrees to 40 degrees
        liftServo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(10);                       // waits 15 ms for the servo to reach the position       
    }
}


void setGrabberClosed(bool closeState) {
  grabberMotor->run(closeState ? FORWARD : BACKWARD );
  grabberMotor->setSpeed(255);

}



// ___ ALGORITHMS ___
void followLine(int forward = true){
  if (onPerpendicularLine) setDriveDir(forward ? "Forward": "Backward");
  else if(!leftLineSensorVal() && !rightLineSensorVal()) setDriveDir(forward ? "Forward" : "Backward");
  else if(leftLineSensorVal() && !rightLineSensorVal()) setDriveDir(forward ? "Left" : "BackRight");
  else if(!leftLineSensorVal() && rightLineSensorVal()) setDriveDir(forward ? "Right" : "BackLeft");
}


bool returnToLine(bool leftOfLine){
  setDriveDir(leftOfLine ? "SlowRight" : "SlowLeft");
  // Just stopped sensing line? (meaning it is now between the sensors)
  return (!rightLineSensorVal() && !leftLineSensorVal()) && (rightLineSensorPrev || leftLineSensorPrev);
}


// ___ MAIN ___
void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  liftServo.attach(9);  // attaches the servo on pin 9 to the servo object
  
  pinMode(leftLineSensor,INPUT);
  pinMode(rightLineSensor,INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledOrange, OUTPUT);
  Set_Ultrasonic();
  Serial.begin(9600);


}


void loop() {
  // Logic
  static int counter = 0;
  static unsigned long timer = 0; 
  // To remember what the currently held block's type is
  static bool isBlockMetal = false; 
  // When scanning off the line, this lets the robot know which side of the line it's on.
  static bool leftOfLine = true;


//GrabberDown();
//delay(1000);
//GrabberUp();
//delay(3000);
//  delay(3000);
//


  while(true) {
    GrabberDown();
    delay(800);
    setGrabberClosed(true);
    delay(1000);
    GrabberUp();
    delay(800);
    GrabberDown();
    delay(800);
    setGrabberClosed(false);
    delay(800);
    setGrabberClosed(true);
    GrabberUp();
    delay(2000);
   
  }

//liftServo.write(125);
 //Serial.println(isBlock());
 //Serial.println(GetDis_Ultrasonic());
  //delay(50);


/*

  switch (state){
    
    case START: // Travel from start to pickup box
      if(up==1) {GrabberUp(); up=0;}
      followLine();
      if (detectedPerpendicularLine) counter = counter + 1;
      if (counter == 3) 
      {
        state = STARTING_BLOCK_SEARCH; 
        timer = t + 200;// 200ms
        GrabberDown();
      }
    break;

    
    case STARTING_BLOCK_SEARCH:
      if (t < timer) setDriveDir("Backward");
      else {state = SWEEP; leftOfLine = true; timer = t + RotateTime;}
    break;

    case SWEEP:
      if (t < timer) { // Turning away from line
        setDriveDir(leftOfLine ? "SlowLeft" : "SlowRight");
      }
      else if (returnToLine(leftOfLine)){ // Returned back to line?
        if (leftOfLine) { // Returning from left
          timer = t + RotateTime;
        } else { // Returning from right
          state = SWEEP_FORWARD;
          timer = t + 300;  // time moving forward
        }
        leftOfLine = !leftOfLine;
      } 
      else if (isBlock() == 1){
        
        state = PICK_UP_BLOCK;
        timer = t + 200;
      }
    break;

    case SWEEP_FORWARD:
      if (t < timer) setDriveDir("Forward");
      else {
        state = SWEEP;
        timer = t + RotateTime;
        leftOfLine = true;
      }
    break;

    case PICK_UP_BLOCK:
      if (t < timer) setDriveDir("SlowForward");
      else {
        setDriveDir("Stop");
        state = IDENTIFY_BLOCK;
        setGrabberClosed(true);
      }
      break;

    case IDENTIFY_BLOCK:
    // TODO identify
      isBlockMetal = true;
      ledActiveT = t + 8000;
      state = RETURN_TO_LINE;
    break;

    case RETURN_TO_LINE:
      if (returnToLine(leftOfLine)) {
        state = GO_TO_DROPOFF;
        counter = 0;
      }
    break;

    case GO_TO_DROPOFF:
      followLine(false); // Going backwards
      if (detectedPerpendicularLine) counter = counter + 1;
      if (counter == 2) state = PICKING_DROPOFF;
    break;
  }


  */
  
  // LED
  digitalWrite(ledOrange, (t%2000)<1000 ? HIGH : LOW);
  digitalWrite((isBlockMetal)? ledRed : ledGreen, (t < ledActiveT)? HIGH : LOW);
  
  // Loop updates 
  updateMotors();
  updateSensing();
  t=millis();
  delay(2);
}
