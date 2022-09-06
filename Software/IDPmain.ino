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
const float SERVO_SPEED = 1;
const unsigned long FORWARD_10cm_TIME = 850; // test done
const unsigned long ROTATE_30_TIME = 580; //test done
const unsigned long SWEEP_TIME = 800;
#define leftLineSensor 7
#define rightLineSensor 11
#define ledRed 0
#define ledGreen 1
#define ledOrange 2
#define HallPin A0
#define SWPin 3


// ___ VARIABLE INITS ___
// LEDs
unsigned long ledActiveT = 0;

// Ultrasonic sensor
int trigPin = 12;    // Trigger
int echoPin = 13;    // Echo
long T_Gnd_min = 9.5;
long T_Block_max = 9.5;
// need to test value after fix ultrasonic


// Cache for the current motor speeds
// The caches are signed to signify forward or backward movement
int leftMotorCurrent = 0;
int rightMotorCurrent = 0;

// Cache for target velocities for the motor speeds
// These are used to allow smooth acceleration from the current motor speeds to the new ones
int leftMotorTarget = 0;
int rightMotorTarget = 0;


// Servo
float servoAngle = 0;
float servoAngleTarget = 150;


// Running average of sensor values, for more accurate read values
RunningAverage leftLineSensorRA(4);
RunningAverage rightLineSensorRA(4);
RunningAverage distSensorRA(4);
RunningAverage isBlockRA(4);

bool rightLineSensorPrev = false;
bool leftLineSensorPrev = false;

// When first detecting a line, detectedPerpendicularLine will tick true for one loop, then go back to false.
bool detectedPerpendicularLine = false;
bool onPerpendicularLine = false;

// State
enum State_enum {START, REACH_COLLECTION_POINT, STARTING_BLOCK_SEARCH, SWEEP, SWEEP_FORWARD, PICK_UP_BLOCK, RETURN_TO_LINE, TURN_AROUND, GO_TO_DROPOFF, PICKING_DROPOFF, DROPPING_BLOCK, FIND_START_LINE, TURNING_TO_START, RETURN_TO_START, END};
uint8_t state = START;             //uint8_t is an 8-bit integer / byte
unsigned long t = 0; // t is incremented after each loop.



// ___ FUNCTIONS ___



// ___ ULTRASONIC SENSOR ___
void Set_Ultrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


long GetDis_Ultrasonic() {
  long duration = 0, cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  for (int m = 0; m < 5; m++) {
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
  duration /= 5;
  cm = (duration / 2) / 29.1;
  if (cm < 50) return (cm);
  return 50;
}


int detectIsBlock() { // Ground->0; Block->1
  long dis = GetDis_Ultrasonic();
  if (dis < T_Block_max) return 1;
  else if (dis > T_Gnd_min) return 0;
  else return 0;
}


bool getIsBlock() {
  return (isBlockRA.getAverage() > 0.5) ? true : false; // Return true if average is bigger than 0.5, else return false
}


// ___ SENSOR INTERFACE ___
void updateSensing() {
  leftLineSensorPrev = leftLineSensorVal();
  rightLineSensorPrev = rightLineSensorVal();
  leftLineSensorRA.addValue(digitalRead(leftLineSensor));
  rightLineSensorRA.addValue(digitalRead(rightLineSensor));
  //isBlockRA.addValue(detectIsBlock());

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


bool detectMetal() {
  RunningAverage RA(10);

  for (int i = 0; i < 10; i++) {
    RA.addValue(analogRead(HallPin));
    delay(5);
  }

  if (RA.getAverage() > 400)// Threshold // test
    return true;
  else return false;
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
  updateServo();
  
}



// ___ GRABBER ___
void updateServo() {
  //servoAngle = liftServo.read();
  if (servoAngleTarget > servoAngle) {
    servoAngle = min(servoAngle + SERVO_SPEED, servoAngleTarget);
  } else {
    servoAngle = max(servoAngle - SERVO_SPEED, servoAngleTarget);
  }
  liftServo.write(servoAngle);
}


void GrabberDown() {
  servoAngleTarget = 50;
}


void GrabberUp() {
  servoAngleTarget = 150;
}


void setGrabberClosed(bool closeState) {
  grabberMotor->run(closeState ?  FORWARD : BACKWARD );
  grabberMotor->setSpeed(255);

}



// ___ DRIVING INTERFACE ___
void setDriveDir(String dir) {
  // Set the direction for the robot to drive in
  // Takes a String parameter
  if (dir == "Stop") {
    leftMotorTarget = 0;
    rightMotorTarget = 0;
  }
  if (dir == "Forward") {
    leftMotorTarget = -255;
    rightMotorTarget = -255;
  }
  if (dir == "SlowForward") {
    leftMotorTarget = -180;
    rightMotorTarget = -180;
  }
  if (dir == "Backward") {
    leftMotorTarget = 255;
    rightMotorTarget = 255;
  }
  if (dir == "Left") {
    leftMotorTarget = 255;
    rightMotorTarget = -255;
  }
  if (dir == "Right") {
    leftMotorTarget = -255;
    rightMotorTarget = 255;
  }
  if (dir == "SlowLeft") {
    leftMotorTarget = 255;
    rightMotorTarget = -255;
  }
  if (dir == "SlowRight") {
    leftMotorTarget = -255;
    rightMotorTarget = 255;
  }
}



// ___ ALGORITHMS ___
void followLine() {
  if (onPerpendicularLine) setDriveDir("Forward");
  else if (!leftLineSensorVal() && !rightLineSensorVal()) setDriveDir("Forward");
  else if (leftLineSensorVal() && !rightLineSensorVal()) setDriveDir("Left");
  else if (!leftLineSensorVal() && rightLineSensorVal()) setDriveDir("Right");
}


bool returnToLine(bool leftOfLine) {
  setDriveDir(leftOfLine ? "SlowRight" : "SlowLeft");
  // Just stopped sensing line? (meaning it is now between the sensors)
  return (!rightLineSensorVal() && !leftLineSensorVal()) && (rightLineSensorPrev || leftLineSensorPrev);
}


// ___ MAIN ___
void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  liftServo.attach(9);  // attaches the servo on pin 9 to the servo object

  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  pinMode(HallPin, INPUT);
  pinMode(SWPin, INPUT);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledOrange, OUTPUT);
  Set_Ultrasonic();
  //Serial.begin(9600);

  setGrabberClosed(true);

  delay(1000);

  servoAngle = liftServo.read();
  while(servoAngle != servoAngleTarget) {
  updateServo();
  delay(10);
  }

  while(!digitalRead(SWPin)); // wait until swPin reads HIGH




}


void loop() {
  // Logic
  static int counter = 0;
  static unsigned long timer = 0;
  // To remember what the currently held block's type is
  static bool isBlockMetal = false;
  // When scanning off the line, this lets the robot know which side of the line it's on.
  static bool leftOfLine = true;
  static int blockNum = 0;
  



  switch (state) {

    case START: // Travel from start to pickup box
      setGrabberClosed(false);
      followLine();
      if (detectedPerpendicularLine) counter = counter + 1;
      if (counter == 2) {
        counter = 0;
        state = REACH_COLLECTION_POINT;
        if (blockNum == 0) timer = t + 14000; // test done
        else timer = t + 12500; // test done
      }


    break;


    case REACH_COLLECTION_POINT:
      if (t < timer) followLine(); // keep moving with grabber up
      else {
        followLine();
        GrabberDown();
        if (detectedPerpendicularLine)
        {
          timer = t + 1 * FORWARD_10cm_TIME ; // 0cm // test
          if (blockNum == 0) {
            state = PICK_UP_BLOCK;
          } else {
            state = STARTING_BLOCK_SEARCH;
          }
        }
      }

      break;


    case STARTING_BLOCK_SEARCH:
      if (t < timer) setDriveDir("Forward");
      else {
        state = SWEEP;  // sweep time // test
        leftOfLine = true;
        timer = t +  SWEEP_TIME;
      }
      break;

    case SWEEP:
      if (t < timer) { // Turning away from line
        setDriveDir(leftOfLine ? "SlowLeft" : "SlowRight");
      }
      else if (returnToLine(leftOfLine)) { // Returned back to line?
        if (leftOfLine) { // Returning from left
          timer = t + SWEEP_TIME;  // sweep time // test
        }
        else { // Returning from right
          state = SWEEP_FORWARD;
          timer = t + FORWARD_10cm_TIME;  // time moving forward
        }
        leftOfLine = !leftOfLine;
      }

      //else if (detectIsBlock() == 1) { // <else if] -> <if>

        state = PICK_UP_BLOCK;
        timer = t + 0;
      }
      break;

    case SWEEP_FORWARD:
      if (t < timer) setDriveDir("Forward");
      else {
        state = SWEEP;
        timer = t + SWEEP_TIME; // sweep time // test
        leftOfLine = true;
      }
      break;

    case PICK_UP_BLOCK:
      // TODO identify
      if (t < timer) setDriveDir("SlowForward");
      else {
        setDriveDir("Stop");
        isBlockMetal = detectMetal();
        ledActiveT = t + 8000;
        
        setGrabberClosed(true);
        timer = t + 2000; // test
        state = RETURN_TO_LINE;
      }
      break;

    case RETURN_TO_LINE:
      if (t < timer);
      else { 
        GrabberUp();
        if (returnToLine(leftOfLine) || (blockNum == 0)) {
          state = TURN_AROUND;
          timer = t + ROTATE_30_TIME * 4; // Time to skip lines // test
        }
      }

      break;

    case TURN_AROUND:
      if (returnToLine(false) && t > timer) {
        
        state = GO_TO_DROPOFF;
        counter = 0;
      }
      break;

    case GO_TO_DROPOFF:
      if (t < timer) setDriveDir("Forward");
      else if (counter == 1) {
        state = PICKING_DROPOFF;
        timer = t + ROTATE_30_TIME * 3;
      } else {
        followLine();
        if (detectedPerpendicularLine) {
          timer = t + FORWARD_10cm_TIME;
          counter = 1;
        }
      }
      break;

    case PICKING_DROPOFF:
      if (t < timer) setDriveDir(isBlockMetal ? "Right" : "Left");
      else {
        blockNum = blockNum + 1;
        state = DROPPING_BLOCK;
        timer = t + 1000; // test
      }
      break;

    case DROPPING_BLOCK:
      setDriveDir("Stop");
      setGrabberClosed(false);
      GrabberUp();
      if (t > timer) {
        state = TURNING_TO_START;
        timer = t + ROTATE_30_TIME * 2.5; // test done
      }
      break;

    case FIND_START_LINE:
      if (returnToLine(isBlockMetal) && t > timer) {
        state = REACH_COLLECTION_POINT;
      }
      break;

    case TURNING_TO_START:
      if (returnToLine(!isBlockMetal) && t > timer) {
        state = RETURN_TO_START;
        counter = 0;
      }
      break;

    case RETURN_TO_START:
      
      if (counter == 1) {
        setDriveDir("Forward");
        if (t > timer) state = END;
      }
      else {
        followLine();
        
        if (detectedPerpendicularLine) {
          timer = t + FORWARD_10cm_TIME * 2;
          counter = 1; 
        }
      }

      case END:
        setDriveDir("Stop");
        GrabberUp();
        setGrabberClosed(false);
      break;

  }


  // LED
  digitalWrite(ledOrange, ((t % 500) < 250) && (leftMotorCurrent != 0 || rightMotorCurrent != 0) ? HIGH : LOW);
  digitalWrite((isBlockMetal) ? ledRed : ledGreen, (t < ledActiveT) ? HIGH : LOW);

  // Loop updates
  updateMotors();
  updateSensing();

  /*
    if((millis()-t)>8) digitalWrite(ledRed,HIGH);
    else digitalWrite(ledRed,LOW);
  */

  //while((unsigned long)(millis() - t) <= 10){ // actual period ~7
  // wait here
  //}
  t = millis();
}
