#include <RunningAverage.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
Servo grabberServo; 
#define leftLineSensor A0
#define rightLineSensor A1
lastCall = "None"


void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  grabberServo.attach(10);  // attaches the servo on pin 10 to the servo object
  pinMode(leftLineSensor,INPUT);
  pinMode(rightLineSensor,INPUT);
  Serial.begin(9600);
}

void forward(){
  int MotorSpeed = 0
  int TargetSpeed = 255
  while (MotorSpeed <= TargetSpeed){
    MotorSpeed = min(MotorSpeed + 10, TargetSpeed) //MotorSpeed += 10;
    leftMotor->setSpeed(MotorSpeed);
    rightMotor->setSpeed(MotorSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(5);
  lastCall = "forward";
  }
}

void backward(){
  int MotorSpeed = 0
  int TargetSpeed = 255
  while (MotorSpeed <= TargetSpeed){
    MotorSpeed = min(MotorSpeed + 10, TargetSpeed) //MotorSpeed += 10;
    leftMotor->setSpeed(MotorSpeed);
    rightMotor->setSpeed(MotorSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(5);
  lastCall = "backward";
  }
}

void left(){
  int MotorSpeed = 0
  int TargetSpeed = 255
  while (MotorSpeed <= TargetSpeed){
    MotorSpeed = min(MotorSpeed + 10, TargetSpeed) //MotorSpeed += 10;
    leftMotor->setSpeed(MotorSpeed);
    rightMotor->setSpeed(MotorSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay(5);
  lastCall = "left";
  }
}


void right(){
  int MotorSpeed = 0
  int TargetSpeed = 255
  while (MotorSpeed <= TargetSpeed){
    MotorSpeed = min(MotorSpeed + 10, TargetSpeed) //MotorSpeed += 10;
    leftMotor->setSpeed(MotorSpeed);
    rightMotor->setSpeed(MotorSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    delay(5);
  lastCall = "right";
  }
}

// This FOR loop also works
//void right(){
//  int TargetSpeed = 255
//  leftMotor->run(FORWARD);
//  rightMotor->run(BACKWARD);
//  for (int MotorSpeed = 0; MotorSpeed <= TargetSpeed; MotorSpeed = min(MotorSpeed + 10, TargetSpeed){
//    leftMotor->setSpeed(MotorSpeed);
//    rightMotor->setSpeed(MotorSpeed);
//    delay(5);
//  lastCall = "right";
//  }
//}


void rest(){
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

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

void FollowLine(){
  
  //line detected by neither
  if(digitalRead(leftLineSensor)==0 && digitalRead(rightLineSensor)==0){
    //Forward
    if (!lastCall == "forward"){
      lastCall = "forward";
      forward();
    }
  }
  //line detected by left sensor
  else if(digitalRead(left)==1 && digitalRead(right)==0){
    //turn left
    if (!lastCall == "left"){
      lastCall = "left";
      left();
    }
  }
  //line detected by right sensor
  else if(digitalRead(left)==0 && digitalRead(right)==1){
    //turn right
    if (!lastCall == "right"){
      lastCall = "right";
      right();
    }
  }
  //line detected by none
  else if(digitalRead(left)==0 && digitalRead(right)==0){
    //lost
    lostLine(lastCall);
   
  }
}

void lostLine(lastCall){
  if (lastCall == "forward"){
    backward();
  else if (lastCall == "backward"){
    forward();
  else if (lastCall == "left"){
    right();
  else if (lastCall == "right"){
    left();
  FollowLine();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
