#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  leftMotor->setSpeed(255);
  rightMotor->setSpeed(255);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  //delay(3000);
  //leftMotor->run(FORWARD);
  //rightMotor->run(BACKWARD);
  delay(2000);
  

}
