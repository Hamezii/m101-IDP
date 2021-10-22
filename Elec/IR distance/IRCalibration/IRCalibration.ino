#include "SharpIR.h"
/* 
 * Don't use <SharpIR.h>, this is different library with "SharpIR.h"
 */



#define ir A0
#define model 1080
int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long duration;
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y, 10-80cm
//                                            20150 for GP2Y0A02Y, 20-150cm
//                                            (working distance range according to the datasheets)

SharpIR SharpIR(ir, model);

int GetDisUltra(){
// The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  long cm = (duration/2) *0.0343+1;     // Divide by 29.1 or multiply by 0.0343
  //inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135

  if(cm<500){
    return cm;
  }
}
  
  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {   

  //unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  int IR_Dis = SharpIR.distance();  // this returns the distance to the object you're measuring

  //if(MeanDis>=80) MeanDis=80;
  //if(MeanDis<=9) MeanDis=9;
  Serial.println(IR_Dis);

  int Ultra_Dis = GetDisUltra();
  
  //Serial.println(Ultra_Dis);
  delay(50);
  
  //Serial.print("Mean distance: ");  // returns it to the serial monitor
  
  //unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  //Serial.print("Time taken (ms): ");
  //Serial.println(pepe2);  

}
