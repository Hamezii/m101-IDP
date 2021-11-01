// Metal detector
// Runs a pulse over the search loop in series with resistor
// Voltage over search loop spikes
// Through a diode this charges a capacitor
// Value of capacitor after series of pulses is read by ADC
 
// Metal objects near search loop change inductance.
// ADC reading depends on inductance.
// changes wrt long-running mean are indicated by LEDs
// LED1 indicates rise in inductance
// LED2 indicates fall in inductance
// the flash rate indicates how large the difference is
 
// wiring:
// 220Ohm resistor on D2
// 10-loop D=10cm seach loop between ground and resistor
// diode (-) on pin A0 and (+) on loop-resistor connection
// 10nF capacitor between A0 and ground
// LED1 in series with 220Ohm resistor on pin 8
// LED2 in series with 220Ohm resistor on pin 9
 
// First time, run with with serial print on and tune value of npulse
// to get capacitor reading between 200 and 300


//#include <Filters.h>

float x[] = {0,0,0};
float y[] = {0,0,0};
int k = 0;



const byte npulse = 1;
const bool debug = true;
 
const byte pin_pulse=3;
const byte pin_cap  =A1;
const byte pin_gnd  =A2;
const byte pin_LED1 =12;
const byte pin_LED2 =11;

 
void setup() {
  if (debug) Serial.begin(9600);
  pinMode(pin_pulse, OUTPUT);
  digitalWrite(pin_pulse, LOW);
  pinMode(pin_cap, INPUT); 
  pinMode(pin_gnd, INPUT);  
  
  pinMode(pin_LED1, OUTPUT);
  digitalWrite(pin_LED1, LOW);
  pinMode(pin_LED2, OUTPUT);
  digitalWrite(pin_LED2, LOW);
}
 
const int nmeas=512;  //measurements to take
long int sumsum=0; //running sum of 64 sums
long int skip=0;   //number of skipped sums
long int diff=0;        //difference between sum and avgsum
long int flash_period=0;//period (in ms)
long unsigned int prev_flash=0; //time stamp of previous flash
 
void loop() {
 
  int minval=1023;
  int maxval=0;
 
  //perform measurement
  long unsigned int sum=0;
  for (int imeas=0; imeas<nmeas+2; imeas++){
    //reset the capacitor
    pinMode(pin_cap,OUTPUT);
    digitalWrite(pin_cap,LOW);
    delayMicroseconds(20);
    pinMode(pin_cap,INPUT);
    //apply pulses
    for (int ipulse = 0; ipulse < npulse; ipulse++) {
      digitalWrite(pin_pulse,HIGH); //takes 3.5 microseconds
      delayMicroseconds(8);
      digitalWrite(pin_pulse,LOW);  //takes 3.5 microseconds
      delayMicroseconds(8);
    }
    //read the charge on the capacitor
    int val = analogRead(pin_cap)-analogRead(pin_gnd); //takes 13x8=104 microseconds
    minval = min(val,minval);
    maxval = max(val,maxval);
    sum+=val;
   
 
    //determine if LEDs should be on or off
    long unsigned int timestamp=millis();
    byte ledstat=0;
    if (timestamp<prev_flash+10){
      if (diff>50){
        ledstat=1;
      }
      if (diff<-50){
        ledstat=1;
      }
      else ledstat = 0;
    }
    if (timestamp>prev_flash+flash_period){
      if (diff>50){
        ledstat=1;
       
      }
      if (diff<-50){
        ledstat=1;
       
      }
      else ledstat = 0;
      prev_flash=timestamp;  
    }
    if (flash_period>11000)ledstat=0;
 
    //switch the LEDs to this setting
    if (ledstat==0){
      digitalWrite(pin_LED1,LOW);
      digitalWrite(pin_LED2,LOW);
    }
    if (ledstat==1){
      digitalWrite(pin_LED1,HIGH);
     
      digitalWrite(pin_LED2,LOW);
    }
    if (ledstat==2){
      digitalWrite(pin_LED1,LOW);
      digitalWrite(pin_LED2,HIGH);
    }
 
  }
 
  //subtract minimum and maximum value to remove spikes
  sum-=minval; sum-=maxval;
 
 
  //process
  if (sumsum==0) sumsum=sum<<7; //set sumsum to expected value
  long int avgsum=(sumsum+64)>>7;
  diff=sum-avgsum;
 
 
  if (abs(diff)<avgsum>>10){      //adjust for small changes
    sumsum=sumsum+sum-avgsum;
    skip=0;
  } else {
    skip++;
  }
  if (skip>128){     // break off in case of prolonged skipping
    sumsum=sum<<7;
    skip=0;
  }
 
  // one permille change = 2 ticks/s
  if (diff==0) flash_period = 1000000;
  else flash_period=avgsum/(2*abs(diff));   

  x[0] = diff;
  float b[] = {0.00024132, 0.00048264, 0.00024132};
  float a[] = {1.95558189, -0.95654717};
  y[0] = a[0]*y[1] + a[1]*y[2] +
               b[0]*x[0] + b[1]*x[1] + b[2]*x[2];

/*
  if(k % 3 ==0)
  {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
    
    // For the serial monitor
    Serial.print(2*x[0]);
    Serial.print(" ");
    Serial.println(2*y[0]);
  }
  */

  delay(1); // Wait 1ms
  for(int i = 1; i >= 0; i--){
    x[i+1] = x[i]; // store xi
    y[i+1] = y[i]; // store yi
  }
  
  k = k+1;

  Serial.println(y[0]);
    /*Serial.println(nmeas);
    Serial.print(" ");
    Serial.println(minval);
    Serial.print(" ");
    Serial.println(maxval);
    Serial.print(" ");
    Serial.println(sum);
    Serial.print(" ");
    Serial.println(avgsum);
    Serial.print(" ");
    Serial.println(diff);
    Serial.print(" ");
    Serial.println(flash_period);
    Serial.print(" "); */
 
 
}
