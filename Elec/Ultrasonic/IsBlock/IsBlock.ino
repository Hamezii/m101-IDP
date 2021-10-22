int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
//long H_Ultra = 8;   // Height of ultrasonic sensor (cm)
long T_Gnd_min = 800, T_Gnd_max = 900;
long T_Block_max = 700, T_Block_far_max = 1000;
// need to test value after fix ulrtasonic

void Set_Ultrasonic();
long GetTime_Ultrasonic();
int IsBlock(); //Ground->0; Block->1; Block a bit far->2

void setup() {
  Serial.begin (9600);
  Set_Ultrasonic();
}
 
void loop() {
  Serial.println(  IsBlock()  );
  //Serial.println(GetTime_Ultrasonic());
  delay(2);
}




void Set_Ultrasonic(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


long GetTime_Ultrasonic(){
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
    delay(1);
  }
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.


  // Convert the time into a distance
  //cm = (duration/2) / 29.1+1;     // Divide by 29.1 or multiply by 0.0343

  if(duration<10000) return(duration/5);
}


int IsBlock(){ // Ground->0; Block->1; Block a bit far->2
  long Time=GetTime_Ultrasonic();
  if((Time<T_Block_max)) return 1;
  else if((Time>T_Gnd_min)&&(Time<T_Gnd_max)) return 0;
  else if((Time>T_Gnd_max)&&(Time<T_Block_far_max)) return 2;
  else return -1;

}
