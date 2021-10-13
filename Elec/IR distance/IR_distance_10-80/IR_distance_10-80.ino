int receivePin = A4;

void setup() {
  Serial.begin (9600);
  //Define inputs
  pinMode(receivePin, INPUT);
}

void loop() {
  float Vir = analogRead(receivePin);
  // get voltage
  delay(100); // test period <=50ms

  if((Vir<600)&&(Vir>60)){
  //Serial.println(Vir);
  float distance = 26.39*pow((Vir*5/1024),(-1.245)); // in cm
  Serial.println(distance);
  }
  else{
    Serial.println(0);

    
  }
  
  
}
