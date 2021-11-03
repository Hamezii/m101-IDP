int HallPin=A0;
byte metalstate = 0;
void setup() {
  pinMode(HallPin, INPUT);
  Serial.begin(9600);
}
void loop() {
  long mag1 = analogRead(HallPin);
  delay(2);
  mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);
    mag1 += analogRead(HallPin);
  delay(2);  
  mag1 += analogRead(HallPin);
  delay(2);

mag1/=10;
  Serial.println(mag1);  // if metal detector can return voltage stably, try to place LEGO at different position to find a suitable threshold
/*
  if (mag1 > 530) { // Threshold about 3V
    Serial.println(1);
    metalstate = 1;
  }
  else {
    Serial.println(0);
    metalstate = 0;
  }
  */
  //Serial.println(metalstate);
}
