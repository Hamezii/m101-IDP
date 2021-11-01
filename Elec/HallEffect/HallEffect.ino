int HallPin=A0;


void setup() {
  pinMode(HallPin, INPUT);
  Serial.begin(9600);

}

void loop() {
  long mag1 = analogRead(HallPin);
  delay(1);
  Serial.println(mag1);  // if metal detector can return voltage stably, try to place LEGO at different position to find a suitable threshold
  delay(2);

 
  /*
  if (mag1 > 600) { // Threshold about 3V
    Serial.println(1);
  }
  else Serial.println(0);
  */
}
