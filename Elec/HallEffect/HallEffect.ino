int HallPin=A0;


void setup() {
  pinMode(HallPin, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int mag1 = analogRead(HallPin);
  delay(1);
  mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
  delay(1);
   mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
   delay(1);
  mag1 += analogRead(HallPin);
  
  Serial.println(mag1/9);
  delay(2);
}
