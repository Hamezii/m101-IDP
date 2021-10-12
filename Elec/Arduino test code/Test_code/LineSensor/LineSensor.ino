

//int lineSensor1 =2;
int lineSensor1 =A5;
// normally, we just use a digital port to read (i.e. 1/2/3...)

// the setup routine runs once when you press reset:
void setup() {
  pinMode(lineSensor1, INPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  //int line1 = digitalRead(lineSensor1);
  int line1 = analogRead(lineSensor1);
  // print out the state of the button:
  Serial.println(line1);
  delay(1);        // delay in between reads for stability
}
