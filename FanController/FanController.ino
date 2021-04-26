int fanPin = 6; // LED connected to digital pin 9

void setup() {
  pinMode(fanPin, OUTPUT); // sets the pin as output
}

void loop() {
  // 15.20 mA - max current
  // 6.85 mA - pin06 current
  // analogRead values go from 0 to 1023, 
  // analogWrite values from 0 to 255
  int output = int(255 * 0.99);
  analogWrite(fanPin, output);
//  digitalWrite(fanPin, HIGH);
}
