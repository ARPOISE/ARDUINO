/*
 PiezoPitchDelay.ino - ARDUINO sketch for 2 potentiometers driving pitch and delay of a piezo.

MIT License

Copyright (C) 2026, Tamiko Thiel and Peter Graf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

For more information on

Tamiko Thiel, see https://TamikoThiel.com/
Peter Graf, see https://mission-base.com/peter/

*/
/*
  Sensor-Setup:
  Pin A0 - Control/middle pin from potentiometer 1, the other two pins of the pot are connected to 5V and Ground
  Pin A1 - Control/middle pin from potentiometer 2, the other two pins of the pot are connected to 5V and Ground
  Pin 9  - Voltage pin of the piezo, the other pin of the piezo is connected to Ground
 */
// Define pins for the potentiometers
const int potPin1 = A0;
const int potPin2 = A1;

// Define pin for the piezo buzzer
const int piezoPin = 9;

int pitchValue = 0;
int delayValue = 0;

void setup() {
  // Define the piezo pin as an OUTPUT
  pinMode(piezoPin, OUTPUT);
  Serial.begin(115200);
}

unsigned long nextSerialPrint = 0;
int serialPrintInterval = 1000;  // In milliseconds

void loop() {
  // Read Pot 1 and map it to the piezo's clear, audible range (A4 to C8)
  int rawPot1 = analogRead(potPin1);
  pitchValue = map(rawPot1, 0, 1023, 440, 4186);
  
  // Read Pot 2 and map it to the delay range (50ms to 500ms)
  int rawPot2 = analogRead(potPin2);
  delayValue = map(rawPot2, 0, 1023, 50, 500);
  
  // Play the frequency on the piezo buzzer if the knob is turned up
  if (rawPot1 > 10) { 
    tone(piezoPin, pitchValue);
  } else {
    pitchValue = 0;
    noTone(piezoPin); // Silence if the pitch knob is fully counter-clockwise
  }
  
  // Wait based on the second potentiometer's position
  delay(delayValue);
  
  // Briefly stop the sound to create a distinct pulsing/beeping effect
  noTone(piezoPin);

  unsigned long now = millis();
  if (now >= nextSerialPrint) {
    nextSerialPrint = now + serialPrintInterval;

    Serial.print("rawPot1 ");
    Serial.print(rawPot1);
    Serial.print(" | ");
    Serial.print("rawPot2 ");
    Serial.print(rawPot2);
    Serial.print(" | ");
    Serial.print("pitchValue ");
    Serial.print(pitchValue);
    Serial.print(" | ");
    Serial.print("delayValue ");
    Serial.println(delayValue);
  }
  delay(10); 
}
