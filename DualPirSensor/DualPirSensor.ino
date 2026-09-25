/*
 DualPirSensor.ino - ARDUINO sketch for 2 PIR sensors, both need to be HIGH to trigger.

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
  Pin 2 - Receive Pin for sensor 1
  Pin 3 - Receive Pin for sensor 2
  Pin 13 - LED, just for debugging
 */
const int ledPin = 13;
const int sensor1Pin = 2;
const int sensor2Pin = 3;

int sensorState = LOW;
int sensor1Value = 0;
int sensor2Value = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  Serial.begin(115200);
}

unsigned long nextSerialPrint = 0;
int serialPrintInterval = 1000;  // In milliseconds

void loop() {
  sensor1Value = digitalRead(sensor1Pin);
  sensor2Value = digitalRead(sensor2Pin);
  if (sensor1Value == HIGH && sensor2Value == HIGH) {
    digitalWrite(ledPin, HIGH);

    if (sensorState == LOW) {
      Serial.println("Motion detected!");
      sensorState = HIGH;
    }
  } else {
    digitalWrite(ledPin, LOW);

    if (sensorState == HIGH) {
      Serial.println("Motion stopped!");
      sensorState = LOW;
    }
  }

  unsigned long now = millis();
  if (now >= nextSerialPrint) {
    nextSerialPrint = now + serialPrintInterval;

    Serial.print("sensorValues ");
    Serial.print(sensor1Value);
    Serial.print(" | ");
    Serial.println(sensor2Value);
  }

  delay(3);  // Short delay
}
