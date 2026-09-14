/*
 DualCapacitiveFoil.ino - ARDUINO sketch for 2 capacitive foils, both need to be touched to trigger.

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

Tamiko Thiel, see www.TamikoThiel.com/
Peter Graf, see www.mission-base.com/peter/

*/
#include "CapacitiveSensor.h"

/*
  Wiring Diagram (1 Send Pin, 2 Receive Pins), Resistors are 10 MegaOhm!

			         +---[ Resistor 1 ]----+----o Receive Pin 1
			         |                     |
			         |                 [ Foil A ]
Send Pin  o----+
			         |
			         |
			         +---[ Resistor 2 ]----+----o Receive Pin 2
			                               |
			                           [ Foil B ]
  Sensor-Setup:
  Pin 7 - Send Pin
  Pin 5 - Receive Pin 1 for foil A
  Pin 9 - Receive Pin 2 for foil B
  Pin 11 - LED, just for debugging
 */
const int LED = 11;

CapacitiveSensor sensor1 = CapacitiveSensor(7, 5);
CapacitiveSensor sensor2 = CapacitiveSensor(7, 9);

void setup()
{
	// Turn off auto calibration
	sensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);
	sensor2.set_CS_AutocaL_Millis(0xFFFFFFFF);

	pinMode(LED, OUTPUT); // Pin 11, driving an LED using PWM

	Serial.begin(74880);
}

long sensorActivationThreshold = 1000; // This might need calibration

unsigned long nextSerialPrint = 0;
int serialPrintInterval = 1000; // In milli seconds

int ledIntensity = 0;
unsigned long nextIntensityCheck = 0;
int intensityCheckInterval = 20; // In milli seconds

void loop()
{
	unsigned long start = millis();

	// n readouts, build average
	long sensorValue1 = sensor1.capacitiveSensor(10);
	long sensorValue2 = sensor2.capacitiveSensor(10);

	unsigned long milliSeconds = millis();
	if (sensorValue1 > sensorActivationThreshold && sensorValue2 > sensorActivationThreshold)
	{
		ledIntensity = 255;
		nextIntensityCheck = milliSeconds + intensityCheckInterval;
	}
	else if (milliSeconds > nextIntensityCheck)
	{
		if (ledIntensity > 0)
		{
			ledIntensity--;
			nextIntensityCheck = milliSeconds + intensityCheckInterval;
		}
		else
		{
			nextIntensityCheck = 0xffffffff;
		}
	}
	analogWrite(LED, ledIntensity);

	milliSeconds = millis();
	if (milliSeconds > nextSerialPrint)
	{
		nextSerialPrint = milliSeconds + serialPrintInterval;

		Serial.print("3\t");                    // Pin number
		Serial.print(milliSeconds - start);     // Time performance
		Serial.print("\t");                     // Tab
		Serial.print(sensorValue1);             // Sensor value
		Serial.print("\t");                     // Tab
		Serial.println(ledIntensity);           // Led intensity

		Serial.print("4\t");                    // Pin number
		Serial.print(milliSeconds - start);     // Time performance
		Serial.print("\t");                     // Tab
		Serial.print(sensorValue2);             // Sensor value
		Serial.print("\t");                     // Tab
		Serial.println(ledIntensity);           // Led intensity
	}

	delay(3); // Short delay
}

