/*
 MeasureDistance.ino - ARDUINO sketch using two HC-SR04 ultrasonic sensors,
                       uses a low pass filter for the sensor readings,
                       triangulates the object position relative to the right sensor,
                       calculates the angle towards the object relative to the midpoint between the sensors.

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

Tamiko Thiel, see https://www.TamikoThiel.com/
Peter Graf, see https://www.mission-base.com/peter/

*/

// Uses the HCSR04 ultrasonic sensor library v2.0.2 by gamegine
// https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/releases
#include <HCSR04.h>

// Uses one trigger pin, connected to both sensors
// And two echo pins, one for each sensor
//
HCSR04 hc(2, new int[2]{ 3, 4 }, 2);  //initialisation class HCSR04 (trigger pin, echo pin)

unsigned long nextSerialPrint = 0;
int serialPrintInterval = 500;  // In milliseconds

void setup() {
  Serial.begin(74880);
}

// Filter the sensor readings
class LowPassFilter {
private:
  float alpha;
  float last_val;
  bool is_initialized;

public:
  // Constructor to set the smoothing factor alpha (0.0 to 1.0)
  LowPassFilter(float alpha_val) {
    alpha = alpha_val;
    last_val = 0.0f;
    is_initialized = false;
  }

  // Process a new noisy reading and return the smoothed value
  float update(float new_val) {
    if (!is_initialized) {
      last_val = new_val;
      is_initialized = true;
      return new_val;
    }

    // Filter formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    last_val = (alpha * new_val) + ((1.0f - alpha) * last_val);
    return last_val;
  }
};

// Initialize the filter with an alpha
// 0.10 – 0.25: Balanced: General analog inputs, joysticks, battery voltage
// 0.50 – 0.80: Low Lag: Motion tracking, etc, interfaces requiring quick reactions
LowPassFilter sensorFilter1(0.25f);
LowPassFilter sensorFilter2(0.25f);

// Distance between your two sensors in meters
const float SENSOR_SPACING = 50.0f;

// Position of the object, relative to the right sensor, from the perspective of the sensors
//
float objectX = 0.0f;
float objectY = 0.0f;
float objectAngle = 0.0f;  // Angle in degrees relative to center-forward

// --- TRIANGULATION FUNCTION ---
// Updates global objectX and objectY. Returns true if valid, false if out of bounds.
bool calculatePosition(float d1, float d2, float D) {
  // 1. Calculate X position along the baseline
  objectX = (pow(d1, 2) + pow(D, 2) - pow(d2, 2)) / (2.0f * D);

  // 2. Check if the math is physically possible
  float y_squared = pow(d1, 2) - pow(objectX, 2);
  if (y_squared < 0) {
    objectX = 0.0f;
    objectY = 0.0f;
    return false;  // Error: Circles do not intersect
  }

  // 3. Calculate Y position out in front
  objectY = sqrt(y_squared);
  return true;  // Success
}

void loop() {
  bool calculateWorked = false;

  float dist1 = sensorFilter1.update(hc.dist(0));
  delay(20);
  float dist2 = sensorFilter2.update(hc.dist(1));
  delay(20);

  if (calculatePosition(dist1, dist2, SENSOR_SPACING)) {
    calculateWorked = true;
    // 1. Calculate X relative to the midpoint observer
    float x_relative_to_midpoint = objectX - (SENSOR_SPACING / 2.0f);

    // 2. Calculate angle from the forward-facing Y axis
    // atan2(x, y) gives 0 radians when looking straight along the Y axis
    float angle_radians = atan2(x_relative_to_midpoint, objectY);

    // 3. Convert to degrees for human readability
    objectAngle = angle_radians * (180.0f / PI);
  }

  unsigned long now = millis();
  if (now >= nextSerialPrint) {
    nextSerialPrint = now + serialPrintInterval;

    Serial.print(dist1);
    Serial.print(" | ");
    Serial.print(dist2);
    Serial.print(" | ");

    if (calculateWorked) {
      // Math succeeded! Print the coordinates
      Serial.print("Object Position -> X: ");
      Serial.print(objectX);
      Serial.print(" cm, Y: ");
      Serial.print(objectY);
      Serial.print(" cm | Angle: ");
      Serial.print(objectAngle);
      Serial.println("°");
    } else {
      Serial.println("Error: Geometric conflict (Target out of bounds)");
    }
  }
}
