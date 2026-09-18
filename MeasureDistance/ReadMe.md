##     MeasureDistance

An ARDUINO sketch using two HC-SR04 ultrasonic sensors, it uses a low pass filter for the sensor readings, triangulates the object position relative to the right sensor, and calculates the angle towards the object relative to the midpoint between the sensors.

This is a picture of the setup:

![Picture](/MeasureDistance/IMG_8927.JPG)

### Parts:

- ARDUINO UNO R4, the one on the right in the picture, and breadboard
- Two HC-SR04 ultrasonic sensors
- Second breadboard, the one on the left, the second ARDUINO is not used, just the breadboard for holding the sensor
- Cables, the crocodile cables are my only cables long enough to separate the two sensors by 0.5m

### Software:

- HCSR04 ultrasonic sensor library v2.0.2 by gamegine, https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/releases

### Wiring:
- The HC-SR04 ultrasonic sensor has for pins, 5V, Ground, trigger and echo. This sketch uses 1 output pin for both sensors and two different echo pins for the sensors.
