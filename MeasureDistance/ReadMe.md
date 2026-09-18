##     MeasureDistance

An ARDUINO sketch using two HC-SR04 ultrasonic sensors, it uses a low pass filter for the sensor readings, triangulates the object position relative to the right sensor, and calculates the angle towards the object relative to the midpoint between the sensors.

This is a picture of the setup:

![Picture](/MeasureDistance/IMG_8927.JPG)

Parts used:

- ARDUINO UNO R4 and breadboard, the one on the right
- Two HC-SR04 ultrasonic sensors
- Second breadboard, the one on the left in the picture, the second ARDUINO is not used, just the breadboard for holding the sensor
- Cables, the crocodile cables are my only cables long enough to separate the two sensors by 0.5m
