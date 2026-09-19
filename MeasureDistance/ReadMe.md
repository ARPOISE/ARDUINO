##     MeasureDistance

An ARDUINO sketch using two HC-SR04 ultrasonic sensors, it uses a low pass filter for the sensor readings, triangulates the object position relative to the right sensor, calculates the angle towards the object relative to the midpoint between the sensors, and uses the servo motor in order to point in the direction of the object sensed.

This is a picture of the setup:

![Picture](/MeasureDistance/IMG_8932.JPG)

### Parts:

- ARDUINO UNO R4, the one on the right in the picture, and breadboard
- Two HC-SR04 ultrasonic sensors
- Second breadboard, the one on the left, the second ARDUINO is not used, just the breadboard for holding the sensor
- The servo motor, for pointing in the direction of the object sensed
- A 100uF capacitor for smoothing out voltage changes, as decoupling capacitor
- Cables, the crocodile cables are my only cables long enough to separate the two sensors by 0.5m

### Software:

- HC-SR04 ultrasonic sensor library, v2.0.2, by gamegine, https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib/releases
- The servo library, https://docs.arduino.cc/libraries/servo/

### Wiring:
- The HC-SR04 ultrasonic sensor has four pins, they are 5V, Ground, trigger and echo. This sketch uses 1 output pin for both sensors and two different echo pins for the sensors.
