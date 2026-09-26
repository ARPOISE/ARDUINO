## DualPirSensor

An ARDUINO sketch to read the inputs from two HW-416A passive infrared sensors. 

The LED is turned on if both sensors are HIGH at the same time.

This is a picture of the setup:

![Picture](/DualPirSensor/IMG_8976.JPG)

### Parts:

- ARDUINO UNO R4 and breadboard
- Two HW-416A passive infrared sensors
- One LED, any color
- One 220 Ohm resistor
- Cables

### Software:
- Arduino standard

### Wiring:
```text
  Sensor-Setup:
  Pin 2 - Receive Pin for sensor 1
  Pin 3 - Receive Pin for sensor 2
  Pin 13 - LED, just for debugging
```
- The HW-416A passive infrared sensor has three pins, they are Ground (black in the picture), output (white), and 5V (red).
### Hints

There are two small orange adjustment screws (potentiometers) on the board.
The one labeled TIME (the one on the left side in the picture) controls how long the output stays HIGH after detecting motion.
If it is turned all the way clockwise, the signal will stay 1 for up to 5 to 7 minutes from a single movement.
Turning the TIME potentiometer all the way counter-clockwise reduces the hold time to its minimum (about 3 seconds),
allowing it to drop back to 0 quickly.

The other potentiometer, labeled SENS (Sensitivity), controls the detection distance.
If it is set to the maximum, the sensor can trigger on invisible air currents, heat from the Arduino itself,
or motion through thin walls.
Your can change this by turning the SENS potentiometer counter-clockwise towards the minimum.

