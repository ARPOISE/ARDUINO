##     FollowTheHand

An ARDUINO sketch using two HC-SR04 and two HY-SRF05 ultrasonic sensors, it triangulates the object position of the object nearest to the sensors, and animates a Unity 3d virtual head to look at that object.

This is a picture of the setup:

![Picture](/FollowTheHand/FollowTheHand.JPG)

The two HY-SRF05 ultrasonic sensors are located on top of the monitor, left and right of the middle of the monitor, they are called TL (top left) and TR (top right) in the code.
The two HY-SR04 ultrasonic sensors are located below the monitor, left and right of the middle of the monitor, they are called BL (bottom left) and BR (bottom right) in the code.

Here is the description from the code:
```text
// ========================================================
// SIMULTANEOUS FOUR-SENSOR 3D HAND TRACKER
// Shared TRIG: D8 (all four sensors)
// ECHO: TL=D3, TR=D4, BL=D5, BR=D6
// Arduino UNO R4 WiFi
//
// Sensor positions:
//
// TL = ( 0, 52, -5)
// TR = (28, 52, -5)
// BL = ( 0,  0,  0)
// BR = (28,  0,  0)
//
// +X = your right
// +Y = upward
// +Z = toward you
//
// All four sensors fire simultaneously.
//
// Position algorithm:
//
// 1. <20 cm corner override.
// 2. Reject <5 cm and >70 cm.
// 3. If four readings valid, discard largest.
// 4. Trilaterate from remaining three.
// 5. If exactly three are valid, use those three.
// 6. Select solution in front of monitor.
//
// Unity output:
//
// POS,x,y,z
// ========================================================
```
The position of the nearest object calculated from the sensor outputs is sent via Serial to a Unity 3d project. This project shows a very simple head with eyes, the head and eyes are animated so that they look at the nearest object. This is what it looks like:

https://github.com/user-attachments/assets/d95815f4-1e74-4599-b4b7-c438d0c05fbc


### Parts:

- ARDUINO UNO R4, and breadboard
- Two HC-SR04 ultrasonic sensors, the ones below the monitor
- Two HY-SRF05 ultrasonic sensors, the ones on to of the monitor
- Second breadboard for holding the sensor on the left
- Cables

### Software

- Standard ARDUINO software.

### Wiring:
The sketch uses digital pin D8 in order to trigger all 4 ultrasound sensor simultaneously. The digital pins D3, D4, D5, and D6 are used for receiving the echos of the four sensors.
![Picture](/FollowTheHand/ArduinoWiring.JPG)
From left to right, ground, echo, trigger, 5V.

Wiring of the HY-SRF05 sensors:
![Picture](/FollowTheHand/HY-SRF06-Wiring.JPG)
From left to right, ground, echo, trigger, 5V.

Wiring of the HY-SR04 sensors:
![Picture](/FollowTheHand/HC-SR04-Wiring.JPG)
From left to right, 5V, trigger, echo, ground,.





