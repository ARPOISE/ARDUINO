## DualCapacitiveFoil

An ARDUINO sketch to read the inputs from two capacity foils. 

The red LED is turned on if both foils are triggered at the same time.

This is a picture of the setup:

![Picture](/DualCapacitiveFoil/images/IMG_8925.JPG)

### Parts:

- ARDUINO UNO R3 and breadboard
- Two 10 MegaOhm resistors
- Two pieces of folded household aluminum foil
- One red LED
- One 220 Ohm resistor
- Cables

### Software:
- CapacitiveSensor Library, v0.5.1, https://github.com/PaulStoffregen/CapacitiveSensor

### Wiring:
```text
/*
  Wiring Diagram (1 Send Pin, 2 Receive Pins), Resistors are 10 MegaOhm!

			         +---[ Resistor 1 ]----+----o Receive Pin 1
			         |                     |
			         |                 [ Foil A ]
Send Pin  o----------+
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
```
