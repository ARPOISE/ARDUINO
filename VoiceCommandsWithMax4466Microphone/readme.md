##     VoiceCommandsWithMax4466Microphone

An ARDUINO sketch for 6 simple voice commands switching on LEDs.

This is a picture of the setup:

![Picture](./IMG_9415.JPG)

The command words to be used to light the LEDs have to be entered in a training session after the sketch is started for the first time. After the training data has been entered, i.e. spoken into the microphone, the data is stored in the EEPROM of the ARDUINO.
When the sketch starts and finds stored training data on the EEPROM, the data is loaded and no training session is started. Entering the message 't' in the Serail Monitor starts a new training session. 
Each word for the five colors and the 'off' command has to be spoken five times. In my example I used the words "red", "yellow", "green", "blue", "white", and "off".
When the sketch is not in training mode and one of the command words is recognized, the corresponding LED is turned on, and all others off. If the 'off' command is recognized, all LEDS are turned off.

### Parts:

- ARDUINO UNO R4 WIFI and breadboard
- One Microphone Amplifier - MAX4466, https://www.adafruit.com/product/1063
- Five 220 Ohm resistors
- Five LEDs I used the colors red, yellow, green, blue, and white
- Cables
