DCC++ Throttle

DCC++ EX throttle implementation using an ESP32. Communication to DCC++ EX is done over Wifi via TCP.

The ESP32 interfaces with an 320x240 SPI LCD, 5 momentary buttons (track power, menu, and three for navigation), a rotary encoder (for train speed and changing direction), and a X27 168 stepper motor for an analog speed indication.

Development is done in Visual Studio Code with PlatformIO.