DCC++ Throttle

Throttle implementation using an Arduino Mega 2560 or ESP32.

If this is run on an Arduino Mega 2560 board, then the communication to the DCC++
base station will be a wired serial connection. If run on an ESP32, the communication
is done via WiFi.

A slight modification to the stock DCC++ base station code is needed in order to
make the WiFi connection work. The ESP32 is expecting a response back but the stock
version of DCC++ does not provide this.
