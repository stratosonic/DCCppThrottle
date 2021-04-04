#include "Sensor.h"
#include "Arduino.h"

Sensor::Sensor() { this->setName("S"); }

Sensor::Sensor(char *name) {
    this->setName(name);
}
