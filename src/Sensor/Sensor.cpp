#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor() { this->setName("S"); }

Sensor::Sensor(String name) {
    this->setName(name);
}
