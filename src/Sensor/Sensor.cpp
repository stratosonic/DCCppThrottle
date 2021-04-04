#include "Sensor.h"
#include "Arduino.h"

Sensor::Sensor() { this->setName("S"); }

Sensor::Sensor(String name) {
    this->setName(name);
}
