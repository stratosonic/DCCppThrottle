#include "Turnout.h"
#include "Arduino.h"

Turnout::Turnout(int id, String name) {
    this->setName(name);
    this->id = id;
}
