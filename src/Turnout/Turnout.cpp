#include "Arduino.h"
#include "Turnout.h"


Turnout::Turnout(int id, String name) {
    this->setName(name);
    this->id = id;
}
