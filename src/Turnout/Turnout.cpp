#include "Turnout.h"
#include "Arduino.h"

Turnout::Turnout(int id, char *name) {
    this->setName(name);
    this->id = id;
}
