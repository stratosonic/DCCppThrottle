#include "Component.h"
#include "Arduino.h"

Component::Component() {
    this->active = false;
}

char *Component::getName() {
    return this->name;
}

void Component::setName(char *name) {
    strcpy(this->name, name);
}

void Component::toggleActive() {
    this->active = !this->active;
}

boolean Component::isActive() {
    return this->active;
}
