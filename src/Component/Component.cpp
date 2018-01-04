#include "Arduino.h"
#include "Component.h"

Component::Component() {
    this->active = false;
}

String Component::getName() {
    return this->name;
}

void Component::setName(String name) {
    this->name = name;
}

void Component::toggleActive() {
    this->active = !this->active;
}

boolean Component::isActive() {
    return this->active;
}
