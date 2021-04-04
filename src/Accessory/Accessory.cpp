#include "Accessory.h"
#include "Arduino.h"

Accessory::Accessory() {
    this->setName("Acc");
}

Accessory::Accessory(char *name, int address, int subAddress) {
    this->setName(name);
    this->address = address;
    this->subAddress = subAddress;
}
