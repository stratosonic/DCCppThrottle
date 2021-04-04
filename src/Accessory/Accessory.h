#include "Arduino.h"
#include "Component/Component.h"

#ifndef ACCESSORY_H
#define ACCESSORY_H

class Accessory : public Component {
  private:
    int address;
    int subAddress;

  public:
    Accessory();
    Accessory(char *name, int address, int subAddress);
};

#endif
