#include "Arduino.h"
#include "Component/Component.h"

#ifndef SENSOR_H
#define SENSOR_H

class Sensor : public Component {
  private:
  public:
    Sensor();
    Sensor(char *name);
};

#endif
