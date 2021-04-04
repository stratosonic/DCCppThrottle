#include "Arduino.h"

#ifndef COMPONENT_H
#define COMPONENT_H

class Component {
  private:
    String name;
    boolean active;

  public:
    Component();
    String getName();
    void setName(String name);
    void toggleActive();
    boolean isActive();
};

#endif
