#include "Arduino.h"

#ifndef COMPONENT_H
#define COMPONENT_H

class Component {
  private:
    char name[13];
    boolean active;

  public:
    Component();
    char *getName();
    void setName(char *name);
    void toggleActive();
    boolean isActive();
};

#endif
