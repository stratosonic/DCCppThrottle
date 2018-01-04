#include "Arduino.h"
#include "Component/Component.h"

#ifndef TURNOUT_H
#define TURNOUT_H


class Turnout : public Component {
private:
int id;
public:
Turnout(int id, String name);
};


#endif
