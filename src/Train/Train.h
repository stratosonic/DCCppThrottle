#include "Arduino.h"

#ifndef TRAIN_H
#define TRAIN_H

#define TRAIN_FORWARD 1
#define TRAIN_REVERSE 0

#define MAX_FORWARD_SPEED 126
#define MAX_REVERSE_SPEED -126

#define MAX_NUMBER_OF_TRAIN_FUNCTIONS 29

class Train {
  private:
    int16_t speed;

  public:
    byte registerIndex;
    int address;
    byte direction;
    byte numberOfFunctions;
    byte activeFunction;
    uint32_t functions;
    Train();
    Train(byte, int, byte);
    byte getRegisterNumber();
    int8_t getSpeed();
    void setSpeed(int);
    byte getDirection();
    void setDirection(byte);
    void increaseSpeed(byte);
    void decreaseSpeed(byte);
    void getSpeedCommand(char *);
    void getEmergencyStopCommand(char *);
    void incrementActiveFunction();
    String getFunctionCommand();
};

#endif
