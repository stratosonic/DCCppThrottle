#include "Arduino.h"

#ifndef SPEED_INDICATOR_H
#define SPEED_INDICATOR_H

class SpeedIndicator {
    enum Direction { CW,
                     CCW };

  private:
    const uint8_t MAX_SPEED = 126;
    const uint16_t STEPS_PER_REVOLUTION = 600 * 2;
    const uint16_t STEP_DELAY = 400;
    const uint16_t MAX_WORKING_STEPS = MAX_SPEED * 9 / 2;
    uint8_t stepPin;
    uint8_t directionPin;
    // -126 to 126
    int8_t gaugePosition;
    int8_t gaugeValue;
    void doSteps(Direction direction, long steps);

  public:
    SpeedIndicator(uint8_t, uint8_t);
    void reset();
    void move(int8_t val);
};

#endif
