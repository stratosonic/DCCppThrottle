#include "SpeedIndicator.h"
#include "Arduino.h"

/**
 * Constructor
 * @param stepPin Step pin
 * @param directionPin Direction pin
 */
SpeedIndicator::SpeedIndicator(uint8_t stepPin, uint8_t directionPin) {
    this->stepPin = stepPin;
    this->directionPin = directionPin;
    pinMode(this->stepPin, OUTPUT);
    pinMode(this->directionPin, OUTPUT);
    this->gaugePosition = 0;
    this->gaugeValue = 0;
}

/**
 * Resets the speed indicator by running it to both extremes and then stops in the center.
 */
void SpeedIndicator::reset() {
    // Rotate CCW all the way
    this->doSteps(CCW, STEPS_PER_REVOLUTION);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Rotate CW all the way
    this->doSteps(CW, STEPS_PER_REVOLUTION);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Rotate back to the middle
    this->doSteps(CCW, STEPS_PER_REVOLUTION / 2);
}

/**
 * Moves the speed indicator to the specified value.
 *
 * @param value the value to move to
 */
void SpeedIndicator::move(int8_t value) {
    if (value > MAX_SPEED) {
        value = MAX_SPEED;
    } else if (value < -MAX_SPEED) {
        value = -MAX_SPEED;
    }

    Direction dir = CCW;
    if (value > gaugePosition) {
        dir = CW;
    }
    // calculate number of steps
    int diff = abs(gaugePosition - value);
    long steps = map(diff, 0, MAX_SPEED, 0, MAX_WORKING_STEPS);

    if (steps > 0) {
        this->doSteps(dir, steps);
        this->gaugePosition = value;
    }
}

/**
 * Moves the pointer in the specified direction for the specified steps.
 *
 * @param direction the direction to move
 * @param steps the number of steps to move
 */
void SpeedIndicator::doSteps(Direction direction, long steps) {
    if (direction == CW) {
        digitalWrite(this->directionPin, HIGH);
    } else {
        digitalWrite(this->directionPin, LOW);
    }

    for (int i = 0; i < steps; i++) {
        digitalWrite(this->stepPin, HIGH);
        delayMicroseconds(STEP_DELAY);
        digitalWrite(this->stepPin, LOW);
        delayMicroseconds(STEP_DELAY);
    }
}