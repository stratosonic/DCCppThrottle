#include "Train.h"
#include "Arduino.h"

Train::Train() {
    Train(0, 0, MAX_NUMBER_OF_TRAIN_FUNCTIONS);
}

Train::Train(byte registerIndex, int address, byte numberOfFunctions) {
    this->registerIndex = registerIndex;
    this->address = address;
    this->speed = 0;
    this->direction = TRAIN_FORWARD;
    this->numberOfFunctions = numberOfFunctions;
    this->activeFunction = 0;
    this->functions = 0; //0b1101011001101001101001001101011;
}

byte Train::getRegisterNumber() {
    return this->registerIndex;
}

int8_t Train::getSpeed() {
    return (int8_t)abs(this->speed);
}

void Train::setSpeed(int speed) {
    this->speed = speed;
    this->direction = (this->speed >= 0) ? 1 : 0;
}

byte Train::getDirection() {
    return this->direction;
}
void Train::setDirection(byte direction) {
    this->direction = direction;
}

void Train::increaseSpeed(byte increaseAmount) {
    this->speed += increaseAmount;

    if (this->speed > MAX_FORWARD_SPEED) {
        this->speed = MAX_FORWARD_SPEED;
    }

    if (this->speed > 0) {
        this->direction = TRAIN_FORWARD;
    } else if (this->speed < 0) {
        this->direction = TRAIN_REVERSE;
    }
}

void Train::decreaseSpeed(byte decreaseAmount) {
    this->speed -= decreaseAmount;

    if (this->speed < MAX_REVERSE_SPEED) {
        this->speed = MAX_REVERSE_SPEED;
    }

    if (this->speed > 0) {
        this->direction = TRAIN_FORWARD;
    } else if (this->speed < 0) {
        this->direction = TRAIN_REVERSE;
    }
}

void Train::getSpeedCommand(char *command) {
    int16_t spd = this->speed;
    if (spd < 0) {
        spd = -spd;
    }
    sprintf(command, "<t %d %d %d %d>", this->registerIndex, this->address, spd, this->direction);
}

void Train::getEmergencyStopCommand(char *command) {
    sprintf(command, "<t %d %d %d %d>", this->registerIndex, this->address, -1, this->direction);
}

void Train::incrementActiveFunction() {
    if (this->activeFunction == this->numberOfFunctions - 1) {
        this->activeFunction = 0;
    } else {
        this->activeFunction++;
    }
}

void Train::getFunctionCommand(char *command) {
    if (bitRead(this->functions, this->activeFunction)) {
        bitClear(this->functions, this->activeFunction);
    } else {
        bitSet(this->functions, this->activeFunction);
    }

    sprintf(command, "<F %d %u %u>",
            this->address,
            this->activeFunction,
            bitRead(this->functions, this->activeFunction));
    // Serial.print("function command: ");
    // Serial.println(command);
}