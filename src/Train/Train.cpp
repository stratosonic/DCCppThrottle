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
    int functionNumber = this->activeFunction;
    this->functions = this->functions ^ (1UL << functionNumber);
    Serial.println(functionNumber);

    uint32_t functionTransmission;

    if (functionNumber < 5) {
        byte mask = 31;
        functionTransmission = this->functions & mask;
        // swap F0 and F4 bits
        byte f0 = bitRead(functionTransmission, 0);
        byte f4 = bitRead(functionTransmission, 4);
        if (f0) {
            bitSet(functionTransmission, 4);
        } else {
            bitClear(functionTransmission, 4);
        }
        if (f4) {
            bitSet(functionTransmission, 0);
        } else {
            bitClear(functionTransmission, 0);
        }
        functionTransmission += 128;
        functionTransmission = (byte)functionTransmission;
        sprintf(command, "<f %d %u>", this->address, functionTransmission);
    } else if (functionNumber < 9) {
        int mask = 15;
        functionTransmission = this->functions & mask << 5;
        functionTransmission = functionTransmission >> 5;
        functionTransmission += 176;
        functionTransmission = (int)functionTransmission;
        sprintf(command, "<f %d %u>", this->address, functionTransmission);
    } else if (functionNumber < 13) {
        int mask = 15;
        functionTransmission = this->functions & mask << 9;
        functionTransmission = functionTransmission >> 9;
        functionTransmission += 160;
        functionTransmission = (int)functionTransmission;
        sprintf(command, "<f %d %u>", this->address, functionTransmission);
    } else if (functionNumber < 21) {
        uint32_t mask = 255;
        functionTransmission = this->functions & mask << 13;
        functionTransmission = functionTransmission >> 13;
        functionTransmission = (uint32_t)functionTransmission;
        sprintf(command, "<f %d %d %u>", this->address, 222, functionTransmission);
    } else {
        uint32_t mask = 255;
        functionTransmission = this->functions & mask << 21;
        functionTransmission = functionTransmission >> 21;
        functionTransmission = (uint32_t)functionTransmission;
        sprintf(command, "<f %d %d %u>", this->address, 223, functionTransmission);
    }
    //Serial.print("Command: ");
    //Serial.println(output);
    //Serial.print("Binary = ");
    //Serial.println(functionTransmission, BIN);
}
