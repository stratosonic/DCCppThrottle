#include "Arduino.h"
#include "Train.h"

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
    this->functions = 0;     //0b1101011001101001101001001101011;
}

byte Train::getRegisterNumber() {
    return this->registerIndex;
}

int8_t Train::getSpeed() {
    return (int8_t) abs(this->speed);
}

void Train::setSpeed(int speed) {
    this->speed = speed;
}

byte Train::getDirection() {
    return this->direction;
}
void Train::setDirection(byte direction) {
    this->direction = direction;
}

void Train::increaseSpeed(byte increaseAmount) {
    this->speed += increaseAmount;

    if(this->speed > MAX_FORWARD_SPEED) {
        this->speed = MAX_FORWARD_SPEED;
    }

    if(this->speed > 0) {
        this->direction = TRAIN_FORWARD;
    } else if(this->speed < 0 ) {
        this->direction = TRAIN_REVERSE;
    }
}

void Train::decreaseSpeed(byte decreaseAmount) {
    this->speed -= decreaseAmount;

    if(this->speed < MAX_REVERSE_SPEED) {
        this->speed = MAX_REVERSE_SPEED;
    }

    if(this->speed > 0) {
        this->direction = TRAIN_FORWARD;
    } else if (this->speed < 0) {
        this->direction = TRAIN_REVERSE;
    }
}

String Train::getSpeedCommand() {
    char output[19];
    sprintf(output, "<t %d %d %d %d>", this->registerIndex, this->address, abs(this->speed), this->direction);
    return String(output);
}

String Train::getEmergencyStopCommand() {
    char output[19];
    sprintf(output, "<t %d %d %d %d>", this->registerIndex, this->address, -1, this->direction);
    return String(output);
}

void Train::incrementActiveFunction() {
    if (this->activeFunction == this->numberOfFunctions - 1) {
        this->activeFunction = 0;
    } else {
        this->activeFunction++;
    }
}

String Train::getFunctionCommand() {
    int functionNumber = this->activeFunction;
    this->functions = this->functions ^ (1UL << functionNumber);
    Serial.println(functionNumber);

    uint32_t functionTransmission;
    char output[18];

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
        sprintf(output, "<f %d %lu>", this->address, functionTransmission);
    } else if (functionNumber < 9) {
        int mask = 15;
        functionTransmission = this->functions & mask << 5;
        functionTransmission = functionTransmission >> 5;
        functionTransmission += 176;
        functionTransmission = (int)functionTransmission;
        sprintf(output, "<f %d %lu>", this->address, functionTransmission);
    } else if (functionNumber < 13) {
        int mask = 15;
        functionTransmission = this->functions & mask << 9;
        functionTransmission = functionTransmission >> 9;
        functionTransmission += 160;
        functionTransmission = (int)functionTransmission;
        sprintf(output, "<f %d %lu>", this->address, functionTransmission);
    } else if (functionNumber < 21) {
        uint32_t mask = 255;
        functionTransmission = this->functions & mask << 13;
        functionTransmission = functionTransmission >> 13;
        functionTransmission = (uint32_t)functionTransmission;
        sprintf(output, "<f %d %d %lu>", this->address, 222, functionTransmission);
    } else {
        uint32_t mask = 255;
        functionTransmission = this->functions & mask << 21;
        functionTransmission = functionTransmission >> 21;
        functionTransmission = (uint32_t)functionTransmission;
        sprintf(output, "<f %d %d %lu>", this->address, 223, functionTransmission);
    }

    Serial.print("Binary = ");
    Serial.println(functionTransmission, BIN);

    //Serial.print(output);
    //Serial.println();

    return String(output);
}
