#include "PuenteH.h"
#include <Arduino.h>

PuenteH::PuenteH(uint8_t speedPin, uint8_t in1Pin, uint8_t in2Pin)
    : speedPin_(speedPin), in1Pin_(in1Pin), in2Pin_(in2Pin) {}

void PuenteH::initializeBridge() {
    pinMode(in1Pin_, OUTPUT);
    pinMode(in2Pin_, OUTPUT);
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
    analogWrite(speedPin_, 0);
}

void PuenteH::setSpeed(uint8_t speed) {
    analogWrite(speedPin_, speed);
}

void PuenteH::moveForward() {
    digitalWrite(in1Pin_, HIGH);
    digitalWrite(in2Pin_, LOW);
}

void PuenteH::moveBackward() {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, HIGH);
}

void PuenteH::stopMotor() {
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
}
