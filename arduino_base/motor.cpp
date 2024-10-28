#include "Motor.h"
#include <Arduino.h>

// Constructor que inicializa la instancia de PuenteH
Motor::Motor(uint8_t speedPin, uint8_t in1Pin, uint8_t in2Pin) 
    : bridge(speedPin, in1Pin, in2Pin) {}

// Inicializa el motor usando PuenteH
void Motor::InitializeMotor() {
    bridge.initializeBridge();
}

// Establece la velocidad usando PuenteH
void Motor::setSpeed(uint8_t speed) {
    bridge.setSpeed(speed);
}

// Mueve el motor hacia adelante usando PuenteH
void Motor::moveForward() {
    bridge.moveForward();
}

// Mueve el motor hacia atrás usando PuenteH
void Motor::moveBackward() {
    bridge.moveBackward();
}

// Detiene el motor usando PuenteH
void Motor::stopMotor() {
    bridge.stopMotor();
}
