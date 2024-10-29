#ifndef Puente_h
#define Puente_h

#include <Arduino.h>


class PuenteH
{
public:
  Puente(uint8_t SpeedPin, uint8_t in1Pin, uint8_t in2Pin)
  void setSpeed(uint8_t speed);
  void moveForward();
  void moveBackward();
  void stopMotor();
private:
  uint8_t speedPin_;
  uint8_t in1Pin_;
  uint8_t in2Pin_;
};

#endif