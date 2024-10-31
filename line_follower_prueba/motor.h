#ifndef Motor_h
#define Motor_h 

#include <Arduino.h>
#include "puente.h"
class Motor
{
public:
  Motor(uint8_t speedPin, uint8_t in1Pin, uint8_t in2Pin);
  void InitializeMotor();
  void StopMotor();
  void MoveForward();
  void MoveBackwards();
  void SetSpeed(uint8_t speed);

private:
  PuenteH bridge;
};

#endif 

