#ifndef Motor_h
#define Motor_h 

#include <Arduino.h>

class Motor
{
public:
  Motor(uint8_t speed, uint8_t in1, uint8_t in2);
  void IntializeMotor();
  void StopMotor();
  void MoveForward();
  void MoveBawkards();
  void SetSpeed(uint8_t pwm, uint8_t speed)

private:
  uint8_t speed;  // Checar estas mas adelante 
  uint8_t in1_;
  uint8_t in2_;
};

#endif 

