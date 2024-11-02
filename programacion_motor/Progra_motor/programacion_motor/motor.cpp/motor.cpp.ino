#include "motor.h"
#include <Arduino.h>

void Motor::InitializeMotor();
{
  pinMode(in1_, OUTPUT);
  pinMode(in2_, OUTPUT);
  digitalWrite(in1_,LOW)
  digitalWrite(in2_,LOW);
  analogWrite(speedPin,0);
}
void Motor::SetSpeed(uint8_t speed)
{
  analogWrite(pwmPin, speed);
  vel = speed;
};

void Motor::MoveForward()
{
  digitalWrite(in1_, HIGH);
  digitalWrite(in2_, LOW);
  analogWrite(speedPin_,vel)
};

void Motor::MoveBackwards()
{
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, HIGH);
  analogWrite(speedPin_,vel)

void Motor::StopMotor()
{
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, LOW);
}
};