#include "Motor.h"
#include <Arduino.h>

void Motor::InitializeMotor();
{
  pinMode(in1_, OUTPUT);
  pinMode(in2_, OUTPUT);
  digitalWrite(in1_,LOW)
  digitalWrite(in2_,LOW);

  analogWrite(speed,0);
}
void Motor::SetSpeed(uint8_t pwm, uint8_t speed)
{
  analogWrite(pwm, speed);
};

void Motor::MoveForward()
{
  digitalWrite(in1_, HIGH);
  digitalWrite(in2_, LOW);
};

void Motor::MoveBawkards()
{
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, HIGH);

void Motor::StopMotor(){
  digitalWrite(in1_, LOW);
  digitalWrite(in1_, LOW);
}
};