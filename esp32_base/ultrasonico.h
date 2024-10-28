#ifndef ULTRASONICO_H
#define ULTRASONICO_H

#include <esp_arduino_version.h>

class Ultrasonico 
{
  public:
  Ultrasonico(uint8_t trigger, uint8_t echo);
  void InitializeUltra();
  void medirDistancia();

private:
  uint8_t trigger_;
  uint8_t echo_;

#endif