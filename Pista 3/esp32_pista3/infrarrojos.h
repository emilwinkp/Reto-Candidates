#ifndef INFRARROJOS_H
#define INFRARROJOS_H
#include "infrarrojo.h"

class Infrarrojos 
{

public:
  Infrarrojo(uint8_t pin_1
            uint8_t pin_2
            uint8_t pin_3)
  
  void InitializeInfras();
  void DetectarObjetos();
  void EvaluarObjetos();

private:
  Infrarrojo infrarrojo1
  Infrarrojo infrarrojo2
  Infrarrojo infrarrojo3
};

#endif