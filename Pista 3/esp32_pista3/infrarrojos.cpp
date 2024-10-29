#include "infrarrojos.h"

Infrarrojos::Infrarrojo(uint8_t pin_1
            uint8_t pin_2
            uint8_t pin_3)
            :infrarrojo1(pin_1), infrarrojo2(pin_2), infrarrojo3(pin_3),
            objetoIzq(false), objetoEnf(false), objetoDer(false)

void Infrarrojos::InitilizeInfras()
{
  infrarrojo1.InitializeInfra();
  infrarrojo2.InitializeInfra();
  infrarrojo3.InitializeInfra();
}

void Infrarrojos::DetectarObjetos() 
{
  infrarrojo1.DetectarObjeto();
  infrarrojo2.DetectarObjeto();
  infrarrojo3.DetectarObjeto();

}

void Infrarrojos::EvaluarObjetos()
{
  
}