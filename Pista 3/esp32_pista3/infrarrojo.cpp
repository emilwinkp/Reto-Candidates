#include "infrarrojo.h"

void Infrarrojo::InitializeInfra()
{
  Serial.begin(9600);
  pinMode(pin_, INPUT);

}

void Infrarrojo::DetectarObjeto()
{
  deteccion = digitalRead(pin_);

  if (deteccion == HIGH) {
    obstaculo = true
  }
}