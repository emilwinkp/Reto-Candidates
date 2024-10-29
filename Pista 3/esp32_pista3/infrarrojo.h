#ifndef INFRARROJO_H
#define INFRARROJO_H

class Infrarrojo
{
public:
  Infrarrojo(uint8_t pin);
  void InitializeInfra();
  bool DetectarObjeto();

private:
  uinnt8_t pin_;
};

#endif