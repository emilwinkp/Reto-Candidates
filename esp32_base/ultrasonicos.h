#ifndef ULTRASONICOS_H
#define ULTRASONICOS_H

#include "Arduino.h"

class Ultrasonico {
public:
  // Constructor
  Ultrasonico(uint8_t triggerPin_1, uint8_t echoPin_2,
              uint8_t triggerPin_2, uint8_t echoPin_2, int muroCercaDist = 10, int muroLejosDist = 45)); // Checar distancias

  // Inicializa los pines del sensor
  void InitializeUltra();

  // Medir distancia
  float medirDistancia();

  // Evaluar si hay un muro cerca o lejos
  void evaluarMuro();

  // Obtener el conteo de muros cercanos y lejanos
  int getMuroCerca() { return muro_cerca; }
  int getMuroLejos() { return muro_lejos; }

private:
  uint8_t trigger_1;
  uint8_t echo_1;
  uint8_t trigger_2;
  uint8_t echo_2;
  uint8_t trigger_3;
  uint8_t echo_3;
  int muroCercaDist;
  int muroLejosDist;
  int muro_cerca;
  int muro_lejos;
};

#endif
