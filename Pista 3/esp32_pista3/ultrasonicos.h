#ifndef ULTRASONICOS_H
#define ULTRASONICOS_H
#include "ultrasonico.h"
//#include "Arduino.h"

class Ultrasonicos {
public:
  // Constructor
  Ultrasonico(uint8_t triggerPin_1, uint8_t echoPin_2,
              uint8_t triggerPin_2, uint8_t echoPin_2,
              uint8_t triggerPin_3, uint8_t echoPin_3,  
              int muroCercaDist, int muroLejosDist, float pelotaDist); // Checar distancias

  // Inicializa los pines del sensor
  void InitializeUltra();
  // Medir distancia
  float medirDistancia();
  // Evaluar si hay un muro cerca o lejos
  void evaluarMuros();
  // Evaluar situacion
  void evaluarSituacion();
  // Obtener el conteo de muros cercanos y lejanos
  int getMuroCerca() { return muro_cerca; }
  int getMuroLejos() { return muro_lejos; }

private:
  Ultrasonico ultrasonico1
  Ultrasonico ultrasonico2
  Ultrasonico ultrasonico3
  int muroCercaDist;
  int muroLejosDist;
  int muro_cerca;
  int muro_lejos;
};

#endif
