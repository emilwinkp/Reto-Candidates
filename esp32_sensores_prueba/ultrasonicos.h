#ifndef ULTRASONICOS_H
#define ULTRASONICOS_H
#include "ultrasonico.h"
#include <HardwareSerial.h>
#include <Arduino.h>

class Ultrasonicos {
public:
  // Constructor
  Ultrasonicos(uint8_t triggerPin_1, uint8_t echoPin_1,
              uint8_t triggerPin_2, uint8_t echoPin_2,
              uint8_t triggerPin_3, uint8_t echoPin_3,  
              float muroCercaDist, float muroLejosDist, float pelotaDist); // Checar distancias

  // Inicializa los pines del sensor
  void InitializeUltras();
  // Medir distancia
  float medirDistancias(uint8_t trigger, uint8_t echo);
  // Evaluar si hay un muro cerca o lejos
  void evaluarMuros();
  // Evaluar situacion
  int evaluarSituacion();
  // Obtener el conteo de muros cercanos y lejanos
  void EnviarComandoArduino();

  int getMuroCerca() { return muro_cerca; }
  int getMuroLejos() { return muro_lejos; }

private:
  Ultrasonico ultrasonico1;
  Ultrasonico ultrasonico2;
  Ultrasonico ultrasonico3;
  int muroCercaDist;
  int muroLejosDist;
  float pelotaDist;
  bool muro_cercaIzq,muro_cercaDer, muro_cercaEnf, pelotaIzq, pelotaDer, pelotaEnf;
  int muro_cerca;
  int muro_lejos;
  int situacion;
  void enviarComando(int comando);

  HardwareSerial Serial2 = HardwareSerial(2); //Puerto serial adicional para comunicación
};

#endif
