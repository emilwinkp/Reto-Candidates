#include "Ultrasonicos.h"

// Constructor
Ultrasonico::Ultrasonico(uint8_t triggerPin_1, uint8_t echoPin_1,
                         uint8_t triggerPin_2, uint8_t echoPin_2,
                         uint8_t triggerPin_3, uint8_t echoPin_3,
                         int muroCercaDist, int muroLejosDist, float pelotaDist)
  : ultrasonico1(triggerPin_1,echoPin_1), ultrasonico2(triggerPin_2, echoPin_2), ultrasonico3(triggerPin_3, echoPin_3),
  muroCercaDist(muroCercaDist), muroLejosDist(muroLejosDist), pelotaDist(pelota distancia)
  muro_cercaIzq(false), muro_cercaDer(false), muro_cercaEnf(false), 
  pelotaIzq(false), pelotaDer(false) muro_lejos(0) {}

// Inicializa los pines del sensor
void Ultrasonico::InitializeUltra() {
  pinMode(triggerPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);
  pinMode(triggerPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);
  pinMode(triggerPin_3, OUTPUT);
  pinMode(echoPin_3, INPUT);
}

// Medir distancia con el sensor ultrasónico
float Ultrasonico::MedirDistancia(uint8_t trigger, uint8_t echo) {
  // Enviar pulso de trigger
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  
  // Leer el tiempo de respuesta del pulso en microsegundos
  long duration = pulseIn(echo, HIGH);
  
  // Calcular distancia en cm (velocidad del sonido = 340 m/s => 58.2 us/cm)
  float distance = duration / 58.2;
  return distance;
}

// Evaluar si hay un muro cerca o lejos
void Ultrasonicos::evaluarMuro() {
  float distancia1 = MedirDistancia(triggerPin_1, echoPin_1);
  if (distancia1 <= muroCercaDist) {
    muro_cercaIzq = true;
  } else if (distancia1 >= muroCercaDist && distancia1 <= pelotaDist) {
    pelotaIzq = true;
  } else {
    muro_lejos++;
  }

  float distancia2 = MedirDistancia(triggerPin_2, echoPin_2);
  if (distancia2 <= muroCercaDist) {
    muro_cercaEnf = true;
  } else { 
    muro lejos++;
  }

  float distancia3 = MedirDistancia(triggerPin_3, echoPin_3);
  if (distancia3 <= muroCercaDist) {
    muro_cercaDer = true;
  } else if (distancia3 >= muroCercaDist && distancia3 <= pelotaDist) {
    pelotaDer = true;
  } else {
    muro_lejos++;
  }

  /*
  // Imprimir los resultados
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.print(" cm | Muros cercanos: ");
  Serial.print(muro_cerca);
  Serial.print(" | Muros lejanos: ");
  Serial.println(muro_lejos);
  */
}
