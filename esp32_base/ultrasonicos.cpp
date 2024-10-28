#include "Ultrasonicos.h"

// Constructor
Ultrasonico::Ultrasonico(uint8_t triggerPin, uint8_t echoPin, int muroCercaDist, int muroLejosDist)
  : trigger(triggerPin), echo(echoPin), muroCercaDist(muroCercaDist), muroLejosDist(muroLejosDist), muro_cerca(0), muro_lejos(0) {}

// Inicializa los pines del sensor
void Ultrasonico::InitializeUltra() {
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
}

// Medir distancia con el sensor ultrasónico
float Ultrasonico::medirDistancia() {
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
void Ultrasonico::evaluarMuro() {
  float distancia = medirDistancia();
  
  if (distancia < muroCercaDist) {
    muro_cerca++;
  } else if (distancia >= muroCercaDist && distancia <= muroLejosDist) {
    muro_lejos++;
  }

  // Imprimir los resultados
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.print(" cm | Muros cercanos: ");
  Serial.print(muro_cerca);
  Serial.print(" | Muros lejanos: ");
  Serial.println(muro_lejos);
}
