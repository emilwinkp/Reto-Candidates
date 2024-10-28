#include "ultrasonico.h"

void Ultrasonico::InitializeUltra(){
  Serial.begin(9600);
  pinMode(trigger_, OUTPUT);
  pinMode(echo_, INPUT);
}

void Ultrasonico::MedirDistancia(){
    // Enviar pulso de trigger
  digitalWrite(trigger_, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_, LOW);
  
  // Leer el tiempo de respuesta del pulso en microsegundos
  long duration = pulseIn(echo_, HIGH);
  
  // Calcular distancia en cm (velocidad del sonido = 340 m/s => 58.2 us/cm)
  float distance = duration / 58.2;
  return distance;
}
}
