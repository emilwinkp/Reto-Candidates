#include "ultrasonico.h"

Ultrasonico::Ultrasonico(uint8_t trigger, uint8_t echo) 
  : trigger_(trigger), echo_(echo) {}

void Ultrasonico::InitializeUltra() {
  Serial.begin(115200);
  pinMode(trigger_, OUTPUT);
  pinMode(echo_, INPUT);
}

float Ultrasonico::medirDistancia() {
  digitalWrite(trigger_, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_, LOW);

  long duration = pulseIn(echo_, HIGH);
  float distance = duration / 58.2;
  Serial.print("Distancia: ");
  Serial.println(distance);
  return distance;
}
