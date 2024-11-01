#include <Arduino.h>
#include "ultrasonicos.h"
#include "ultrasonico.h"

//Ultrasonicos ult(14,32,14,15,14,0,5,30,10);
Ultrasonico ult(14,15);
int situacion = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Hola");
  ult.InitializeUltra();
  // put your setup code here, to run once:

}

void loop() {
  /*
  situacion = ult.evaluarSituacion(); // Asegúrate de asignar el valor retornado
  Serial.print("Situación: ");
  Serial.println(situacion);
  delay(1000);
  */
  ult.medirDistancia();
  delay(100);
  
  
}

