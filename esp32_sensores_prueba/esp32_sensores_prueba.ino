#include <Arduino.h>
#include "ultrasonicos.h"
#include "ultrasonico.h"

//Ultrasonicos ult(14,32,14,15,2,0,10,40,30);
Ultrasonico ul(14,32);
Ultrasonico ul2(14,15);
Ultrasonico ul3(2,0);

int situacion = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Hola");
  ul.InitializeUltra();
  ul2.InitializeUltra();
  ul3.InitializeUltra();
  delay(5000);
  // put your setup code here, to run once:

}

void loop() {
  /*
  situacion = ult.evaluarSituacion(); // Asegúrate de asignar el valor retornado
  Serial.print("Situación: ");
  Serial.println(situacion);
  delay(1000);
  */
  //ult.evaluarSituacion();
  ul.medirDistancia();
  //ult.evaluarMuros();
  delay(3000);
  
  
}

