#include "GiroscopioMPU6050.h"
#include "ultrasonicos.h"
#include <HardwareSerial.h>

GiroscopioMPU6050 mpu;
Ultrasonico ultrasonicos(14,15,14,32,14,0,10,45,22.25) //Pines y distancias

// Situaciones 
/*
1. Esquina izquierda
2. Pasillo
3. Esquina derecha
4. 
*/

}
void setup() {
  Serial.begin(9600);
  mpu.InitializeMPU();
  ultrasonicos.InitializeUltra();
  // put your setup code here, to run once:
}

void loop() {
  if (color = verde){ // Si esta en la plataforma verde (sensor por programar) se debe de mover 30 cm hacia adelante
    Serial.println()
  }
  ultrasonicos.evaluarSituacion();
  switch (situacion){
    case 1:
      Serial.println();
      break;
    case 2:
      Serial.println();
      break;
    case 3:
      Serial.println();
      break;
    case 4:
      Serial.println();
      break;
    case 5:
      Serial.println();
      break;
    case 6:
      Serial.println();
      break;
    
  }
  mpu.sendAngulo();
  delay(100)
  // put your main code here, to run repeatedly:

}
