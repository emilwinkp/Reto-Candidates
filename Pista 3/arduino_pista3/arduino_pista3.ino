#include "GiroscopioMPU6050.h"
#include "motores.h"

Motores mismotores (1,2,3,4,5,6);
GiroscopioMPU6050 mpu;

pelota = false;

void setup() {
  Serial.begin(9600);
  mpu.InitializeMPU();
  mismotores.InitializeDriver();

}

void loop() {
  
  // put your main code here, to run repeatedly:

}
