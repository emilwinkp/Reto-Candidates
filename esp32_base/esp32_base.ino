#include "GiroscopioMPU6050.h"
#include "ultrasonicos.h"
#include <HardwareSerial.h>

GiroscopioMPU6050 mpu;

void setup() {
  Serial.begin(9600)
  mpu.InitializeMPU();
  // put your setup code here, to run once:

}

void loop() {
  mpu.sendAngulo();
  delay(100)
  // put your main code here, to run repeatedly:

}
