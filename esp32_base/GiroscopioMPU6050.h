#ifndef GIROSCOPIO_MPU6050_H
#define GIROSCOPIO_MPU6050_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> 

class GiroscopioMPU6050 {
public: 
  GiroscopioMPU6050();
  void InitializeMPU();
  float readAnguloActual();
  void sendAngulo();

private:
  Adafruit_MPU6050 mpu;
  float angleOffset; 
};

#endif