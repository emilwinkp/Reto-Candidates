#ifndef Motores_h
#define Motores_h

#include "motor.h"
#include <Arduino.h>
#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>

class Motores 
{
public:
  // Constructor para inicializar ambos motores
  Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
          uint8_t speed2, uint8_t in1_2, uint8_t in2_2)
  // Funciones para controlar motores
  void InitializeMotors();
  void InitializeDriver();
  void SetAllSpeeds(uint8_t speed);
  void StopMotors();
  void MoveForward();
  void MoveBackwards();
 
  // PID con encoder
  void ControlWithPID(int target_position1, int target_position2);
  void readEncoder();  // Función para leer el encoder
  
  // IMU para giros 
  void MoveMotorsImu (float target_angle); //Checar comunicacion con esp32

private:
  Motor motor1;
  Motor motor2;
  Adafruit_MPU6050 mpu;
  // Variables PID
  float kp, ki, kd;
  float eprev;       // Error anterior
  float eintegral;   // Integral del error
  long prevT;        // Tiempo anterior

  // Variables de encoder
  int pos1, pos2;           // Posición de los encoders
  uint8_t pwm; // Checar driver

  // Pines del encoder 
  uint8_t encoderPinA1, encoderPinB1;
  uint8_t encoderPinB2, encoderPinB2;
  
  //Variables para almacenar angulo
  float anguloZ = 0;
};

#endif
