#ifndef Motores_h
#define Motores_h

#include "Motor.h"
#include <Arduino.h>

class Motores 
{
public:
  // Constructor para inicializar ambos motores
  Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
          uint8_t speed2, uint8_t in1_2, uint8_t in2_2,
          uint8_t encoderPinA1,uint8_t encoderPinB1
          uint8_t encoderPinA2, uint8_t encoderPinB2);

  // Funciones para controlar motores
  void InitializeMotors();
  void InitializeDriver();
  void SetAllSpeeds(uint8_t speed);
  void StopMotors();
  void MoveForward();
  void MoveBackwards();
  void MoveAngle(); // A considerar con giroscopio 
  
  // PID con encoder
  void ControlWithPID(int target_position);
  void readEncoder();  // Función para leer el encoder

private:
  Motor motor1;
  Motor motor2;

  // Variables PID
  float kp, ki, kd;
  float eprev;       // Error anterior
  float eintegral;   // Integral del error
  long prevT;        // Tiempo anterior

  // Variables de encoder
  int pos1;
  int pos2;           // Posición del encoder

  // Pines del encoder (Ejemplo con un solo encoder para ambos motores)
  uint8_t encoderPinA1, encoderPinB1;
  uint8_t encoderPinB2, encoderPinB2;
};

#endif
