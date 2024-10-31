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
          uint8_t speed2, uint8_t in1_2, uint8_t in2_2);
  // Funciones para controlar motores
  void InitializeMotors();
  void InitializeDriver();
  void InitializeEncoders();
  void SetAllSpeeds(uint8_t speed);
  void StopMotors();
  void MotoresMoveForward();
  void MotoresMoveBackwards();
  // PID con encoder
  void ControlWithPID(int target_position1, int target_position2);
  static void readEncoder();  // Función para leer el encoder
  static void readEncoderA1();
  static void readEncoderA2();
  // IMU para giros 
  void MoveMotorsImu (float target_angle); //Checar comunicacion con esp32
  
private:
  Motor motor1;
  Motor motor2;
  // Variables PID
  float kp, ki, kd;
  float eprev1, eprev2;
  float eintegral1, eintegral2;  // Integral del error
  long prevT;        // Tiempo anterior

  // Variables de encoder
  int pos1, pos2;          // Posición de los encoders
  static Motores* instance;
  uint8_t pwm; // Checar driver

  // Pines del encoder 
  uint8_t encoderPinA1, encoderPinB1;
  uint8_t encoderPinA2, encoderPinB2;
  
  //Variables para almacenar angulo
  float anguloZ = 0;
};

#endif
