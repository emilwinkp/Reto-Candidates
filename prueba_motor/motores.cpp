#include "motores.h"
#include "GiroscopioMPU6050.h"
#include <Arduino.h>

Motores* Motores::instance = nullptr;
GiroscopioMPU6050 gir;
// Constructor de la clase
Motores::Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
                 uint8_t speed2, uint8_t in1_2, uint8_t in2_2)
  : motor1(speed1, in1_1, in2_1), motor2(speed2, in1_2, in2_2), 
  kp(2.0), ki(1.0), kd(1.0), eprev1(0.0), eprev2(0.0), eintegral1(0.0), eintegral2(0.0), prevT(0),
  pos1(0), pos2(0), encoderPinA1(10), encoderPinB1(8), encoderPinA2(12), encoderPinB2(11){
  instance = this;
}
  //pwm(pwm_salida) // Pines del encoder (modifica según sea necesario)
void Motores::InitializeEncoders() {
  // Configuración de pines para el encoder
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

  // Configuración de interrupción para el encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), Motores::readEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), Motores::readEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), Motores::readEncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), Motores::readEncoderA2, CHANGE);
 // checar si es necesario
}

// Inicializa los motores
void Motores::InitializeMotors() {
  motor1.InitializeMotor();
  motor2.InitializeMotor();
  resetEncoder();
}

void Motores::readEncoder() {
  readEncoderA1();
  readEncoderA2();
}

void Motores::resetEncoder(){
  pos1 = 0;
  pos2 = 0;
}

void Motores::readEncoderA1() {
  if (instance) {
    bool a = digitalRead(instance->encoderPinA1);
    bool b = digitalRead(instance->encoderPinB1);
    // Dirección del encoder 1
    if (a != b) {
      instance->pos1++;
    } else {
      instance->pos1--;
    }
  }
}

// Método estático para leer el encoder 2 (A2 y B2)
void Motores::readEncoderA2() {
  if (instance) {
    bool a = digitalRead(instance->encoderPinA2);
    bool b = digitalRead(instance->encoderPinB2);
    // Dirección del encoder 2
    if (a != b) {
      instance->pos2++;
    } else {
      instance->pos2--;
    }
  }
}

void Motores::SpeedMotores(uint8_t speed1, uint8_t speed2){
  motor1.SetSpeed(speed1);
  motor2.SetSpeed(speed2);
}
// Función de control PID
void Motores::ControlWithPID(int target_position1, int target_position2) {
  // Calcular el tiempo transcurrido
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  // Convertir a segundos
  prevT = currT;

  // Calcular el error para motor 1
  int error1 = target_position1 - pos1;

  // Derivada para motor 1
  float dedt1 = (error1 - eprev1) / deltaT;

  // Integral para motor 1
  eintegral1 += error1 * deltaT;

  // Señal de control para motor 1
  float control_signal1 = kp * error1 + kd * dedt1 + ki * eintegral1;

  eprev1 = error1;
  // Potencia del motor y direccion para motor 1
  float power1 = fabs(control_signal1);
  if (power1 > 255) {
    power1 = 255;  // Limitar el valor máximo a 255
  }
  int direction1 = control_signal1 >= 0 ? 1 : -1;

  // aplicar control para motor 1
  if (direction1 == 1) {
    motor1.MoveForward();
  } else {
    motor1.MoveBackwards();
  }
  // Ajustar la velocidad de motor 1
  motor1.SetSpeed(power1);

  // PID para motor 2
  int error2 = target_position2 - pos2;
  float dedt2 = (error2 - eprev2) / deltaT;
  eintegral2 += error2 * deltaT;
  float control_signal2 = kp * error2 + kd * dedt2 + ki * eintegral2;
  eprev2 = error2; 

  float power2 = fabs(control_signal2);
  if (power2 > 255) {
    power2 = 255;  // Limitar el valor máximo a 255
  }
  int direction2 = control_signal2 >= 0 ? 1 : -1;

  // aplicar control para motor 2
  if (direction2 == 1) {
    motor2.MoveForward();
  } else {
    motor2.MoveBackwards();
  }
  // Ajustar la velocidad de motor 1
  motor2.SetSpeed(power2);
  Serial.print("Power 2");
  Serial.print(power2);
  Serial.print("Power 1");
  Serial.print(power1);
}

void Motores::StopMotors(){
  motor1.StopMotor();
  motor2.StopMotor();
}

void Motores::InitializeDriver(){
  InitializeMotors();
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
}

void Motores::SetAllSpeeds(uint8_t speed){
  motor1.SetSpeed(speed);
  motor2.SetSpeed(speed);
}

void Motores::MotoresMoveForward(){
  motor1.MoveForward();
  motor2.MoveForward();
}

void Motores::MoveMotorsImu(float targetAngle) {
    float currentAngle = gir.readAnguloActual();
    float error = targetAngle - currentAngle;
    int baseSpeed = 200;
    // Control PID para corregir el ángulo
    float pTerm = kp * error;
    eintegral1 += error * ki;
    float dTerm = kd * (error - eprev1);
    eprev1 = error;

    // Ajuste de velocidad de los motores basándose en el control PID
    int adjustment = static_cast<int>(pTerm + eintegral1 + dTerm);
    int leftMotorSpeed = constrain(baseSpeed - adjustment, 0, 255);
    int rightMotorSpeed = constrain(baseSpeed + adjustment, 0, 255);

    SpeedMotores(leftMotorSpeed, rightMotorSpeed);
    MotoresMoveForward();
}


void Motores::MotoresMoveBackwards(){
  motor1.MoveBackwards();
  motor2.MoveBackwards();
}
 // Mover respecto a un angulo
