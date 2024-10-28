#include "motores.h"
#include <Arduino.h>
// Constructor de la clase
Motores::Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
                 uint8_t speed2, uint8_t in1_2, uint8_t in2_2)
  : motor1(speed1, in1_1, in2_2), motor2(speed2, in1_2, in2_2),
    kp(1.0), ki(0.0), kd(0.0), eprev(0.0), eintegral(0.0), prevT(0), pos1(0),pos2(0),
    encoderPinA1(encoderPinA1), encoderPinB1(encoderPinB1),
    encoderPinA2(encoderPinA2), encoderPinB1(encoderPinB2),
    pwm(pwm_salida)  // Pines del encoder (modifica según sea necesario)
{
  // Configuración de pines para el encoder
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

  // Configuración de interrupción para el encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), std::bind(&Motores::readEncoder, this), RISING); // checar si es necesario
}

// Inicializa los motores
void Motores::InitializeMotors() {
  motor1.IntializeMotor();
  motor2.IntializeMotor();
}

void Motores::readEncoder() {
  // Función para leer el encoder 1
  int a = digitalRead(encoderPinA1);
  int b = digitalRead(encoderPinB1);
  if (a != b) {
    pos1++;
  } else {
    pos1--;
  }
// Función para leer el encoder 2
  int a2 = digitalRead(encoderPinA2);
  int b2 = digitalRead(encoderPinB2);
  if (a != b) {
    pos2++;
  }  else {
    pos2--; 
  }
  }
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
  float dedt1 = (error1 - eprev) / deltaT;

  // Integral para motor 1
  eintegral1 += error1 * deltaT;

  // Señal de control para motor 1
  float control_signal1 = kp * error + kd * dedt + ki * eintegral;

  eprev1 = error1
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

 // Mover respecto a un angulo
void MoveMotorsImu(float target_angle){
  float angulo_inicial = readAnguloInicial();
  float angulo_actual = angulo_inicial;

  if (target_angle >0){
    //Giro a la derecha
    motor1.SetSpeed(200);
    motor1.MoveForward();
    motor2.SetSpeed(200);
    motor2.MoveBackwards();
  } else {
    motor1.SetSpeed(200);
    motor1.MoveBackwards();
    motor2.SetSpeed(200);
    motor2.MoveForward();
  }
  // Girar hasta alcanzar algulo deseado
  while abs((angulo_actual - angulo_inicial)< target_angle){
    angulo_actual = readAnguloActual();
  }
  StopMotors();
}

void. 
