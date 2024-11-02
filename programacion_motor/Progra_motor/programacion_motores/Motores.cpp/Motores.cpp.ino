#include "Motores.h"
#include "esp_arduino_version.h"
// Constructor de la clase
Motores::Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
                 uint8_t speed2, uint8_t in1_2, uint8_t in2_2)
  : motor1(speed1, in1_1, in2_2), motor2(speed2, in1_2, in2_2),
    kp(1.0), ki(0.0), kd(0.0), eprev(0.0), eintegral(0.0), prevT(0), pos(0),
    encoderPinA(2), encoderPinB(3)  // Pines del encoder (modifica según sea necesario)
{
  // Configuración de pines para el encoder
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Configuración de interrupción para el encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), std::bind(&Motores::readEncoder, this), RISING);
}

// Función para leer el encoder
void Motores::readEncoder() {
  int b = digitalRead(encoderPinB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}

// Función de control PID
void Motores::ControlWithPID(int target_position) {
  // Calcular el tiempo transcurrido
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  // Convertir a segundos
  prevT = currT;

  // Calcular el error
  int error = target_position - pos;

  // Derivada
  float dedt = (error - eprev) / deltaT;

  // Integral
  eintegral += error * deltaT;

  // Señal de control
  float control_signal = kp * error + kd * dedt + ki * eintegral;

  // Potencia del motor
  float power = fabs(control_signal);
  if (power > 255) {
    power = 255;  // Limitar el valor máximo a 255
  }

  // Dirección del motor
  int direction = control_signal >= 0 ? 1 : -1;

  // Controlar ambos motores en la misma dirección
  if (direction == 1) {
    motor1.MoveForward();
    motor2.MoveForward();
  } else {
    motor1.MoveBackwards();
    motor2.MoveBackwards();
  }

  // Ajustar la velocidad de ambos motores
  motor1.SetSpeed(PWM, power);
  motor2.SetSpeed(PWM, power);

  // Almacenar el error anterior
  eprev = error;
}

// Inicializa los motores
void Motores::InitializeMotors() {
  motor1.IntializeMotor();
  motor2.IntializeMotor();
}
