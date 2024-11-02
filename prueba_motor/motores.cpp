#include "motores.h"
#include "GiroscopioMPU6050.h"
#include <Arduino.h>

Motores* Motores::instance = nullptr;
GiroscopioMPU6050 gir;

// Constructor de la clase
Motores::Motores(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
                 uint8_t speed2, uint8_t in1_2, uint8_t in2_2)
  : motor1(speed1, in1_1, in2_1), motor2(speed2, in1_2, in2_2), 
  kp(1.0), ki(3.0), kd(0.1  ), eprev1(0.0), eprev2(0.0), eintegral1(0.0), eintegral2(0.0), prevT(0),
  pos1(0), pos2(0), encoderPinA1(10), encoderPinB1(8), encoderPinA2(13), encoderPinB2(12) {
  instance = this;
}

// Configuración de pines para los encoders
void Motores::InitializeEncoders() {
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

  // Configuración de interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), Motores::readEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), Motores::readEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), Motores::readEncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), Motores::readEncoderA2, CHANGE);
}

void Motores::readEncoder(){
  readEncoderA1();
  readEncoderA2();
}
// Reiniciar los valores de los encoders
void Motores::resetEncoder() {
  pos1 = 0;
  pos2 = 0;
}

int Motores::Pos1() {
    return pos1;
}

int Motores::Pos2() {
    return pos2;
}
// Función estática para leer el encoder 1 (A1 y B1)
void Motores::readEncoderA1() {
    if (instance) {
        bool a = digitalRead(instance->encoderPinA1);
        bool b = digitalRead(instance->encoderPinB1);

        if (a == b) {
            instance->pos1++;  // Incrementar si la dirección es positiva
        } else {
            instance->pos1--;  // Decrementar si la dirección es negativa
        }
    }
}

void Motores::readEncoderA2() {
    if (instance) {
        bool a = digitalRead(instance->encoderPinA2);
        bool b = digitalRead(instance->encoderPinB2);

        if (a == b) {
            instance->pos2++;  // Incrementar si la dirección es positiva
        } else {
            instance->pos2--;  // Decrementar si la dirección es negativa
        }
    }
}

void Motores::SpeedMotores(uint8_t speed1, uint8_t speed2){
  motor1.SetSpeed(speed1);
  motor2.SetSpeed(speed2);
}
// Función de control PID (No sirve)
void Motores::ControlWithPID(int target_position1, int target_position2) {
  // Inicializar el tiempo y las variables del PID
  long currT = micros();
  float deltaT = 0;
  
  while (abs(pos1 - target_position1) > 5 || abs(pos2 - target_position2) > 5) {  // Condición de salida

    // Calcular el tiempo transcurrido
    currT = micros();
    deltaT = ((float)(currT - prevT)) / 1.0e6;  // Convertir a segundos
    prevT = currT;

    // PID para motor 1
    int error1 = target_position1 - pos1;
    float dedt1 = (error1 - eprev1) / deltaT;
    eintegral1 += error1 * deltaT;
    float control_signal1 = kp * error1 + kd * dedt1 + ki * eintegral1;
    eprev1 = error1;
    float power1 = constrain(fabs(control_signal1), 0, 255);

    // Establecer dirección y velocidad del motor 1
    if (control_signal1 >= 0) {
      motor1.MoveForward();
    } else {
      motor1.MoveBackwards();
    }
    motor1.SetSpeed(power1);

    // PID para motor 2
    int error2 = target_position2 - pos2;
    float dedt2 = (error2 - eprev2) / deltaT;
    eintegral2 += error2 * deltaT;
    float control_signal2 = kp * error2 + kd * dedt2 + ki * eintegral2;
    eprev2 = error2;
    float power2 = constrain(fabs(control_signal2), 0, 255);

    // Establecer dirección y velocidad del motor 2
    if (control_signal2 >= 0) {
      motor2.MoveForward();
    } else {
      motor2.MoveBackwards();
    }
    motor2.SetSpeed(power2);

    // Monitoreo y feedback en consola
    Serial.print("Power Motor 1: ");
    Serial.println(power1);
    Serial.print("Power Motor 2: ");
    Serial.println(power2);

    // Leer la posición actual de los encoders
    readEncoder();

    // Breve pausa para evitar sobrecargar la CPU
    delay(10);
  }

  // Detener motores al alcanzar el objetivo
  StopMotors();
  Serial.println("Objetivo alcanzado.");
}

void Motores::InitializeMotors(){
  motor1.InitializeMotor();
  motor2.InitializeMotor();
  resetEncoder();
}
void Motores::StopMotors(){
  motor1.StopMotor();
  motor2.StopMotor();
}

void Motores::InitializeDriver(){
  InitializeMotors();
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

void Motores::MotoresMoveBackwards(){
  motor1.MoveBackwards();
  motor2.MoveBackwards();
}

void Motores::MoveMotorsImu(float target_angle){
  gir.InitializeMPU();
  float angulo_inicial = gir.readAnguloInicial();
  float angulo_actual = angulo_inicial;
  float error, dedt, control_signal;
  float eintegral = 0;
  float eprev = target_angle;
  unsigned long startTime = millis();
  
  Serial.print("angulo_inicial");
  Serial.print(angulo_inicial);
  
  float ang = fabs(angulo_actual - angulo_inicial)< fabs(target_angle);
  Serial.print("faltante");
  Serial.print(ang);
  
  // Girar hasta alcanzar algulo deseado
  while (fabs(angulo_actual - angulo_inicial)< fabs(target_angle)){ //checar como pausar en caso de no encontrar el angulo
    angulo_actual = gir.readAnguloActual(); 
    error = target_angle - (angulo_actual - angulo_inicial);

    eintegral += error*0.01;
    dedt = (error - eprev)/0.01;
    eprev = error;
    

    control_signal = kp * error + ki * eintegral + kd * dedt;
    float velocidad = fabs(control_signal);
    if (velocidad > 255) velocidad = 255;
  
    if (target_angle> 0){
    //Giro a la derecha
      motor1.SetSpeed(velocidad);
      motor1.MoveForward();
      motor2.SetSpeed(velocidad);
      motor2.MoveBackwards();
    } else {
      motor1.SetSpeed(velocidad);
      motor1.MoveBackwards();
      motor2.SetSpeed(velocidad);
      motor2.MoveForward();
    }

    if (millis() - startTime > 3000) {
      break;
    }
    Serial.print("velocidad");
    Serial.print(velocidad);
    delay(10);
  }

  StopMotors();
}
/*
void Motores::Mover30adelante() {
    resetEncoder();
    const int targetSteps = 36; // Ajusta según tu configuración
    int error3, error4;
    float control_signal2, control_signal2;
    
    MotoresMoveForward();

    while (pos1 < targetSteps && pos2 < targetSteps) {
        readEncoder();

        // Calcula el error como la distancia restante hasta el objetivo
        error1 = targetSteps - pos1;
        error2 = targetSteps - pos2;

        // Control PID para cada motor
        control_signal1 = kp * error1 + kd * (error1 - eprev1) + ki * eintegral1;
        control_signal2 = kp * error2 + kd * (error2 - eprev2) + ki * eintegral2;

        // Ajustar velocidades
        motor1.SetSpeed(constrain(control_signal1, 0, 255));
        motor2.SetSpeed(constrain(control_signal2, 0, 255));

        // Actualizar variables de PID
        eprev1 = error1;
        eprev2 = error2;
        eintegral1 += error1;
        eintegral2 += error2;

        // Monitorear encoders
        Serial.print("Pos1: ");
        Serial.print(pos1);
        Serial.print(" Pos2: ");
        Serial.println(pos2);

        delay(10);
    }

    StopMotors();
}
*/
void Motores::Mover30atras() {
  resetEncoder();
  const int targetSteps = 36;  // Ajusta este valor según el número de pasos que representa 30 cm

  // Mover hacia atrás hasta que ambos encoders alcancen el target
  MotoresMoveBackwards();
  SpeedMotores(200, 200);

  while (abs(pos1) < targetSteps && abs(pos2) < targetSteps) {
    readEncoder();
    Serial.print("Pos1: ");
    Serial.print(pos1);
    Serial.print(" Pos2: ");
    Serial.println(pos2);
    delay(10);
  }

  StopMotors();
}


void Motores::GirarDer() {
  gir.InitializeMPU();
  float targetAngle = 90;
  float anguloActual = gir.readAnguloActual();
  
  motor1.MoveForward();
  motor2.MoveBackwards();
  SpeedMotores(200,200);

  while (fabs(anguloActual) < targetAngle) {
    anguloActual = gir.readAnguloActual();
    Serial.print("Ángulo actual: ");
    Serial.println(anguloActual);
    delay(10);
  }

  StopMotors();
}

void Motores::GirarIzq() {
  gir.InitializeMPU();
  float targetAngle = -90;
  float anguloActual = gir.readAnguloActual();
  
  motor1.MoveBackwards();
  motor2.MoveForward();
  SpeedMotores(200,200);

  while (fabs(anguloActual) > targetAngle) {
    anguloActual = gir.readAnguloActual();
    Serial.print("Ángulo actual: ");
    Serial.println(anguloActual);
    delay(10);
  }

  StopMotors();
}
