#include "motores.h"
#include "GiroscopioMPU6050.h"

Motores mismotores(6, 3, 2, 5, 4, 7);

float anguloin = 0;
float anguloact = 0;
GiroscopioMPU6050 giro;

void setup() {
  Serial.begin(9600);
  mismotores.InitializeMotors();
  mismotores.InitializeDriver();
  mismotores.InitializeEncoders();
  Serial.println("Inicializando motores...");
  giro.InitializeMPU();
}

void loop() {
  anguloact = giro.readAnguloActual();
  // Mover hacia adelant
  mismotores.SpeedMotores(200,200);
  delay(10);
  mismotores.readEncoder();
  delay(10);
  mismotores.MotoresMoveForward();
  delay(10);
  mismotores.MoveMotorsImu(3.14);
  delay(3000);
  mismotores.StopMotors();
  delay(10);
  // Imprimir ángulos si es necesario
  Serial.print("Angulo Inicial: ");
  Serial.println(anguloin);
  Serial.print("Angulo Actual: ");
  Serial.println(anguloact);
 // Pausa antes de repetir el ciclo
}
