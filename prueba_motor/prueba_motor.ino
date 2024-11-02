#include "motores.h"
#include "GiroscopioMPU6050.h"
#include "QTRsensor.h"

Motores mismotores(6, 3, 2, 5, 4, 7);

/*
float anguloin = 0;
float anguloact = 0;
GiroscopioMPU6050 giro;
*/

uint8_t sensorPins[6] = {A0, A1, A2, A3, A4, A5};  
SensorQTR qtrSensor(sensorPins, 6);               

void setup() {
  Serial.begin(9600);
  mismotores.InitializeMotors();
  mismotores.InitializeDriver();
  mismotores.MoveMotorsImu(90);
  //qtrSensor.InitializeQTR(sensorPins, 6);
  //for (uint16_t i = 0; i < 400; i++) {
    //qtr.calibrate();

   // Inicializar el giroscopio
}
void loop() {
  mismotores.MotoresMoveForward();
  delay(100);
  mismotores.SpeedMotores(200, 200);
}
  /*
  qtrSensor.readSensors();            // Lee los valores del sensor
  int color = qtrSensor.detectColor(); // Detecta el color basado en la reflectancia

    // Imprime el color detectado en el monitor serial
  Serial.print("Color detectado: ");
  if (color == 78) Serial.println("Negro");
  else if (color == 82) Serial.println("Rojo");
  else if (color == 86) Serial.println("Verde");
  else Serial.println("Blanco");

  delay(500); // Pausa para evitar lecturas excesivas
}/*
  /*
  mismotores.SpeedMotores(200, 200);
  Serial.println("Motores a velocidad 200");

  // Mover hacia atrás
  mismotores.MotoresMoveForward();
  mismotores.ControlWithPID(36,36);
  Serial.println("Moviendo hacia atrás");
  delay(1000);

  // Detener
  
  Serial.println("Detenido");
  delay(1000);

  // Mover hacia adelante
  mismotores.MotoresMoveForward();
  Serial.println("Moviendo hacia adelante");
  delay(1000);

  // Detener nuevamente
  mismotores.StopMotors();
  Serial.println("Detenido");

  // Imprimir ángulos si es necesario
  delay(2000); 
  */
  /*
  anguloin = giro.readAnguloInicial();
  delay(10);
  anguloact = giro.readAnguloActual();
  delay(2000);

  delay(20);
  */
  // Establecer velocidad de los motores
  
  //anguloact = giro.readAnguloActual();
  // Mover hacia adelant
  /*
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
  */
 

  // Imprimir ángulos si es necesario
  

 // Pausa antes de repetir el ciclo
 
 
