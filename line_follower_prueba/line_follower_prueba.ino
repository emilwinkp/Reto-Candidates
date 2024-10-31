#include <QTRSensors.h>
#include "motores.h"

Motores mismotores(6,3,2,5,4,7);
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

const int rightMax = 255;
const int leftMax = 255;
const int baseSpeed = 150;
const float Kp = 0.05;
const float Kd = 2.0;
/*
#define Kp 0.05
#define Kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMax 200 // max speed of the robot
#define leftMax 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
*/
void setup() {
  mismotores.InitializeDriver();
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);
  
  delay(500); // Segundos para poner el robot en el suelo
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  
   /* comment out for serial printing
    
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
}

int ultimoError = 0;

void loop() {
  unsigned int sensores[6];
  uint16_t posicion = qtr.readLineBlack(sensorValues); 

  // Agregar timeout en caso de ser requerido para patrones

  int error = posicion - 2500; // Margen de error respecto al centro de la linea

  int motorSpeed = Kp * error + Kd * (error - ultimoError);
  ultimoError = error;

  int rightSpeed = baseSpeed + motorSpeed;
  int leftSpeed = baseSpeed - motorSpeed;

  if (rightSpeed > rightMax) rightSpeed = rightMax;
  if (leftSpeed > leftMax) leftSpeed = leftMax;
  if (rightSpeed <0) rightSpeed = 0;
  if (leftSpeed < 0) leftSpeed = 0;

  mismotores.SpeedMotores(rightSpeed,leftSpeed);
  mismotores.MotoresMoveForward();
  
  Serial.print("Posición: ");
  Serial.print(posicion);
  Serial.print("\tError: ");
  Serial.print(error);
  Serial.print("\tAjuste Izquierdo: ");
  Serial.print(leftSpeed);
  Serial.print("\tAjuste Derecho: ");
  Serial.println(rightSpeed);

  delay(100);
}
