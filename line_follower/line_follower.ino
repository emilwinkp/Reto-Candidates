#include "QTRSensors.h"
#include "motores.h"

Motores mymotores(1,2,3,4,5,6);

#define kp 0.05
#define Kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {  0, 1, 2, 3, 4, 5} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];

void setup() {
  mymotores.InitializeDriver()
  qtra.calibrate(QTR_EMITTERS_ON);   
  delay(20);
  
delay(10000); // Segundos para poner el robot en el suelo
  
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

int ultimoeError = 0

void loop() {
  unsigned int sensores[8];
  int posicion = qtra.readLine(sensors); 

  // Agregar timeout en caso de ser requerido para patrones

  int error = posicion - 2500; // Margen de error respecto al centro de la linea

  int motorSpeed = Kp * error + Kd * (error - ultimoError);
  ultimoError = error;

  int rightSpeed = rightBaseSpeed + motorSpeed;
  int leftSpeed = leftBaseSpeed - motorSpeed;

  if (rightSpeed > rightMax) rightSpeed = rightMax;
  if (leftSpeed > leftMax) leftSpeed = leftMax;
  if (rightSpeed <0) rightSpeed = 0;
  if (leftSpeed < 0) leftSpeed = 0;

  {
    motor1.SetSpeed(leftSpeed);
    motor2.SetSpeed(rightSpeed);
  }
}
