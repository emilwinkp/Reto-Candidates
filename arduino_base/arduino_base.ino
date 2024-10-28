echo "# Reto-Candidates" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/emilwinkp/Reto-Candidates.git
git push -u origin main

#include "motores.h"
#include "ultrasonicos.h"
#include "infrarojos.h"
#include "SoftwareSerial.h"

SoftwareSerial espSerial(10, 11); // RX, TX para comunicación con ESP32
//unsigned long previousMillis = 0;
//long intervalo = 1000;
float target_angle;

Motores myMotors()

void setup() {
  Serial.begin(9600);           // Comunicación con el ordenador
  espSerial.begin(9600);        // Comunicación con el ESP32
  
  // put your setup code here, to run once:

}

void loop() {
 if (espSerial.available()) {
    float targetAngle = espSerial.parseFloat();  // Leer ángulo enviado por ESP32
    Serial.println(targetAngle)
 }

 
  /*
  unsigned long currentMillies = millies();
  if ((currentMillies - previousMillis) >= intervalo)
  {
    previousMillies = currentMillies;
    myMotores.readEncoder();

  }
  */
  
  // put your main code here, to run repeatedly:

}
