echo "# Reto-Candidates" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/emilwinkp/Reto-Candidates.git
git push -u origin main

#include "motores.h"
#include "ultrasonicos.h"
unsigned long previousMillis = 0;
long intervalo = 1000;

Motores myMotors()

void setup() {
  
  // put your setup code here, to run once:

}

void loop() {
  unsigned long currentMillies = millies();
  if ((currentMillies - previousMillis) >= intervalo){
    previousMillies = currentMillies;
    myMotores.readEncoder();

  }
  // put your main code here, to run repeatedly:

}
