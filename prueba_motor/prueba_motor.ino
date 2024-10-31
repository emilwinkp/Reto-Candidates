#include "motores.h"

Motores mismotores(6,3,2,5,4,7);

void setup() {
  Serial.begin(9600);
  mismotores.InitializeDriver();
  // put your setup code here, to run once:

}

void loop() {
  mismotores.SetAllSpeeds(100);
  mismotores.MotoresMoveForward();// put your main code here, to run repeatedly:

;}
