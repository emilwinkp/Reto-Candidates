#include "HardwareSerial.h"
#include "rgb.h"
#include "ultrasonicos.h"
#include "infrarojos.h"

Ultrasonicos ultrasonicos(1,2,3,4,5,6,10,30,20);
Infrarojo infrarrojos(1,2,3,4,5,6);
ColorSensor rgb;

void setup() {
  Serial.begin(115200);
  rgb.begin();
  ultrasonicos.InitializeUltra();
  infrarrojos.InitializeInfra(); 
  // put your setup code here, to run once:

}

void loop() {
  color_act = rgb.getColorName();
  if (color_act == "Verde") {
    Serial.write(65, DEC);
  }
  // put your main code here, to run repeatedly:

}
