#include <LiquidCrystal_I2C.h>

#include "ultrasonicos.h"
#include <Arduino.h>
#include "rgb.h"

Ultrasonicos ult(33,32,14,15,13,0,10,45,30);
ColorSensor rgb;
TwoWire I2CBus = TwoWire(0); // Bus I2C 0 en el ESP32
LiquidCrystal_I2C lcd(0x27, 16, 2);


void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.begin(115200);
  ult.InitializeUltras();
  Serial.println("Iniciando prueba de sensor ultrasónico...");
  rgb.begin();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  delay(2000);
  

  
}

void loop() {
  
  rgb.readColor();
  color = getColorName();
  delay(1000);
  
  


}
