#include "rgb.h"
#include <ColorConverterLib.h>

ColorSensor::ColorSensor()
    : tcs(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X), clear(0), red(0), green(0), blue(0),
      hue(0), saturation(0), value(0) {}

bool ColorSensor::begin() {
    if (!tcs.begin()) {
        Serial.println("Error al iniciar TCS34725");
        return false;
    }
    return true;
}

void ColorSensor::readColor() {
    tcs.setInterrupt(false);
    delay(60); // Toma 50 ms para capturar el color
    tcs.getRawData(&red, &green, &blue, &clear);
    tcs.setInterrupt(true);

    // Hacer la medición RGB relativa
    uint32_t sum = clear;
    float r = red / (float)sum;
    float g = green / (float)sum;
    float b = blue / (float)sum;

    // Escalar RGB a bytes
    r *= 256;
    g *= 256;
    b *= 256;

    // Convertir a HSV
    ColorConverter::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);

    valor = hue*360;
    Serial.print("hue: ");
    Serial.println(hue);
    Serial.print("saturation: ");
    Serial.println(saturation);
    Serial.print("value: ");
    Serial.println(value);
    Serial.print("VALOR: ");
    Serial.println(valor);
}

void ColorSensor::getColorName() const {
    if (valor < 15)
  {
  lc.setCursor(0,0);
  lc.print("Rojo");
  delay(2000);
  lc.clear();
  }
  else if (valor < 45)
  {
  lc.setCursor(0,0);
  lc.print("Naranja");
  delay(2000);
  lc.clear();
  Serial.println("Naranja");
  }
  else if (valor < 90)
  {
  lc.setCursor(0,0);
  lc.print("Amarillo");
  delay(2000);
  lc.clear();
  Serial.println("Amarillo");
  }
  else if (valor < 150)
  {
  lc.setCursor(0,0);
  lc.print("Verde");
  delay(2000);
  lc.clear();
  Serial.println("Verde");
  
  }
  else if (valor < 210)
  {
  lc.setCursor(0,0);
  lc.print("Cyan");
  delay(2000);
  lc.clear();
  Serial.println("Cyan");
  }
  else if (valor < 270)
  {
  lc.setCursor(0,0);
  lc.print("Morado");
  delay(2000);
  lc.clear();
  Serial.println("Morado");
  }
  else if (valor < 350)
  {
    lc.setCursor(0,0);
  lc.print("Magenta");
  delay(2000);
  lc.clear();
  Serial.println("Magenta");
  }
  else
  {
  lc.setCursor(0,0);
  lc.print("Blanco");
  delay(2000);
  lc.clear();
  Serial.println("Blanco");
  }
}
