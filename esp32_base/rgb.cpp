#include "ColorSensor.h"

ColorSensor::ColorSensor()
    : tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X), clear(0), red(0), green(0), blue(0),
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
    RGBConverterLib::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);
}

String ColorSensor::getColorName() const {
    if (hue < 15) {
        return "Red";
    } else if (hue < 45) {
        return "Orange";
    } else if (hue < 90) {
        return "Yellow";
    } else if (hue < 150) {
        return "Green";
    } else if (hue < 210) {
        return "Cyan";
    } else if (hue < 270) {
        return "Blue";
    } else if (hue < 330) {
        return "Magenta";
    } else {
        return "Red";
    }
}
