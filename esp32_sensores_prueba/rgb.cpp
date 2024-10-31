#include "rgb.h"
#include <ColorConverterLib.h>

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
    ColorConverter::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);
}

String ColorSensor::getColorName() const {
    if (value < 50 && saturation < 50) {
        return "Negro";  // Negro, con bajo valor y saturación
    } else if (hue < 15 || hue >= 330) {
        return "Rojo";    // Rojo
    } else if (hue >= 45 && hue < 75) {
        return "Amarillo"; // Amarillo
    } else if (hue >= 75 && hue < 150) {
        return "Verde";  // Verde
    } else if (hue >= 270 && hue < 330) {
        return "Magenta"; // Magenta
    } else if (hue >= 240 && hue < 270) {
        return "Púrpura"; // Púrpura
    } else {
        return "Unknown"; // Colores no especificados
    }
}
