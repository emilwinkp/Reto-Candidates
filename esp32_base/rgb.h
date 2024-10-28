#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "RGBConverterLib.h"

class ColorSensor {
public:
    ColorSensor();
    bool begin();
    void readColor();
    String getColorName() const;

private:
    Adafruit_TCS34725 tcs;
    uint16_t clear, red, green, blue;
    double hue, saturation, value;

    void printColorName(double hue) const;
};

#endif
