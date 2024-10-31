#ifndef QTRsensor_h
#define QTRsensor_h

#include <Arduino.h>
#include <QTRSensors.h>

class SensorQTR {
private:
    QTRSensors qtr;  // Usa QTRSensorsAnalog en vez de QTRSensors directamente
    unsigned int sensorValues[6];
    const int THRESHOLD_BLACK = 800;
    const int THRESHOLD_GREEN = 400;
    const int THRESHOLD_RED = 600;

public:
    SensorQTR(uint8_t* pins, uint8_t sensorCount);
    void InitializeQTR(uint8_t* pins, uint8_t sensorCount);
    void readSensors();
    int detectColor();
    int reflectanciaProm();
};

#endif
