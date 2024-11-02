#include "QTRsensor.h"

#define THRESHOLD_BLACK 800   // Ajusta según la sensibilidad del sensor
#define THRESHOLD_RED 600
#define THRESHOLD_GREEN 400
// Constructor
SensorQTR::SensorQTR(uint8_t* pins, uint8_t sensorCount) {
  InitializeQTR(pins, sensorCount); // No es necesario llamar a init en el constructor
}

// Inicialización del sensor
void SensorQTR::InitializeQTR(uint8_t* pins, uint8_t sensorCount) {
  qtr.setTypeAnalog(); 
  qtr.setSamplesPerSensor(10);  // Cambia setSamplePeriod a setSamplesPerSensor
  qtr.setSensorPins(pins, sensorCount);          // Configura el tipo de sensor
}

// Lectura de los valores de los sensores
void SensorQTR::readSensors() {
    qtr.read(sensorValues);
}

// Detecta el color en los sensores y devuelve la inicial en ASCII
int SensorQTR::detectColor() {
    int valorprom = reflectanciaProm();

    if (valorprom >= THRESHOLD_BLACK) {
        return 78;  // Negro
    } else if (valorprom >= THRESHOLD_RED) {
        return 82;  // Rojo
    } else if (valorprom >= THRESHOLD_GREEN) {
        return 86;  // Verde
    } else {
        return 66;  // Blanco
    }
}

// Obtiene el valor promedio de reflectancia
int SensorQTR::reflectanciaProm() {
    int valorprom = 0;
    for (int i = 0; i < 6; i++) {
        valorprom += sensorValues[i];
    }
    return valorprom / 6;
}
