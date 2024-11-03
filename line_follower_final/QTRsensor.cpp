#include "QTRsensor.h"

// Constructor
SensorQTR::SensorQTR(uint8_t* pins, uint8_t sensorCount) {
  InitializeQTR(pins, sensorCount); // No es necesario llamar a init en el constructor
}

// Inicialización del sensor
void SensorQTR::InitializeQTR(uint8_t* pins, uint8_t sensorCount) {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
  qtr.setEmitterPin(2);
  
  delay(500); // Segundos para poner el robot en el suelo
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);         // Configura el tipo de sensor
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
