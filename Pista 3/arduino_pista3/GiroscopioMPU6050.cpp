#include "GiroscopioMPU6050.h"
#include <HardwareSerial.h>  // Biblioteca para comunicación serial

GiroscopioMPU6050::GiroscopioMPU6050() : angleOffset(0.0) {}

void GiroscopioMPU6050::InitializeMPU() {
    if (!mpu.begin()) {
        Serial.println("No se pudo encontrar el MPU6050, verifique la conexión.");
        while (1);
    }
    delay(100);
    sensors_event_t event;
    mpu.getGyroSensor()->getEvent(&event);
    angleOffset = event.gyro.z;  // Almacenar el ángulo inicial como offset
}

float GiroscopioMPU6050::readAnguloActual() {
    sensors_event_t event;
    mpu.getGyroSensor()->getEvent(&event);
    float currentAngle = event.gyro.z - angleOffset;  // Calcular ángulo relativo
    return currentAngle;
}

void GiroscopioMPU6050::sendAngulo() {
    float angle = getAngle();
    Serial.println(angle);  // Enviar el ángulo al Arduino
}
