#include "GiroscopioMPU6050.h"
#include <HardwareSerial.h>  // Biblioteca para comunicación serial

GiroscopioMPU6050::GiroscopioMPU6050() : angleOffset(0.0) {}

void GiroscopioMPU6050::InitializeMPU() {
    if (!mpu.begin(0x68)) { // Inicializa el MPU6050 en la dirección I2C 0x68
    Serial.println("No se pudo encontrar el sensor MPU6050. Verifique la conexión.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 iniciado correctamente");
  
  // Configuración adicional del sensor si es necesaria
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);
  sensors_event_t event;
  mpu.getGyroSensor()->getEvent(&event);
  angleOffset = event.gyro.z;  // Almacenar el ángulo inicial como offset
}

float GiroscopioMPU6050::readAnguloInicial(){
  return angleOffset;
}

float GiroscopioMPU6050::readAnguloActual() {
  sensors_event_t event;
  mpu.getGyroSensor()->getEvent(&event);
  
  // Obtener el tiempo actual y calcular el delta tiempo en segundos
  unsigned long tiempoActual = millis();
  float deltaTiempo = (tiempoActual - ultimaLectura) / 1000.0;  // Convertir a segundos
  ultimaLectura = tiempoActual;
  
  // Integrar la velocidad angular para obtener el ángulo acumulado
  anguloActual += (event.gyro.z * deltaTiempo);  // Asumiendo que la velocidad angular está en grados/s

  // Restar el offset si es necesario
  return anguloActual - angleOffset;

}
