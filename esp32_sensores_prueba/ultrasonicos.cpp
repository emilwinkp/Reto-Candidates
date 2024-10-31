#include "Ultrasonicos.h"
// Constructor
Ultrasonicos::Ultrasonicos(uint8_t triggerPin_1, uint8_t echoPin_1,
                         uint8_t triggerPin_2, uint8_t echoPin_2,
                         uint8_t triggerPin_3, uint8_t echoPin_3,
                         int muroCercaDist, int muroLejosDist, float pelotaDist)
  : ultrasonico1(triggerPin_1, echoPin_1), ultrasonico2(triggerPin_2, echoPin_2), ultrasonico3(triggerPin_3, echoPin_3),
  muroCercaDist(muroCercaDist), muroLejosDist(muroLejosDist), pelotaDist(pelotaDist),
  muro_cercaIzq(false), muro_cercaDer(false), muro_cercaEnf(false), 
  pelotaIzq(false), pelotaDer(false), pelotaEnf(false), muro_lejos(0), situacion(0) {}

// Inicializa los pines del sensor
void Ultrasonicos::InitializeUltras() {
  ultrasonico1.InitializeUltra();
  ultrasonico2.InitializeUltra();
  ultrasonico3.InitializeUltra();
  Serial2.begin(9600, SERIAL_8N1, 1, 3);
}

// Medir distancia con el sensor ultrasónico
float Ultrasonicos::medirDistancia(uint8_t trigger, uint8_t echo) {
  // Enviar pulso de trigger
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  
  // Leer el tiempo de respuesta del pulso en microsegundos
  long duration = pulseIn(echo, HIGH);
  
  // Calcular distancia en cm (velocidad del sonido = 340 m/s => 58.2 us/cm)
  float distance = duration / 58.2;
  return distance;
}

// Evaluar si hay un muro cerca o lejos
void Ultrasonicos::evaluarMuros() {
  float distancia1 = ultrasonico1.MedirDistancia(); 
  if (distancia1 <= muroCercaDist) {
    muro_cercaIzq = true;
  } else if (distancia1 >= muroCercaDist && distancia1 <= pelotaDist) {
    pelotaIzq = true;
  } else {
    muro_lejos++;
  }

  float distancia2 = ultrasonico2.MedirDistancia();
  if (distancia2 <= muroCercaDist) {
    muro_cercaEnf = true;
  } else if (distancia2 > muroCercaDist && distancia2 <= pelotaDist) {
    pelotaEnf = true;
  } else { 
    muro_lejos++;
  }

  float distancia3 = ultrasonico3.MedirDistancia();
  if (distancia3 <= muroCercaDist) {
    muro_cercaDer = true;
  } else if (distancia3 >= muroCercaDist && distancia3 <= pelotaDist) {
    pelotaDer = true;
  } else {
    muro_lejos++;
  }
}

int Ultrasonicos::evaluarSituacion() {
  evaluarMuros(); // Llama a la función para actualizar los estados de los sensores
  if (muro_cercaIzq && muro_cercaEnf && !muro_cercaDer) {
    situacion = 1; //Esquina izquierda
  } else if (muro_cercaIzq && !muro_cercaEnf && muro_cercaDer) {
    situacion = 2; //Pasillo
  } else if (!muro_cercaIzq && muro_cercaEnf && muro_cercaDer) {
    situacion = 3; //Esquina derecha
  } else if (!muro_cercaIzq && pelotaEnf && !muro_cercaDer) {
    situacion = 4; //Pelota enfrente
  } else if (muro_cercaIzq && !muro_cercaEnf && pelotaDer) {
    situacion = 5; // Pelota a la derecha
  } else if (pelotaIzq && !muro_cercaEnf && muro_cercaDer) {
    situacion = 6; //Pelota a la izquierda
  } else if (!muro_cercaIzq && muro_cercaEnf && !muro_cercaDer) {
    situacion = 7; // Muro enfrente y sin muros cerca a los lados
  } else if (muro_cercaIzq && !muro_cercaEnf && !muro_cercaDer && !pelotaDer) {
    situacion = 8; //Muro izquierda y espacio a la derecha
  } else if (!muro_cercaIzq && !muro_cercaEnf && muro_cercaDer) {
    situacion = 9; //Muro derecha y espacio a la izquierda
  } else if (!muro_cercaIzq && !muro_cercaEnf && pelotaDer) {
    situacion = 10; //Pelota a la derecha de la salida
  } else if (!muro_cercaIzq && !muro_cercaEnf && pelotaDer) {
    situacion = 11; //Pelota a la izquierda de la salida
  }
  else {
    situacion = 12; // Si ninguna situación coincide, puedes definir una situación por defecto.
  }
  return situacion;
}

void Ultrasonicos::EnviarComandoArduino() {
  situacion = evaluarSituacion();
  switch (situacion) {
    case 1:
      Serial2.println(1);
      break;
    case 2:
      Serial2.println(2);
      break;
    case 3:
      Serial2.println(3);
      break;
    case 4:
      Serial2.println(4);
      break;
    case 5:
      Serial2.println(5);
      break;
    case 6:
      Serial2.println(6);
      break;
    case 7:
      Serial2.println(7);
      break;
    case 8:
      Serial2.println(8);
      break;
    case 9:
      Serial2.println(9);
      break;
    case 10:
      Serial2.println(10);
      break;
    case 11:
      Serial2.println(11);
      break;
    case 12:
      Serial2.println(12);
      break;
  }
}

