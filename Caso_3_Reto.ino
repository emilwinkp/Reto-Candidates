#include <QTRSensors.h>

const int motorIzquierdoAdelante = 5;  
const int motorIzquierdoAtras = 6;     
const int motorDerechoAdelante = 9;     
const int motorDerechoAtras = 10;       

enum Direccion { ADELANTE, IZQUIERDA, DERECHA, ATRAS };
Direccion direccionActual;

bool visitado[5][3]; 
int x = 0; 
int y = 0; 

void setup() {
  pinMode(motorIzquierdoAdelante, OUTPUT);
  pinMode(motorIzquierdoAtras, OUTPUT);
  pinMode(motorDerechoAdelante, OUTPUT);
  pinMode(motorDerechoAtras, OUTPUT);

  Serial.begin(9600);
  direccionActual = ADELANTE;

  memset(visitado, 0, sizeof(visitado));
}

void loop() {

  if (Serial.available() > 0) {
    int distanciaFrontal = Serial.parseInt(); 
    int distanciaIzquierda = Serial.parseInt();
    int distanciaDerecha = Serial.parseInt(); 
    int valorQTR = Serial.parseInt(); 

    if (valorQTR < 2000) {
      detenerMotores();
      Serial.println("¡Negro detectado! Deteniendo.");
      return; 
    }

    visitado[x][y] = true;

    if (distanciaFrontal > 20 && !visitado[x][y + 1]) { 
      moverAdelante();
      y++; 
    } else if (distanciaDerecha > 20 && !visitado[x + 1][y]) { 
      girarDerecha();
      moverAdelante();
      x++; 
      direccionActual = DERECHA;
    } else if (distanciaIzquierda > 20 && !visitado[x - 1][y]) { 
      girarIzquierda();
      moverAdelante();
      x--; 
      direccionActual = IZQUIERDA;
    } else {
      
      if (direccionActual == ADELANTE) {
        girarIzquierda();
        moverAdelante(); 
        direccionActual = ATRAS;
      } else {
        girarDerecha();
        moverAdelante(); 
        direccionActual = ATRAS;
      }
    }
  }

  delay(100);
}

void moverAdelante() {
  digitalWrite(motorIzquierdoAdelante, HIGH);
  digitalWrite(motorDerechoAdelante, HIGH);
  digitalWrite(motorIzquierdoAtras, LOW);
  digitalWrite(motorDerechoAtras, LOW);
}

void girarIzquierda() {
  digitalWrite(motorIzquierdoAdelante, LOW);
  digitalWrite(motorDerechoAdelante, HIGH);
  digitalWrite(motorIzquierdoAtras, LOW);
  digitalWrite(motorDerechoAtras, LOW);
}

void girarDerecha() {
  digitalWrite(motorIzquierdoAdelante, HIGH);
  digitalWrite(motorDerechoAdelante, LOW);
  digitalWrite(motorIzquierdoAtras, LOW);
  digitalWrite(motorDerechoAtras, LOW);
}

void detenerMotores() {
  digitalWrite(motorIzquierdoAdelante, LOW);
  digitalWrite(motorDerechoAdelante, LOW);
  digitalWrite(motorIzquierdoAtras, LOW);
  digitalWrite(motorDerechoAtras, LOW);
}
