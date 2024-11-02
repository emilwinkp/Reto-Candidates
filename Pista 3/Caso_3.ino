#include <QTRSensors.h>

// Pines de sensores ultrasónicos
const int trigPinFront = 2;
const int echoPinFront = 3;
const int trigPinLeft = 4;
const int echoPinLeft = 5;
const int trigPinRight = 6;
const int echoPinRight = 7;

const int in1 = 8;   // Motor izquierdo
const int in2 = 9;   // Motor izquierdo
const int in3 = 10;  // Motor derecho
const int in4 = 11;  // Motor derecho
const int enA = 5;   // Pin de velocidad del motor izquierdo
const int enB = 6; // Pin de velocidad del motor derecho

// Pines del sensor de color QTR
const int colorSensorPin = A0; // Usa el pin correspondiente a tu sensor

// Variables para guardar distancias
int distanceFront, distanceLeft, distanceRight;

// Funciones de movimiento (puedes personalizar con tus configuraciones de motor)
void moveForward() {
    // Motor izquierdo hacia adelante
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor derecho hacia adelante
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Puedes ajustar la velocidad según sea necesario
  analogWrite(enA, 200); // Ajusta según el PWM necesario
  analogWrite(enB, 200);
}
void turnLeft() {
    // Motor izquierdo hacia atrás
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Motor derecho hacia adelante
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Ajusta la velocidad si es necesario
  analogWrite(enA, 200); // Velocidad del motor izquierdo
  analogWrite(enB, 200); // Velocidad del motor derecho
}
void turnRight() {
   // Motor izquierdo hacia adelante
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor derecho hacia atrás
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Ajusta la velocidad si es necesario
  analogWrite(enA, 200); // Velocidad del motor izquierdo
  analogWrite(enB, 200); // Velocidad del motor derecho
}
void stop() {
  // Motor izquierdo detenido
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  // Motor derecho detenido
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Puedes poner las velocidades en 0 para asegurarte de que esté completamente detenido
  analogWrite(enA, 0); 
  analogWrite(enB, 0);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; 
}

String detectColor() {
  int sensorValue = analogRead(colorSensorPin);
  if (sensorValue < 100) return "NEGRO";
  else if (sensorValue < 300) return "MAGENTA";
  else if (sensorValue < 500) return "PURPURA";
  else if (sensorValue < 700) return "AMARILLO";
  else if (sensorValue < 900) return "VERDE";
  else return "ROJO";
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(colorSensorPin, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  analogWrite(enA, 200); // Velocidad del motor izquierdo
  analogWrite(enB, 200); // Velocidad del motor derecho

}

void loop() {
  
  distanceFront = getDistance(trigPinFront, echoPinFront);
  distanceLeft = getDistance(trigPinLeft, echoPinLeft);
  distanceRight = getDistance(trigPinRight, echoPinRight);

  String color = detectColor();
  Serial.println(color);

  if (color == "VERDE") {
    Serial.println("Inicio del laberinto");
    moveForward();
  }
  else if (color == "ROJO") {
    Serial.println("Fin del laberinto");
    stop();
    while (true);
  }
  else if (color == "NEGRO") {
    Serial.println("Obstáculo detectado en el suelo");
    stop();
    delay(500);
    turnRight();
  }
  else {
    if (distanceFront < 15) {
      Serial.println("Obstáculo en frente");
      stop();
      delay(500);
      if (distanceLeft > distanceRight) {
        turnLeft();
      } else {
        turnRight();
      }
    } else {
      moveForward();
    }
  }
  
  delay(100); 
}

struct Cell {
  bool visited;
  bool obstacle;
};

const int gridSize = 20; // Tamaño máximo estimado del laberinto
Cell map[gridSize][gridSize]; // Mapa dinámico del laberinto
int posX = gridSize / 2, posY = gridSize - 1; // Establecer posición inicial al "centro-abajo"

// Ajusta según el tamaño real del laberinto
void markCurrentCellAsVisited() {
  map[posY][posX].visited = true;
}

void exploreMaze() {
  markCurrentCellAsVisited();
  detectAndMarkObstacles();

  // Ejemplo de movimiento en dirección no visitada
  if (!map[posY - 1][posX].visited && !map[posY - 1][posX].obstacle) {
    // Mover hacia arriba
    moveUp();
    posY--;
  } else if (!map[posY][posX + 1].visited && !map[posY][posX + 1].obstacle) {
    // Mover a la derecha
    moveRight();
    posX++;
  } else if (!map[posY + 1][posX].visited && !map[posY + 1][posX].obstacle) {
    // Mover hacia abajo
    moveDown();
    posY++;
  } else if (!map[posY][posX - 1].visited && !map[posY][posX - 1].obstacle) {
    // Mover a la izquierda
    moveLeft();
    posX--;
  } else {
    // Retroceder si no hay celdas no visitadas alrededor
    backtrack();
  }
}
