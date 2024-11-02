const int pinTrigIzquierdo = 32;        
const int pinEchoIzquierdo = 33;        
const int pinTrigDerecho = 25;           
const int pinEchoDerecho = 26;         
const int pinTrigFrontal = 27;        
const int pinEchoFrontal = 14;           

void setup_1() {
  Serial.begin(9600); 
  pinMode(pinTrigIzquierdo, OUTPUT);
  pinMode(pinEchoIzquierdo, INPUT);
  pinMode(pinTrigDerecho, OUTPUT);
  pinMode(pinEchoDerecho, INPUT);
  pinMode(pinTrigFrontal, OUTPUT);
  pinMode(pinEchoFrontal, INPUT);
}

void loop_1() {
  int distanciaFrontal = medirDistancia(pinTrigFrontal, pinEchoFrontal);
  int distanciaIzquierda = medirDistancia(pinTrigIzquierdo, pinEchoIzquierdo);
  int distanciaDerecha = medirDistancia(pinTrigDerecho, pinEchoDerecho);

  Serial.print(distanciaFrontal);
  Serial.print(",");
  Serial.print(distanciaIzquierda);
  Serial.print(",");
  Serial.print(distanciaDerecha);
  Serial.print(",");
  Serial.println(0); 

  delay(100); 
}

int medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  long duracion = pulseIn(pinEcho, HIGH);
  int distancia = duracion * 0.034 / 2; 
  
  return distancia;
}
