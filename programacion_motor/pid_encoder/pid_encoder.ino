#define ENCA 2 // Amarillo
#define ENCB 3 // Blanco

int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrumpt(digitalPinToInterrumpt(ENCA),readEncoder,RISING);

  // put your setup code here, to run once:
}

void loop() {
  Serial.print(pos);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
}