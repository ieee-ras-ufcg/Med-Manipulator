int IN1 = 8;  // Controle bobina A
int IN2 = 9;  // Controle bobina A
int IN3 = 10; // Controle bobina B
int IN4 = 11; // Controle bobina B

int passo = 0;
unsigned long ultimoTempo = 0;
unsigned long intervalo = 3000; 
bool sentidoHorario = true;
bool motorLigado = false; 

void setup() {
  Serial.begin(9600);
  Serial.println("Digite: 1 - Sentido horário; 2 - Sentido Anti-horário; 3 - Parar motor");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == '1') {
      sentidoHorario = true;
      motorLigado = true;
    } 
    else if (cmd == '2') {
      sentidoHorario = false;
      motorLigado = true;
    } 
    else if (cmd == '0') {
      pararMotor();
      motorLigado = false; 
    }
  }

  if (motorLigado) {
    unsigned long agora = micros();
    if (agora - ultimoTempo >= intervalo) {
      passoMotor();
      ultimoTempo = agora;
    }
  }
}

void passoMotor() {
  switch (passo) {
    case 0: digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  break;
    case 1: digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  break;
    case 2: digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); break;
    case 3: digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); break;
  }

  if (sentidoHorario)
    passo = (passo + 1) % 4;
  else
    passo = (passo - 1 + 4) % 4;
}

void pararMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
