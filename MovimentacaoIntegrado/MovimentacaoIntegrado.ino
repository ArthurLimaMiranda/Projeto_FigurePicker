#include <Arduino.h>

// Definições dos pinos de direção - Ponte 1 (EIXO X - ESQUERDA/DIREITA)
#define IN1 33   // Motor A - Sentido 1
#define IN2 32   // Motor A - Sentido 2
#define IN3 12   // Motor B - Sentido 1
#define IN4 13   // Motor B - Sentido 2

// Definições dos pinos de direção - Ponte 2 (EIXO Y - FRENTE/TRÁS)
#define IN5 15   // Motor C - Sentido 1
#define IN6 16   // Motor C - Sentido 2

// Definições dos botões
#define left 21
#define right 18
#define left_2 5
#define right_2 19
#define stop 27
#define front 22
#define back 23

// Pinos dos Encoders para cada motor (apenas canal A)
#define ENCODER_A 34  // Encoder Motor A - Eixo X
#define ENCODER_C 14  // Encoder Motor C - Eixo Y

// Variáveis de controle de posição
long targetPositionX = 0;
long targetPositionY = 0;
bool movingToTargetX = false;
bool movingToTargetY = false;

// Variáveis para reset
bool resettingX = false;
bool resettingY = false;

// Variável para armazenar último movimento
String lastCommand = "STOP";
String lastPrintedCommand = "";

// Direção atual de cada motor
int motorDirectionA = 0;  // 1: direita, -1: esquerda, 0: parado
int motorDirectionB = 0;  // 1: direita, -1: esquerda, 0: parado  
int motorDirectionC = 0;  // 1: frente, -1: trás, 0: parado

// Estrutura para armazenar dados dos encoders
struct EncoderData {
  volatile long contador;
  volatile int estadoAnterior;
  volatile boolean contadorAtualizado;
  volatile char direcao;
  volatile unsigned long ultimoTempoInterrupcao;
  int pino;
  int* motorDirection;
};

// Instâncias dos encoders para motores A e C
EncoderData encoderA = {0, 0, false, ' ', 0, ENCODER_A, &motorDirectionA};
EncoderData encoderC = {0, 0, false, ' ', 0, ENCODER_C, &motorDirectionC};

const unsigned long DEBOUNCE_TIME = 10;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 10;

// Variáveis para controle de impressão
bool buttonWasPressed = false;
String currentMovement = "";

// Protótipos das funções
void IRAM_ATTR handleEncoderA();
void IRAM_ATTR handleEncoderC();
void updateEncoder(EncoderData &encoder);
void printEncoderStatus();
void resetAllEncoders();
void updateMotorDirections(int dirA, int dirB, int dirC);
void autoTestSequence();
void moveToPositionX(long position);
void moveToPositionY(long position);
void stopAllMotors();
void stopMotorsX();
void stopMotorY();
void moveLeft();
void moveRight();
void moveForward();
void moveBackward();
void resetX();
void resetY();
void printMovement(String movement);

void setup() {
  Serial.begin(115200);
  
  // Configura botões
  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(left_2, INPUT_PULLUP);
  pinMode(right_2, INPUT_PULLUP);
  pinMode(stop, INPUT_PULLUP);
  pinMode(front, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  
  // Configura pinos de direção
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  
  // Configura pinos dos encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  
  // Lê estado inicial dos encoders
  encoderA.estadoAnterior = digitalRead(ENCODER_A);
  encoderC.estadoAnterior = digitalRead(ENCODER_C);
  
  // Configura interrupções dos encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), handleEncoderC, CHANGE);

  // Para todos os motores inicialmente
  stopAllMotors();
  
  Serial.println("=== SISTEMA 3 MOTORES - EIXO X/Y ===");
  Serial.println("Motores A/B: ESQUERDA/DIREITA (EIXO X) - Encoder A");
  Serial.println("Motor C: FRENTE/TRÁS (EIXO Y) - Encoder C");
  Serial.println("Controle por posição com encoders");
  printHelp();
}

void loop() {
  // Controle por botões físicos
  handleButtons();
  
  // Controle por serial
  handleSerialCommands();
  
  // Verifica movimento para posição X
  if (movingToTargetX) {
    if (abs(encoderA.contador - targetPositionX) <= 10) { // Margem de erro de 10 pulsos
      stopMotorsX();
      movingToTargetX = false;
      Serial.print("EIXO X chegou na posição: ");
      Serial.println(targetPositionX);
    }
  }
  
  // Verifica movimento para posição Y
  if (movingToTargetY) {
    if (abs(encoderC.contador - targetPositionY) <= 10) { // Margem de erro de 10 pulsos
      stopMotorY();
      movingToTargetY = false;
      Serial.print("EIXO Y chegou na posição: ");
      Serial.println(targetPositionY);
    }
  }
  
  // Verifica reset X
  if (resettingX) {
    if (digitalRead(right) == LOW && digitalRead(right_2) == LOW) {
      stopMotorsX();
      resettingX = false;
      encoderA.contador = 0;
      Serial.println("EIXO X resetado - Posição definida como 0");
    }
  }
  
  // Verifica reset Y
  if (resettingY) {
    if (digitalRead(back) == LOW) {
      stopMotorY();
      resettingY = false;
      encoderC.contador = 0;
      Serial.println("EIXO Y resetado - Posição definida como 0");
    }
  }

  delay(10);
}

// Função para imprimir movimento
void printMovement(String movement) {
  if (movement != lastPrintedCommand) {
    Serial.println(movement);
    printEncoderStatus();
    lastPrintedCommand = movement;
  }
}

// Funções de interrupção para cada encoder
void IRAM_ATTR handleEncoderA() { updateEncoder(encoderA); }
void IRAM_ATTR handleEncoderC() { updateEncoder(encoderC); }

void updateEncoder(EncoderData &encoder) {
  unsigned long tempoAtual = micros();
  if (tempoAtual - encoder.ultimoTempoInterrupcao < DEBOUNCE_TIME) {
    return;
  }
  encoder.ultimoTempoInterrupcao = tempoAtual;
  
  int estadoAtual = digitalRead(encoder.pino);
  
  // Lógica para detecção de movimento
  if (encoder.estadoAnterior != estadoAtual) {
    if (*(encoder.motorDirection) > 0) {
      encoder.contador++;
      encoder.direcao = 'F'; // Frente/Direita
    } else if (*(encoder.motorDirection) < 0) {
      encoder.contador--;
      encoder.direcao = 'T'; // Trás/Esquerda
    }
    
    encoder.contadorAtualizado = true;
  }
  
  encoder.estadoAnterior = estadoAtual;
}

// Função para mover eixo X para posição específica
void moveToPositionX(long position) {
  targetPositionX = position;
  long currentPosition = encoderA.contador;
  
  if (currentPosition < position) {
    moveRight();
    movingToTargetX = true;
    Serial.print("Movendo EIXO X para DIREITA - Alvo: ");
    Serial.println(position);
  } else if (currentPosition > position) {
    moveLeft();
    movingToTargetX = true;
    Serial.print("Movendo EIXO X para ESQUERDA - Alvo: ");
    Serial.println(position);
  } else {
    stopMotorsX();
    Serial.println("EIXO X já está na posição desejada");
  }
}

// Função para mover eixo Y para posição específica
void moveToPositionY(long position) {
  targetPositionY = position;
  long currentPosition = encoderC.contador;
  
  if (currentPosition < position) {
    moveForward();
    movingToTargetY = true;
    Serial.print("Movendo EIXO Y para FRENTE - Alvo: ");
    Serial.println(position);
  } else if (currentPosition > position) 
    moveBackward();
    movingToTargetY = true;
    Serial.print("Movendo EIXO Y para TRÁS - Alvo: ");
    Serial.println(position);
  } else {
    stopMotorY();
    Serial.println("EIXO Y já está na posição desejada");
  }
}

// Função para reset do eixo X
void resetX() {
  Serial.println("Iniciando reset EIXO X - Movendo para DIREITA até fim de curso");
  resettingX = true;
  moveRight();
}

// Função para reset do eixo Y
void resetY() {
  Serial.println("Iniciando reset EIXO Y - Movendo para TRÁS até fim de curso");
  resettingY = true;
  moveBackward();
}

// Função para atualizar as direções de todos os motores
void updateMotorDirections(int dirA, int dirB, int dirC) {
  motorDirectionA = dirA;
  motorDirectionB = dirB;
  motorDirectionC = dirC;
}

void printEncoderStatus() {
  Serial.println("=== STATUS ENCODERS ===");
  Serial.print("Motor A (X): ");
  Serial.print(encoderA.contador);
  Serial.print(" pulses - ");
  Serial.println(encoderA.direcao == 'F' ? "DIREITA" : (encoderA.direcao == 'T' ? "ESQUERDA" : "PARADO"));
  
  Serial.print("Motor C (Y): ");
  Serial.print(encoderC.contador);
  Serial.print(" pulses - ");
  Serial.println(encoderC.direcao == 'F' ? "FRENTE" : (encoderC.direcao == 'T' ? "TRÁS" : "PARADO"));
  Serial.println("=======================");
  
  encoderA.contadorAtualizado = false;
  encoderC.contadorAtualizado = false;
}

void resetAllEncoders() {
  noInterrupts();
  encoderA.contador = 0;
  encoderC.contador = 0;
  encoderA.direcao = ' ';
  encoderC.direcao = ' ';
  interrupts();
  
  Serial.println("Todos os encoders foram resetados!");
}

void handleButtons() {
  if (millis() - lastButtonTime < BUTTON_DEBOUNCE) {
    return;
  }
  
  bool anyButtonPressed = (digitalRead(stop) == LOW) ||
                         (digitalRead(left) == LOW && digitalRead(left_2) == LOW) ||
                         (digitalRead(right) == LOW && digitalRead(right_2) == LOW) ||
                         (digitalRead(front) == LOW) ||
                         (digitalRead(back) == LOW);
                         
  if (!anyButtonPressed) {
    buttonWasPressed = false;
    currentMovement = "";
    lastPrintedCommand = "";
  }
  
  if(digitalRead(stop) == LOW){
    if (!buttonWasPressed || currentMovement != "STOP") {
      printMovement("PARANDO TODOS OS MOTORES");
      stopAllMotors();
      movingToTargetX = false;
      movingToTargetY = false;
      resettingX = false;
      resettingY = false;
      lastCommand = "STOP";
      currentMovement = "STOP";
      buttonWasPressed = true;
    }
    lastButtonTime = millis();
  }
  else if(digitalRead(left) == LOW && digitalRead(left_2) == LOW){
    if (!buttonWasPressed || currentMovement != "LEFT") {
      printMovement("MOVENDO EIXO X: ESQUERDA");
      movingToTargetX = false;
      resettingX = false;
      moveLeft();
      lastCommand = "LEFT";
      currentMovement = "LEFT";
      buttonWasPressed = true;
    }
    lastButtonTime = millis();
  }
  else if(digitalRead(right) == LOW && digitalRead(right_2) == LOW){
    if (!buttonWasPressed || currentMovement != "RIGHT") {
      printMovement("MOVENDO EIXO X: DIREITA");
      movingToTargetX = false;
      resettingX = false;
      moveRight();
      lastCommand = "RIGHT";
      currentMovement = "RIGHT";
      buttonWasPressed = true;
    }
    lastButtonTime = millis();
  }
  
  else if(digitalRead(front) == LOW){
    if (!buttonWasPressed || currentMovement != "FORWARD") {
      printMovement("MOVENDO EIXO Y: FRENTE");
      movingToTargetY = false;
      resettingY = false;
      moveForward();
      lastCommand = "FORWARD";
      currentMovement = "FORWARD";
      buttonWasPressed = true;
    }
    lastButtonTime = millis();
  }
  else if(digitalRead(back) == LOW){
    if (!buttonWasPressed || currentMovement != "BACKWARD") {
      printMovement("MOVENDO EIXO Y: TRÁS");
      movingToTargetY = false;
      resettingY = false;
      moveBackward();
      lastCommand = "BACKWARD";
      currentMovement = "BACKWARD";
      buttonWasPressed = true;
    }
    lastButtonTime = millis();
  }
}

void handleSerialCommands() {
  if(Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command.length() > 0) {
      char cmd = command.charAt(0);
      
      switch(cmd) {
        case 'w': case 'W': // Frente (Eixo Y)
          movingToTargetY = false;
          resettingY = false;
          moveForward();
          Serial.println("Movendo EIXO Y: FRENTE");
          lastCommand = "FORWARD";
          break;
          
        case 's': case 'S': // Trás (Eixo Y)
          movingToTargetY = false;
          resettingY = false;
          moveBackward();
          Serial.println("Movendo EIXO Y: TRÁS");
          lastCommand = "BACKWARD";
          break;
          
        case 'a': case 'A': // Esquerda (Eixo X)
          movingToTargetX = false;
          resettingX = false;
          moveLeft();
          Serial.println("Movendo EIXO X: ESQUERDA");
          lastCommand = "LEFT";
          break;
          
        case 'd': case 'D': // Direita (Eixo X)
          movingToTargetX = false;
          resettingX = false;
          moveRight();
          Serial.println("Movendo EIXO X: DIREITA");
          lastCommand = "RIGHT";
          break;
          
        case 'x': case 'X': // Parar
          stopAllMotors();
          movingToTargetX = false;
          movingToTargetY = false;
          resettingX = false;
          resettingY = false;
          Serial.println("PARANDO");
          lastCommand = "STOP";
          break;
          
        case 'p': case 'P': // Mostrar status dos encoders
          printEncoderStatus();
          break;

        case 'z': case 'Z': // Resetar todos os encoders
          resetAllEncoders();

        case 'r': case 'R': // Reset de eixos
          if (command.length() > 1) {
            char axis = command.charAt(1);
            if (axis == 'x' || axis == 'X') {
              resetX();
            } else if (axis == 'y' || axis == 'Y') {
              resetY();
            } else {
              Serial.println("Comando reset inválido. Use RX ou RY");
            }
          } else {
            Serial.println("Comando reset inválido. Use RX ou RY");
          }
          break;

        case 'g': case 'G': // Ir para posição
          if (command.length() > 1) {
            char axis = command.charAt(1);
            long position = command.substring(2).toInt();
            if (axis == 'x' || axis == 'X') {
              moveToPositionX(position);
            } else if (axis == 'y' || axis == 'Y') {
              moveToPositionY(position);
            } else {
              Serial.println("Comando goto inválido. Use GX123 ou GY456");
            }
          } else {
            Serial.println("Comando goto inválido. Use GX123 ou GY456");
          }
          break;
          
        case 'h': case 'H': // Ajuda
          printHelp();
          break;
          
        case 'i': case 'I': // Informações
          printStatus();
          break;
          
        default:
          Serial.println("Comando inválido! Digite 'H' para ajuda.");
          break;
      }
    }
  }
}

void printHelp() {
  Serial.println();
  Serial.println("=== COMANDOS SERIAL ===");
  Serial.println("W - Mover EIXO Y para FRENTE");
  Serial.println("S - Mover EIXO Y para TRÁS");
  Serial.println("A - Mover EIXO X para ESQUERDA");
  Serial.println("D - Mover EIXO X para DIREITA");
  Serial.println("X - PARAR todos os motores");
  Serial.println("GX<num> - Move EIXO X para posição (ex: GX1000)");
  Serial.println("GY<num> - Move EIXO Y para posição (ex: GY500)");
  Serial.println("RX - Reset EIXO X (move até fim de curso)");
  Serial.println("RY - Reset EIXO Y (move até fim de curso)");
  Serial.println("M - Sequência de teste automático");
  Serial.println("P - Status dos encoders");
  Serial.println("Z - Resetar encoders (zerar contagem)");
  Serial.println();
  Serial.println("I - Informações do sistema");
  Serial.println("H - Mostrar esta ajuda");
  Serial.println("========================");
}

void printStatus() {
  Serial.println();
  Serial.println("=== STATUS DO SISTEMA ===");
  Serial.print("Posição EIXO X: "); Serial.println(encoderA.contador);
  Serial.print("Posição EIXO Y: "); Serial.println(encoderC.contador);
  Serial.print("Movendo para alvo X: "); Serial.println(movingToTargetX ? "SIM" : "NÃO");
  Serial.print("Movendo para alvo Y: "); Serial.println(movingToTargetY ? "SIM" : "NÃO");
  Serial.print("Resetando X: "); Serial.println(resettingX ? "SIM" : "NÃO");
  Serial.print("Resetando Y: "); Serial.println(resettingY ? "SIM" : "NÃO");
  Serial.print("Último comando: "); Serial.println(lastCommand);
  Serial.println("=========================");
}

// Função para parar todos os motores
void stopAllMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  
  updateMotorDirections(0, 0, 0);
}

// Função para parar apenas os motores do eixo X
void stopMotorsX() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  motorDirectionA = 0;
  motorDirectionB = 0;
}

// Função para parar apenas o motor do eixo Y
void stopMotorY() {
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  
  motorDirectionC = 0;
}

// Movimento EIXO X: ESQUERDA
void moveLeft() {
  // Motores A e B para esquerda
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  
  stopMotorY();
  
  updateMotorDirections(-1, -1, 0);
}

// Movimento EIXO X: DIREITA
void moveRight() {
  // Motores A e B para direita
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  
  stopMotorY();
  
  updateMotorDirections(1, 1, 0);
}

// Movimento EIXO Y: FRENTE
void moveForward() {
  
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  
  stopMotorsX();
  
  updateMotorDirections(0, 0, 1);
}

// Movimento EIXO Y: TRÁS
void moveBackward() {
  
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  
  stopMotorsX();
  
  updateMotorDirections(0, 0, -1);
}