#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// CONFIGURAÇÃO ESP-NOW
uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x03, 0xE7, 0xB8};

// Estrutura para comunicação ESP-NOW
typedef struct struct_message {
    char command[10];
    long value;
    long position;
} struct_message;

struct_message myData;
struct_message receivedData;

// Prototypes
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
void IRAM_ATTR handleEncoderA();
void updateEncoder(struct EncoderData &encoder);
void handleButtons();
void handleSerialCommands();
void printHelp();
void printStatus();
void moveToPositionX(long position);
void stopAllMotors();
void stopMotorsX();
void moveBack();
void moveFront();
void resetX();
void printMovement(String movement);
void sendCommandToSecondary(String cmd, long value = 0, long position = 0);
void printEncoderStatus();
void resetAllEncoders();
void updateMotorDirections(int dirA, int dirB);

// Definições dos pinos de direção - (EIXO X - FREMTE/TRÁS)
#define IN1 32   // Motor A - Sentido 1
#define IN2 33   // Motor A - Sentido 2
#define IN3 12   // Motor B - Sentido 1
#define IN4 13   // Motor B - Sentido 2

// Definições dos botões
#define front 21
#define back 18
#define front_2 5
#define back_2 19
#define stop 27

#define ENCODER_A 34  // Encoder Motor A - Eixo X

// Variáveis de controle de posição
long targetPositionX = 0;
bool movingToTargetX = false;
bool resettingX = false;
bool flopFlopping = false;
String lastCommand = "STOP";
String lastPrintedCommand = "";

int motorDirectionA = 0;
int motorDirectionB = 0;

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

EncoderData encoderA = {0, 0, false, ' ', 0, ENCODER_A, &motorDirectionA};

const unsigned long DEBOUNCE_TIME = 1;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 10;

bool buttonWasPressed = false;
String currentMovement = "";

// CALLBACKS ESP-NOW
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.print("Status do envio: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len < (int)sizeof(receivedData)) {
        Serial.println("Tamanho de dados recebido inválido");
        return;
    }
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Confirmação recebida: ");
    Serial.print(receivedData.command);
    Serial.print(" - Valor: ");
    Serial.println(receivedData.value);

    Serial.print("De MAC: ");
    for (int i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        Serial.printf("%02X", recv_info->src_addr[i]);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(100);

    // Configuração de pinos
    pinMode(front, INPUT_PULLUP);
    pinMode(back, INPUT_PULLUP);
    pinMode(front_2, INPUT_PULLUP);
    pinMode(back_2, INPUT_PULLUP);
    pinMode(stop, INPUT_PULLUP);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);

    encoderA.estadoAnterior = digitalRead(ENCODER_A);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, CHANGE);

    // Configuração WiFi e ESP-NOW
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    
    // Registrar callbacks
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    
    // Registrar peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Falha ao adicionar peer");
    }

    stopAllMotors();

    Serial.println("=== ESP PRINCIPAL - EIXO X ===");
    Serial.print("Endereço MAC: ");
    Serial.println(WiFi.macAddress());
    printHelp();
}

void loop() {
  handleButtons();
  handleSerialCommands();

  // Verifica movimento para posição X
  if (movingToTargetX) {
    if (abs(encoderA.contador - targetPositionX) <= 3) {
      stopMotorsX();
      movingToTargetX = false;
      Serial.print("EIXO X chegou na posição: ");
      Serial.println(targetPositionX);
    }
  }

  // Verifica reset X
  if (resettingX) {
    if (digitalRead(back) == LOW && digitalRead(back_2) == LOW) {
      stopMotorsX();
      resettingX = false;
      noInterrupts();
      resetAllEncoders();
      interrupts();
      Serial.println("EIXO X resetado - Posição definida como 0");
    }
  }

  delay(10);
}

// Comunicação com a ESP Secundária
void sendCommandToSecondary(String cmd, long value, long position) {
    memset(&myData, 0, sizeof(myData));
    strncpy(myData.command, cmd.c_str(), sizeof(myData.command)-1);
    myData.value = value;
    myData.position = position;
    esp_err_t ret = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if(ret != ESP_OK){
        Serial.print("esp_now_send falhou: ");
        Serial.println(ret);
    }
}

// Função para mover eixo X para posição específica
void moveToPositionX(long position) {
    targetPositionX = position;
    long currentPosition = encoderA.contador;

    if (currentPosition < position) {
        moveFront();
        movingToTargetX = true;
        Serial.print("Movendo EIXO X para FRENTE - Alvo: ");
        Serial.println(position);
    } else if (currentPosition > position) {
        moveBack();
        movingToTargetX = true;
        Serial.print("Movendo EIXO X para TRÁS - Alvo: ");
        Serial.println(position);
    } else {
        stopMotorsX();
        Serial.println("EIXO X já está na posição");
    }
}

// Função para reset do eixo X
void resetX() {
    Serial.println("Iniciando reset EIXO X");
    resettingX = true;
    moveBack();
}

// Função para controle de print de movimento
void printMovement(String movement) {
    if (movement != lastPrintedCommand) {
        Serial.println(movement);
        lastPrintedCommand = movement;
    }
}

// Função para controle dos botões
void handleButtons() {
    if (millis() - lastButtonTime < BUTTON_DEBOUNCE) return;

    bool frontPressed = (digitalRead(front) == LOW && digitalRead(front_2) == LOW);
    bool backPressed = (digitalRead(back) == LOW && digitalRead(back_2) == LOW);
    bool stopPressed = (digitalRead(stop) == LOW);
    
    bool anyButtonPressed = frontPressed || backPressed || stopPressed;

    if (!anyButtonPressed) {
        buttonWasPressed = false;
        currentMovement = "";
        lastPrintedCommand = "";
        return;
    }

    if(digitalRead(stop) == LOW){
        if (!buttonWasPressed || currentMovement != "STOP") {
            printMovement("PARANDO TODOS OS MOTORES");
            stopAllMotors();
            movingToTargetX = false;
            resettingX = false;
            lastCommand = "STOP";
            currentMovement = "STOP";
            buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
    else if(digitalRead(front) == LOW && digitalRead(front_2) == LOW){
        if (!buttonWasPressed || currentMovement != "FRONT") {
          printEncoderStatus();
          if(flopFlopping){
            moveBack();
            printMovement("MOVENDO EIXO X: FRENTE");
          }
          movingToTargetX = false;
          lastCommand = "FRONT";
          currentMovement = "FRONT";
          buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
    else if(digitalRead(back) == LOW && digitalRead(back_2) == LOW){
        if (!buttonWasPressed || currentMovement != "BACK") {
          printEncoderStatus();
          if(flopFlopping){
            moveFront();
            printMovement("MOVENDO EIXO X: BACK");
          }
          movingToTargetX = false;
          lastCommand = "BACK";
          currentMovement = "BACK";
          buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
}

// Função para controle da comunicação serial
void handleSerialCommands() {
    if(Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if(command.length() > 0) {
            char cmd = command.charAt(0);

            switch(cmd) {
                case 'w': case 'W':
                    movingToTargetX = false;
                    resettingX = false;
                    moveFront();
                    Serial.println("Movendo EIXO X: BACK");
                    lastCommand = "BACK";
                    break;

                case 's': case 'S':
                    movingToTargetX = false;
                    resettingX = false;
                    moveBack();
                    Serial.println("Movendo EIXO X: FRENTE");
                    lastCommand = "FRONT";
                    break;

                case 'a': case 'A':
                    sendCommandToSecondary("MOVE", 1);
                    Serial.println("Comando enviado: EIXO Y ESQUERDA");
                    break;

                case 'd': case 'D':
                    sendCommandToSecondary("MOVE", -1);
                    Serial.println("Comando enviado: EIXO Y DIREITA");
                    break;

                case 'x': case 'X':
                    stopAllMotors();
                    sendCommandToSecondary("STOP");
                    movingToTargetX = false;
                    resettingX = false;
                    Serial.println("PARANDO TODOS OS MOTORES");
                    lastCommand = "STOP";
                    break;

                case 'p': case 'P':
                    printEncoderStatus();
                    break;

                case 'z': case 'Z':
                    resetAllEncoders();
                    sendCommandToSecondary("RESET_ENC");
                    break;

                case 'r': case 'R':
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        if (axis == 'x' || axis == 'X') {
                            resetX();
                        } else if (axis == 'y' || axis == 'Y') {
                            Serial.println("Enviando: RESET EIXO Y");
                            sendCommandToSecondary("RESET");
                        }
                    }
                    break;

                case 'g': case 'G':
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        long position = command.substring(2).toInt();
                        if (axis == 'x' || axis == 'X') {
                            moveToPositionX(position);
                        } else if (axis == 'y' || axis == 'Y') {
                            sendCommandToSecondary("GOTO", 0, position);
                            Serial.print("Enviando: EIXO Y para posição ");
                            Serial.println(position);
                        }
                    }
                    break;

                case 'h': case 'H':
                    printHelp();
                    break;

                case 'i': case 'I':
                    printStatus();
                    break;
            }
        }
    }
}

// Funções de interrupção para encoder
void IRAM_ATTR handleEncoderA() { updateEncoder(encoderA); }

// Funções de atualização para encoder
void updateEncoder(EncoderData &encoder) {
    unsigned long tempoAtual = micros();
    if (tempoAtual - encoder.ultimoTempoInterrupcao < DEBOUNCE_TIME) return;
    encoder.ultimoTempoInterrupcao = tempoAtual;

    int estadoAtual = digitalRead(encoder.pino);
    if (encoder.estadoAnterior != estadoAtual) {
        if (*(encoder.motorDirection) > 0) {
            encoder.contador++;
            encoder.direcao = 'F';
        } else if (*(encoder.motorDirection) < 0) {
            encoder.contador--;
            encoder.direcao = 'T';
        }
        encoder.contadorAtualizado = true;
    }
    encoder.estadoAnterior = estadoAtual;
}

void printEncoderStatus() {
    Serial.println("=== STATUS ENCODERS ===");
    Serial.print("Motor A (X): ");
    Serial.print(encoderA.contador);
    Serial.print(" pulses - ");
    Serial.println(encoderA.direcao == 'F' ? "BACK" : (encoderA.direcao == 'T' ? "FRENTE" : "PARADO"));
    Serial.println("=======================");
    encoderA.contadorAtualizado = false;
}

void resetAllEncoders() {
    noInterrupts();
    encoderA.contador = 0;
    encoderA.direcao = ' ';
    interrupts();
    Serial.println("Encoder do EIXO X resetado!");
}

void updateMotorDirections(int dirA, int dirB) {
    motorDirectionA = dirA;
    motorDirectionB = dirB;
}

void stopAllMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    updateMotorDirections(0, 0);
}

void stopMotorsX() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    motorDirectionA = 0;
    motorDirectionB = 0;
}

void moveBack() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    updateMotorDirections(-1, -1);
}

void moveFront() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    updateMotorDirections(1, 1);
}

void printHelp() {
    Serial.println();
    Serial.println("=== COMANDOS SERIAL ===");
    Serial.println("W - Mover EIXO Y para FRENTE");
    Serial.println("S - Mover EIXO Y para TRÁS");
    Serial.println("A - Mover EIXO X para FRENTE");
    Serial.println("D - Mover EIXO X para BACK");
    Serial.println("X - PARAR todos os motores");
    Serial.println("GX<num> - Move EIXO X para posição");
    Serial.println("GY<num> - Move EIXO Y para posição");
    Serial.println("RX - Reset EIXO X");
    Serial.println("RY - Reset EIXO Y");
    Serial.println("P - Status dos encoders");
    Serial.println("Z - Resetar encoders");
    Serial.println("I - Informações do sistema");
    Serial.println("H - Ajuda");
    Serial.println("========================");
}

void printStatus() {
    Serial.println();
    Serial.println("=== STATUS DO SISTEMA ===");
    Serial.print("Posição EIXO X: "); Serial.println(encoderA.contador);
    Serial.print("Movendo para alvo X: "); Serial.println(movingToTargetX ? "SIM" : "NÃO");
    Serial.print("Resetando X: "); Serial.println(resettingX ? "SIM" : "NÃO");
    Serial.print("Último comando: "); Serial.println(lastCommand);
    Serial.println("=========================");
}
