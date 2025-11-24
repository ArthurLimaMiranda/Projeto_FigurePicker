#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// CONFIGURAÇÃO ESP-NOW
uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x85, 0x8A, 0xB8};

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
void IRAM_ATTR handleEncoderC();
void updateEncoder(EncoderData &encoder);
void moveLeft();
void moveRight();
void stopMotorsY();
void moveToPositionY(long position);
void resetY();
void resetAllEncoders();
void printEncoderStatus();
void stopAllMotors();
void handleButtons();
void handleSerialCommands();
void printHelp();
void printStatus();
void updateMotorDirections(int dirC);

// Definições dos pinos de direção - (EIXO Y - ESQUERDA/DIREITA)
#define IN5 33   // Motor C - Sentido 1
#define IN6 32   // Motor C - Sentido 2

// Definições dos botões
#define right 25
#define left 26

#define ENCODER_C 27  // Encoder Motor C - Eixo Y

// Variáveis de controle
long targetPositionY = 0;
bool movingToTargetY = false;
bool resettingY = false;
bool flopFlopping = false;
String lastCommand = "STOP";
String lastPrintedCommand = "";

int motorDirectionC = 0;

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

EncoderData encoderC = {0, 0, false, ' ', 0, ENCODER_C, &motorDirectionC};

const unsigned long DEBOUNCE_TIME = 1;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 10;

bool buttonWasPressed = false;
String currentMovement = "";

// CALLBACKS ESP-NOW
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len < (int)sizeof(receivedData)) {
        Serial.println("Tamanho de dados recebido inválido");
        return;
    }
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    Serial.print("Comando recebido: ");
    Serial.print(receivedData.command);
    Serial.print(" - Valor: ");
    Serial.println(receivedData.value);
    
    // Processa os comandos recebidos
    if (strcmp(receivedData.command, "MOVE") == 0) {
        if(receivedData.value == 1) {
            moveLeft();
            Serial.println("EIXO Y: Movendo para ESQUERDA");
        } else if (receivedData.value == -1) {
            moveRight();
            Serial.println("EIXO Y: Movendo para DIREITA");
        }
    } 
    else if (strcmp(receivedData.command, "STOP") == 0) {
        stopMotorsY();
        Serial.println("EIXO Y: Parando");
    }
    else if (strcmp(receivedData.command, "GOTO") == 0) {
        moveToPositionY(receivedData.position);
        Serial.print("EIXO Y: Indo para posição ");
        Serial.println(receivedData.position);
    }
    else if (strcmp(receivedData.command, "RESET") == 0) {
        resetY();
        Serial.println("EIXO Y: Iniciando reset");
    }
    else if (strcmp(receivedData.command, "RESET_ENC") == 0) {
        resetAllEncoders();
        Serial.println("EIXO Y: Encoder resetado");
    }
    else if (strcmp(receivedData.command, "STATUS") == 0) {
        printEncoderStatus();
    }
    strcpy(myData.command, "CONFIRM");
    myData.value = receivedData.value;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void setup() {
    Serial.begin(115200);
    delay(100);

    // Configuração de pinos
    pinMode(left, INPUT_PULLUP);
    pinMode(right, INPUT_PULLUP);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(ENCODER_C, INPUT_PULLUP);

    encoderC.estadoAnterior = digitalRead(ENCODER_C);
    attachInterrupt(digitalPinToInterrupt(ENCODER_C), handleEncoderC, CHANGE);
    
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

    Serial.println("=== ESP PRINCIPAL - EIXO Y ===");
    Serial.print("Endereço MAC: ");
    Serial.println(WiFi.macAddress());
    printHelp();
}

void loop() {
    handleButtons();
    handleSerialCommands();

    // Verifica movimento para posição Y
    if (movingToTargetY) {
        if (abs(encoderC.contador - targetPositionY) <= 3) {
            stopMotorsY();
            movingToTargetY = false;
            Serial.print("EIXO Y chegou na posição: ");
            Serial.println(targetPositionY);
        }
    }

    // Verifica reset Y
    if (resettingY) {
        if (digitalRead(right) == LOW) {
            stopMotorsY();
            resettingY = false;
            noInterrupts();
            encoderC.contador = 0;
            interrupts();
            Serial.println("EIXO Y resetado - Posição definida como 0");
        }
    }

    delay(10);
}

// Função para mover eixo Y para posição específica
void moveToPositionY(long position) {
    targetPositionY = position;
    long currentPosition = encoderC.contador;

    if (currentPosition < position) {
        moveLeft();
        movingToTargetY = true;
        Serial.print("Movendo EIXO Y para DIREITA - Alvo: ");
        Serial.println(position);
    } else if (currentPosition > position) {
        moveRight();
        movingToTargetY = true;
        Serial.print("Movendo EIXO Y para ESQUERDA - Alvo: ");
        Serial.println(position);
    } else {
        stopMotorsY();
        Serial.println("EIXO Y já está na posição");
    }
}

// Função para reset do eixo Y
void resetY() {
    Serial.println("Iniciando reset EIXO Y");
    resettingY = true;
    moveRight();
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

    bool anyButtonPressed = (digitalRead(left) == LOW) ||
                           (digitalRead(right) == LOW);

    if (!anyButtonPressed) {
        buttonWasPressed = false;
        currentMovement = "";
        lastPrintedCommand = "";
        return;
    }

    if(digitalRead(left) == LOW){
        if (!buttonWasPressed || currentMovement != "ESQUERDA") {
          printEncoderStatus();
          if(flopFlopping){
            moveRight();
            printMovement("MOVENDO EIXO Y: ESQUERDA");
          }
          movingToTargetY = false;
          lastCommand = "ESQUERDA";
          currentMovement = "ESQUERDA";
          buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
    else if(digitalRead(right) == LOW){
        if (!buttonWasPressed || currentMovement != "DIREITA") {
          printEncoderStatus();
          if(flopFlopping){
            moveLeft();
            printMovement("MOVENDO EIXO Y: DIREITA");
          }
          movingToTargetY = false;
          lastCommand = "DIREITA";
          currentMovement = "DIREITA";
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
                case 'a': case 'A':
                    movingToTargetY = false;
                    resettingY = false;
                    moveLeft();
                    Serial.println("Movendo EIXO Y: ESQUERDA");
                    lastCommand = "ESQUERDA";
                    break;

                case 'd': case 'D':
                    movingToTargetY = false;
                    resettingY = false;
                    moveRight();
                    Serial.println("Movendo EIXO Y: DIREITA");
                    lastCommand = "DIREITA";
                    break;

                case 'x': case 'X':
                    stopAllMotors();
                    movingToTargetY = false;
                    resettingY = false;
                    Serial.println("PARANDO TODOS OS MOTORES");
                    lastCommand = "STOP";
                    break;

                case 'p': case 'P':
                    printEncoderStatus();
                    break;

                case 'z': case 'Z':
                    resetAllEncoders();
                    break;

                case 'r': case 'R':
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        if (axis == 'y' || axis == 'Y') {
                            Serial.println("Enviando: RESET EIXO Y");
                            resetY();
                        }
                    }
                    break;

                case 'g': case 'G':
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        long position = command.substring(2).toInt();
                        if (axis == 'y' || axis == 'Y') {
                            Serial.print("Enviando: EIXO Y para posição ");
                            moveToPositionY(position);
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
void IRAM_ATTR handleEncoderC() {
    updateEncoder(encoderC);
}

// Funções de atualização para encoder
void updateEncoder(EncoderData &encoder) {
    unsigned long tempoAtual = micros();
    if (tempoAtual - encoder.ultimoTempoInterrupcao < DEBOUNCE_TIME) return;
    encoder.ultimoTempoInterrupcao = tempoAtual;

    int estadoAtual = digitalRead(encoder.pino);
    if (encoder.estadoAnterior != estadoAtual) {
        if (*(encoder.motorDirection) > 0) {
            encoder.contador++;
            encoder.direcao = 'E';
        } else if (*(encoder.motorDirection) < 0) {
            encoder.contador--;
            encoder.direcao = 'D';
        }
        encoder.contadorAtualizado = true;
    }
    encoder.estadoAnterior = estadoAtual;
}

void printEncoderStatus() {
    Serial.println("=== STATUS ENCODERS ===");
    Serial.print("Motor C (Y): ");
    Serial.print(encoderC.contador);
    Serial.print(" pulses - ");
    Serial.println(encoderC.direcao == 'D' ? "DIREITA" : (encoderC.direcao == 'E' ? "ESQUERDA" : "PARADO"));
    Serial.println("=======================");
    encoderC.contadorAtualizado = false;
}

void resetAllEncoders() {
    noInterrupts();
    encoderC.contador = 0;
    encoderC.direcao = ' ';
    interrupts();
    Serial.println("Encoder do EIXO Y resetado!");
}

void updateMotorDirections(int dirC) {
    motorDirectionC = dirC;
}

void stopAllMotors() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    updateMotorDirections(0);
}

void stopMotorsY() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    motorDirectionC = 0;
}

void moveLeft() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
    updateMotorDirections(1);
}

void moveRight() {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
    updateMotorDirections(-1);
}

void printHelp() {
    Serial.println();
    Serial.println("=== COMANDOS SERIAL ===");
    Serial.println("W - Mover EIXO Y para ESQUERDA");
    Serial.println("S - Mover EIXO Y para TRÁS");
    Serial.println("A - Mover EIXO Y para ESQUERDA");
    Serial.println("D - Mover EIXO Y para DIREITA");
    Serial.println("X - PARAR todos os motores");
    Serial.println("GX<num> - Move EIXO Y para posição");
    Serial.println("GY<num> - Move EIXO Y para posição");
    Serial.println("RX - Reset EIXO Y");
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
    Serial.print("Posição EIXO Y: "); Serial.println(encoderC.contador);
    Serial.print("Movendo para alvo Y: "); Serial.println(movingToTargetY ? "SIM" : "NÃO");
    Serial.print("Resetando Y: "); Serial.println(resettingY ? "SIM" : "NÃO");
    Serial.print("Último comando: "); Serial.println(lastCommand);
    Serial.println("=========================");
}
