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

// Variável para controle de delay entre comandos
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_DELAY = 10000;
bool waitingForDelay = false;

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
bool canSendCommand();

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
    waitingForDelay = false;
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

// Verifica se pode enviar comando (delay de 10 segundos)
bool canSendCommand() {
    if (waitingForDelay) {
        unsigned long currentTime = millis();
        if (currentTime - lastCommandTime < COMMAND_DELAY) {
            Serial.print("Aguarde ");
            Serial.print((COMMAND_DELAY - (currentTime - lastCommandTime)) / 1000);
            Serial.println(" segundos antes do próximo comando");
            return false;
        }
        waitingForDelay = false;
    }
    return true;
}

// Comunicação com a ESP Secundária
void sendCommandToSecondary(String cmd, long value, long position) {
    if (!canSendCommand()) {
        return;
    }
    
    memset(&myData, 0, sizeof(myData));
    strncpy(myData.command, cmd.c_str(), sizeof(myData.command)-1);
    myData.value = value;
    myData.position = position;
    
    esp_err_t ret = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if(ret != ESP_OK){
        Serial.print("esp_now_send falhou: ");
        Serial.println(ret);
        waitingForDelay = false;
    } else {
        lastCommandTime = millis();
        waitingForDelay = true;
        Serial.println("Comando enviado - Aguardando 10 segundos para próximo");
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
                case 'w': case 'W': // Eixo Z - Cima
                    sendCommandToSecondary("MOVE_Z", 1);
                    Serial.println("Comando enviado: EIXO Z CIMA");
                    break;

                case 's': case 'S': // Eixo Z - Baixo
                    sendCommandToSecondary("MOVE_Z", -1);
                    Serial.println("Comando enviado: EIXO Z BAIXO");
                    break;

                case 'a': case 'A': // Eixo Y - Esquerda
                    sendCommandToSecondary("MOVE_Y", 1);
                    Serial.println("Comando enviado: EIXO Y ESQUERDA");
                    break;

                case 'd': case 'D': // Eixo Y - Direita
                    sendCommandToSecondary("MOVE_Y", -1);
                    Serial.println("Comando enviado: EIXO Y DIREITA");
                    break;

                case 'q': case 'Q': // Garra - Abrir
                    sendCommandToSecondary("GRIP", 80); // 80 = aberto
                    Serial.println("Comando enviado: GARRA ABRIR");
                    break;

                case 'e': case 'E': // Garra - Fechar
                    sendCommandToSecondary("GRIP", 10); // 10 = fechado
                    Serial.println("Comando enviado: GARRA FECHAR");
                    break;

                case 'x': case 'X': // Parar tudo
                    stopAllMotors();
                    sendCommandToSecondary("STOP_ALL", 0);
                    movingToTargetX = false;
                    resettingX = false;
                    Serial.println("PARANDO TODOS OS MOTORES");
                    lastCommand = "STOP";
                    break;

                case 'p': case 'P': // Status
                    printEncoderStatus();
                    sendCommandToSecondary("STATUS", 0);
                    break;

                case 'z': case 'Z': // Reset encoders
                    resetAllEncoders();
                    sendCommandToSecondary("RESET_ENC", 0);
                    break;

                case 'r': case 'R': // Reset eixos
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        if (axis == 'x' || axis == 'X') {
                            resetX();
                        } else if (axis == 'y' || axis == 'Y') {
                            Serial.println("Enviando: RESET EIXO Y");
                            sendCommandToSecondary("RESET_Y", 0);
                        } else if (axis == 'z' || axis == 'Z') {
                            Serial.println("Enviando: RESET EIXO Z");
                            sendCommandToSecondary("RESET_Z", 0);
                        }
                    }
                    break;

                case 'g': case 'G': // Ir para posição
                    if (command.length() > 1) {
                        char axis = command.charAt(1);
                        long position = command.substring(2).toInt();
                        if (axis == 'x' || axis == 'X') {
                            moveToPositionX(position);
                        } else if (axis == 'y' || axis == 'Y') {
                            sendCommandToSecondary("GOTO_Y", 0, position);
                            Serial.print("Enviando: EIXO Y para posição ");
                            Serial.println(position);
                        } else if (axis == 'z' || axis == 'Z') {
                            sendCommandToSecondary("GOTO_Z", 0, position);
                            Serial.print("Enviando: EIXO Z para posição ");
                            Serial.println(position);
                        }
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
    Serial.println("=== COMANDOS SERIAL (ESP PRINCIPAL) ===");
    Serial.println("CONTROLE EIXO X (Local):");
    Serial.println("  Frente/Trás: Botões físicos");
    Serial.println("  GX<num>     - Move EIXO X para posição");
    Serial.println("  RX          - Reset EIXO X");
    Serial.println();
    Serial.println("CONTROLE REMOTO (ESP Secundária):");
    Serial.println("  EIXO Y:");
    Serial.println("    A         - ESQUERDA");
    Serial.println("    D         - DIREITA"); 
    Serial.println("    GY<num>   - Move para posição Y");
    Serial.println("    RY        - Reset EIXO Y");
    Serial.println("  EIXO Z:");
    Serial.println("    W         - CIMA");
    Serial.println("    S         - BAIXO");
    Serial.println("    GZ<num>   - Move para posição Z");
    Serial.println("    RZ        - Reset EIXO Z");
    Serial.println("  GARRA:");
    Serial.println("    Q         - ABRIR");
    Serial.println("    E         - FECHAR");
    Serial.println();
    Serial.println("COMANDOS GERAIS:");
    Serial.println("  X          - PARAR todos os motores");
    Serial.println("  P          - Status dos encoders");
    Serial.println("  Z          - Resetar encoders");
    Serial.println("  I          - Informações do sistema");
    Serial.println("  H          - Mostrar esta ajuda");
    Serial.println();
    Serial.println("NOTA: Comandos remotos têm delay de 10 segundos");
    Serial.println("================================");
}

void printStatus() {
    Serial.println();
    Serial.println("=== STATUS DO SISTEMA ===");
    Serial.print("Posição EIXO X: "); Serial.println(encoderA.contador);
    Serial.print("Movendo para alvo X: "); Serial.println(movingToTargetX ? "SIM" : "NÃO");
    Serial.print("Resetando X: "); Serial.println(resettingX ? "SIM" : "NÃO");
    Serial.print("Último comando: "); Serial.println(lastCommand);
    Serial.print("Próximo comando em: ");
    if (waitingForDelay) {
        unsigned long remaining = COMMAND_DELAY - (millis() - lastCommandTime);
        Serial.print(remaining / 1000);
        Serial.println(" segundos");
    } else {
        Serial.println("PRONTO");
    }
    Serial.println("=========================");
}