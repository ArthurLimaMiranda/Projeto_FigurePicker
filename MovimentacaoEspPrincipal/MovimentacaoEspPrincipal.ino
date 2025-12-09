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

// Estruturas para processamento de coordenadas
struct Coordinate {
    float x;
    float y;
};

const int MAX_COORDS = 20;
Coordinate circles[MAX_COORDS];
Coordinate squares[MAX_COORDS];
int circleCount = 0;
int squareCount = 0;

// Estado da sequência automática
enum SequenceState {
    IDLE,
    MOVING_TO_X,
    WAITING_ESP2_PICKUP,
    MOVING_DROP_X_AFTER_PICKUP,
    MOVING_Y_TO_DROP,
    WAITING_DROP,
    RESETTING_AXES,
    SEQUENCE_COMPLETE
};

SequenceState currentState = IDLE;
int currentIndex = 0;
bool processingCircles = true;
unsigned long stateStartTime = 0;
bool sequenceActive = false;

// Conversões
const float PULSES_PER_CM_X = 17.40;
const float PULSES_PER_CM_Y = 18.5;
const long X_BOARD_OFFSET = 110;
const unsigned long ESP2_Y_ARRIVE_TIME = 3000;
const unsigned long ESP2_PICKUP_DOWN_TIME = 5000;
const unsigned long ESP2_PICKUP_TIME = 7000;
const unsigned long ESP2_CLAW_TIME = 2000;
int currentPickUpPoint= 0;

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
void parseCoordinates(String input);
void startSequence();
void processSequence();
long cmToPulsesX(float cm);
long cmToPulsesY(float cm);

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

  if (sequenceActive) {
        processSequence();
    }

    // Verifica movimento para posição X
    if (movingToTargetX && !sequenceActive) {
        if (abs(encoderA.contador - targetPositionX) <= 3) {
            stopMotorsX();
            movingToTargetX = false;
            Serial.print("EIXO X chegou na posição: ");
            Serial.println(targetPositionX);
        }
    }

    // Verifica reset X
    if (resettingX && !sequenceActive) {
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

// Converte cm para pulsos X
long cmToPulsesX(float cm) {
    return (long)(cm * PULSES_PER_CM_X);
}

// Converte cm para pulsos Y
long cmToPulsesY(float cm) {
    return (long)(cm * PULSES_PER_CM_Y);
}

// Processa a sequência automática
void processSequence() {
    unsigned long currentTime = millis();
    
    switch (currentState) {
        case IDLE:
            break;
            
        case MOVING_TO_X: {
            if (abs(encoderA.contador - targetPositionX) <= 3) {
                stopMotorsX();
                Serial.println("Chegou na posição X. Enviando coordenada Y para ESP2");
                Coordinate* currentCoords = processingCircles ? circles : squares;
                long yPulses = cmToPulsesY(currentCoords[currentIndex].y);
                
                sendCommandToSecondary("GOTO_Y", 0, yPulses);                
                currentState = WAITING_ESP2_PICKUP;
                currentPickUpPoint = 0;
                stateStartTime = currentTime;
                Serial.println("Aguardando ESP2 realizar pickup");
            }
            break;
        }
        
        case WAITING_ESP2_PICKUP:
            if ((currentTime - stateStartTime >= ESP2_Y_ARRIVE_TIME) && currentPickUpPoint == 0) {
                Serial.println("Abrindo a garra");
                sendCommandToSecondary("GRIP", 130);
                currentPickUpPoint++;
                stateStartTime = currentTime;
            }

            if ((currentTime - stateStartTime >= ESP2_CLAW_TIME) && currentPickUpPoint == 1) {
                Serial.println("Descendo a garra");
                sendCommandToSecondary("MOVE_Z", -1);
                currentPickUpPoint++;
                stateStartTime = currentTime;
            }

            if ((currentTime - stateStartTime >= ESP2_PICKUP_DOWN_TIME) && currentPickUpPoint == 2) {
                Serial.println("Fechando a garra");
                sendCommandToSecondary("GRIP", 88);
                currentPickUpPoint++;
                stateStartTime = currentTime;
            }

            if ((currentTime - stateStartTime >= ESP2_CLAW_TIME) && currentPickUpPoint == 3) {
                Serial.println("Subindo a garra");
                sendCommandToSecondary("MOVE_Z", 1);
                currentPickUpPoint++;
                stateStartTime = currentTime;
            }
            
            if ((currentTime - stateStartTime >= ESP2_PICKUP_TIME) && currentPickUpPoint == 4) {
                Serial.println("Pickup completo. Indo para a area de drop");
                moveFront();
                currentState = MOVING_DROP_X_AFTER_PICKUP;
            }
            break;
            
        case MOVING_DROP_X_AFTER_PICKUP:
            if (digitalRead(front) == LOW && digitalRead(front_2) == LOW) {
                stopMotorsX();
                noInterrupts();
                encoderA.contador = 0;
                interrupts();
                Serial.println("X na área de drop. Movendo Y para posição de drop...");

                if (processingCircles) {
                    sendCommandToSecondary("MOVE_Y", 1);
                    Serial.println("Movendo para DROP (círculo)");
                } else {
                    sendCommandToSecondary("MOVE_Y", -1);
                    Serial.println("Movendo para DROP (quadrado)");
                }
                
                currentState = MOVING_Y_TO_DROP;
                stateStartTime = currentTime;
            }
            break;
            
        case MOVING_Y_TO_DROP:
            if (currentTime - stateStartTime >= ESP2_Y_ARRIVE_TIME) {
                Serial.println("Y chegou na posição. Abrindo garra...");
                sendCommandToSecondary("GRIP", 110, 0);
                currentState = WAITING_DROP;
                stateStartTime = currentTime;
            }
            break;
            
        case WAITING_DROP:
            if (currentTime - stateStartTime >= ESP2_CLAW_TIME) {
                Serial.println("Drop completo. Fechando garra e resetando eixos...");
                sendCommandToSecondary("GRIP", 88, 0);
                delay(500);
                sendCommandToSecondary("RESET_Y", 0, 0);
                resetX();
                currentState = RESETTING_AXES;
                stateStartTime = currentTime;
            }
            break;
            
        case RESETTING_AXES:
            if (digitalRead(back) == LOW && digitalRead(back_2) == LOW) {
                stopMotorsX();
                resetAllEncoders();
                sendCommandToSecondary("RESET_ENC", 0);
                
                currentIndex++;

                Coordinate* currentCoords = processingCircles ? circles : squares;
                int currentCount = processingCircles ? circleCount : squareCount;
                
                if (currentIndex < currentCount) {
                    Serial.print("Próxima coordenada: ");
                    Serial.print(currentIndex + 1);
                    Serial.print("/");
                    Serial.println(currentCount);
                    
                    long xTarget = cmToPulsesX(currentCoords[currentIndex].x) + X_BOARD_OFFSET;
                    moveToPositionX(xTarget);
                    currentState = MOVING_TO_X;
                } else if (processingCircles && squareCount > 0) {
                    
                    Serial.println("=== CÍRCULOS COMPLETOS. INICIANDO QUADRADOS ===");
                    processingCircles = false;
                    currentIndex = 0;
                    
                    long xTarget = cmToPulsesX(squares[0].x) + X_BOARD_OFFSET;
                    moveToPositionX(xTarget);
                    currentState = MOVING_TO_X;
                } else {
                    currentState = SEQUENCE_COMPLETE;
                    Serial.println("=== SEQUÊNCIA COMPLETA ===");
                    sequenceActive = false;
                }
            }
            break;
            
        case SEQUENCE_COMPLETE:
            sequenceActive = false;
            currentState = IDLE;
            break;
    }
}

// Inicia a sequência automática
void startSequence() {
    if (circleCount == 0 && squareCount == 0) {
        Serial.println("ERRO: Nenhuma coordenada cadastrada!");
        return;
    }
    
    Serial.println("=== INICIANDO SEQUÊNCIA AUTOMÁTICA ===");
    Serial.print("Círculos: ");
    Serial.println(circleCount);
    Serial.print("Quadrados: ");
    Serial.println(squareCount);
    
    Serial.println("Resetando eixos...");
    resetX();
    sendCommandToSecondary("RESET_Y", 0, 0);
    delay(5000);
    
    resetAllEncoders();
    sendCommandToSecondary("RESET_ENC", 0);
    
    processingCircles = (circleCount > 0);
    currentIndex = 0;
    sequenceActive = true;
    
    Coordinate* firstCoords = processingCircles ? circles : squares;
    long xTarget = cmToPulsesX(firstCoords[0].x) + X_BOARD_OFFSET;
    
    Serial.print("Movendo para primeira posição X: ");
    Serial.print(firstCoords[0].x);
    Serial.println(" cm");
    
    moveToPositionX(xTarget);
    currentState = MOVING_TO_X;
}

// Parse coordenadas do formato c[[x1,y1],[x2,y2],...] ou q[[x1,y1],[x2,y2],...]
void parseCoordinates(String input) {
    input.trim();
    
    bool isCircle = (input.charAt(0) == 'c' || input.charAt(0) == 'C');
    bool isSquare = (input.charAt(0) == 'q' || input.charAt(0) == 'Q');
    
    if (!isCircle && !isSquare) {
        Serial.println("ERRO: Formato inválido. Use c[...] ou q[...]");
        return;
    }
    
    input = input.substring(1);
    input.trim();
    
    int count = 0;
    Coordinate* coords = isCircle ? circles : squares;
    
    if (isCircle) {
        circleCount = 0;
    } else {
        squareCount = 0;
    }
    
    int startIdx = 0;
    int depth = 0;
    String currentNumber = "";
    float currentX = 0, currentY = 0;
    bool readingX = true;
    
    for (unsigned int i = 0; i < input.length() && count < MAX_COORDS; i++) {
        char c = input.charAt(i);
        
        switch (c) {
            case '[':
                depth++;
                if (depth == 2) {
                    currentNumber = "";
                    currentX = 0;
                    currentY = 0;
                    readingX = true;
                }
                break;
                
            case ']':
                if (depth == 2) {
                    
                    if (currentNumber.length() > 0) {
                        currentY = currentNumber.toFloat();
                        currentNumber = "";
                    }
                    
                    coords[count].x = currentX;
                    coords[count].y = currentY;
                    count++;
                }
                depth--;
                break;
                
            case ',':
                if (depth == 2) {
                    if (readingX) {
                        currentX = currentNumber.toFloat();
                        currentNumber = "";
                        readingX = false;
                    }
                }
                break;
                
            case ' ':
            case '\t':
            
                break;
                
            default:
                if (depth == 2 && (isdigit(c) || c == '.' || c == '-')) {
                    currentNumber += c;
                }
                break;
        }
    }
    
    if (isCircle) {
        circleCount = count;
        Serial.print("Círculos cadastrados: ");
        Serial.println(circleCount);
        for (int i = 0; i < circleCount; i++) {
            Serial.print("  [");
            Serial.print(circles[i].x);
            Serial.print(", ");
            Serial.print(circles[i].y);
            Serial.println("]");
        }
    } else {
        squareCount = count;
        Serial.print("Quadrados cadastrados: ");
        Serial.println(squareCount);
        for (int i = 0; i < squareCount; i++) {
            Serial.print("  [");
            Serial.print(squares[i].x);
            Serial.print(", ");
            Serial.print(squares[i].y);
            Serial.println("]");
        }
    }
}

// Verifica se pode enviar comando
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
    if (sequenceActive) return;

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

    if(stopPressed){
        if (!buttonWasPressed || currentMovement != "STOP") {
            printMovement("PARANDO TODOS OS MOTORES");
            stopAllMotors();
            movingToTargetX = false;
            resettingX = false;
            sequenceActive = false;
            lastCommand = "STOP";
            currentMovement = "STOP";
            buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
    else if(frontPressed){
        if (!buttonWasPressed || currentMovement != "FRONT") {
          printEncoderStatus();
          moveFront();
          printMovement("MOVENDO EIXO X: FRENTE");
          movingToTargetX = false;
          lastCommand = "FRONT";
          currentMovement = "FRONT";
          buttonWasPressed = true;
        }
        lastButtonTime = millis();
    }
    else if(backPressed){
        if (!buttonWasPressed || currentMovement != "BACK") {
          printEncoderStatus();
          moveBack();
          printMovement("MOVENDO EIXO X: BACK");
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
            
            if (cmd == 'c' || cmd == 'C' || cmd == 'q' || cmd == 'Q') {
                if (command.indexOf('[') != -1) {
                    parseCoordinates(command);
                    return;
                }
            }

            switch(cmd) {
                case 't': case 'T':
                    Serial.println("Comando enviado: EIXO X FRENTE");
                    moveFront();
                    break;

                case 'y': case 'Y':
                    Serial.println("Comando enviado: EIXO X TRÁS");
                    moveBack();
                    break;

                case 'w': case 'W':
                    sendCommandToSecondary("MOVE_Z", 1);
                    Serial.println("Comando enviado: EIXO Z CIMA");
                    break;

                case 's': case 'S':
                    if (command.length() > 1 && command.charAt(1) == 't') {
                        startSequence();
                    } else {
                        sendCommandToSecondary("MOVE_Z", -1);
                        Serial.println("Comando enviado: EIXO Z BAIXO");
                    }
                    break;

                case 'a': case 'A':
                    sendCommandToSecondary("MOVE_Y", 1);
                    Serial.println("Comando enviado: EIXO Y ESQUERDA");
                    break;

                case 'd': case 'D':
                    sendCommandToSecondary("MOVE_Y", -1);
                    Serial.println("Comando enviado: EIXO Y DIREITA");
                    break;

                case 'q': case 'Q':
                    sendCommandToSecondary("GRIP", 130);
                    Serial.println("Comando enviado: GARRA ABRIR");
                    break;

                case 'e': case 'E':
                    sendCommandToSecondary("GRIP", 88);
                    Serial.println("Comando enviado: GARRA FECHAR");
                    break;

                case 'x': case 'X':
                    stopAllMotors();
                    sendCommandToSecondary("STOP_ALL", 0);
                    movingToTargetX = false;
                    resettingX = false;
                    sequenceActive = false;
                    Serial.println("PARANDO TODOS OS MOTORES");
                    lastCommand = "STOP";
                    break;

                case 'p': case 'P':
                    printEncoderStatus();
                    sendCommandToSecondary("STATUS", 0);
                    break;

                case 'z': case 'Z':
                    resetAllEncoders();
                    sendCommandToSecondary("RESET_ENC", 0);
                    break;

                case 'r': case 'R':
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

                case 'g': case 'G':
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

                case 'h': case 'H':
                    printHelp();
                    break;

                case 'i': case 'I':
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
    Serial.println("SEQUÊNCIA AUTOMÁTICA:");
    Serial.println("  c[[x1,y1],[x2,y2],...] - Cadastrar círculos");
    Serial.println("  q[[x1,y1],[x2,y2],...] - Cadastrar quadrados");
    Serial.println("  ST                     - Iniciar sequência");
    Serial.println();
    Serial.println("CONTROLE EIXO X (Local):");
    Serial.println("  T/Y        - Frente/Trás");
    Serial.println("  GX<num>    - Move EIXO X para posição");
    Serial.println("  RX         - Reset EIXO X");
    Serial.println();
    Serial.println("CONTROLE REMOTO (ESP Secundária):");
    Serial.println("  EIXO Y: A/D, GY<num>, RY");
    Serial.println("  EIXO Z: W/S, GZ<num>, RZ");
    Serial.println("  GARRA: Q (abrir), E (fechar)");
    Serial.println();
    Serial.println("COMANDOS GERAIS:");
    Serial.println("  X - PARAR | P - Status | Z - Reset encoders");
    Serial.println("  H - Ajuda | I - Info sistema");
    Serial.println("================================");
}

void printStatus() {
    Serial.println();
    Serial.println("=== STATUS DO SISTEMA ===");
    Serial.print("Posição EIXO X: "); Serial.println(encoderA.contador);
    Serial.print("Sequência ativa: "); Serial.println(sequenceActive ? "SIM" : "NÃO");
    Serial.print("Círculos cadastrados: "); Serial.println(circleCount);
    Serial.print("Quadrados cadastrados: "); Serial.println(squareCount);
    Serial.print("Último comando: "); Serial.println(lastCommand);
    Serial.println("=========================");
}