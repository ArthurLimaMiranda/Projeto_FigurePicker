
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

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

// Estruturas para encoders
struct EncoderData {
  volatile long contador;
  volatile int estadoAnterior;
  volatile boolean contadorAtualizado;
  volatile char direcao;
  volatile unsigned long ultimoTempoInterrupcao;
  int pino;
  int* motorDirection;
};

struct Encoder2Data {
  volatile long contador;
  volatile int estadoAnteriorA;
  volatile int estadoAnteriorB;
  volatile boolean contadorAtualizado;
  volatile char direcao;
  volatile unsigned long ultimoTempoInterrupcao;
  int pinoA;
  int pinoB;
  int* motorDirection;
};

// Definições dos pinos de direção - (EIXO Y - ESQUERDA/DIREITA)
#define IN5 33   // Motor C - Sentido 1 (Y)
#define IN6 32   // Motor C - Sentido 2 (Y)

// Definições dos pinos de direção - (EIXO Z - CIMA/BAIXO)
#define IN7 14   // Motor D - Sentido 1 (Z)
#define IN8 13   // Motor D - Sentido 2 (Z)

// Definições dos botões
#define right 25  // Botão direito eixo Y
#define left 26   // Botão esquerdo eixo Y
#define up 12     // Botão cima eixo Z

// Encoders
#define ENCODER_C 27  // Encoder Motor C - Eixo Y (1 canal)
#define ENCODER_D_A 16  // Encoder Motor D - Eixo Z - Canal A
#define ENCODER_D_B 17  // Encoder Motor D - Eixo Z - Canal B

// Garra
#define SERVO_PIN 4

// Variáveis de controle
// Eixo Y
long targetPositionY = 0;
bool movingToTargetY = false;
bool resettingY = false;
int motorDirectionC = 0;

// Eixo Z
long targetPositionZ = 0;
bool movingToTargetZ = false;
bool resettingZ = false;
int motorDirectionD = 0;
const long Z_GROUND_POSITION = 300;

// Garra
Servo gripperServo;
bool gripperOpen = false;

String lastCommand = "STOP";
String lastPrintedCommand = "";

// Encoders instanciados
EncoderData encoderC = {0, 0, false, ' ', 0, ENCODER_C, &motorDirectionC};
Encoder2Data encoderD = {0, 0, 0, false, ' ', 0, ENCODER_D_A, ENCODER_D_B, &motorDirectionD};

const unsigned long DEBOUNCE_TIME = 1;
unsigned long lastButtonTime = 0;
const unsigned long BUTTON_DEBOUNCE = 10;

bool buttonWasPressed = false;
String currentMovement = "";

int gripperTarget = -1;
int gripperCurrent = 80;
unsigned long lastGripperStepTime = 0;
const int GRIP_STEP = 3;
const unsigned long GRIP_STEP_INTERVAL = 50;
unsigned long gripStartTime = 0;
const unsigned long GRIP_TIMEOUT = 1200;
bool gripperMoving = false;

// Flags para enviar ACK fora do callback
bool sendAckPending = false;
struct_message ackData;

// Prototypes
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

void IRAM_ATTR handleEncoderC();
void IRAM_ATTR handleEncoderD_A();
void IRAM_ATTR handleEncoderD_B();
void updateEncoder(EncoderData &encoder);
void updateEncoder2(Encoder2Data &encoder);
void moveLeft();
void moveRight();
void moveUp();
void moveDown();
void stopMotorsY();
void stopMotorsZ();
void moveToPositionY(long position);
void moveToPositionZ(long position);
void resetY();
void resetZ();
void resetAllEncoders();
void printEncoderStatus();
void stopAllMotors();
void handleButtons();
void updateMotorDirections(int dirC, int dirD);
void controlGripper(int state);
void handleSerialCommands();
void requestGripperMove(int angle);
void processGripperMovementLoop();

// CALLBACKS ESP-NOW
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("OnDataSent - status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len < (int)sizeof(receivedData)) {
        return;
    }
    
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    receivedData.command[sizeof(receivedData.command)-1] = '\0';

    if (strcmp(receivedData.command, "MOVE_Y") == 0) {
        if (receivedData.value == 1) {
            moveLeft();
            movingToTargetY = false;
            resettingY = false;
        } else if (receivedData.value == -1) {
            moveRight();
            movingToTargetY = false;
            resettingY = false;
        }
    }
    else if (strcmp(receivedData.command, "MOVE_Z") == 0) {
        if (receivedData.value == 1) {
            moveUp();
            movingToTargetZ = false;
            resettingZ = false;
        } else if (receivedData.value == -1) {
            moveDown();
            movingToTargetZ = false;
            resettingZ = false;
        }
    }
    else if (strcmp(receivedData.command, "STOP_ALL") == 0) {
        stopAllMotors();
        movingToTargetY = false;
        movingToTargetZ = false;
        resettingY = false;
        resettingZ = false;
    }
    else if (strcmp(receivedData.command, "GOTO_Y") == 0) {
        moveToPositionY(receivedData.position);
    }
    else if (strcmp(receivedData.command, "GOTO_Z") == 0) {
        moveToPositionZ(receivedData.position);
    }
    else if (strcmp(receivedData.command, "RESET_Y") == 0) {
        resetY();
    }
    else if (strcmp(receivedData.command, "RESET_Z") == 0) {
        resetZ();
    }
    else if (strcmp(receivedData.command, "RESET_ENC") == 0) {
        resetAllEncoders();
        memset(&ackData, 0, sizeof(ackData));
        strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
        ackData.value = 0;
        sendAckPending = true;
    }
    else if (strcmp(receivedData.command, "STATUS") == 0) {
        memset(&ackData, 0, sizeof(ackData));
        strncpy(ackData.command, "STATUS_REQ", sizeof(ackData.command)-1);
        ackData.value = 0;
        sendAckPending = true;
    }
    else if (strcmp(receivedData.command, "GRIP") == 0) {
        requestGripperMove((int)receivedData.value);
    }
}

// ==================== SETUP / LOOP ====================
void setup() {
    Serial.begin(115200);
    delay(100);

    // Configuração de pinos
    pinMode(left, INPUT_PULLUP);
    pinMode(right, INPUT_PULLUP);
    pinMode(up, INPUT_PULLUP);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT);
    pinMode(IN8, OUTPUT);
    pinMode(ENCODER_C, INPUT_PULLUP);
    pinMode(ENCODER_D_A, INPUT_PULLUP);
    pinMode(ENCODER_D_B, INPUT_PULLUP);

    // Configuração dos encoders
    encoderC.estadoAnterior = digitalRead(ENCODER_C);
    encoderD.estadoAnteriorA = digitalRead(ENCODER_D_A);
    encoderD.estadoAnteriorB = digitalRead(ENCODER_D_B);

    // Configuração das interrupções
    attachInterrupt(digitalPinToInterrupt(ENCODER_C), handleEncoderC, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_D_A), handleEncoderD_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_D_B), handleEncoderD_B, CHANGE);

    // Configuração do servo da garra
    gripperServo.attach(SERVO_PIN);
    controlGripper(80);

    // Configuração WiFi e ESP-NOW
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }

    // Registrar callbacks (com novas assinaturas)
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Registrar peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Falha ao adicionar peer");
    }

    stopAllMotors();

    Serial.println("=== ESP SECUNDÁRIA - EIXOS Y e Z + GARRA ===");
    Serial.print("Endereço MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("Pronta para receber comandos...");
}

void loop() {
    
    handleSerialCommands();
    handleButtons();

    // Verifica movimento para posição Y
    if (movingToTargetY) {
        if (abs(encoderC.contador - targetPositionY) <= 3) {
            stopMotorsY();
            movingToTargetY = false;
            Serial.print("EIXO Y chegou na posição: ");
            Serial.println(targetPositionY);
        }
    }

    // Verifica movimento para posição Z
    if (movingToTargetZ) {
        if (abs(encoderD.contador - targetPositionZ) <= 3) {
            stopMotorsZ();
            movingToTargetZ = false;
            Serial.print("EIXO Z chegou na posição: ");
            Serial.println(targetPositionZ);
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
            // Confirma reset por ACK
            memset(&ackData, 0, sizeof(ackData));
            strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
            ackData.value = 0;
            sendAckPending = true;
        }
    }

    // Verifica reset Z
    if (resettingZ) {
        if (digitalRead(up) == LOW) {
            stopMotorsZ();
            resettingZ = false;
            noInterrupts();
            encoderD.contador = 0;
            interrupts();
            Serial.println("EIXO Z resetado - Posição definida como 0 (topo)");
            // Confirma reset por ACK
            memset(&ackData, 0, sizeof(ackData));
            strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
            ackData.value = 0;
            sendAckPending = true;
        }
    }

    // Verifica descida do eixo Z
    if (motorDirectionD == -1 && encoderD.contador >= Z_GROUND_POSITION) {
        stopMotorsZ();
        Serial.println("EIXO Z: Chegou perto do chão - Parando");yy
        memset(&ackData, 0, sizeof(ackData));
        strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
        ackData.value = encoderD.contador;
        sendAckPending = true;
    }

    processGripperMovementLoop();

    static unsigned long lastAckAttempt = 0;
    static int ackRetries = 0;
    const unsigned long ACK_RETRY_INTERVAL = 400;
    const int ACK_MAX_RETRIES = 3;

    if (sendAckPending) {
        unsigned long now = millis();
        if (now - lastAckAttempt >= ACK_RETRY_INTERVAL) {
            lastAckAttempt = now;
            esp_err_t r = esp_now_send(broadcastAddress, (uint8_t *)&ackData, sizeof(ackData));
            if (r == ESP_OK) {
                sendAckPending = false;
                ackRetries = 0;
                Serial.print("ACK enviado: ");
                Serial.print(ackData.command);
                Serial.print(" - ");
                Serial.println(ackData.value);
            } else {
                ackRetries++;
                Serial.print("Falha ao enviar ACK (tentativa ");
                Serial.print(ackRetries);
                Serial.print(") - erro: ");
                Serial.println(r);
                if (ackRetries >= ACK_MAX_RETRIES) {
                    sendAckPending = false;
                    ackRetries = 0;
                    Serial.println("Desistindo de tentar enviar ACK depois de várias falhas.");
                }
            }
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
        Serial.print("Movendo EIXO Y para ESQUERDA - Alvo: ");
        Serial.println(position);
    } else if (currentPosition > position) {
        moveRight();
        movingToTargetY = true;
        Serial.print("Movendo EIXO Y para DIREITA - Alvo: ");
        Serial.println(position);
    } else {
        stopMotorsY();
        Serial.println("EIXO Y já está na posição");
    }
}

// Função para reset do eixo Y
void resetY() {
    Serial.println("Iniciando reset EIXO Y - Movendo para DIREITA até fim de curso");
    resettingY = true;
    moveRight();
}

// Função para mover eixo Z para posição específica
void moveToPositionZ(long position) {
    targetPositionZ = position;
    long currentPosition = encoderD.contador;

    if (currentPosition < position) {
        moveUp();
        movingToTargetZ = true;
        Serial.print("Movendo EIXO Z para CIMA - Alvo: ");
        Serial.println(position);
    } else if (currentPosition > position) {
        moveDown();
        movingToTargetZ = true;
        Serial.print("Movendo EIXO Z para BAIXO - Alvo: ");
        Serial.println(position);
    } else {
        stopMotorsZ();
        Serial.println("EIXO Z já está na posição");
    }
}

// Função para reset do eixo Z
void resetZ() {
    Serial.println("Iniciando reset EIXO Z - Movendo para CIMA até fim de curso");
    resettingZ = true;
    moveUp();
}

void requestGripperMove(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    gripperTarget = angle;
    gripperMoving = true;
    gripStartTime = millis();
    lastGripperStepTime = 0; // força passo imediato no loop
}

// Função que realiza os passos da garra
void processGripperMovementLoop() {
    if (!gripperMoving) return;

    unsigned long now = millis();

    if (now - gripStartTime > GRIP_TIMEOUT) {
        gripperMoving = false;
        memset(&ackData, 0, sizeof(ackData));
        strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
        ackData.value = -1;
        sendAckPending = true;
        return;
    }

    if (now - lastGripperStepTime < GRIP_STEP_INTERVAL) return;
    lastGripperStepTime = now;

    if (gripperCurrent < gripperTarget) {
        int next = gripperCurrent + GRIP_STEP;
        if (next > gripperTarget) next = gripperTarget;
        gripperCurrent = next;
        gripperServo.write(gripperCurrent);
    } else if (gripperCurrent > gripperTarget) {
        int next = gripperCurrent - GRIP_STEP;
        if (next < gripperTarget) next = gripperTarget;
        gripperCurrent = next;
        gripperServo.write(gripperCurrent);
    }

    if (gripperCurrent == gripperTarget) {
        gripperMoving = false;
        memset(&ackData, 0, sizeof(ackData));
        strncpy(ackData.command, "CONFIRM", sizeof(ackData.command)-1);
        ackData.value = gripperCurrent;
        sendAckPending = true;
    }
}

// Função para controle da garra
void controlGripper(int state) {
    gripperServo.write(state);
}

// Função para controle dos botões
void handleButtons() {
    if (millis() - lastButtonTime < BUTTON_DEBOUNCE) return;

    bool anyButtonPressed = (digitalRead(left) == LOW) ||
                       (digitalRead(right) == LOW) ||
                       (digitalRead(up) == LOW);


    if (!anyButtonPressed) {
        buttonWasPressed = false;
        currentMovement = "";
        lastPrintedCommand = "";
        return;
    }

    if(digitalRead(left) == LOW){
      if (!buttonWasPressed || currentMovement != "ESQUERDA") {
        printEncoderStatus();
        movingToTargetY = false;
        resettingY = false;
        lastCommand = "ESQUERDA";
        currentMovement = "ESQUERDA";
        Serial.println("EIXO Y: ESQUERDA (botão local)");
        buttonWasPressed = true;
      }
      lastButtonTime = millis();
    }
    else if(digitalRead(right) == LOW){
      if (!buttonWasPressed || currentMovement != "DIREITA") {
        printEncoderStatus();
        movingToTargetY = false;
        resettingY = false;
        lastCommand = "DIREITA";
        currentMovement = "DIREITA";
        Serial.println("EIXO Y: DIREITA (botão local)");
        buttonWasPressed = true;
      }
      lastButtonTime = millis();
    }
    else if(digitalRead(up) == LOW){
      if (!buttonWasPressed || currentMovement != "CIMA") {
        printEncoderStatus();
        movingToTargetZ = false;
        resettingZ = false;
        lastCommand = "CIMA";
        currentMovement = "CIMA";
        Serial.println("EIXO Z: CIMA (botão local)");
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

                case 'w': case 'W':
                    movingToTargetZ = false;
                    resettingZ = false;
                    moveUp();
                    Serial.println("Movendo EIXO Z: CIMA");
                    lastCommand = "CIMA";
                    break;

                case 's': case 'S':
                    movingToTargetZ = false;
                    resettingZ = false;
                    moveDown();
                    Serial.println("Movendo EIXO Z: DOWN");
                    lastCommand = "BAIXO";
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

                case 'h': case 'H':
                    if (command.length() > 1) {
                      long position = command.substring(1).toInt();
                      Serial.print("Enviando: GARRA para posição ");
                      controlGripper(position);
                        
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
            }
        }
    }
}

// Funções de interrupção para encoder
void IRAM_ATTR handleEncoderC() {
    updateEncoder(encoderC);
}
void IRAM_ATTR handleEncoderD_A() {
    updateEncoder2(encoderD);
}
void IRAM_ATTR handleEncoderD_B() {
    updateEncoder2(encoderD);
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
void updateEncoder2(Encoder2Data &encoder) {
    unsigned long tempoAtual = micros();
    if (tempoAtual - encoder.ultimoTempoInterrupcao < DEBOUNCE_TIME) return;
    encoder.ultimoTempoInterrupcao = tempoAtual;

    int estadoAtualA = digitalRead(encoder.pinoA);
    int estadoAtualB = digitalRead(encoder.pinoB);

    if (encoder.estadoAnteriorA != estadoAtualA || encoder.estadoAnteriorB != estadoAtualB) {
        if (estadoAtualA == estadoAtualB) {
            if (*(encoder.motorDirection) > 0) {
                encoder.contador++;
                encoder.direcao = 'U';
            } else if (*(encoder.motorDirection) < 0) {
                encoder.contador--;
                encoder.direcao = 'D';
            }
        } else {
            if (*(encoder.motorDirection) > 0) {
                encoder.contador--;
                encoder.direcao = 'D';
            } else if (*(encoder.motorDirection) < 0) {
                encoder.contador++;
                encoder.direcao = 'U';
            }
        }

        encoder.contadorAtualizado = true;
    }

    encoder.estadoAnteriorA = estadoAtualA;
    encoder.estadoAnteriorB = estadoAtualB;
}

void printEncoderStatus() {
    Serial.println("=== STATUS ENCODERS ===");
    Serial.print("Motor C (Y): ");
    Serial.print(encoderC.contador);
    Serial.print(" pulses - ");
    Serial.println(encoderC.direcao == 'E' ? "ESQUERDA" : (encoderC.direcao == 'D' ? "DIREITA" : "PARADO"));

    Serial.print("Motor D (Z): ");
    Serial.print(encoderD.contador);
    Serial.print(" pulses - ");
    Serial.println(encoderD.direcao == 'U' ? "CIMA" : (encoderD.direcao == 'D' ? "BAIXO" : "PARADO"));

    Serial.print("Garra: ");
    Serial.println(gripperOpen ? "ABERTA" : "FECHADA");
    Serial.println("=======================");

    encoderC.contadorAtualizado = false;
    encoderD.contadorAtualizado = false;
}

void resetAllEncoders() {
    noInterrupts();
    encoderC.contador = 0;
    encoderD.contador = 0;
    encoderC.direcao = ' ';
    encoderD.direcao = ' ';
    interrupts();
    Serial.println("Todos os encoders resetados!");
}

void updateMotorDirections(int dirC, int dirD) {
    motorDirectionC = dirC;
    motorDirectionD = dirD;
}

void stopAllMotors() {
    stopMotorsY();
    stopMotorsZ();
}

void stopMotorsY() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    motorDirectionC = 0;
}

void stopMotorsZ() {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    motorDirectionD = 0;
}

void moveLeft() {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
    motorDirectionC = 1;
}

void moveRight() {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
    motorDirectionC = -1;
}

void moveUp() {
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
    motorDirectionD = 1;
}

void moveDown() {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    motorDirectionD = -1;
}