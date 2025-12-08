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

// CALLBACKS ESP-NOW
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("OnDataSent - status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    const uint8_t *mac = recv_info->src_addr;

    if (len < (int)sizeof(receivedData)) {
        Serial.println("Tamanho de dados recebido inválido");
        return;
    }
    
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    receivedData.command[9] = '\0';

    Serial.print("Comando recebido: ");
    Serial.print(receivedData.command);
    Serial.print(" - Valor: ");
    Serial.println(receivedData.value);

    // Processa os comandos recebidos
    if (strcmp(receivedData.command, "MOVE_Y") == 0) {
        if(receivedData.value == 1) {
            moveLeft();
            Serial.println("EIXO Y: Movendo para ESQUERDA");
        } else if (receivedData.value == -1) {
            moveRight();
            Serial.println("EIXO Y: Movendo para DIREITA");
        }
    }
    else if (strcmp(receivedData.command, "MOVE_Z") == 0) {
        if(receivedData.value == 1) {
            moveUp();
            Serial.println("EIXO Z: Movendo para CIMA");
        } else if (receivedData.value == -1) {
            moveDown();
            Serial.println("EIXO Z: Movendo para BAIXO");
        }
    }
    else if (strcmp(receivedData.command, "STOP_ALL") == 0) {
        stopAllMotors();
        Serial.println("TODOS OS MOTORES: Parando");
    }
    else if (strcmp(receivedData.command, "GOTO_Y") == 0) {
        moveToPositionY(receivedData.position);
        Serial.print("EIXO Y: Indo para posição ");
        Serial.println(receivedData.position);
    }
    else if (strcmp(receivedData.command, "GOTO_Z") == 0) {
        moveToPositionZ(receivedData.position);
        Serial.print("EIXO Z: Indo para posição ");
        Serial.println(receivedData.position);
    }
    else if (strcmp(receivedData.command, "GRIP") == 0) {
        controlGripper(receivedData.value);
        if(receivedData.value == 1) {
            Serial.println("GARRA: Abrindo");
        } else {
            Serial.println("GARRA: Fechando");
        }
    }
    else if (strcmp(receivedData.command, "RESET_Y") == 0) {
        resetY();
        Serial.println("EIXO Y: Iniciando reset");
    }
    else if (strcmp(receivedData.command, "RESET_Z") == 0) {
        resetZ();
        Serial.println("EIXO Z: Iniciando reset");
    }
    else if (strcmp(receivedData.command, "RESET_ENC") == 0) {
        resetAllEncoders();
        Serial.println("ENCODERS: Resetados");
    }
    else if (strcmp(receivedData.command, "STATUS") == 0) {
        printEncoderStatus();
    }

    // Envia confirmação
    strcpy(myData.command, "CONFIRM");
    myData.value = receivedData.value;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
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
        }
    }

    // Verifica descida do eixo Z
    if (motorDirectionD == -1 && encoderD.contador >= Z_GROUND_POSITION) {
        stopMotorsZ();
        Serial.println("EIXO Z: Chegou perto do chão - Parando");
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

// Função para controle da garra
void controlGripper(int state) {
    gripperServo.write(state);
}

// Função para controle dos botões
void handleButtons() {
    if (millis() - lastButtonTime < BUTTON_DEBOUNCE) return;

    bool anyButtonPressed = (digitalRead(left) == LOW) ||
                           (digitalRead(right) == LOW);
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