# Hardware - Sistema de Controle Físico

Sistema de controle robótico baseado em ESP32 com comunicação ESP-NOW para manipulação de garra cartesiana.

## Visão Geral

Este módulo implementa o controle físico da garra robótica usando dois microcontroladores ESP32 que se comunicam via protocolo ESP-NOW (comunicação wireless ponto-a-ponto).

## Estrutura

```
Hardware/
├── Macs.txt                          # Endereços MAC dos ESP32
├── MacFinder/
│   └── MacFinder.ino                # Utilitário para descobrir endereço MAC
├── MovimentacaoEspPrincipal/
│   └── MovimentacaoEspPrincipal.ino # Controle do ESP Principal (Eixo X)
├── MovimentacaoEspSecundaria/
│   └── MovimentacaoEspSecundaria.ino # Controle do ESP Secundário (Eixos Y e Z)
└── MovimentacaoIntegrado/
    └── MovimentacaoIntegrado.ino    # Versão integrada (legacy)
```

## Arquitetura do Sistema

### Topologia de Comunicação

```
┌─────────────────────────┐
│   ESP Principal         │
│   (Eixo X)              │
│   MAC: a0:dd:6c:85:8a:b8│
└───────────┬─────────────┘
            │ ESP-NOW
            │ (Wireless)
            ▼
┌─────────────────────────┐
│   ESP Secundário        │
│   (Eixos Y e Z)         │
│   MAC: a0:dd:6c:03:e7:b8│
└─────────────────────────┘
```

## Hardware Components

### ESP Principal
- **Função**: Controla movimento no eixo X (Frente/Trás)
- **Motores**: 2 motores DC (Motor A e Motor B)
- **Encoder**: Encoder rotativo para feedback de posição
- **Comunicação**: Serial + ESP-NOW

#### Pinout ESP Principal
```
Motores:
- IN1 (GPIO 32): Motor A - Sentido 1
- IN2 (GPIO 33): Motor A - Sentido 2
- IN3 (GPIO 12): Motor B - Sentido 1
- IN4 (GPIO 13): Motor B - Sentido 2

Encoder:
- CLK (GPIO 14): Canal A do encoder
- DT (GPIO 27): Canal B do encoder

Botões:
- BTN_UP (GPIO 18): Movimento para frente
- BTN_DOWN (GPIO 19): Movimento para trás
```

### ESP Secundário
- **Função**: Controla movimentos nos eixos Y (Esquerda/Direita) e Z (Subir/Descer)
- **Motores**: 4 motores DC (2 para Y, 2 para Z)
- **Encoders**: 2 encoders (um para cada eixo)
- **Comunicação**: ESP-NOW (recebe comandos do ESP Principal)

## Protocolo ESP-NOW

### Estrutura de Mensagem
```cpp
typedef struct struct_message {
    char command[10];  // Comando: "MOVEY", "MOVEZ", "RESET", etc.
    long value;        // Valor associado (ex: direção)
    long position;     // Posição alvo (para movimentos absolutos)
} struct_message;
```

### Comandos Disponíveis

#### Comandos Locais (ESP Principal)
- `FRONT` / `F`: Move para frente
- `BACK` / `B`: Move para trás
- `STOPX`: Para motores do eixo X
- `RESETX`: Reseta encoder do eixo X
- `GOTOX <pos>`: Move para posição absoluta no eixo X
- `STATUS`: Exibe status dos encoders

#### Comandos Remotos (Enviados ao ESP Secundário)
- `MOVEY <dir>`: Move no eixo Y (1=direita, -1=esquerda)
- `MOVEZ <dir>`: Move no eixo Z (1=subir, -1=descer)
- `STOPY`: Para motores do eixo Y
- `STOPZ`: Para motores do eixo Z
- `RESETY`: Reseta encoder do eixo Y
- `RESETZ`: Reseta encoder do eixo Z
- `GOTOY <pos>`: Move para posição absoluta no eixo Y
- `GOTOZ <pos>`: Move para posição absoluta no eixo Z

## Setup e Configuração

### 1. Descobrir Endereços MAC

Primeiro, descubra os endereços MAC dos seus ESP32:

```bash
# Grave MacFinder.ino em cada ESP32
# Abra o Serial Monitor (115200 baud)
# Anote os endereços MAC exibidos
```

### 2. Configurar Endereços

Edite os arquivos `.ino` com os MACs corretos:

**MovimentacaoEspPrincipal.ino:**
```cpp
// MAC do ESP Secundário
uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x03, 0xE7, 0xB8};
```

**MovimentacaoEspSecundaria.ino:**
```cpp
// MAC do ESP Principal (se necessário para resposta)
uint8_t broadcastAddress[] = {0xA0, 0xDD, 0x6C, 0x85, 0x8A, 0xB8};
```

### 3. Upload do Código

1. Instale a biblioteca ESP-NOW (já incluída no Arduino ESP32)
2. Grave `MovimentacaoEspPrincipal.ino` no ESP Principal
3. Grave `MovimentacaoEspSecundaria.ino` no ESP Secundário

### 4. Conexão Serial

Conecte ao ESP Principal via Serial (115200 baud) para enviar comandos.

## Interface Serial

### Comandos Disponíveis
```
Comandos do Eixo X (Local):
  FRONT ou F         - Move para frente
  BACK ou B          - Move para trás  
  STOPX              - Para motores X
  RESETX             - Reseta encoder X
  GOTOX <posicao>    - Move para posição absoluta X

Comandos dos Eixos Y e Z (Remoto):
  MOVEY <dir>        - Move Y (1=direita, -1=esquerda)
  MOVEZ <dir>        - Move Z (1=subir, -1=descer)
  STOPY              - Para motores Y
  STOPZ              - Para motores Z
  RESETY             - Reseta encoder Y
  RESETZ             - Reseta encoder Z
  GOTOY <posicao>    - Move para posição absoluta Y
  GOTOZ <posicao>    - Move para posição absoluta Z

Comandos Gerais:
  STATUS             - Mostra status dos encoders
  HELP ou ?          - Mostra esta ajuda
```

## Encoders

### Sistema de Feedback

Cada encoder fornece feedback em tempo real da posição:
- **Resolução**: Incremental (pulsos por rotação)
- **Tipo**: Encoder rotativo quadratura
- **Função**: Controle de posição absoluta e relativa

### Leitura de Encoder
```cpp
struct EncoderData {
    volatile long counter;
    volatile int lastStateA;
    int pinA;
    int pinB;
};
```

## Características

### Controle de Motores
- ✅ Controle de direção via ponte H
- ✅ Movimento suave com PWM (opcional)
- ✅ Proteção contra comandos simultâneos
- ✅ Delay configurável entre comandos

### Sistema de Segurança
- ✅ Debouncing em botões físicos
- ✅ Timeout entre comandos (10 segundos padrão)
- ✅ Confirmação de envio ESP-NOW
- ✅ Status de comunicação

### Feedback
- ✅ Posição em tempo real via encoder
- ✅ Status de comunicação ESP-NOW
- ✅ Confirmação de comandos recebidos
- ✅ Indicação de movimentos ativos

## Troubleshooting

### ESP-NOW não conecta
1. Verifique os endereços MAC
2. Certifique-se que ambos ESPs estão ligados
3. Verifique distância (máx ~100m em campo aberto)

### Encoder não conta
1. Verifique conexões dos pinos CLK e DT
2. Teste com movimento manual
3. Verifique interrupções ativadas

### Motores não respondem
1. Verifique alimentação dos motores
2. Teste ponte H separadamente
3. Verifique pinout IN1-IN4

## Especificações Técnicas

- **Microcontrolador**: ESP32
- **Frequência**: 240 MHz (dual core)
- **Comunicação**: ESP-NOW (2.4 GHz)
- **Alcance**: ~100 metros (linha de visão)
- **Latência**: <10ms
- **Baudrate Serial**: 115200

## Fluxo de Operação

1. ESP Principal recebe comando via Serial ou botão
2. Se comando local (X): executa diretamente
3. Se comando remoto (Y/Z): envia via ESP-NOW para secundário
4. ESP Secundário recebe e executa comando
5. Feedback via encoder em ambos ESPs
6. Status retornado ao usuário

## Notas de Desenvolvimento

- Delay de 10 segundos entre comandos pode ser ajustado em `COMMAND_DELAY`
- Encoders usam interrupções para precisão máxima
- Comunicação ESP-NOW é assíncrona
- Serial Monitor mostra debug completo
