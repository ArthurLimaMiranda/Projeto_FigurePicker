# Projeto FigurePicker - Versão Prévia (11/11/2025)

> **Nota:** Esta é uma versão anterior do projeto, especificamente a entrega realizada em **11 de novembro de 2025**. 

---

## Execução da Simulação (Vídeo)

Visualização do comportamento dinâmico do manipulador e resposta do controle visual.

![Demonstração da Simulação](recursos/SIM-COPPELIA.mp4)

---

##  1. Camada Física e Cinemática

A simulação modela um robô de topologia cartesiana (PPP - Prismático, Prismático, Prismático), operando em um espaço de trabalho tridimensional limitado.

![Projeto e Hierarquia](recursos/image.png)

### Topologia das Juntas
* **Atuadores:** 3 Juntas Prismáticas (`/JX`, `/JY`, `/JZ`) ortogonais entre si.
* **Efetuador Final:** Garra de acionamento paralelo composta por duas juntas deslizantes (`/JL`, `/JR`) que operam em simetria oposta.
* **Modo de Controle:** As juntas operam em *Position Control* (Malha fechada de posição), onde o motor aplica força até atingir a coordenada alvo (`sim.setJointTargetPosition`).

### Restrições Espaciais (Hard Limits)
Para garantir a integridade da simulação, foram implementados limites de software que restringem o volume de trabalho ($\mathbb{R}^3$):

| Eixo | Mínimo (m) | Máximo (m) | Resolução de Passo |
| :--- | :--- | :--- | :--- |
| **X** | -0.175 | 0.250 | 0.025 |
| **Y** | -0.200 | 0.200 | 0.025 |
| **Z** | -0.240 | 0.050 | Variável |

---

##  2. Lógica de Controle (Script)

O controle é centralizado em um único *Threaded Script* (Lua) que gerencia a máquina de estados do robô.

### Fluxo de Execução
1.  **Inicialização (`sysCall_init`):** Mapeamento dos *handles* dos objetos, ativação dos drivers de motor e renderização da GUI XML.
2.  **Loop de Sensoriamento (`sysCall_sensing`):**
    * Captura o buffer de imagem do sensor.
    * Executa o algoritmo de segmentação de cor.
    * Calcula o vetor de erro (diferença entre o centro da imagem e o centro do objeto).
3.  **Loop de Atuação (Event-Driven):**
    * A atualização de posição ocorre via *interrupções de UI* (cliques manuais) ou via *correção automática* (quando o sensor detecta o alvo).
    * A função `moverGarra()` atua como despachante final, enviando os valores de `alvoX`, `alvoY` e `alvoZ` para a API do simulador.

!

---

##  3. Sistema de Visão e Processamento

A simulação utiliza um sensor de visão RGB (`/VS`) acoplado ao efetuador (Eye-in-Hand configuration) para realizar o rastreamento ativo.

### Algoritmo de Detecção
O processamento de imagem é realizado pixel-a-pixel no array bruto da câmera:

1.  **Thresholding (Limiarização):** Filtra pixels que satisfazem a condição lógica:
    $$Pixel_{vermelho} \iff (R > 150) \land (G < 80) \land (B < 80)$$
2.  **Cálculo de Centroide:** Realiza a média aritmética das coordenadas $(x, y)$ de todos os pixels ativos para encontrar o centro de massa do objeto.
3.  **Feedback de Posição:**
    O erro de posição é realimentado no sistema de controle para ajustar as coordenadas $X$ e $Y$ do robô proporcionalmente ao desvio detectado:
    ```lua
    alvoX = alvoX + (deltaX * ganho)
    alvoY = alvoY - (deltaY * ganho)
    ```

---

##  4. Interface Homem-Máquina (IHM)

A simulação integra uma IHM nativa (`simUI`) que sobrepõe a viewport 3D.

![Interface de Controle](recursos/controle.png)

* **Visualização:** Renderiza o buffer da câmera do robô em tempo real (`simUI.setImageData`).
* **Telemetria:** Exibe as coordenadas numéricas atuais dos alvos.
* **Intervenção:** Permite override manual das coordenadas calculadas pela visão.