# Sistema de Visão Computacional para Garra Robótica
## Sprint - Migração de Câmera Acoplada para Câmera Externa

### 📋 Resumo do Projeto

Este projeto implementa um sistema de visão computacional para controle de uma garra robótica no CoppeliaSim. Durante esta sprint, foi realizada a migração completa do sistema de câmera acoplada à garra para um sistema com câmera externa posicionada em tripé.

---

## 🔄 Modelo Anterior vs Modelo Atual

### **Modelo Anterior: Câmera Acoplada**
- Câmera montada diretamente na garra
- Sistema de perseguição por pixel simples
- Movimento direto baseado na posição do objeto na imagem
- Menos complexidade matemática
- Campo de visão limitado à área próxima da garra

### **Modelo Atual: Câmera Externa (Tripé)**
- Câmera posicionada em tripé fixo com ângulo de -45°
- Projeção geométrica complexa (raycasting 3D)
- Transformação de coordenadas: Pixel → Mundo → Junta
- Calibração completa dos eixos XY e Z
- Visão ampla de toda a área de trabalho

---

## 🎯 Objetivos Alcançados

### 1. **Calibração do Sistema de Coordenadas**

#### Calibração XY
- Descoberta do mapeamento entre coordenadas do mundo e juntas
- Identificação de inversão do eixo Y
- Cálculo de offsets: X = +0.175m, Y = +0.300m

**Fórmulas Finais:**
```
MundoX = JX × 1.0 + 0.175
MundoY = JY × (-1.0) + 0.300
```

#### Calibração Z
- Descoberta dos limites físicos da junta Z
- Definição de alturas de operação seguras
- Range: -0.250 a +0.350

**Alturas Configuradas:**
- Navegação: 0.15m (altura inicial)
- Descida fina: 0.005m por clique

### 2. **Sistema de Projeção 3D**

Implementação de cálculo matemático completo:
1. Captura de pixel vermelho na imagem (256×256)
2. Normalização de coordenadas de tela
3. Cálculo do vetor de direção da câmera
4. Transformação para sistema de coordenadas mundial
5. Interseção do raio com o plano do objeto (Z = 0.05m)
6. Conversão para coordenadas das juntas

### 3. **Sistema de Trava XY**

Solução para interferência visual quando a garra entra no campo de visão:
- Botão "Iniciar Descida" trava posições X e Y
- Permite descida controlada sem movimentos indesejados
- Destrava automaticamente ao abrir a garra
- Previne oscilações causadas pela detecção da própria garra

### 4. **Controle Fino de Altura**

- Passo reduzido para descida: 0.5cm por clique
- Permite aproximação precisa do objeto
- Evita colisões com o cubo de 10cm de altura

---

## 🛠️ Componentes Técnicos

### Estrutura Física
- **Câmera (VS)**: Posição [0.2125, -0.150, 0.800], ângulo -45°
- **Área de trabalho**: 
  - X: -0.050 a 0.475m (52.5cm)
  - Y: 0.050 a 0.550m (50cm)
  - Z: -0.250 a 0.350m (60cm)
- **Objeto alvo**: Cubo vermelho 10×10×10cm

### Detecção de Cor
Filtro RGB para vermelho:
```lua
R > 130 AND R > G + 50 AND R > B + 50
```

### Parâmetros de Controle
- Tolerância XY: 0.5cm (precisão de posicionamento)
- Passo XY: 2.5cm (ajuste manual lateral)
- Passo Z: 0.5cm (descida granular)

---

## 📐 Algoritmo Principal

```
1. DETECÇÃO
   └─> Identificar pixels vermelhos na imagem da câmera
   └─> Calcular centroide dos pixels detectados

2. PROJEÇÃO 3D
   └─> Converter pixel (px, py) para coordenadas normalizadas
   └─> Aplicar matriz de transformação da câmera
   └─> Calcular interseção do raio com plano Z = 0.05m
   └─> Obter posição (mundoX, mundoY) no mundo real

3. CONVERSÃO DE COORDENADAS
   └─> Aplicar fórmulas de calibração
   └─> Converter (mundoX, mundoY) para (juntaX, juntaY)
   └─> Aplicar limites de segurança

4. CONTROLE DE MOVIMENTO
   └─> Se não travado: seguir objeto em XY
   └─> Se travado: manter XY fixo, permitir apenas Z
   └─> Enviar comandos para as juntas
```

---

## 🎮 Fluxo de Operação

1. **Inicialização**
   - Sistema detecta objeto vermelho
   - Garra segue automaticamente em XY
   - Altura Z em 0.15m (segura)

2. **Posicionamento**
   - Ajustar XY automaticamente (ou manual com setas)
   - Observar linha verde (debug) apontando para o objeto

3. **Descida Controlada**
   - Clicar "INICIAR DESCIDA" (trava XY)
   - Usar Z- para descer em passos de 0.5cm
   - Aproximar da altura do cubo (~0.05-0.10m)

4. **Captura**
   - Clicar "Fechar Garra"
   - Garra captura o objeto

5. **Reset**
   - Clicar "Abrir Garra"
   - Sistema destrava XY automaticamente
   - Volta ao modo de rastreamento normal

---

## 🔧 Scripts Desenvolvidos

### Script Principal
**`script.lua`**
- Sistema completo de visão e controle
- Projeção 3D e conversão de coordenadas
- Interface de controle

## 🚀 Melhorias em Relação ao Modelo Anterior

| Aspecto | Modelo Anterior | Modelo Atual |
|---------|----------------|--------------|
| **Campo de visão** | Limitado (câmera na garra) | Amplo (câmera externa) |
| **Complexidade** | Baixa (pixel → movimento) | Alta (projeção 3D completa) |
| **Precisão** | Dependente da distância | Calibrada e consistente |
| **Interferência** | Problema na garra | Resolvido com trava XY |
| **Calibração** | Não necessária | Sistema completo |
| **Área de trabalho** | Pequena | Grande (52×50cm) |
| **Manutenibilidade** | Acoplamento físico | Independente da garra |

---

## 📊 Desafios Superados

### 1. **Mapeamento de Coordenadas**
- **Problema**: Coordenadas do mundo não correspondiam diretamente às juntas
- **Solução**: Script de calibração que testou a garra em múltiplas posições
- **Resultado**: Fórmulas exatas de conversão com offset e inversão de Y

### 2. **Inversão do Eixo Y**
- **Problema**: JY aumenta → MundoY diminui (comportamento contra-intuitivo)
- **Solução**: Aplicação de escala negativa (-1.0) na conversão
- **Resultado**: Movimento correto da garra

### 3. **Interferência Visual**
- **Problema**: Garra entra no campo de visão ao descer, causando oscilações
- **Solução**: Sistema de trava manual que congela XY durante descida
- **Resultado**: Descida estável e controlada

### 4. **Projeção 3D Complexa**
- **Problema**: Câmera a 45° requer cálculo geométrico avançado
- **Solução**: Implementação de raycasting com matriz de transformação
- **Resultado**: Detecção precisa da posição 3D do objeto

### 5. **Controle de Altura**
- **Problema**: Passos muito grandes causavam colisões
- **Solução**: Passo reduzido de 2.5cm para 0.5cm em Z
- **Resultado**: Descida precisa sem colisões

---

## 🎓 Conceitos Aplicados

- **Visão Computacional**: Detecção de cor, processamento de imagem
- **Geometria 3D**: Projeção perspectiva, raycasting, interseção raio-plano
- **Álgebra Linear**: Transformações de coordenadas, matrizes de rotação
- **Robótica**: Cinemática, controle de juntas, espaço de trabalho
- **Calibração**: Mapeamento de sistemas de referência, identificação de parâmetros
- **Engenharia de Software**: Máquina de estados, modularização, debugging

---

## 📝 Próximos Passos Sugeridos

1. **Automação Completa**: Sistema que desce automaticamente quando bem posicionado
2. **Múltiplos Objetos**: Detecção e seleção entre vários cubos
3. **Cores Diferentes**: Expandir detecção para outras cores
4. **Feedback de Força**: Detectar quando garra toca o objeto
5. **Machine Learning**: Melhorar detecção com redes neurais (YOLO)
6. **Trajetórias Otimizadas**: Planejamento de caminho mais eficiente

---
Projeto acadêmico desenvolvido para fins educacionais no SENAI.
