# Classifier - Classificador Tradicional de Formas

Sistema de classificação de formas geométricas usando Machine Learning tradicional baseado em características extraídas de imagens.

## Visão Geral

Este módulo implementa um classificador binário para distinguir entre cubos e esferas usando técnicas de Machine Learning clássicas (não deep learning). O classificador é treinado com características extraídas diretamente dos pixels das imagens.

## Estrutura

```
Classifier/
├── img_classifier.py         # Script de classificação de imagens
├── train_classifier.py       # Script de treinamento do modelo
└── models/
    ├── melhor_modelo.pkl     # Modelo treinado serializado
    └── model_params.json     # Parâmetros e configurações
```

## Instalação

### Dependências
```bash
pip install numpy scikit-image scikit-learn matplotlib seaborn joblib opencv-python
```

## Funcionamento

### Pipeline de Classificação

1. **Carregamento de Imagem**
   - Leitura via OpenCV
   - Conversão de cores (BGR → RGB ou Grayscale)

2. **Pré-processamento**
   - Redimensionamento para 64x64 pixels
   - Normalização (opcional)
   - Flatten (conversão para vetor 1D)

3. **Extração de Características**
   - RGB: 64 × 64 × 3 = 12,288 features
   - Grayscale: 64 × 64 = 4,096 features

4. **Predição**
   - Carregamento do modelo treinado (.pkl)
   - Classificação usando modelo sklearn
   - Retorno da classe predita

## Uso

### Classificar Imagem

```bash
python img_classifier.py
```

**Fluxo:**
1. Abre diálogo de seleção de arquivo
2. Carrega imagem selecionada
3. Processa e classifica
4. Exibe resultado com label sobreposto

### Treinar Modelo

```bash
python train_classifier.py
```

**Parâmetros configuráveis:**
```python
IMG_SIZE = (64, 64)           # Tamanho das imagens
USE_COLOR = True              # True=RGB, False=Grayscale
CLASSIFIER = 'SVM'            # 'SVM', 'RandomForest', 'KNN'
TEST_SIZE = 0.2               # Porcentagem para teste
RANDOM_STATE = 42             # Seed para reprodutibilidade
```

## Características Técnicas

### Extração de Features

#### Modo RGB (Padrão)
```python
IMG_SIZE = (64, 64)
CHANNELS = 3
TOTAL_FEATURES = 64 × 64 × 3 = 12,288
```

Cada pixel contribui com 3 valores (R, G, B).

#### Modo Grayscale
```python
IMG_SIZE = (64, 64)
CHANNELS = 1
TOTAL_FEATURES = 64 × 64 = 4,096
```

Cada pixel contribui com 1 valor (intensidade).

### Classificadores Suportados

#### 1. SVM (Support Vector Machine)
```python
from sklearn.svm import SVC
model = SVC(kernel='rbf', C=1.0, gamma='scale')
```
- **Pros**: Ótimo para alta dimensionalidade, robusto
- **Contras**: Lento com datasets grandes

#### 2. Random Forest
```python
from sklearn.ensemble import RandomForestClassifier
model = RandomForestClassifier(n_estimators=100, max_depth=10)
```
- **Pros**: Rápido, interpretável, robusto a overfitting
- **Contras**: Pode consumir mais memória

#### 3. K-Nearest Neighbors
```python
from sklearn.neighbors import KNeighborsClassifier
model = KNeighborsClassifier(n_neighbors=5)
```
- **Pros**: Simples, sem treinamento
- **Contras**: Lento em inferência, sensível a ruído

## Métricas de Avaliação

### Métricas Calculadas
```python
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

accuracy = accuracy_score(y_true, y_pred)
precision = precision_score(y_true, y_pred)
recall = recall_score(y_true, y_pred)
f1 = f1_score(y_true, y_pred)
```

### Matriz de Confusão
```python
from sklearn.metrics import confusion_matrix
import seaborn as sns

cm = confusion_matrix(y_true, y_pred)
sns.heatmap(cm, annot=True, fmt='d')
```

## Visualização

### Exibição de Resultado

```python
# Carrega imagem
img_display = cv2.imread(image_path)

# Adiciona label predito
label_text = f"Class: {label_map.get(prediction)}"
cv2.putText(img_display, label_text, (10, 30), 
           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Mostra imagem
cv2.imshow("Classification Result", img_display)
cv2.waitKey(0)
```

## Configuração

### img_classifier.py

```python
# Configurações principais
IMG_SIZE = (64, 64)                    # Tamanho de redimensionamento
LABEL_MAP = {0: "cubo", 1: "esfera"}   # Mapeamento de classes
MODEL_PATH = 'models/melhor_modelo.pkl' # Caminho do modelo

# Modo de cor
USE_COLOR = True  # True=RGB, False=Grayscale
```

### train_classifier.py

```python
# Dataset
DATASET_PATH = 'data/'
CLASSES = ['cubo', 'esfera']

# Hyperparameters
IMG_SIZE = (64, 64)
TEST_SIZE = 0.2
VALIDATION_SIZE = 0.1
RANDOM_STATE = 42

# Modelo
CLASSIFIER_TYPE = 'SVM'  # 'SVM', 'RandomForest', 'KNN'

# Salvamento
MODEL_SAVE_PATH = 'models/melhor_modelo.pkl'
PARAMS_SAVE_PATH = 'models/model_params.json'
```

## Estrutura do Dataset

### Organização Esperada
```
data/
├── cubo/
│   ├── cubo_001.jpg
│   ├── cubo_002.jpg
│   └── ...
└── esfera/
    ├── esfera_001.jpg
    ├── esfera_002.jpg
    └── ...
```

### Formato das Imagens
- **Formatos suportados**: JPG, PNG, JPEG
- **Tamanho**: Qualquer (será redimensionado)
- **Cor**: RGB ou Grayscale

## Exemplos de Uso

### Exemplo 1: Classificação Simples
```python
import cv2
import joblib

# Carrega modelo
model = joblib.load('models/melhor_modelo.pkl')

# Carrega e processa imagem
image = cv2.imread('test_image.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_resized = cv2.resize(image, (64, 64))
image_flat = image_resized.flatten()

# Classifica
prediction = model.predict([image_flat])[0]
label = {0: "cubo", 1: "esfera"}[prediction]

print(f"Classe predita: {label}")
```

### Exemplo 2: Batch Classification
```python
import os
from glob import glob

# Carrega modelo
model = joblib.load('models/melhor_modelo.pkl')

# Processa múltiplas imagens
image_paths = glob('test_images/*.jpg')
results = []

for path in image_paths:
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image, (64, 64))
    image_flat = image_resized.flatten()
    
    prediction = model.predict([image_flat])[0]
    results.append({
        'path': path,
        'class': {0: "cubo", 1: "esfera"}[prediction]
    })

# Mostra resultados
for result in results:
    print(f"{result['path']}: {result['class']}")
```

## Análise de Performance

### Vantagens
✅ **Rápido**: Inferência em milissegundos  
✅ **Leve**: Modelo pequeno (~1-10 MB)  
✅ **Simples**: Fácil de entender e debugar  
✅ **Sem GPU**: Roda em qualquer CPU  
✅ **Interpretável**: Features são pixels diretamente  

### Limitações
❌ **Menor Precisão**: Tipicamente 85-90% vs 95%+ do YOLO  
❌ **Sensível a Ruído**: Pixels brutos são afetados por iluminação  
❌ **Sem Localização**: Apenas classifica, não detecta posição  
❌ **Features Fixas**: Não aprende features automaticamente  
❌ **Escala Limitada**: Funciona bem apenas em imagens simples  

## Troubleshooting

### Modelo não encontrado
```python
# Verifica se o arquivo existe
import os
if not os.path.exists('models/melhor_modelo.pkl'):
    print("Modelo não encontrado! Execute train_classifier.py primeiro.")
```

### Erro de dimensão
```python
# Certifica-se que a imagem está no formato correto
print(f"Shape esperada: {64 * 64 * 3} para RGB")
print(f"Shape recebida: {image_flat.shape}")
```

### Baixa acurácia
- Aumentar tamanho do dataset
- Adicionar data augmentation
- Experimentar diferentes classificadores
- Ajustar hiperparâmetros
- Considerar usar features mais sofisticadas (HOG, SIFT, etc.)

## Recursos Adicionais

### Melhorias Possíveis

1. **Feature Engineering**
   - HOG (Histogram of Oriented Gradients)
   - SIFT (Scale-Invariant Feature Transform)
   - Color histograms
   - Texture features (LBP)

2. **Ensemble Methods**
   - Voting classifier
   - Stacking
   - Boosting (XGBoost, LightGBM)

3. **Preprocessing**
   - Normalização
   - Equalização de histograma
   - Remoção de background
   - Edge detection

## Comparação: Classifier vs YOLO

| Aspecto | Classifier | YOLOv11s |
|---------|-----------|----------|
| **Precisão** | 85-90% | 95%+ |
| **Velocidade** | ~5-10ms | ~20-50ms |
| **Tamanho** | 1-10 MB | 20-50 MB |
| **Localização** | ❌ Não | ✅ Sim |
| **Multiple Objects** | ❌ Não | ✅ Sim |
| **GPU Required** | ❌ Não | ⚠️ Recomendado |
| **Treinamento** | Minutos | Horas |
| **Dataset Size** | 100-1000 | 1000-10000+ |

## Notas

- O classificador funciona melhor com imagens centradas e recortadas
- Iluminação consistente é crucial para boa performance
- Para produção, considere usar YOLO para detecção + este classificador para refinamento
- Modelo .pkl é específico da versão do sklearn - recrie se necessário

## Workflow Típico

```bash
# 1. Coletar e organizar dataset
mkdir -p data/cubo data/esfera

# 2. Treinar modelo
python train_classifier.py

# 3. Testar classificação
python img_classifier.py

# 4. Avaliar resultados e iterar
```