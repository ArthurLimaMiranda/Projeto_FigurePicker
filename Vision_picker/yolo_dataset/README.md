# YOLO Dataset Tools - Preparação e Augmentation

Conjunto de scripts para preparação, conversão e augmentation de datasets para treinamento de modelos YOLO.

## Visão Geral

Este módulo contém ferramentas essenciais para preparar datasets de imagens para treinamento de modelos YOLO, incluindo conversão de formatos, organização de dados, data augmentation e validação de labels.

## Estrutura

```
yolo_dataset/
├── augmentation_com_labels.py          # Augmentation preservando labels YOLO
├── converter_heic_para_jpg.py          # Converte HEIC → JPG
├── converter_labelme_to_yolo.py        # Converte LabelMe → YOLO
├── preparar_anotacao_final.py          # Preparação final do dataset
├── remover_heic.py                     # Remove arquivos HEIC após conversão
└── split_train_val.py                  # Divide dataset em train/val/test
```

## Instalação

### Dependências
```bash
pip install opencv-python pillow pillow-heif albumentations pyyaml numpy
```

## Scripts Disponíveis

### 1. converter_heic_para_jpg.py

Converte imagens do formato HEIC (usado por iPhones) para JPG.

#### Uso
```bash
python converter_heic_para_jpg.py
```

#### Funcionalidades
- ✅ Busca recursiva por arquivos .heic e .HEIC
- ✅ Conversão com preservação de qualidade
- ✅ Mantém estrutura de diretórios
- ✅ Log de progresso

#### Código Base
```python
from PIL import Image
import pillow_heif

pillow_heif.register_heif_opener()

heic_image = Image.open('image.heic')
heic_image.convert('RGB').save('image.jpg', 'JPEG', quality=95)
```

---

### 2. remover_heic.py

Remove arquivos HEIC após conversão bem-sucedida.

#### Uso
```bash
python remover_heic.py
```

⚠️ **Atenção**: Use apenas após confirmar que as conversões foram bem-sucedidas!

#### Funcionalidades
- ✅ Busca recursiva por .heic/.HEIC
- ✅ Confirmação antes de deletar
- ✅ Log de arquivos removidos
- ✅ Contagem de espaço liberado

---

### 3. converter_labelme_to_yolo.py

Converte anotações do formato LabelMe (JSON) para formato YOLO (TXT).

#### Formato LabelMe (JSON)
```json
{
  "imagePath": "image.jpg",
  "imageHeight": 480,
  "imageWidth": 640,
  "shapes": [
    {
      "label": "cubo",
      "points": [[x1, y1], [x2, y2], [x3, y3], [x4, y4]],
      "shape_type": "polygon"
    }
  ]
}
```

#### Formato YOLO (TXT)
```
<class_id> <x_center> <y_center> <width> <height>
```
Valores normalizados entre 0 e 1.

#### Uso
```bash
python converter_labelme_to_yolo.py
```

#### Conversão
```python
# LabelMe polygon → YOLO bounding box
x_min, y_min = min(points, key=lambda p: p[0])[0], min(points, key=lambda p: p[1])[1]
x_max, y_max = max(points, key=lambda p: p[0])[0], max(points, key=lambda p: p[1])[1]

# Normaliza
x_center = ((x_min + x_max) / 2) / img_width
y_center = ((y_min + y_max) / 2) / img_height
width = (x_max - x_min) / img_width
height = (y_max - y_min) / img_height

# Formato YOLO
yolo_label = f"{class_id} {x_center} {y_center} {width} {height}"
```

---

### 4. split_train_val.py

Divide dataset em conjuntos de treino, validação e teste.

#### Uso
```bash
python split_train_val.py
```

#### Configuração
```python
TRAIN_RATIO = 0.7   # 70% treino
VAL_RATIO = 0.2     # 20% validação
TEST_RATIO = 0.1    # 10% teste
```

#### Estrutura de Saída
```
dataset/
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── labels/
    ├── train/
    ├── val/
    └── test/
```

#### Funcionalidades
- ✅ Split aleatório com seed fixo
- ✅ Mantém pareamento imagem-label
- ✅ Validação de integridade
- ✅ Relatório de estatísticas

---

### 5. augmentation_com_labels.py

Aplica data augmentation preservando as anotações YOLO.

#### Uso
```bash
python augmentation_com_labels.py
```

#### Transformações Aplicadas

```python
import albumentations as A

transform = A.Compose([
    # Flips
    A.HorizontalFlip(p=0.5),
    A.VerticalFlip(p=0.2),
    
    # Geometric
    A.Rotate(limit=15, border_mode=cv2.BORDER_CONSTANT, value=0, p=0.5),
    A.RandomScale(scale_limit=0.2, p=0.3),
    
    # Color
    A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
    A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=30, val_shift_limit=20, p=0.5),
    A.RGBShift(r_shift_limit=20, g_shift_limit=20, b_shift_limit=20, p=0.3),
    
    # Noise
    A.GaussNoise(var_limit=(10.0, 50.0), p=0.3),
    A.GaussianBlur(blur_limit=(3, 7), p=0.3),
    A.MotionBlur(blur_limit=7, p=0.2),
    
    # Dropout
    A.CoarseDropout(max_holes=8, max_height=32, max_width=32, p=0.3),
    
], bbox_params=A.BboxParams(
    format='albumentations',  # [x_min, y_min, x_max, y_max]
    label_fields=['class_labels'],
    min_area=16,
    min_visibility=0.3
))
```

#### Parâmetros
```python
IMAGES_DIR = 'images/train'
LABELS_DIR = 'labels/train'
OUTPUT_DIR = 'augmented_dataset'
AUGMENTATIONS_PER_IMAGE = 5  # Gera 5 variações por imagem
```

#### Funcionalidades
- ✅ Preserva bounding boxes durante transformações
- ✅ Remove boxes com área mínima < threshold
- ✅ Remove boxes com visibilidade < threshold
- ✅ Nomenclatura automática (img_001_aug_0.jpg)
- ✅ Validação de labels YOLO

---

### 6. preparar_anotacao_final.py

Script de preparação final do dataset antes do treinamento.

#### Uso
```bash
python preparar_anotacao_final.py
```

#### Funcionalidades
- ✅ Validação de integridade (pareamento img-label)
- ✅ Verificação de labels vazios
- ✅ Normalização de nomes de arquivo
- ✅ Criação de data.yaml
- ✅ Estatísticas do dataset
- ✅ Visualização de exemplos

#### Validações Realizadas
```python
# 1. Cada imagem tem um label
# 2. Cada label tem uma imagem
# 3. Labels não estão vazios
# 4. Coordenadas estão normalizadas (0-1)
# 5. Class IDs são válidos
# 6. Bounding boxes estão dentro da imagem
```

---

## Workflow Completo

### Pipeline Recomendado

```bash
# 1. Converter HEIC para JPG
python converter_heic_para_jpg.py

# 2. Remover arquivos HEIC originais
python remover_heic.py

# 3. Converter anotações LabelMe para YOLO
python converter_labelme_to_yolo.py

# 4. Dividir em train/val/test
python split_train_val.py

# 5. Aplicar augmentation
python augmentation_com_labels.py

# 6. Preparação final e validação
python preparar_anotacao_final.py

# 7. Treinar modelo (no diretório yolov11s)
cd ../yolov11s
python train.py
```

---

## Estruturas de Dataset

### Antes do Processamento
```
raw_data/
├── image_001.heic
├── image_001.json  (LabelMe)
├── image_002.heic
└── image_002.json
```

### Após Conversão
```
converted/
├── image_001.jpg
├── image_001.txt  (YOLO)
├── image_002.jpg
└── image_002.txt
```

### Após Split
```
dataset/
├── images/
│   ├── train/  (70%)
│   ├── val/    (20%)
│   └── test/   (10%)
└── labels/
    ├── train/
    ├── val/
    └── test/
```

### Após Augmentation
```
augmented_dataset/
├── images/
│   └── train/
│       ├── img_001.jpg
│       ├── img_001_aug_0.jpg
│       ├── img_001_aug_1.jpg
│       └── ...
└── labels/
    └── train/
        ├── img_001.txt
        ├── img_001_aug_0.txt
        ├── img_001_aug_1.txt
        └── ...
```

---

## Exemplos de Uso

### Exemplo 1: Conversão Completa

```python
from pathlib import Path
import subprocess

# 1. Converter HEIC
subprocess.run(['python', 'converter_heic_para_jpg.py'])

# 2. Converter Labels
subprocess.run(['python', 'converter_labelme_to_yolo.py'])

# 3. Split dataset
subprocess.run(['python', 'split_train_val.py'])

print("Dataset preparado!")
```

### Exemplo 2: Augmentation Customizado

```python
import albumentations as A
import cv2
import numpy as np

def custom_augmentation(image, boxes, labels):
    transform = A.Compose([
        A.HorizontalFlip(p=0.5),
        A.Rotate(limit=10, p=0.3),
        A.RandomBrightnessContrast(p=0.5),
    ], bbox_params=A.BboxParams(
        format='yolo',
        label_fields=['class_labels']
    ))
    
    augmented = transform(image=image, bboxes=boxes, class_labels=labels)
    return augmented['image'], augmented['bboxes'], augmented['class_labels']
```

### Exemplo 3: Validação de Labels

```python
def validate_yolo_label(label_path, img_width, img_height):
    """Valida formato YOLO"""
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                return False, "Formato inválido"
            
            cls_id, x, y, w, h = map(float, parts)
            
            # Verifica normalização
            if not (0 <= x <= 1 and 0 <= y <= 1 and 0 <= w <= 1 and 0 <= h <= 1):
                return False, "Coordenadas fora do range [0, 1]"
            
            # Verifica box dentro da imagem
            x1, y1 = x - w/2, y - h/2
            x2, y2 = x + w/2, y + h/2
            
            if x1 < 0 or y1 < 0 or x2 > 1 or y2 > 1:
                return False, "Bounding box fora da imagem"
    
    return True, "OK"
```

---

## Troubleshooting

### Erro: HEIC não suportado
```bash
pip install pillow-heif
```

### Labels não convertendo
```python
# Verifica formato JSON
import json
with open('label.json', 'r') as f:
    data = json.load(f)
    print(data.keys())  # Deve ter 'shapes'
```

### Augmentation quebrando labels
```python
# Aumenta min_visibility e min_area
bbox_params=A.BboxParams(
    format='yolo',
    min_area=100,      # Aumentado de 16
    min_visibility=0.5  # Aumentado de 0.3
)
```

### Dataset desbalanceado após split
```python
# Usa stratified split
from sklearn.model_selection import train_test_split

# Coleta labels
labels = [get_majority_class(f) for f in files]

train, test = train_test_split(files, stratify=labels, test_size=0.3)
```

---

## Recursos Adicionais

### Visualização de Augmentation

```python
import cv2
import matplotlib.pyplot as plt

def visualize_augmentation(image_path, label_path):
    """Visualiza imagem com bounding boxes"""
    img = cv2.imread(image_path)
    h, w = img.shape[:2]
    
    with open(label_path, 'r') as f:
        for line in f:
            cls_id, x, y, width, height = map(float, line.strip().split())
            
            # Desnormaliza
            x1 = int((x - width/2) * w)
            y1 = int((y - height/2) * h)
            x2 = int((x + width/2) * w)
            y2 = int((y + height/2) * h)
            
            # Desenha box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"Class {int(cls_id)}", (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()
```

### Estatísticas do Dataset

```python
def dataset_statistics(labels_dir):
    """Calcula estatísticas do dataset"""
    from collections import Counter
    
    class_counts = Counter()
    box_sizes = []
    
    for label_file in Path(labels_dir).glob('*.txt'):
        with open(label_file, 'r') as f:
            for line in f:
                cls_id, x, y, w, h = map(float, line.strip().split())
                class_counts[int(cls_id)] += 1
                box_sizes.append(w * h)
    
    print("Distribuição de Classes:")
    for cls_id, count in class_counts.items():
        print(f"  Classe {cls_id}: {count} objetos")
    
    print(f"\nTamanho médio de box: {np.mean(box_sizes):.4f}")
    print(f"Tamanho min/max: {min(box_sizes):.4f} / {max(box_sizes):.4f}")
```

---

## Integração com YOLO

### Criar data.yaml

```python
import yaml

data = {
    'path': 'dataset',
    'train': 'images/train',
    'val': 'images/val',
    'test': 'images/test',
    'nc': 2,
    'names': {0: 'cubo', 1: 'esfera'}
}

with open('data.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False)
```

### Usar com Ultralytics

```python
from ultralytics import YOLO

# Treinar
model = YOLO('yolo11s.pt')
model.train(data='data.yaml', epochs=100, imgsz=640)
```

---

## Notas Importantes

- ⚠️ Sempre faça backup antes de remover arquivos HEIC
- ⚠️ Valide labels após cada conversão
- ⚠️ Augmentation pode aumentar dataset em 5-10x
- ⚠️ Use seed fixo para reprodutibilidade
- ⚠️ Monitore espaço em disco durante augmentation