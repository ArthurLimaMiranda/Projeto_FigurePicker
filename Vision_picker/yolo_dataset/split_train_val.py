import os
import shutil
import random

# Configurações
base_path = "dataset_aumentado"
train_image_path = os.path.join(base_path, "train/images")
train_label_path = os.path.join(base_path, "train/labels")
val_image_path = os.path.join(base_path, "val/images")
val_label_path = os.path.join(base_path, "val/labels")

# Criar pastas de validação se não existirem
os.makedirs(val_image_path, exist_ok=True)
os.makedirs(val_label_path, exist_ok=True)

# Obter lista de arquivos de imagens
image_files = [f for f in os.listdir(train_image_path) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]

# Calcular 10% das imagens
num_val_samples = int(len(image_files) * 0.1)
print(f"Total de imagens no treino: {len(image_files)}")
print(f"Movendo {num_val_samples} imagens para validação")

# Selecionar aleatoriamente 10% das imagens
val_samples = random.sample(image_files, num_val_samples)

# Mover imagens e labels
for image_file in val_samples:
    # Caminhos completos
    src_image = os.path.join(train_image_path, image_file)
    src_label = os.path.join(train_label_path, os.path.splitext(image_file)[0] + ".txt")
    
    dst_image = os.path.join(val_image_path, image_file)
    dst_label = os.path.join(val_label_path, os.path.splitext(image_file)[0] + ".txt")
    
    # Verificar se o label existe
    if os.path.exists(src_label):
        # Mover arquivos
        shutil.move(src_image, dst_image)
        shutil.move(src_label, dst_label)
        print(f"Movido: {image_file} e seu label")
    else:
        print(f"Aviso: Label não encontrado para {image_file}")

print(f"Validação criada com {len(val_samples)} amostras")
print(f"Imagens restantes no treino: {len(os.listdir(train_image_path))}")