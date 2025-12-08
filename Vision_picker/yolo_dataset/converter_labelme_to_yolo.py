# converter_labelme_to_yolo.py
import json
import os
from pathlib import Path
import shutil

def converter_labelme_para_yolo():
    """Converte anotações do LabelMe para formato YOLO"""
    
    print("🔄 CONVERTENDO LABELME PARA YOLO")
    print("=" * 50)
    
    # 1. Verifica labels.txt usando os.path.join
    labels_file = os.path.join(".", "labels.txt")
    
    if not os.path.exists(labels_file):
        print(f"❌ {labels_file} não encontrado!")
        return
    
    with open(labels_file, "r", encoding="utf-8") as f:
        classes = [line.strip() for line in f if line.strip()]
    
    if not classes:
        print("❌ labels.txt está vazio! Adicione classes (cubo, esfera)")
        return
    
    class_map = {name: idx for idx, name in enumerate(classes)}
    print(f"📋 Classes: {class_map}")
    
    # 2. Garante pastas YOLO existem usando os.makedirs
    train_labels_dir = os.path.join("labels", "train")
    test_labels_dir = os.path.join("labels", "test")
    
    os.makedirs(train_labels_dir, exist_ok=True)
    os.makedirs(test_labels_dir, exist_ok=True)
    
    # 3. Processa TREINO
    print(f"\n📁 CONVERTENDO TREINO...")
    processar_split("labelme_train", "train", class_map)
    
    # 4. Processa TESTE
    print(f"\n📁 CONVERTENDO TESTE...")
    processar_split("labelme_test", "test", class_map)
    
    # 5. Cria dataset.yaml usando os.path.join
    criar_dataset_yaml(classes)
    
    print(f"\n" + "=" * 50)
    print("✅ CONVERSÃO COMPLETA!")
    print("=" * 50)
    print(f"\n🎯 Agora pode treinar o modelo com:")
    print(f'   python treinar_yolov11.py')
    print(f'   OU execute: {os.path.join(".", "4_treinar_yolo.bat")}')

def processar_split(labelme_dir, split_name, class_map):
    """Processa um conjunto (train ou test)"""
    
    input_dir = os.path.join(".", labelme_dir)
    output_labels_dir = os.path.join("labels", split_name)
    
    if not os.path.exists(input_dir):
        print(f"⚠️  Pasta não encontrada: {input_dir}")
        return
    
    json_files = list(Path(input_dir).glob("*.json"))
    
    if not json_files:
        print(f"   ⚠️  Nenhum arquivo .json em {input_dir}/")
        return
    
    print(f"   📂 {len(json_files)} arquivos .json encontrados")
    
    conversoes_bem_sucedidas = 0
    
    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            base_name = json_file.stem
            img_width = data['imageWidth']
            img_height = data['imageHeight']
            
            # Conteúdo YOLO
            yolo_lines = []
            
            for shape in data['shapes']:
                label = shape['label']
                
                if label not in class_map:
                    print(f"      ⚠️  Classe '{label}' ignorada (não está em labels.txt)")
                    continue
                
                class_id = class_map[label]
                points = shape['points']
                
                # Calcula bounding box
                x_coords = [p[0] for p in points]
                y_coords = [p[1] for p in points]
                
                x_min = min(x_coords)
                y_min = min(y_coords)
                x_max = max(x_coords)
                y_max = max(y_coords)
                
                # Normaliza para YOLO (0-1)
                x_center = (x_min + x_max) / 2 / img_width
                y_center = (y_min + y_max) / 2 / img_height
                width = (x_max - x_min) / img_width
                height = (y_max - y_min) / img_height
                
                # Valida
                if not (0 <= x_center <= 1 and 0 <= y_center <= 1):
                    print(f"      ⚠️  Bbox inválida em {base_name}: {label}")
                    continue
                
                if width <= 0 or height <= 0:
                    print(f"      ⚠️  Dimensões zero em {base_name}: {label}")
                    continue
                
                yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
            
            # Salva arquivo .txt YOLO
            if yolo_lines:
                txt_file = os.path.join(output_labels_dir, f"{base_name}.txt")
                with open(txt_file, 'w', encoding='utf-8') as f:
                    f.write("\n".join(yolo_lines))
                conversoes_bem_sucedidas += 1
            else:
                print(f"      ⚠️  Nenhuma anotação válida em {base_name}")
                
        except Exception as e:
            print(f"      ❌ Erro ao processar {json_file.name}: {e}")
    
    print(f"   ✅ Convertidos: {conversoes_bem_sucedidas}/{len(json_files)}")

def criar_dataset_yaml(classes):
    """Cria arquivo dataset.yaml para YOLO"""
    
    # Caminho absoluto usando os.path.abspath
    abs_path = os.path.abspath(".")
    
    yaml_content = f"""# Dataset YOLO - Cubos e Esferas
# Caminho absoluto
path: {abs_path}

# Caminhos relativos
train: images/train
val: images/test

# Número de classes
nc: {len(classes)}

# Nomes das classes
names: {classes}
"""
    
    yaml_file = os.path.join(".", "dataset.yaml")
    with open(yaml_file, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    
    print(f"\n📄 dataset.yaml criado em: {yaml_file}")
    print("-" * 40)
    print(yaml_content)
    print("-" * 40)

if __name__ == "__main__":
    converter_labelme_para_yolo()