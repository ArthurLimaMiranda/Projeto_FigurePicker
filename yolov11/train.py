from roboflow import Roboflow
from ultralytics import YOLO
from pathlib import Path
import shutil
import random
import yaml
import cv2
import matplotlib.pyplot as plt
from IPython.display import Image, display
from pathlib import Path



API_KEY = "RGZ4JUpt6YMS1ZreoZAT"

WORKSPACE = "w0"
PROJECT = "3d-geom-shape-detector"
VERSION = 1

TRAIN_PCT = 70   # Treino
VAL_PCT = 20     # Validação
TEST_PCT = 10    # Teste

OUTPUT_DIR = "dataset_custom_split"  # Onde salvar o dataset

EPOCHS = 100        
BATCH_SIZE = 16      
IMG_SIZE = 640       
MODEL = "yolo11s"   

import torch
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# =============================================================================

print("="*70)
print("⚙️  CONFIGURAÇÕES")
print("="*70)
print(f"Dataset: {WORKSPACE}/{PROJECT}")
print(f"Splits: {TRAIN_PCT}% / {VAL_PCT}% / {TEST_PCT}%")
print(f"Épocas: {EPOCHS}")
print(f"Batch: {BATCH_SIZE}")
print(f"Device: {DEVICE}")
print(f"Modelo: {MODEL}")
print("="*70)


def download_and_split_dataset(
    api_key,
    workspace,
    project,
    version=1,
    train_pct=70,
    val_pct=20,
    test_pct=10,
    output_dir="dataset_split"
):
    """
    Baixa dataset do Roboflow e separa em train/val/test
    """

    print("="*70)
    print("BAIXANDO DATASET DO ROBOFLOW")
    print("="*70)

    rf = Roboflow(api_key=api_key)

    print(f"\nAcessando: {workspace}/{project} v{version}")
    project_obj = rf.workspace(workspace).project(project)

    print(f"\n Baixando dataset...")
    dataset = project_obj.version(version).download("yolov8")

    print(f"\n Download concluído!")
    print(f" Localização temporária: {dataset.location}")

    print(f"\n Coletando arquivos...")
    dataset_path = Path(dataset.location)

    all_files = []

    for split_dir in ['train', 'valid', 'test']:
        img_dir = dataset_path / split_dir / 'images'
        lbl_dir = dataset_path / split_dir / 'labels'

        if img_dir.exists() and lbl_dir.exists():
            for img_file in img_dir.glob('*.*'):
                if img_file.suffix.lower() in ['.jpg', '.jpeg', '.png']:
                    lbl_file = lbl_dir / (img_file.stem + '.txt')

                    if lbl_file.exists():
                        all_files.append({
                            'image': img_file,
                            'label': lbl_file
                        })

    print(f" Total de arquivos: {len(all_files)}")

    total = train_pct + val_pct + test_pct
    if abs(total - 100) > 0.1:
        factor = 100 / total
        train_pct *= factor
        val_pct *= factor
        test_pct *= factor

    random.seed(42)
    random.shuffle(all_files)

    n_total = len(all_files)
    n_train = int(n_total * train_pct / 100)
    n_val = int(n_total * val_pct / 100)
    n_test = n_total - n_train - n_val

    print(f"\n SEPARAÇÃO:")
    print(f"   Treino: {n_train} ({n_train/n_total*100:.1f}%)")
    print(f"   Validação: {n_val} ({n_val/n_total*100:.1f}%)")
    print(f"   Teste: {n_test} ({n_test/n_total*100:.1f}%)")

    train_files = all_files[:n_train]
    val_files = all_files[n_train:n_train+n_val]
    test_files = all_files[n_train+n_val:]

    output_path = Path(output_dir)

    for split in ['train', 'val', 'test']:
        (output_path / split / 'images').mkdir(parents=True, exist_ok=True)
        (output_path / split / 'labels').mkdir(parents=True, exist_ok=True)

    print(f"\n Criando dataset em: {output_path}")

    for split_name, files in [('train', train_files), ('val', val_files), ('test', test_files)]:
        print(f"\n Copiando {split_name}... ({len(files)} arquivos)")

        for item in files:
            img_dest = output_path / split_name / 'images' / item['image'].name
            shutil.copy2(item['image'], img_dest)

            lbl_dest = output_path / split_name / 'labels' / item['label'].name
            shutil.copy2(item['label'], lbl_dest)

    original_yaml = dataset_path / 'data.yaml'
    if original_yaml.exists():
        with open(original_yaml) as f:
            original_config = yaml.safe_load(f)
            class_names = original_config.get('names', ['cube', 'sphere'])
    else:
        class_names = ['cube', 'sphere']

    config = {
        'path': str(output_path.absolute()),
        'train': 'train/images',
        'val': 'val/images',
        'test': 'test/images',
        'nc': len(class_names),
        'names': class_names
    }

    yaml_path = output_path / 'data.yaml'
    with open(yaml_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"\n DATASET PRONTO!")
    print(f" Localização: {output_path}")
    print(f"  Classes ({len(class_names)}): {class_names}")

    try:
        shutil.rmtree(dataset_path)
    except:
        pass

    return str(output_path), class_names

dataset_path, class_names = download_and_split_dataset(
    api_key=API_KEY,
    workspace=WORKSPACE,
    project=PROJECT,
    version=VERSION,
    train_pct=TRAIN_PCT,
    val_pct=VAL_PCT,
    test_pct=TEST_PCT,
    output_dir=OUTPUT_DIR
)

print("\n" + "="*70)
print(" DATASET PREPARADO")
print("="*70)


print("="*70)
print("FILTRANDO DATASET - APENAS CUBOS E ESFERAS")
print("="*70)

def filter_cubes_spheres(dataset_path, output_path="dataset_filtered"):
    """
    Filtra dataset mantendo apenas cubos e esferas

    Args:
        dataset_path: Caminho do dataset original
        output_path: Caminho do dataset filtrado
    """
    dataset_path = Path(dataset_path)
    output_path = Path(output_path)

    # Ler configuração original
    yaml_path = dataset_path / 'data.yaml'
    with open(yaml_path) as f:
        config = yaml.safe_load(f)

    original_classes = config['names']
    print(f"\nClasses originais: {original_classes}")
    print(f"   Total: {len(original_classes)} classes")

    cube_id = None
    sphere_id = None

    for idx, class_name in enumerate(original_classes):
        class_lower = class_name.lower()
        if 'cube' in class_lower and 'cuboid' not in class_lower:
            cube_id = idx
        elif 'sphere' in class_lower or 'ball' in class_lower:
            sphere_id = idx

    if cube_id is None or sphere_id is None:
        print(f"\n Não encontrou cube ou sphere!")
        print(f"   Cube ID: {cube_id}")
        print(f"   Sphere ID: {sphere_id}")
        print(f"\n Classes disponíveis: {original_classes}")

        print(f"\n Tentando identificar classes...")
        for idx, name in enumerate(original_classes):
            print(f"   {idx}: {name}")

        if cube_id is None:
            for idx, name in enumerate(original_classes):
                if 'cube' in name.lower() and 'cuboid' not in name.lower():
                    cube_id = idx
                    break

        if sphere_id is None:
            for idx, name in enumerate(original_classes):
                if 'sphere' in name.lower():
                    sphere_id = idx
                    break

        if cube_id is None or sphere_id is None:
            print(f"\n❌ Erro: Não foi possível identificar cube e sphere!")
            print(f"   Defina manualmente os IDs corretos")
            return None

    print(f"\n Classes identificadas:")
    print(f"   Cube: ID {cube_id} ({original_classes[cube_id]})")
    print(f"   Sphere: ID {sphere_id} ({original_classes[sphere_id]})")

    keep_ids = {cube_id, sphere_id}

    id_mapping = {}
    if cube_id < sphere_id:
        id_mapping[cube_id] = 0  # cube
        id_mapping[sphere_id] = 1  # sphere
    else:
        id_mapping[sphere_id] = 0  # sphere (será remapeado para 1)
        id_mapping[cube_id] = 1    # cube (será remapeado para 0)

    final_mapping = {
        cube_id: 0,     # cube sempre 0
        sphere_id: 1    # sphere sempre 1
    }

    print(f"\n Mapeamento de IDs:")
    print(f"   {original_classes[cube_id]} ({cube_id}) → Cube (0)")
    print(f"   {original_classes[sphere_id]} ({sphere_id}) → Sphere (1)")

    # Estatísticas
    stats = {
        'total_images': 0,
        'kept_images': 0,
        'removed_images': 0,
        'cube_count': 0,
        'sphere_count': 0
    }

    for split in ['train', 'val', 'test']:
        img_dir = dataset_path / split / 'images'
        lbl_dir = dataset_path / split / 'labels'

        if not img_dir.exists():
            continue

        out_img_dir = output_path / split / 'images'
        out_lbl_dir = output_path / split / 'labels'
        out_img_dir.mkdir(parents=True, exist_ok=True)
        out_lbl_dir.mkdir(parents=True, exist_ok=True)

        print(f"\n Processando {split}...")

        # Processar cada imagem
        for img_file in img_dir.glob('*.*'):
            if img_file.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
                continue

            stats['total_images'] += 1

            lbl_file = lbl_dir / (img_file.stem + '.txt')

            if not lbl_file.exists():
                continue

            new_labels = []

            with open(lbl_file) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        class_id = int(parts[0])

                        if class_id in keep_ids:
                            new_id = final_mapping[class_id]
                            new_labels.append(f"{new_id} {parts[1]} {parts[2]} {parts[3]} {parts[4]}")

                            if new_id == 0:
                                stats['cube_count'] += 1
                            else:
                                stats['sphere_count'] += 1

            if new_labels:
                shutil.copy2(img_file, out_img_dir / img_file.name)

                with open(out_lbl_dir / (img_file.stem + '.txt'), 'w') as f:
                    f.write('\n'.join(new_labels) + '\n')

                stats['kept_images'] += 1
            else:
                stats['removed_images'] += 1

    new_config = {
        'path': str(output_path.absolute()),
        'train': 'train/images',
        'val': 'val/images',
        'test': 'test/images',
        'nc': 2,
        'names': ['cube', 'sphere']
    }

    new_yaml_path = output_path / 'data.yaml'
    with open(new_yaml_path, 'w') as f:
        yaml.dump(new_config, f, default_flow_style=False)

    print(f"\n" + "="*70)
    print(" ESTATÍSTICAS DA FILTRAGEM")
    print("="*70)
    print(f"\n Imagens:")
    print(f"   Total original: {stats['total_images']}")
    print(f"   Mantidas: {stats['kept_images']}")
    print(f"   Removidas: {stats['removed_images']}")
    print(f"   % mantido: {stats['kept_images']/stats['total_images']*100:.1f}%")

    print(f"\n  Objetos:")
    print(f"   Cubos: {stats['cube_count']}")
    print(f"   Esferas: {stats['sphere_count']}")
    print(f"   Total: {stats['cube_count'] + stats['sphere_count']}")

    print(f"\n Dataset filtrado salvo em: {output_path}")
    print(f" Configuração: {new_yaml_path}")

    return str(output_path)

filtered_dataset_path = filter_cubes_spheres(
    dataset_path=dataset_path,
    output_path="dataset_filtered"
)

if filtered_dataset_path:
    dataset_path = filtered_dataset_path

    print(f"\n" + "="*70)
    print(" FILTRAGEM CONCLUÍDA!")
    print("="*70)
    print(f"\n O treinamento agora usará apenas CUBOS e ESFERAS")
    print(f" Novo dataset: {dataset_path}")
else:
    print(f"\n" + "="*70)
    print("❌ ERRO NA FILTRAGEM")
    print("="*70)
    print(f"\n  O treinamento usará o dataset completo")


print("="*70)
print("🚀 INICIANDO TREINAMENTO")
print("="*70)

from pathlib import Path

filtered_path = Path("dataset_filtered")
if filtered_path.exists():
    dataset_path = "dataset_filtered"
    print(f"\n Usando dataset FILTRADO (apenas cube e sphere)")
    print(f" Caminho: {dataset_path}")

    yaml_path = filtered_path / 'data.yaml'
    if yaml_path.exists():
        import yaml
        with open(yaml_path) as f:
            config = yaml.safe_load(f)
        print(f"  Classes: {config.get('names', [])}")
        print(f" Número de classes: {config.get('nc', 0)}")
else:
    print(f"\n  Dataset filtrado não encontrado")
    print(f" Usando dataset original: {dataset_path}")
    print(f"\n Para treinar apenas com cube e sphere:")
    print(f"   Execute a célula de filtragem antes desta célula")

model = YOLO(f'{MODEL}.pt')

print(f"\n  Configurações de Treinamento:")
print(f"   Modelo: {MODEL}")
print(f"   Dataset: {dataset_path}")
print(f"   Épocas: {EPOCHS}")
print(f"   Batch: {BATCH_SIZE}")
print(f"   Imagem: {IMG_SIZE}x{IMG_SIZE}")
print(f"   Device: {DEVICE}")

print(f"\n Treinando...")
print(f"   (Isso pode levar alguns minutos...)\n")

results = model.train(
    data=f'{dataset_path}/data.yaml',
    epochs=EPOCHS,
    batch=BATCH_SIZE,
    imgsz=IMG_SIZE,
    device=DEVICE,
    project='runs/detect',
    name='cubes_spheres',
    exist_ok=True,
    pretrained=True,
    patience=20,
    save=True,
    plots=True,
    verbose=True
)

print("\n" + "="*70)
print("TREINAMENTO CONCLUÍDO!")
print("="*70)

print(f"\nResultados salvos em:")
print(f"   runs/detect/cubes_spheres/")
print(f"\n Melhor modelo:")
print(f"   runs/detect/cubes_spheres/weights/best.pt")
print(f"\n Métricas:")
print(f"   runs/detect/cubes_spheres/results.png")