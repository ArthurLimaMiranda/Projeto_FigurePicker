#!/usr/bin/env python3
"""
Script Completo de Retreinamento YOLOv11s
- Baixa dataset do Roboflow
- FILTRA apenas imagens com CUBOS e ESFERAS (descarta outras formas)
- Redistribui em 70% train / 20% val / 10% test
- Aplica augmentation APENAS nas esferas para equilibrar com cubos
- Treina o modelo

Autor: Sistema FigurePicker
"""

from roboflow import Roboflow
from ultralytics import YOLO
from pathlib import Path
import shutil
import random
import yaml
import cv2
import numpy as np
import torch
import sys
from datetime import datetime


API_KEY = "RGZ4JUpt6YMS1ZreoZAT"
WORKSPACE = "w0"
PROJECT = "3d-geom-shape-detector"
VERSION = 1

# Divisao do dataset
TRAIN_PCT = 70
VAL_PCT = 20
TEST_PCT = 10

# Diretorios
OUTPUT_DIR = "dataset_balanced"

# Modelo
MODEL = "yolo11s"

# Parametros de treinamento
EPOCHS = 150
BATCH_SIZE = 16
IMG_SIZE = 640

# Device
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# ============================================================================
# BANNER
# ============================================================================

def print_banner():
    print("\n" + "="*70)
    print("RETREINAMENTO YOLOV11S - DETECTOR CUBO/ESFERA")
    print("="*70)
    print(f"\nData/Hora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Modelo: {MODEL.upper()}")
    print(f"Device: {DEVICE}")
    print(f"Epocas: {EPOCHS}")
    print(f"Batch: {BATCH_SIZE}")
    print(f"\nDivisao do Dataset:")
    print(f"   Train: {TRAIN_PCT}%")
    print(f"   Val:   {VAL_PCT}%")
    print(f"   Test:  {TEST_PCT}%")
    print(f"\nBalanceamento:")
    print(f"   Augmentation APENAS em esferas")
    print(f"   Objetivo: igualar quantidade de cubos e esferas")
    print(f"\nFILTRAGEM ATIVA:")
    print(f"   - Mantendo apenas CUBOS e ESFERAS/BALLS")
    print(f"   - Descartando outras formas (triangulos, cilindros, etc)")
    print("="*70 + "\n")

print_banner()

# ============================================================================
# ETAPA 1: BAIXAR E IDENTIFICAR CLASSES
# ============================================================================

def download_and_identify_classes():
    """Baixa dataset e identifica IDs de cubos e esferas"""
    print("="*70)
    print("ETAPA 1/6: BAIXANDO DATASET DO ROBOFLOW")
    print("="*70)
    
    try:
        rf = Roboflow(api_key=API_KEY)
        print(f"\nConectado ao Roboflow")
        
        project_obj = rf.workspace(WORKSPACE).project(PROJECT)
        print(f"Baixando dataset...")
        
        dataset = project_obj.version(VERSION).download("yolov8")
        dataset_path = Path(dataset.location)
        
        print(f"\nDownload concluido: {dataset_path}")
        
        # Ler data.yaml para ver as classes
        yaml_file = dataset_path / 'data.yaml'
        with open(yaml_file) as f:
            data_config = yaml.safe_load(f)
        
        original_classes = data_config.get('names', [])
        print(f"\nClasses no dataset original:")
        for idx, name in enumerate(original_classes):
            print(f"   ID {idx}: {name}")
        
        # Identificar IDs de cubos e esferas
        cube_ids = []
        sphere_ids = []
        
        for idx, name in enumerate(original_classes):
            name_lower = name.lower()
            
            # Identificar CUBOS (qualquer cor)
            if 'cube' in name_lower and 'cuboid' not in name_lower:
                cube_ids.append(idx)
                print(f"\n   CUBO identificado: ID {idx} ({name})")
            
            # Identificar ESFERAS/BALLS
            elif 'sphere' in name_lower or 'ball' in name_lower:
                sphere_ids.append(idx)
                print(f"   ESFERA/BALL identificada: ID {idx} ({name})")
        
        if not cube_ids or not sphere_ids:
            print(f"\nERRO: Nao foi possivel identificar cubos e esferas!")
            print(f"Cubos encontrados: {cube_ids}")
            print(f"Esferas encontradas: {sphere_ids}")
            sys.exit(1)
        
        print(f"\nResumo da identificacao:")
        print(f"   IDs de CUBOS: {cube_ids}")
        print(f"   IDs de ESFERAS: {sphere_ids}")
        
        return dataset_path, cube_ids, sphere_ids, original_classes
    
    except Exception as e:
        print(f"\nERRO ao baixar dataset: {e}")
        sys.exit(1)

# ============================================================================
# ETAPA 2: COLETAR E FILTRAR ARQUIVOS
# ============================================================================

def collect_and_filter_files(dataset_path, cube_ids, sphere_ids):
    """Coleta arquivos e filtra apenas os que tem cubos e/ou esferas"""
    print("\n" + "="*70)
    print("ETAPA 2/6: FILTRANDO IMAGENS (APENAS CUBOS E ESFERAS)")
    print("="*70)
    
    all_files = []
    stats = {
        'total_images': 0,
        'kept_images': 0,
        'discarded_images': 0,
        'cube_only': 0,
        'sphere_only': 0,
        'both': 0,
        'other_classes': 0
    }
    
    valid_ids = set(cube_ids + sphere_ids)
    
    for split in ['train', 'valid', 'test']:
        img_dir = dataset_path / split / 'images'
        lbl_dir = dataset_path / split / 'labels'
        
        if not img_dir.exists():
            continue
        
        print(f"\nProcessando split: {split}")
        
        for img_file in img_dir.glob('*.*'):
            if img_file.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
                continue
            
            lbl_file = lbl_dir / (img_file.stem + '.txt')
            if not lbl_file.exists():
                continue
            
            stats['total_images'] += 1
            
            # Ler labels e verificar classes
            with open(lbl_file, 'r') as f:
                lines = f.readlines()
            
            # Extrair IDs das classes presentes
            present_classes = set()
            for line in lines:
                parts = line.strip().split()
                if len(parts) >= 5:
                    class_id = int(parts[0])
                    present_classes.add(class_id)
            
            # Verificar se tem APENAS cubos e/ou esferas
            has_cube = any(cid in present_classes for cid in cube_ids)
            has_sphere = any(sid in present_classes for sid in sphere_ids)
            has_other = any(cid not in valid_ids for cid in present_classes)
            
            # FILTRAR: manter apenas se tem cubo/esfera E nao tem outras classes
            if (has_cube or has_sphere) and not has_other:
                # Criar nova label remapeada
                new_labels = []
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        class_id = int(parts[0])
                        
                        # Remapear IDs: 0=cube, 1=sphere
                        if class_id in cube_ids:
                            new_class_id = 0
                        elif class_id in sphere_ids:
                            new_class_id = 1
                        else:
                            continue
                        
                        new_labels.append(f"{new_class_id} {parts[1]} {parts[2]} {parts[3]} {parts[4]}")
                
                if new_labels:
                    all_files.append({
                        'image': img_file,
                        'label': lbl_file,
                        'new_labels': new_labels,
                        'has_cube': has_cube,
                        'has_sphere': has_sphere
                    })
                    
                    stats['kept_images'] += 1
                    
                    if has_cube and has_sphere:
                        stats['both'] += 1
                    elif has_cube:
                        stats['cube_only'] += 1
                    elif has_sphere:
                        stats['sphere_only'] += 1
            else:
                stats['discarded_images'] += 1
                if has_other:
                    stats['other_classes'] += 1
    
    print(f"\n{'='*70}")
    print("ESTATISTICAS DA FILTRAGEM")
    print('='*70)
    print(f"\nTotal de imagens: {stats['total_images']}")
    print(f"Imagens mantidas: {stats['kept_images']}")
    print(f"Imagens descartadas: {stats['discarded_images']}")
    print(f"\nDetalhes das imagens mantidas:")
    print(f"   Apenas cubos: {stats['cube_only']}")
    print(f"   Apenas esferas: {stats['sphere_only']}")
    print(f"   Cubos e esferas: {stats['both']}")
    print(f"\nImagens com outras classes: {stats['other_classes']}")
    
    if stats['kept_images'] == 0:
        print("\nERRO: Nenhuma imagem foi mantida apos filtragem!")
        sys.exit(1)
    
    return all_files

# ============================================================================
# ETAPA 3: DIVIDIR DATASET EM 70/20/10
# ============================================================================

def split_dataset(all_files):
    """Divide arquivos em train/val/test (70/20/10)"""
    print("\n" + "="*70)
    print("ETAPA 3/6: DIVIDINDO DATASET (70/20/10)")
    print("="*70)
    
    random.seed(42)
    random.shuffle(all_files)
    
    n_total = len(all_files)
    n_train = int(n_total * TRAIN_PCT / 100)
    n_val = int(n_total * VAL_PCT / 100)
    n_test = n_total - n_train - n_val
    
    train_files = all_files[:n_train]
    val_files = all_files[n_train:n_train + n_val]
    test_files = all_files[n_train + n_val:]
    
    print(f"\nDivisao realizada:")
    print(f"   Train: {len(train_files):4} arquivos ({len(train_files)/n_total*100:.1f}%)")
    print(f"   Val:   {len(val_files):4} arquivos ({len(val_files)/n_total*100:.1f}%)")
    print(f"   Test:  {len(test_files):4} arquivos ({len(test_files)/n_total*100:.1f}%)")
    
    return {
        'train': train_files,
        'val': val_files,
        'test': test_files
    }

# ============================================================================
# ETAPA 4: FUNCAO DE AUGMENTATION
# ============================================================================

def augment_image(image):
    """Aplica augmentations aleatorios na imagem"""
    h, w = image.shape[:2]
    
    if random.random() < 0.5:
        image = cv2.flip(image, 1)
    
    alpha = random.uniform(0.7, 1.3)
    beta = random.randint(-30, 30)
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    
    if random.random() < 0.3:
        ksize = random.choice([3, 5])
        image = cv2.GaussianBlur(image, (ksize, ksize), 0)
    
    if random.random() < 0.3:
        noise = np.random.randn(h, w, 3) * 15
        image = np.clip(image.astype(float) + noise, 0, 255).astype(np.uint8)
    
    if random.random() < 0.5:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(float)
        hsv[:, :, 1] *= random.uniform(0.7, 1.3)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
        image = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    
    if random.random() < 0.4:
        angle = random.uniform(-10, 10)
        M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1.0)
        image = cv2.warpAffine(image, M, (w, h), borderMode=cv2.BORDER_REFLECT)
    
    return image

# ============================================================================
# ETAPA 5: CRIAR DATASET BALANCEADO
# ============================================================================

def create_balanced_dataset(split_files):
    """Cria dataset balanceado com augmentation"""
    print("\n" + "="*70)
    print("ETAPA 4/6: CRIANDO DATASET BALANCEADO")
    print("="*70)
    
    output_path = Path(OUTPUT_DIR)
    
    if output_path.exists():
        print(f"\nRemovendo dataset antigo...")
        shutil.rmtree(output_path)
    
    print(f"\nCriando estrutura de diretorios...")
    for split in ['train', 'val', 'test']:
        (output_path / split / 'images').mkdir(parents=True, exist_ok=True)
        (output_path / split / 'labels').mkdir(parents=True, exist_ok=True)
    
    print(f"Estrutura criada: {output_path.absolute()}")
    
    # Estatisticas
    stats = {
        'cubes_original': 0,
        'spheres_original': 0,
        'spheres_augmented': 0
    }
    
    # PASSO 1: Contar cubos e esferas no TRAIN para calcular augmentation
    print(f"\nPasso 1: Contando cubos e esferas no conjunto de treino...")
    
    train_cubes = 0
    train_spheres = 0
    
    for file_info in split_files['train']:
        if file_info['has_cube']:
            train_cubes += 1
        if file_info['has_sphere']:
            train_spheres += 1
    
    print(f"   Cubos no train: {train_cubes}")
    print(f"   Esferas no train: {train_spheres}")
    
    # Calcular fator de augmentation necessário
    if train_spheres > 0:
        aug_multiplier = max(1, int(train_cubes / train_spheres))
    else:
        aug_multiplier = 1
    
    print(f"\nFator de augmentation calculado: {aug_multiplier}x")
    print(f"Objetivo: {train_spheres} esferas -> {train_spheres * aug_multiplier} esferas")
    
    # PASSO 2: Processar cada split
    for split_name, files in split_files.items():
        print(f"\n{'='*70}")
        print(f"Processando {split_name.upper()}")
        print('='*70)
        
        out_img_dir = output_path / split_name / 'images'
        out_lbl_dir = output_path / split_name / 'labels'
        
        local_stats = {'cubes': 0, 'spheres': 0, 'aug': 0}
        
        for idx, file_info in enumerate(files, 1):
            img_file = file_info['image']
            has_cube = file_info['has_cube']
            has_sphere = file_info['has_sphere']
            new_labels = file_info['new_labels']
            
            if has_cube:
                local_stats['cubes'] += 1
                stats['cubes_original'] += 1
            if has_sphere:
                local_stats['spheres'] += 1
                stats['spheres_original'] += 1
            
            # Copiar imagem
            shutil.copy2(img_file, out_img_dir / img_file.name)
            
            # Salvar nova label (remapeada)
            new_lbl_file = out_lbl_dir / (img_file.stem + '.txt')
            with open(new_lbl_file, 'w') as f:
                f.write('\n'.join(new_labels) + '\n')
            
            # Augmentation APENAS em esferas E APENAS no TRAIN
            if split_name == 'train' and has_sphere:
                num_aug = aug_multiplier
                
                image = cv2.imread(str(img_file))
                if image is None:
                    continue
                
                for i in range(num_aug):
                    try:
                        aug_image = augment_image(image.copy())
                        
                        aug_img_name = f"{img_file.stem}_aug{i:02d}{img_file.suffix}"
                        aug_img_path = out_img_dir / aug_img_name
                        cv2.imwrite(str(aug_img_path), aug_image)
                        
                        aug_lbl_name = f"{img_file.stem}_aug{i:02d}.txt"
                        aug_lbl_path = out_lbl_dir / aug_lbl_name
                        with open(aug_lbl_path, 'w') as f:
                            f.write('\n'.join(new_labels) + '\n')
                        
                        local_stats['aug'] += 1
                        stats['spheres_augmented'] += 1
                    
                    except Exception as e:
                        print(f"   AVISO: Erro aug {i}: {e}")
            
            if idx % 50 == 0:
                print(f"   Processados: {idx}/{len(files)}...", end='\r')
        
        print(f"\n{split_name}: {len(files)} originais + {local_stats['aug']} augmentados")
        print(f"   Cubos: {local_stats['cubes']}, Esferas: {local_stats['spheres']}")
    
    print(f"\n{'='*70}")
    print("ESTATISTICAS FINAIS DO BALANCEAMENTO")
    print('='*70)
    
    total_cubes = stats['cubes_original']
    total_spheres = stats['spheres_original'] + stats['spheres_augmented']
    
    print(f"\nCUBOS:")
    print(f"   Total: {total_cubes:4}")
    
    print(f"\nESFERAS:")
    print(f"   Originais:   {stats['spheres_original']:4}")
    print(f"   Augmentadas: {stats['spheres_augmented']:4}")
    print(f"   Total:       {total_spheres:4}")
    
    ratio = total_cubes / total_spheres if total_spheres > 0 else 0
    print(f"\nPROPORCAO FINAL:")
    print(f"   Cubos:Esferas = {ratio:.2f}:1")
    
    if 0.8 <= ratio <= 1.2:
        print(f"   STATUS: Perfeitamente balanceado!")
    elif 0.5 <= ratio <= 1.5:
        print(f"   STATUS: Bem balanceado")
    else:
        print(f"   STATUS: Balanceamento razoavel")
    
    config = {
        'path': str(output_path.absolute()),
        'train': 'train/images',
        'val': 'val/images',
        'test': 'test/images',
        'nc': 2,
        'names': ['cube', 'sphere']
    }
    
    yaml_path = output_path / 'data.yaml'
    with open(yaml_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f"\nYAML criado: {yaml_path}")
    
    print(f"\nVerificacao final:")
    for split in ['train', 'val', 'test']:
        img_count = len(list((output_path / split / 'images').glob('*.*')))
        lbl_count = len(list((output_path / split / 'labels').glob('*.txt')))
        print(f"   {split:5} -> {img_count:4} imagens, {lbl_count:4} labels")
        
        if img_count == 0:
            print(f"   ERRO: {split} esta vazio!")
            sys.exit(1)
    
    return output_path, yaml_path

# ============================================================================
# ETAPA 6: TREINAR MODELO
# ============================================================================

def train_model(yaml_path):
    """Treina o modelo YOLOv11"""
    print("\n" + "="*70)
    print("ETAPA 5/6: TREINANDO MODELO")
    print("="*70)
    
    print(f"\nCarregando {MODEL}.pt...")
    model = YOLO(f'{MODEL}.pt')
    print(f"Modelo carregado!")
    
    batch_size = BATCH_SIZE
    
    print(f"\nIniciando treinamento...")
    print(f"{'='*70}\n")
    
    try:
        results = model.train(
            data=str(yaml_path),
            epochs=EPOCHS,
            batch=batch_size,
            imgsz=IMG_SIZE,
            device=DEVICE,
            project='runs/detect',
            name=f'cubes_spheres_{MODEL}',
            exist_ok=True,
            pretrained=True,
            patience=30,
            save=True,
            plots=True,
            verbose=True,
            
            lr0=0.01,
            lrf=0.01,
            momentum=0.937,
            weight_decay=0.0005,
            warmup_epochs=3,
            box=7.5,
            cls=0.5,
            dfl=1.5,
            
            hsv_h=0.015,
            hsv_s=0.7,
            hsv_v=0.4,
            degrees=10,
            translate=0.1,
            scale=0.5,
            fliplr=0.5,
            mosaic=0.5,
            mixup=0.1,
        )
        
        return results
    
    except Exception as e:
        print(f"\nERRO: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

# ============================================================================
# ETAPA 7: VALIDAR
# ============================================================================

def validate_model():
    """Valida o modelo"""
    print("\n" + "="*70)
    print("ETAPA 6/6: VALIDANDO MODELO")
    print("="*70)
    
    model_path = Path(f'runs/detect/cubes_spheres_{MODEL}/weights/best.pt')
    
    if not model_path.exists():
        print(f"\nModelo nao encontrado")
        return
    
    model = YOLO(str(model_path))
    metrics = model.val()
    
    print(f"\n{'='*70}")
    print("METRICAS FINAIS")
    print('='*70)
    print(f"\n   mAP50-95:  {metrics.box.map:.3f}")
    print(f"   mAP50:     {metrics.box.map50:.3f}")
    print(f"   Precisao:  {metrics.box.mp:.3f}")
    print(f"   Recall:    {metrics.box.mr:.3f}")
    
    # Calcular acuracia e F1
    try:
        precision = metrics.box.mp
        recall = metrics.box.mr
        
        if precision > 0 and recall > 0:
            f1_score = 2 * (precision * recall) / (precision + recall)
            print(f"   F1-Score:  {f1_score:.3f}")
            
            accuracy_approx = (precision + recall) / 2
            print(f"   Acuracia (aprox): {accuracy_approx:.3f}")
    except:
        pass
    
    # Metricas por classe
    if hasattr(metrics.box, 'maps') and len(metrics.box.maps) > 0:
        print(f"\n   Por Classe:")
        class_names = ['cube', 'sphere']
        for i, (name, map_val) in enumerate(zip(class_names, metrics.box.maps)):
            print(f"      {name:8} - mAP50-95: {map_val:.3f}")

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Pipeline principal"""
    try:
        # Etapa 1: Download e identificar classes
        dataset_path, cube_ids, sphere_ids, original_classes = download_and_identify_classes()
        
        # Etapa 2: Coletar e filtrar
        all_files = collect_and_filter_files(dataset_path, cube_ids, sphere_ids)
        
        # Etapa 3: Dividir em 70/20/10
        split_files = split_dataset(all_files)
        
        # Etapa 4: Criar dataset balanceado
        output_path, yaml_path = create_balanced_dataset(split_files)
        
        # Etapa 5: Treinar
        results = train_model(yaml_path)
        
        # Etapa 6: Validar
        validate_model()
        
        print("\n" + "="*70)
        print("PROCESSO COMPLETO FINALIZADO!")
        print("="*70)
        print(f"\nModelo: runs/detect/cubes_spheres_{MODEL}/weights/best.pt")
        print(f"Graficos: runs/detect/cubes_spheres_{MODEL}/results.png")
        print("\n" + "="*70)
    
    except KeyboardInterrupt:
        print("\n\nInterrompido pelo usuario")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nErro fatal: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()